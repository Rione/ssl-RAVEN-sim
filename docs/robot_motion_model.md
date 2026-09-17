# 実機同定モデル（ロボット運動・ボール）

RAVEN の MPC（`app/config/control.yaml` の `optimal_control`）は実機で詰めた設定になっている。
その MPC が前提にしている「台」は `app/config/system_model_real_<mac>_ID<n>.yaml`、
「球」は `system_model` の `ball_model`（コードは `common/physics/BallSpeedModel` /
`BallPhysics`）で同定済みのもの。

sim の台が指令どおり即座に動き、球が別の落ち方をしていると、MPC は実機ぶんの補償を
そのまま出して行き過ぎる。**実機で正しい設定なのに sim で変な挙動になる、の正体がこれ。**
だから sim 側を実機に寄せる。合わせるのは sim であって、RAVEN の設定ではない。

## 台（`[RobotModel]` / `[RobotModel.<id>]`）

指令から実際の機体速度までを、RAVEN の同定モデルと同じ順で通す
（`src/models/robot.cpp::advanceActuation`）。

| 段 | 設定キー | RAVEN の yaml |
|---|---|---|
| むだ時間 | `DeadTimeSec` | `robot.input_dead_time_sec` |
| 定常ゲイン | `GainVx` `GainVy` `GainVxFromUy` `GainVyFromUx` | `robot.gain_vx` `gain_vy` `gain_vx_from_uy` `gain_vy_from_ux` |
| 角速度の上限 | `MaxAngularVelRadS` | `physics.yaml: max_angular_velocity` |
| 車輪周速の予算 | `WheelRimSpeedBudgetMmS` | `robot.wheel_rim_speed_budget_mm_s` |
| 一次遅れ | `TauVxSec` `TauVySec` `TauOmegaSec` | `robot.tau_vx` `tau_vy` `tau_omega` |
| 軸別の牽引限界 | `TractionAccelXMmS2` `TractionAccelYMmS2` `TractionDecelXMmS2` `TractionDecelYMmS2` | `robot.traction_accel_x/y_mm_s2` `traction_decel_x/y_mm_s2` |
| 角加速度の上限 | `MaxAngularAccelRadS2` | `physics.yaml: max_angular_acceleration` |

読み方は 2 段。`[RobotModel]` が全 ID の既定値、`[RobotModel.<id>]` はその ID の**差分**で、
書かれたキーだけが既定値を上書きする。`Enabled=false` でモデルを丸ごと切ると、
従来の `MotionControl`（等方な加減速制限）経由の挙動に戻る。

### ID ごとの割り当て

実機は 3 台ぶんしか同定値が無く、しかもそれぞれ**世代が違う**。

| 世代 | 母体 | 性格 |
|---|---|---|
| A | ID2 (`d83add4cb8bd`) | 牽引が強い（前後 5670 / 横 5104 mm/s²）が、ゲインが低く（0.747 / 0.629）むだ時間が長い（0.141 s） |
| B | ID4 (`d83add1a09be`) | 前後は素直（ゲイン 0.942）だが横が弱い（0.569、牽引 1715 mm/s²） |
| C | ID11 (`e0d55de88825`) | いちばん軽い応答（τ 0.030 / 0.025 s）で牽引が最も低い（1215 mm/s²） |

- **ID 2 / 4 / 11 は同定値そのまま。**
- それ以外の ID は A/B/C のどれかを母体にしたクローン。id とキー名から決まる固定の
  ばらつき（τ・むだ時間 ±8〜10%、牽引 ±12%、ゲイン ±5%、周速予算 ±4%）を掛けてある。
  実行のたびに変わらないので、走らせ比べが成立する。
- 生成スクリプトの中身（世代の割り当てとばらつきの幅）は本ファイルの表と
  `config/config_v2.ini` のコメントが唯一の記録。値を作り直したいときはこの表の母体値から
  同じ手順でよい。

### 注意

- **青黄ともに同じ ID には同じ台**を割り当てる。相手チームも実機相当になる。
- むだ時間 0.11〜0.14 s は RAVEN が**自分の閉ループごと**同定した値で、vision → 判断 →
  無線 → ドライバまでを含む。sim では経路の一部が存在しないが、RAVEN の MPC が補償して
  いるのはこの値なので、同じ値を再現するのが MPC から見て正しい台になる。
  ここは仮定なので、実測と突き合わせるなら `DeadTimeSec` から調整する。
- 1 m/s 級のステップでは**牽引限界が先に効く**ので、立ち上がりは τ ではなく
  `TractionAccel*` で決まる。τ が支配するのは小さい指令変化のほう。
- `[Physics]` の `AccSpeedupAbsoluteMax` などは `MotionControl`（`config/config.ini` を読む
  別系統）の設定で、`Enabled=true` のあいだは経路に入らない。

## 球（`[BallModel]`）

RAVEN の `BallSpeedModel` と同じ **滑走 → 転がりの 2 段一定減速**モデル
（`src/qml/sim/GameObjects.qml::applyBallFriction`）。

| 設定キー | RAVEN | 既定値 |
|---|---|---|
| `AccSlideMmS2` | `ball_model.acc_slide_mm_s2` | -2159.324207613644 |
| `AccRollMmS2` | `ball_model.acc_roll_mm_s2` | -213.609470182153 |
| `KSwitch` | `ball_model.k_switch` | 2/3 |
| `DirectKickNormalRestitution` | `ball_model.direct_kick.normal_restitution` | 0.8 |
| `DirectKickTangentRetention` | `ball_model.direct_kick.tangent_retention` | 1.0 |

肝は**切り替えの基準が蹴り出しの速さ v0** であること（`v_switch = KSwitch · v0`）。
物理的な転がり条件（剛球なら 5/7·v0）とは別物で、RAVEN が 2/3 で同定しているので
sim もそれに従う。v0 は `ballLaunchSpeed` が持ち、キック・速度つき配置・衝突・
外から押されたとき、のいずれでも取り直す。

摩擦係数ではなく **mm/s² の減速度そのもの**を合わせるのが要点。RAVEN はパスの初速逆算
（`initialSpeedFor`）も到達時刻（`arrivalTime`）も到達速度（`speedAfterTravel`）も
この 3 つの数から引いているので、ここがずれると「ここで受け取れる」が毎回外れる。

跳ね返りは PhysX の材質で入れる。PhysX は接触する 2 つの材質を平均するので、
ロボット側（`botMaterial`）の反発係数は `2·DirectKickNormalRestitution − BallRestitution`
にしてある。`tangent_retention = 1.0` は「接線方向は落ちない」なので、球とロボットの
摩擦はどちらも 0。地面の接線力は `applyBallFriction()` が丸ごと持っているため、
球の材質の摩擦も 0（PhysX 側にも摩擦があると滑走相が二重に減速する）。

設定パネルの `Ball Slide Decel` / `Ball Roll Decel` がこの 2 つの減速度。
（以前の `Ball Dynamic Friction` / `Rolling Friction` は係数だったので、置き換えた。）

## 検証の入口

- `Robot::advanceActuation` … むだ時間・定常ゲイン・軸別の牽引限界・周速予算・素通しは
  数値で確認済み（ID4 のステップ応答で むだ時間 0.117 s、定常 942 / 569 mm/s、
  加速ピーク 3971 / 1715 mm/s²、周速 2250 mm/s 打ち止め）。
- `applyBallFriction` の積分（switch をまたぐフレームを刻む処理）は RAVEN の閉形式
  `restTravelDistance` と停止距離で 0.07% 以内まで一致する。
- 台と球が組み合わさったときの挙動（MPC を実際に走らせたときの追従）は
  **RAVEN を繋いで走らせないと確かめられない**。まずは ID 2 / 4 / 11 を使って、
  実機のログと並べるのが早い。
