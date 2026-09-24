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
| 角速度の上限 | `MaxAngularVelRadS` | `robot.max_angular_velocity` |
| 車輪周速の予算 | `WheelRimSpeedBudgetMmS` | `robot.wheel_rim_speed_budget_mm_s` |
| 一次遅れ | `TauVxSec` `TauVySec` `TauOmegaSec` | `robot.tau_vx` `tau_vy` `tau_omega` |
| 軸別の牽引限界 | `TractionAccelXMmS2` `TractionAccelYMmS2` `TractionDecelXMmS2` `TractionDecelYMmS2` | `robot.traction_accel_x/y_mm_s2` `traction_decel_x/y_mm_s2` |
| 角加速度の上限 | `MaxAngularAccelRadS2` | `robot.max_angular_acceleration` |

読み方は 2 段。`[RobotModel]` が全 ID の既定値、`[RobotModel.<id>]` はその ID の**差分**で、
書かれたキーだけが既定値を上書きする。`Enabled=false` でモデルを丸ごと切ると、
従来の `MotionControl`（等方な加減速制限）経由の挙動に戻る。

### ⚠ RAVEN 側とペアで設定する

**RAVEN が使うモデルと sim の実際のモデルを揃える。** RAVEN の MPC と EKF は、RAVEN が
選んだ `Target.SIM` に対応する機体モデルを前提にする。RAVEN の sim 経路は共通の
`system_model_sim.yaml` をベースにし、`system_model_sim_ID<n>.yaml` の `robot` 節を ID ごとに
重ねる。sim 専用ファイルが無いと同じ ID の実機ファイルへフォールバックするため、real と sim の
値が異なると意図しないずれが起きる。

`tools/gen_robot_models.py --write-raven` は全 ID の sim 専用 overlay を生成する。現在は、実機 ID 2 の
同定値を全 ID に同じように適用する。これにより、sim 上の全機が少なくとも ID 2 の動きをする基準を作る。
個体差はまだモデルに入れない。RAVEN の sim 実行は real 用ファイルに依存しない。
**実機モードのモデルファイルはこの生成で変更しない。**

`SimRobotModelCoverageTest` は sim と RAVEN のモデルを ID ごとに照合する。比較する項目は、時間定数、
むだ時間、定常ゲインと軸間ゲイン、前後・横の加減速限界、車輪周速予算、角速度・角加速度の上限。
値を変更したら、片側だけ編集せず **`tools/gen_robot_models.py`** から両方を生成する。

```bash
python3 tools/gen_robot_models.py --write-raven <ssl-RAVEN>/app/config
```

このスクリプトが `[RobotModel.<id>]` と RAVEN の yaml を**同じ表から**出す。
モデルファイルの一致は「RAVEN と sim が同じ台を想定している」ことの確認であり、
その表が現在の実機を再現していることは別途実測で確かめる。

### 全機を ID 2 のモデルに揃える

`[RobotModel.0]` から `[RobotModel.15]` まで、物理パラメータはすべて ID 2 の同定値を使う。
これは「どの機体も実際にID 2と完全に同じ」という主張ではなく、**まず全機で ID 2 の動きを再現できる**
ことを目標にした基準モデル。個体差を反映するのは、各機の計測値や根拠が揃ってからにする。

ID 2 のゲイン (`0.88 / 0.83`) は 2026-09-19 の試合記録から更新され、加速限界
(`4269 / 3740 mm/s²`) は同日の段差走行から置いた暫定値。以前のsim ID 2は9月18日の値
(`0.756 / 0.638`, `5549 / 3740 mm/s²`) のまま残っていたため、今回の比較開始に合わせて更新した。
減速限界 `6000 mm/s²` はRAVENの共通既定値を使った仮置きで、ID 2の実測値ではない。

モデル表の値が最新の実機を再現しているかは、RAVEN と sim の値を揃えるテストだけでは証明できない。
実機の動きとの比較で別途確かめる。

### 注意

- **青黄ともに同じ ID には同じ台**を割り当てる。相手チームも実機相当になる。
- ID 2 基準の回転速度上限は 8.42 rad/s、角加速度上限は 35 rad/s²。角加速度は実測値ではなく、
  60 Hz の Vision では立ち上がりを十分に分解できないため置いた共通の計画上限。
- むだ時間 0.087 s は RAVEN が**自分の閉ループごと**同定した値で、vision → 判断 →
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
- sim を実際に走らせた実測（ボールだけ、3000 mm/s で配置、vision から計測）でも、
  同じ距離での速さが RAVEN の `speedAfterTravel` と **0.3〜−5%** で一致する。
  距離が伸びるほど sim がわずかに遅い（8.8 m で −8.5%）ぶんは未解明で、転がり相に
  a_roll の数 % ぶんの余分な減速が乗っている。
- 台と球が組み合わさったときの挙動（MPC を実際に走らせたときの追従）は
  **RAVEN を繋いで走らせないと確かめられない**。まずは ID 2 / 4 / 11 を使って、
  実機のログと並べるのが早い。

## 未解決

- **ボールがロボットの口に正面から入ったときの跳ね返りが 0.8 にならない。**
  0918 実測: 3000 mm/s でロボットへ撃つと、跳ね返りは 0.41 相当。材質は
  球・ロボットとも `DirectKickNormalRestitution` (0.8) なので、PhysX の平均としては
  0.8 のはず。口のドリブラ／取り合いの判定が噛んでいる可能性が高く、材質の問題とは
  切り分けられていない。側面に当てたときの値は未計測。
- 転がり相の −数 % の余分な減速（上記）。
