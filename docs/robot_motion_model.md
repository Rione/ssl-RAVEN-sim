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
| 並進の速さの上限 (0 は上限なし) | `MaxLinearVelMmS` | — |
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
| `AccSlideMmS2` | `ball_model.acc_slide_mm_s2` | -5675 |
| `AccRollMmS2` | `ball_model.acc_roll_mm_s2` | -297 |
| `KSwitch` | `ball_model.k_switch` | 0.54 |
| `DirectKickNormalRestitution` | `ball_model.direct_kick.normal_restitution` | 0.8 |
| `DirectKickTangentRetention` | `ball_model.direct_kick.tangent_retention` | 1.0 |

肝は**切り替えの基準が蹴り出しの速さ v0** であること（`v_switch = KSwitch · v0`）。
物理的な転がり条件（剛球なら 5/7·v0）とは別物で、RAVEN の `ball_model.k_switch` は 0.54 なので
sim もそれに従う。v0 は `ballLaunchSpeed` が持ち、キック・速度つき配置・衝突・
外から押されたとき、のいずれでも取り直す。

摩擦係数ではなく **mm/s² の減速度そのもの**を合わせるのが要点。RAVEN はパスの初速逆算
（`initialSpeedFor`）も到達時刻（`arrivalTime`）も到達速度（`speedAfterTravel`）も
この 3 つの数から引いているので、ここがずれると「ここで受け取れる」が毎回外れる。

### 口の板（`DirectKickNormalRestitution` = e、`DirectKickTangentRetention` = t）

機体の向きを板の法線 f、その左を板に沿う向きとする。

- **蹴らないときの跳ね返り**は PhysX の材質で入れる。口には垂直な板（`BoxShape`、前の面は中心から 74 mm）を
  置いてあり、球はそこに当たる。止まった機体の板で跳ね返った球の、板に垂直な速さの比が e になるよう、
  ロボット側（`botMaterial`）の反発を決める。PhysX は 2 つの材質の反発の平均を「2 つの物の離れる速さ / 近づく速さ」に
  使い、機体 (M = 2.5 kg) も球 (m = 0.046 kg) に押し返されるので、球だけを見た比は (e_材·M − m) / (M + m)。
  そこで e_材 = e·(1 + m/M) + m/M とし、球の材質（`BallRestitution`、壁との跳ね返りのため）との平均が
  e_材 になるよう `botMaterial` = 2·e_材 − `BallRestitution`（0〜1 に収まる範囲。e が小さく
  `BallRestitution` が大きいと届かない）。摩擦は球・ロボットとも 0 なので、沿う成分は落ちない（t は使わない）。
- **蹴ったとき**は `directKickVelocity()` が球の速度を直接決める。板のその点の速度 p（機体の並進 + 回転の ω×r）に
  対する来た球の速度を、垂直の成分 v_n（板へ向かうと負）と沿う成分 v_t に分け、
  `出る球 = p + (−e·v_n + v_k)·f + t·v_t·(左)`。v_k は蹴りの前への速さ（指令を `MaxLinearKickSpeed` /
  `MaxChipKickSpeed` で押さえ、`KickerFriction` を掛けたもの）、チップの上への速さはそのまま上向き。
  来た速度は直近 3 刻みのうち板へ最も強く向かっていたもの（窓に入ったと判じた刻みでは、PhysX がもう跳ね返して
  いることがある）。球が板へ向かっていない（持っている・止まっている・離れていく）ときの垂直の成分は v_k と
  今の速さの大きい方なので、止まった機体が止まった球や持った球を蹴れば、出るのは蹴りの速さそのもの。
  RAVEN の `Rebound`（`出る球 = (v_k + e·v_n)·f + (t·v_t)·n`）を、動く機体に広げた形。
- 鍵が無いときは e = 0.8・t = 1.0（RAVEN の既定）。

球の材質の摩擦も 0。地面の接線力は `applyBallFriction()` が丸ごと持っているため
（PhysX 側にも摩擦があると滑走相が二重に減速する）。

設定パネルの `Ball Slide Decel` / `Ball Roll Decel` がこの 2 つの減速度。
（以前の `Ball Dynamic Friction` / `Rolling Friction` は係数だったので、置き換えた。）

## Sumatra の前提に揃える（`--model sumatra`）

TIGERs の Sumatra は sim に機体の能力を送らず、自分の表（Sumatra の
`config/botParamsDatabase.json` の `"Simulation"`）で計画する。sim の機体がそれより弱いと、
Sumatra の計画が sim の都合で外れる。Sumatra と試合をさせるときは、両チームの機体をこの表に揃える。

```bash
python3 tools/gen_robot_models.py --model sumatra --write-ini
```

`config_v2.ini` の `[RobotModel*]`・`[Physics]` の蹴る捕るの鍵・`[BallModel]` の減速を表の値で
書き換える（ほかの行には触らない）。`[RobotModel.<id>]` は消し、全番号・両チーム共通の
`[RobotModel]` 1 つにする。値と出どころは `tools/gen_robot_models.py` の `SUMATRA_*` と ini の注釈。

| 項目 | 値 | Sumatra |
|---|---|---|
| むだ時間・一次遅れ・ゲイン | 0・0・1 | 遅れとゲインの項が無い（指令どおりに動く台） |
| 加速 / 減速 | 3500 / 6000 mm/s²（前後・横とも） | `accMaxFast` / `brkMax` |
| 並進の速さの上限 | 4000 mm/s | `velMaxFast` |
| 角速度 / 角加速度 | 20 rad/s / 50 rad/s² | `velMaxW` / `accMaxW` |
| 車輪周速の予算 | 0（なし） | 並進と回転を別々に縛るだけ |
| 蹴りの初速の上限 | ストレート 7.5 m/s・チップ 5.5 m/s（3 次元） | `maxAbsoluteStraightVelocity` / `maxAbsoluteChipVelocity` |
| キッカーの再充電 | 0 s | sim の機体はいつも満充電 |
| 捕れる相対速度 | 4000 mm/s | パスの受け側は最大 3.2 m/s |
| 球の減速 | 滑り −3000・転がり −260・切り替え 0.64 | `BallParameters`（sim の geometry は球のモデルを送らない） |
| 口の板の反発 / 沿う成分の保持 | 0.47 / 1.0（実機の板、RAVEN の `system_model_real.yaml`） | Sumatra は 0.55 / 0.35 と予測する（`BallParameters` の SIMULATOR、止めずに蹴る計画は `ConstantLossRedirectConsultant`） |

蹴る・捕るの鍵（`[Physics]`、鍵が無いときの既定は括弧内）:

- `DribblerCatchMaxSpeedMmS`（1500）… 口の窓に入ってきたときの、機体の上で球と重なる点に対する
  球の速さがこれ未満なら捕れる。これ以上で入った球は跳ね返り、窓を出るまで捕れない（蹴るのはよい）。
- `KickerRechargeSec`（1）… 蹴ってから次を蹴れるまで。ドリブラは止めない。
  蹴った球は、口の窓を出るまでその台が捕らず蹴り直さない（こちらは時間でなく窓が空いたかで決まる）。
- `MaxLinearKickSpeed` / `MaxChipKickSpeed`（10 m/s）… 両方の指令の口（mocSim と
  ssl-simulation-protocol）に同じく掛かる。
- `KickerFriction`（0.8）… 蹴りの初速に掛ける係数。

RAVEN 側の台の模型（`system_model_sim_ID<n>.yaml`）も同じ表から出す:
`python3 tools/gen_robot_models.py --model sumatra --write-raven <ssl-RAVEN>/app/config`。
RAVEN の `SimRobotModelCoverageTest` は `[RobotModel.<id>]` だけを読むので、共通の 1 節の ini では
何も照合せずに通る。同じテストは RAVEN の sim 用の模型が ID 2 の基準と同じことも確かめているので、
Sumatra の表で生成した RAVEN の模型はそこで落ちる。

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

- 機体の側面（口の板の外）に当てたときの跳ね返りは測っていない。材質は口の板と同じ。
- 転がり相の −数 % の余分な減速（上記）。
