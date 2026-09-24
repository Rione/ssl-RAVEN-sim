# シミュレータの Vision 更新周期と時間刻み

> **更新日:** 2026-09-24
> **調査対象:** `ssl-RAVEN-sim` の描画・物理更新・Vision 送信と、RAVEN 受信の一部
> **主な実測環境:** Mac 1 台、青・黄各 11 台
> **ブランチ:** `fix/improvement-sim`（時間刻み対応コミット `38e5da1`）

## 先に要点

- Vision は独立した 60 Hz タイマーからではなく、`PhysicsWorld.onFrameDone` ごとに 1 パケット送られる。
- Qt Quick 3D Physics は描画と同期し、描画フレーム 1 回につき物理更新は最大 1 回。PC の描画性能やフレームの揺れが Vision の実時間 Hz に影響しうる。
- 以前は物理更新の実際の刻み幅に関係なく、ロボットモデルや Vision 時刻を 1/60 秒としていた。現在のブランチでは Qt から受け取った刻み幅を物理モデルと `t_capture` に渡す。
- 修正後の Mac で、Vision を約 10 秒直接受信した結果は **601 パケット、60.00 Hz**。短い黄チーム機体の移動指令テストでも、指令後の位置変化を Vision で確認した。
- Windows で「50〜60 Hzだった」という報告があるが、同時刻の FPS や詳細ログは未取得。原因はまだ確定していない。

## 更新の流れ

```text
描画フレーム
  └─ PhysicsWorld.onFrameDone(timestep [ms])
      ├─ GameObjects.updateGameObjects(timestep)
      └─ GameObjects.syncGameObjects(timestep)
          └─ Observer.updateObjects(..., timestepMs)
              ├─ ロボット運動・合成エンコーダの時間刻みを計算
              └─ Sender.send(..., simDtSec)
                  └─ Vision を送信し、t_capture += simDtSec
```

Qt の `PhysicsWorld` は描画と lockstep で動き、「描画フレームの完了後にシミュレーションフレームが始まり、描画 1 回につき物理更新は最大 1 回」と説明されている。`frameDone` の引数はシミュレーション時間刻み（ms）。[Qt PhysicsWorld の仕様](https://doc.qt.io/qt-6/qml-qtquick3d-physics-physicsworld.html)

現在の `src/qml/Main.qml` は `maximumTimestep = 1000/60 ms`、`minimumTimestep = 14 ms`。60 Hz を目標に小さな揺れを吸収する設定だが、60 Hz を保証するものではない。描画が遅い環境で 1 描画フレーム中に複数回シミュレーションを進める構造でもない。

画面上の **FPS** は `FrameAnimation.smoothFrameTime` から計算し、**Vision TX** は実際に成功した送信数を 500 ms ごとに集計する。Qt は `FrameAnimation` を FPS の測定にも使えるとしている。[Qt FrameAnimation の仕様](https://doc.qt.io/qt-6/qml-qtquick-frameanimation.html) ただし FPS と Vision TX は平滑化・集計の方法が異なるので、画面上の瞬間値を厳密に同じ時間窓の測定値として比較しないこと。

## 実装した変更

コミット `38e5da1` (`fix(sim): use actual physics timestep for vision`) に含まれる変更:

1. Qt の `onFrameDone(timestep)` をゲーム更新とオブジェクト同期へ渡す。
2. `Observer` で ms から秒へ変換し、ロボット運動モデル、合成エンコーダ、Vision 送信に使う。
3. `Sender` の `t_capture` を固定 `captureCount / 60` から、受け取った `simDtSec` の累積へ変更する。`t_sent` は引き続き `t_capture` と同じ値。
4. FPS 表示を固定値由来の表示から `FrameAnimation` による描画周期の測定へ変更。
5. PhysicsWorld の minimum timestep を 16.667 ms から 14 ms に下げ、名目 60 Hz 付近の小さな揺れで更新機会を逃しにくくする。

不正または 0 以下の時間刻みは、Observer では 1/60 秒へフォールバックし、Sender では不正値の送信を拒否する。

## 実測

### 修正前

2026-09-23、Mac、青・黄各 11 台、Vision マルチキャスト `224.5.23.2:10694` を直接受信。RAVEN は未接続。

| 計測 | 受信数 | 計測区間 | 受信 Hz |
|---|---:|---:|---:|
| 1 回目 | 1,316 | 29.992 秒 | 43.845 |
| 2 回目 | 1,258 | 28.909 秒 | 43.482 |

2 回目は `frame_number` に欠落がなく、受信プログラムによる取りこぼしよりも、シミュレータの送信頻度自体が約 43.5 Hz だったと判断した。`t_capture` の実時間に対する進みは約 0.725 倍だった。

### 修正後

2026-09-24、Mac 上で Vision マルチキャストを直接受信して約 10.010 秒計測。

| 項目 | 結果 |
|---|---:|
| 受信パケット | 601 |
| 実時間での受信頻度 | 60.00 Hz |
| `t_capture` の時間軸上の頻度 | 60.58 Hz |
| `t_capture` / 実時間の比 | 0.9904 |
| `t_capture` の平均刻み | 16.508 ms |
| `frame_number` の最大飛び | 1（欠番なし） |
| `t_sent - t_capture` | 0 ms |

また、シミュレータへ黄チーム ID 0 の前進指令（0.4 m/s、約 0.6 秒）を送り、Vision 上で `x` が約 208.7 mm、`y` が約 16.7 mm変化することを確認した。これは移動指令から物理更新、Vision 観測までの短い経路テストであり、RAVEN の戦略全体や対戦性能を確認したものではない。

### RAVEN との接続

RAVEN を headless で起動した際、Vision receiver が最初の Vision データを受信したログを確認した。一方、Field View の Swing 画面は headless 環境のため `HeadlessException` となり、戦略の起動・対戦までは検証していない。

接続直後に出た時計ずれ警告（比率 0.9614）は短い計測窓の値だった。別途行った約 10 秒の直接受信では 0.9904 だったため、短時間の警告を定常値として扱わない。

## Windows と Mac の差について

Windows で 50〜60 Hz、Mac ではほぼ 60 Hzという利用者からの報告がある。Windows 側の機器・Qt/FPS・同じ条件の連続ログは未取得であり、現時点で原因は断定できない。

コードと Qt の実行モデルからは、Windows で描画フレームの頻度や間隔が揺れ、それに追従して物理更新・Vision 送信も揺れている可能性がある。候補として GPU/ドライバ、Qt の描画バックエンド、画面リフレッシュ条件、電源設定、機体数や描画負荷があるが、いずれもまだ仮説。

Vision TX の表示は 500 ms 窓で更新するため、短時間の変動も表示される。FPS 表示は `smoothFrameTime` による平滑値なので、画面の FPS と Vision TX の数字が一時的に一致しないことだけで異常とは判断できない。

## なぜ Hz の低下が気になるか

- RAVEN が次の Vision を受け取るまで、新しい位置・速度の観測が更新されない。
- UDP 到着間隔が不規則だと、戦略が見るボールや相手ロボットの情報の古さも不規則になる。
- `t_capture` はシミュレーション時間なので、実時間の Vision 到着 Hz と併せて見る必要がある。修正前のように送信周期だけでなくシミュレーション時刻も固定 60 Hz にすると、実際の物理刻みとカメラ時計がずれることがある。

## 次の調査

Windows PC の設定変更は持ち主の協力が必要なため、いったん後回しにする。再開するときは設定を変える前に、同じ実行中に次を記録して原因を切り分ける。

- 30 秒程度の FPS と Vision TX
- Vision `frame_number` の欠落、受信時刻、`t_capture`
- 画面リフレッシュレート、機体数、Qt/ビルド情報
- 可能なら `frameDone` 呼び出し間隔と timestep の平均・最大・分位点

FPS と Vision Hz が一緒に下がれば、描画・物理更新の連動が有力。FPS が安定しているのに Vision Hz が低い場合は、送信カウンタ、UDP送信エラー、受信側の欠落を分けて調べる。

全環境で一定 Hz を目指すなら、描画から物理更新を切り離す固定時間刻みループ（または描画とは独立したシミュレータ実行方式）が将来案。ただし現行 `PhysicsWorld` の lockstep 制約を越える設計変更になるため、まずは複数環境での同期ログを取ってから検討する。

## 再現確認の手順

1. `fix/improvement-sim` を取得して、ビルドしてから `build/bin/m2-Sim` を起動する。
2. 青・黄の機体数を記録する。設定は変更せず、Vision TX と FPS を確認する。
3. 可能なら Vision `frame_number` と `t_capture` も約 30 秒記録し、パケット欠落と時計の進みを確認する。
4. 変更前後や PC 間を比較するときは、同じ機体数・画面条件・計測時間を使う。
