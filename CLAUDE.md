# ssl-RAVEN-sim

RoboCup SSL 向けロボットサッカーシミュレータ。Qt6 + QML + Qt Quick 3D Physics。
ターゲットは単一実行ファイル `m2-Sim`。

## ビルドと実行

```powershell
.\build-windows.ps1              # ビルド（Qt MinGW + vcpkg + Ninja）
cd build; .\bin\m2-Sim.exe       # 実行
```

- **必ず `build/` から起動する。** QML と設定ファイルを相対パス
  (`../src/qml/Main.qml`, `../config/config_v2.ini`) で読むため、
  他のディレクトリから起動すると何も読めない。
- **QML の変更にビルドは不要。** 実行時にファイルから読み込むので、編集して再起動すれば反映される。
  ビルドが必要なのは C++ を変更したときだけ。

## 構成

C++ 約 1,980 行 / QML 約 3,550 行。**行数でも責務でも QML 側が主**。

| パス | 役割 |
|---|---|
| `src/observer.*` | 中枢。設定・通信結線・60Hz タイマ・エンコーダ合成 |
| `src/networks/` | UDP 受信 3 クラス・送信 2 クラス |
| `src/models/` | Robot の状態と遅延モデル、座標投影 |
| `src/qml/sim/` | **シミュレーション本体**（物理・キック・摩擦） |
| `src/qml/settings/` | 設定パネル UI |
| `src/qml/viz/` | 2D ミニマップ |

- **物理演算は C++ ではなく QML 側にある**（`src/qml/sim/GameObjects.qml`）。
  C++ は I/O と設定、それに**ロボットの運動モデル**（`src/models/robot.cpp` の
  `advanceActuation`）に徹している。指令から機体速度までの実機同定モデルだけは
  C++ 側で、QML はその結果を剛体に渡すだけ。
- C++ と QML の境界は 2 本だけ。下りは `Q_PROPERTY` とシグナル、
  上りは `observer.updateObjects()`（`src/qml/sim/Sync.qml`）。
- アプリの起動の引き金は `src/qml/Main.qml:141` の `Observer { }`。
  C++ 側に起動処理は無い。

## 注意点

- **座標系が場所によって違う。** QML シーン座標は `y` が高さ・`z` が前後、
  SSL vision 座標は `y` が前後・`z` が高さ。
  さらに `QVector3D` の `.z()` が「見出し角[度]」を意味する箇所がある。
- **設定ファイルが 2 系統ある。** `config/config_v2.ini` は `Observer` が読み、
  `config/config.ini` は `MathUtils` と `MotionControl` が読む。
  両方に `[Physics]` があり一部キーが重複しているため、片方を直しても他方に反映されない。
- `config_v2.ini` の `[Geometery]` セクションはどこからも読まれていない。
  フィールド寸法は `sender.cpp` / `Field.qml` / `VField.qml` に個別に直書きされている。
- `test/` は `CMakeLists.txt` から参照されておらずビルドされない。
- 意味が通らないコードに出会ったら、理解不足より先に**実装の残骸**を疑う。
  未使用の宣言やコメントアウトされた旧実装が各所に残っている。

## 詳細ドキュメント

- 既知の不具合: `ai/findings/`（1 件 = 1 ファイル）
- 操作方法: `docs/key_mouse.md`
- 実機同定モデル（台の運動・球の減速）: `docs/robot_motion_model.md`
- エンコーダフィードバック: `docs/encoder_feedback.md`
- 3D モデルの導入: `docs/import_model.md`
