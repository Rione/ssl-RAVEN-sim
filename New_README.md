# ssl-RAVEN-sim (m2-Sim)

RoboCup Soccer Small Size League 向けのロボットサッカーシミュレータ。
Qt6 + QML + Qt Quick 3D Physics。ビルド成果物は実行ファイル `m2-Sim` 1 つ。

![M2 Simulation Image](docs/images/readme_v5.png)

> この文書は `README.md` の内容を実機で検証して書き直したものです。
> 旧 `README.md` との差分は「[付録 A: 旧 README からの変更点](#付録-a-旧-readme-からの変更点)」にまとめてあります。

---

## 目次

1. [必要なもの](#1-必要なもの)
2. [セットアップ（Windows）](#2-セットアップwindows)
3. [ビルド](#3-ビルド)
4. [DLL の配置（Windows）](#4-dll-の配置windows)
5. [起動](#5-起動)
6. [3D モデルについて](#6-3d-モデルについて)
7. [設定ファイルと通信ポート](#7-設定ファイルと通信ポート)
8. [操作方法](#8-操作方法)
9. [うまくいかないとき](#9-うまくいかないとき)
10. [macOS / Linux の現状](#10-macos--linux-の現状)
11. [ドキュメント](#11-ドキュメント)

---

## 1. 必要なもの

`CMakeLists.txt` の `find_package` が要求しているのは次の 3 つだけです。

| 依存 | 用途 | 備考 |
|---|---|---|
| **Qt 6**（Core / Gui / Widgets / Network / Quick / **Quick3D** / **Quick3DPhysics** / Qml） | 本体・3D 描画・物理 | **Quick3D と Quick3DPhysics は Qt の追加ライブラリ**。Qt のインストーラで明示的に選ばないと入りません |
| **Boost**（`boost::asio`） | vision とエンコーダフィードバックの UDP 送信 | Windows は vcpkg で導入 |
| **Protobuf** | 通信パケットの定義 | Windows は vcpkg で導入 |

検証済みの組み合わせ（この構成で起動を確認しています）:

```
Qt        6.10.3 (MinGW 13.1.0 64-bit)
MinGW     C:/Qt/Tools/mingw1310_64
CMake     C:/Qt/Tools/CMake_64
Ninja     C:/Qt/Tools/Ninja
vcpkg     C:/ws/vcpkg  (triplet: x64-mingw-dynamic)
```

> **修正済み**: 以前は Qt のバージョンが 3 か所で食い違っていました（旧 README が 6.8、`CMakeLists.txt` が
> 存在しない `C:/Qt/6.10.0`、`build-windows.ps1` だけが実在する 6.10.3）。現在はどちらも特定バージョンを
> 直書きせず、`QTDIR` 環境変数があればそれを、無ければ `C:\Qt\6.*` の中で最も新しいものを使います。
> Qt を更新しても記述が古くならず、明示的に `-DCMAKE_PREFIX_PATH` を渡せばそちらが優先されます。

---

## 2. セットアップ（Windows）

### 2.1 Qt のインストール

Qt のインストーラ（`C:\Qt\MaintenanceTool.exe`）から、次をすべて入れてください。

- `Qt 6.10.3` > `MinGW 13.1.0 64-bit`
- `Qt 6.10.3` > `Additional Libraries` > **`Qt Quick 3D`**
- `Qt 6.10.3` > `Additional Libraries` > **`Qt Quick 3D Physics`**
- `Qt` > `Developer and Designer Tools` > `MinGW 13.1.0 64-bit` / `CMake` / `Ninja`

**Quick 3D Physics を入れ忘れると、CMake の configure 段階で `find_package(Qt6 ...)` が失敗します。** 既定では入らないので必ず選択してください。

### 2.2 vcpkg のインストール

```powershell
git clone https://github.com/microsoft/vcpkg C:\ws\vcpkg
C:\ws\vcpkg\bootstrap-vcpkg.bat
```

`C:\ws\vcpkg` 以外の場所に置く場合は、`build-windows.ps1` の `$VcpkgRoot` を書き換えてください。

Boost と Protobuf は `build-windows.ps1` が自動で入れるので、手動インストールは不要です（手動でやる場合は
`vcpkg install boost-asio:x64-mingw-dynamic protobuf:x64-mingw-dynamic`）。

### 2.3 クローン

```powershell
git clone https://github.com/Rione/ssl-RAVEN-sim.git
cd ssl-RAVEN-sim
```

**3D モデルのダウンロードは不要です。** `assets/` はリポジトリに含まれています（→ [6 章](#6-3d-モデルについて)）。

---

## 3. ビルド

```powershell
.\build-windows.ps1
```

このスクリプトが行うこと:

1. 使用する Qt を決定（`QTDIR` があればそれ、無ければ `C:\Qt\6.*` の最新）
2. vcpkg で `boost-asio` と `protobuf` を導入（既に入っていればスキップ）
3. Ninja ジェネレータで CMake の configure
4. ビルド → `build/bin/m2-Sim.exe` を生成
5. `windeployqt` で Qt の DLL とプラグインを配置（→ [4 章](#4-dll-の配置windows)）

このスクリプトを使えば、ビルドから実行可能な状態になるまで 1 コマンドで完了します。

コードを変更したあとの再ビルドは、スクリプトを再実行するか次のコマンドで行います。

```powershell
C:\Qt\Tools\CMake_64\bin\cmake.exe --build build --parallel
```

> **QML の変更にビルドは不要です。** QML は実行時にファイルから読まれるので、編集して再起動すれば反映されます。
> ビルドが必要なのは C++ を変更したときだけです。

---

## 4. DLL の配置（Windows）

ビルド直後の `build/bin/` には `m2-Sim.exe` しかありません。**この状態では Qt の DLL が無く、
起動すらできません**（`Qt6Core.dll が見つかりません`）。

**`build-windows.ps1` を使っていれば、この作業は自動で行われます。** スクリプトの最後で
`windeployqt` が実行され、`build/bin/` に `Qt6Core.dll` などの DLL と `platforms/`・`imageformats/`・
`qml/` といったプラグインのディレクトリが作られます。

手動でビルドした場合や、配置が壊れた場合は次を実行してください。

```powershell
& "$env:QTDIR\bin\windeployqt.exe" --qmldir src\qml build\bin\m2-Sim.exe
```

`QTDIR` を設定していない場合は、使用中の Qt のパス（例: `C:\Qt\6.10.3\mingw_64`）に読み替えてください。

> 旧 `build-windows.ps1` は代替手段として `cmake --install` を案内していましたが、
> `CMakeLists.txt` に `install()` ルールが 1 つも無いため、**このコマンドは何もしません。**
> 現在はこの誤った案内を削除しています。

---

## 5. 起動

```powershell
C:\Qt\Tools\CMake_64\bin\cmake.exe --build build --target run
```

または、実行ファイルを直接起動する場合:

```powershell
cd build
.\bin\m2-Sim.exe
```

> **必ず `build/` をカレントディレクトリにして起動してください。**
> `main.cpp:21` が `../src/qml/Main.qml` を、`observer.cpp:3` が `../config/config_v2.ini` を
> **カレントディレクトリ基準の相対パス**で読むためです。別の場所から起動すると何も読めません。
> `--target run` はカレントディレクトリが `build/` になるので、この条件を自動的に満たします。

起動すると、コンソールに次の 3 行が出てウィンドウ（タイトル `m2-sim`）が開きます。

```
Listening on port 20694
Listening on port 10301
Listening on port 10302
```

---

## 6. 3D モデルについて

`assets/` にあるモデルとテクスチャ（32 ファイル / 約 61MB）は**リポジトリに含まれています**。
クローンしただけで動くので、ダウンロードも配置も必要ありません。

**なぜこれが重要か**: `src/qml/sim/Field.qml` と `GameObjects.qml` は `assets/` 配下を
**ディレクトリごと `import`** しています。

```qml
import "../../../assets/models/stadium/"
import "../../../assets/models/bot/Rione/viz" as BlueBody
```

QML では、**存在しないディレクトリの `import` は致命的エラー**です（`.mesh` ファイルが 1 つ欠けている
だけなら描画されないだけで起動はします）。そのため `assets/` が無いと Main.qml 全体のロードが失敗し、
**ビルドは成功するのに起動してもウィンドウが出ない**、という分かりにくい壊れ方をします。

自分のロボットのモデルに差し替えたい場合は
[how to import custom 3D model](docs/import_model.md) を参照してください。
サンプルモデルは [こちら](https://drive.google.com/drive/folders/17iXSCv_ecgYn4Mx0ziXjV6I9hVO665dg?usp=share_link)
からも入手できます。

---

## 7. 設定ファイルと通信ポート

### 設定ファイル

| ファイル | 読むクラス | 備考 |
|---|---|---|
| `config/config_v2.ini` | `Observer` | GUI の Setting パネルから変更すると、このファイルに書き戻されます |
| `config/config.ini` | `MotionControl` / `MathUtils` | `[Physics]` の Vel / Acc 系 6 キーのみ使用 |

> **設定ファイルは 2 系統あり、`[Physics]` セクションのキーが一部重複しています。**
> 値が食い違っていても片方しか効きません（例: `AccBrakeAbsoluteMax` は `config.ini` の値だけが使われます）。
> 詳細は [docs/architecture.md](docs/architecture.md) の 5.11 を参照してください。

> `config_v2.ini` はアプリがウィンドウサイズ等を書き戻すため、**起動するだけで `git status` が汚れます。**
> コミット時に意図しない差分を混ぜないよう注意してください。

### 通信ポート（`config_v2.ini` の現在値）

| 方向 | 内容 | アドレス:ポート |
|---|---|---|
| 受信 | コマンド（`mocSim_Packet`） | `0.0.0.0:20694` |
| 受信 | 青チーム制御（`RobotControl`） | `0.0.0.0:10301` |
| 受信 | 黄チーム制御（`RobotControl`） | `0.0.0.0:10302` |
| 送信 | vision（`SSL_WrapperPacket`） | `224.5.23.2:10694` |
| 送信 | エンコーダフィードバック（`PiToMw`） | `224.5.69.4:16941` |

パケットの詳細な定義は [docs/architecture.md](docs/architecture.md) の 4 章にあります。

---

## 8. 操作方法

キーボードとマウスの操作は [docs/key_mouse.md](docs/key_mouse.md) を参照してください。

![collision demo](./docs/gif/collision.gif)

---

## 9. うまくいかないとき

| 症状 | 原因 | 対処 |
|---|---|---|
| configure で `find_package(Qt6 ...)` が失敗する | Quick 3D / Quick 3D Physics が未インストール、または Qt のパスが違う | [2.1](#21-qt-のインストール) を確認。手動 configure なら `-DCMAKE_PREFIX_PATH` を渡す |
| 起動しても**ウィンドウが出ない** | `assets/` が無い、または `build/` 以外から起動した | `assets/` の存在を確認。`build/` から起動する |
| `Qt6Core.dll が見つかりません` | `windeployqt` 未実行 | [4 章](#4-dll-の配置windows) を実行 |
| `Failed to bind UDP socket to port ...` | 同じポートを別プロセスが使用中 | 既存の `m2-Sim` を終了するか、Setting でポートを変更 |
| QML を直したのに反映されない | アプリを再起動していない | 再起動する（ビルドは不要） |

起動に失敗した場合、**コンソールに QML のエラーが出ます**。GUI からではなくターミナルから起動して、
そのメッセージを確認してください。

---

## 10. macOS / Linux の現状

### macOS — **修正済み（ただし実機未検証）**

以前はビルドできませんでした。`CMakeLists.txt` の Darwin ブロックが Homebrew の絶対パスを直接
指定していましたが、そこで指定されていた 2 つのフォーミュラは Homebrew 側で **disabled**
（インストール不可）になっているためです。

| フォーミュラ | disabled になった日 |
|---|---|
| `protobuf@21` | 2026-01-08 |
| `boost@1.85` | 2026-04-05 |

旧 README の `brew install ... boost@1.85 ... protobuf@21` は、**この行自体がエラーで止まります。**
既存の Mac が動き続けていたのは、`brew` が導入済みのものを消さないためです。

**現在は次のように修正してあります。**

- `CMakeLists.txt` の Darwin ブロックからバージョン付きの絶対パスを削除し、`brew --prefix` で
  Homebrew の場所を問い合わせる形にしました（Apple Silicon と Intel のパス差も自動で吸収します）
- protobuf を CONFIG モードで探し、`protobuf::libprotobuf` **ターゲット**をリンクするようにしました。
  以前の `${Protobuf_LIBRARIES}` は生のライブラリパスで、protobuf 22 以降が必要とする Abseil への
  依存が伝わらず、`absl::` の未定義シンボルでリンクが失敗します

セットアップ手順:

```bash
brew update
brew install qt cmake boost protobuf
cd ssl-RAVEN-sim
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
cmake --build build --target run
```

> **この手順は Mac 実機で検証できていません。** 変更が Windows のビルドを壊していないことは
> 確認済みです（素の `cmake` 構成からのフルビルドと `build-windows.ps1` の両方で成功、起動も確認）。
> Mac で試したときに失敗したら、そのログを添えて報告してください。

### Linux — **未検証**

`CMakeLists.txt` の Linux 分岐は空（`# Linux settings here`）で、システムの Qt / Protobuf が
見つかる前提になっています。Ubuntu の apt で入る Qt は本プロジェクトが要求するバージョンに届かないことが
多く、Quick 3D Physics も含まれません。

未マージのブランチ `feat/windows-support` に
`e1e442f feat: add Linux (Ubuntu 24.04) build support` というコミットがあります。Linux で動かす場合は
まずこれを確認してください。

---

## 11. ドキュメント

| 文書 | 内容 |
|---|---|
| [docs/architecture.md](docs/architecture.md) | 構造・1 Tick の処理順序・通信仕様のリファレンス |
| [docs/key_mouse.md](docs/key_mouse.md) | キーボード / マウス操作 |
| [docs/import_model.md](docs/import_model.md) | 3D モデルの差し替え手順（balsam / cooker） |
| [docs/encoder_feedback.md](docs/encoder_feedback.md) | 合成エンコーダフィードバックの仕様 |
| [ai/findings/](ai/findings/) | 既知の不具合（1 件 = 1 ファイル） |

### 関連ツール

- [ssl-game-controller](https://github.com/RoboCup-SSL/ssl-game-controller)
- [ssl-autorefs](https://github.com/RoboCup-SSL/ssl-autorefs)

## ライセンス

GNU General Public License version 3 (GPL v3)。詳細は [LICENSE](LICENSE) を参照してください。

---

## 付録 A: 旧 README からの変更点

旧 `README.md` の記述を実機・実ファイルと突き合わせた結果、次の誤りが見つかりました。
「対応」欄が **コード修正** のものは、ドキュメントだけでなく実ファイルを直しています。

| # | 旧 README の記述 | 実際 | 対応 |
|---|---|---|---|
| 1 | 「Qt6: Version 6.8 is supported」 | 3 か所で食い違い。`CMakeLists.txt` は存在しない `C:/Qt/6.10.0` を指し、`build-windows.ps1` だけが実在する 6.10.3 を渡していた | **コード修正**: 双方からバージョン直書きを削除。`QTDIR` か `C:\Qt\6.*` の最新を使う |
| 2 | Qt Quick 3D / Quick 3D Physics の記載なし | `CMakeLists.txt` が `REQUIRED` で要求。Qt の追加ライブラリなので既定では入らない | 文書化 |
| 3 | `eigen` `yaml-cpp` `assimp` `bullet` `vulkan-volk` をインストールせよ | **いずれもプロジェクト内で 1 か所も使われていない**（`find_package` にも無い）。必要なのは Qt6 / Boost / Protobuf のみ | 文書化 + `README.md` から削除 |
| 4 | macOS: `brew install ... boost@1.85 ... protobuf@21` | 両方とも Homebrew で disabled 済みで、**この行自体が失敗する** | **コード修正**: Darwin ブロックの絶対パス直書きを削除し `brew --prefix` 方式へ。protobuf を CONFIG モード + `protobuf::libprotobuf` に変更（→ [10 章](#10-macos--linux-の現状)） |
| 5 | Windows の手順が一切ない | 動作が確認できている唯一の経路が Windows | 本文書の 2〜5 章として追加 + `README.md` にも節を追加 |
| 6 | `mkdir build && cd build && cmake .. && make` | Windows は Ninja + vcpkg ツールチェーン。当時は素の `cmake ..` では Qt が見つからなかった | **コード修正**: Qt 自動検出を追加し、素の `cmake -S . -B build` でも通るようにした（検証済み） |
| 7 | `windeployqt` の記載なし | **実行しないと起動できない**（DLL 不足） | **コード修正**: `build-windows.ps1` が自動実行するようにした |
| 8 | `cmake --install` で DLL を配置せよ（`build-windows.ps1` の案内） | `CMakeLists.txt` に `install()` ルールが 1 つも無く、**このコマンドは何もしない** | **コード修正**: 誤った案内を削除 |
| 9 | 3D モデルを Google Drive からダウンロードして `assets/` に配置せよ | `assets/` をリポジトリに含めたため不要。**この手順の欠落が「ビルドは通るのに起動してもウィンドウが出ない」の原因だった** | **コード修正**: `assets/` をコミット |
| 10 | `build/` から起動する必要性の記載なし | `../src/qml/Main.qml` と `../config/config_v2.ini` をカレントディレクトリ基準で読むため必須 | 文書化 |
| 11 | `~/ws/m2-sim/` というパス | リポジトリ名は `ssl-RAVEN-sim` | 修正 |
| 12 | `CONTRIBUTING.md` へのリンク | **ファイルが存在しない** | `README.md` のリンクを削除し、issue / PR と `ai/findings/` への案内に差し替え |
| 13 | `docs/key_mouse.md` へのリンクなし | 存在するのに参照されていなかった | 双方の README に追加 |
| 14 | Ubuntu の手順を他と同列に記載 | `CMakeLists.txt` の Linux 分岐は空で未検証 | 未検証である旨を明記 |

### 検証方法

上記のうち起動に関わる項目は、実際に次の手順で確認しています。

1. リポジトリを新規クローンし、`assets/` が無い状態で `build/bin/m2-Sim.exe` を起動
   → ウィンドウが出ず、`QQmlApplicationEngine failed to load component` が出力される
2. `assets/` を含むクローンで同じ操作
   → ウィンドウ（`Title='m2-sim'`）が開き、`Listening on port 20694/10301/10302` が出力される
3. `cmake --build build --target run` でも同様にウィンドウが開くことを確認

### 未解決の項目

- `assets/textures/Ri-one.png` が `src/qml/sim/Field.qml:110` から参照されていますが、ファイルが存在しません。
  参照している `Texture` を使うマテリアルはコメントアウトされているため起動には影響しませんが、
  ファイルを持っている人がいれば追加してください。
- 本文書は日本語で書いています。公開リポジトリの正面の README として使う場合は、英語版も用意するか
  併記するかを決めてください。
