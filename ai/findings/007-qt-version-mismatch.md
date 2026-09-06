# Qt のパス指定がビルドスクリプトと CMakeLists で食い違う

- **状態**: 未修正
- **箇所**: `CMakeLists.txt:33`, `build-windows.ps1:12`, `build-windows.ps1:67`
- **影響**: 環境によって意図しない Qt が使われる。環境構築時に原因を特定しづらい
- **発見日**: 2026-09-01
- **ビルド**: 該当（ビルド設定）

## 症状

ビルドスクリプトが指定した Qt とは別のバージョンが使われる可能性がある。
複数の Qt を入れている環境では、原因が分かりにくい形でビルドが通らない／
実行時に別バージョンの DLL を掴む、といった症状になる。

## 原因

指定が 2 箇所にあり、値が違う。

| ファイル | 行 | 値 |
|---|---|---|
| `build-windows.ps1` | 12 | `C:/Qt/6.10.3/mingw_64` |
| `CMakeLists.txt` | 33 | `C:/Qt/6.10.0/mingw_64` |
| `build-windows.ps1` | 67 | `C:/Qt/6.10.0/mingw_64`（windeployqt の案内文） |

`build-windows.ps1` は 6.10.3 を `-DCMAKE_PREFIX_PATH` で渡している。

```powershell
$QtDir = "C:/Qt/6.10.3/mingw_64"
...
-DCMAKE_PREFIX_PATH="$QtDir"
```

しかし `CMakeLists.txt:33` が、それを**先頭に別の値を差し込む形で上書き**している。

```cmake
set(CMAKE_PREFIX_PATH "C:/Qt/6.10.0/mingw_64" ${CMAKE_PREFIX_PATH})
```

結果として探索順が `6.10.0` → `6.10.3` になる。
`6.10.0` が入っていない環境では見つからず `6.10.3` にフォールバックするため
**たまたま動く**が、両方入っている環境では 6.10.0 が選ばれる。

さらに `build-windows.ps1:67` の案内文だけ 6.10.0 を指しているため、
表示されたコマンドをそのままコピーすると別バージョンの `windeployqt` を叩くことになる。

## 補足: 同種のハードコード

`CMakeLists.txt:14-36` は OS 別に依存パスを直書きしている。

- macOS: `/usr/local/Cellar/protobuf@21/21.12_1` とパッチバージョンまで固定
- Windows: 上記の Qt パス
- Linux: `# Linux settings here` のみで**中身が空**

protobuf のバージョンも環境間で乖離している（macOS は `protobuf@21`、
Windows の vcpkg は 6.33.4）。

## 修正案

`CMakeLists.txt:33` の `set(CMAKE_PREFIX_PATH ...)` を削除し、
パス指定を `build-windows.ps1` 側（`-DCMAKE_PREFIX_PATH`）に一本化する。

```cmake
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    # Qt のパスは -DCMAKE_PREFIX_PATH で渡す（build-windows.ps1 を参照）
endif()
```

同時に `build-windows.ps1:67` の案内文の `6.10.0` を `$QtDir` に置き換える。

```powershell
Write-Host "  $QtDir/bin/windeployqt.exe --qmldir src/qml $BuildDir/bin/m2-Sim.exe"
```

## 確認方法

configure 後に、実際に使われた Qt を確認する。

```powershell
Select-String -Path build\CMakeCache.txt -Pattern "Qt6_DIR|CMAKE_PREFIX_PATH"
```

## メモ

「同じコードなのに人によって挙動が違う」が最も出やすい箇所。
新しくメンバーが入るたびに再発するため、2〜3 人体制なら早めに直す価値がある。

なお `CMakeLists.txt` には現在、未コミットの変更が 1 行ある
（`find_package(Boost REQUIRED)` → `find_package(Boost REQUIRED CONFIG)`）。
本件を直すときに一緒に整理するとよい。
