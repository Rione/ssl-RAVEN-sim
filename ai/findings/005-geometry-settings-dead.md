# 設定パネルの Geometry セクションが機能しない

- **状態**: 未修正
- **箇所**: `src/qml/settings/Setting.qml:32-49`, `:179-184`, `src/qml/settings/List.qml:40-47`
- **影響**: 動かない UI が表示されている。触っても何も起きず、原因も分かりにくい
- **発見日**: 2026-09-01
- **ビルド**: 不要（QML のみ。ただし本格対応には C++ が必要）

## 症状

設定パネルに `Geometry` セクションが表示され、`Line Thickness` のスライダーも動く。
しかし**値を変えて Save を押しても何も変わらない**。

## 原因

3 段階すべてで経路が切れている。

### 1. UI 側の値が observer に繋がっていない

`Setting.qml:32-39` が **リテラル `0` で固定**されている。

```qml
property real tempFieldWidth: 0        // ← observer.fieldWidth ではない
property real tempFieldHeight: 0
property real tempLineThickness: 0
property real tempGoalWidth: 0
property real tempGoalHeight: 0
property real tempGoalDepth: 0
property real tempPenaltyAreaWidth: 0
property real tempPenaltyAreaDepth: 0
```

本来の記述は直下の `Setting.qml:42-49` に**すべてコメントアウトで残っている**。

```qml
// property real tempFieldWidth: observer.fieldWidth
// property real tempFieldHeight: observer.fieldHeight
// ...
```

### 2. Save ボタンでも反映されない

`Setting.qml:179-184` の代入も**全行コメントアウト**。

```qml
// observer.lineThickness = tempLineThickness;
// observer.goalWidth = tempGoalWidth;
// ...
```

### 3. そもそも C++ 側に受け皿が無い

`src/observer.h:23-54` の `Q_PROPERTY` 一覧に、
`fieldWidth` / `goalWidth` / `lineThickness` などは**1 つも存在しない**。
コメントアウトを外しても、そのままでは動かない。

### 4. 設定ファイルの値も誰も読んでいない

`config/config_v2.ini:33-43` に `[Geometery]` セクションがあり
10 項目が定義されているが、**C++ 側にこれを読むコードが無い**。

```ini
[Geometery]          ← セクション名が誤記（Geometry ではない）
FieldLength=15400
FieldWidth=12400
...
```

## フィールド寸法が実際に定義されている場所

設定ファイルは死んでいる一方、寸法は**4 系統に分散**している。

| 定義場所 | 値 | 用途 |
|---|---|---|
| `config/config_v2.ini` `[Geometery]` | `FieldLength=15400` 等 | **誰も読まない** |
| `src/networks/sender.cpp:121-125` | `field_length=12000`, `field_width=9000` | vision で外部へ送信 |
| `src/qml/sim/Field.qml` | `±5995`, `±4495` 等の直書き | 3D 画面の描画 |
| `src/qml/viz/VField.qml:11-17` | `12000`, `9000`, `1000`, `1840` | 2D ミニマップの描画 |

1 つ変更しても他が追従しないため、**フィールド寸法を変える改修は現状かなり危険**。

## 修正案

対応の重さが 2 段階ある。

### A. 表示だけ止める（軽い・QML のみ）

`Setting.qml:59` の `menuModel` から `Geometry` の `ListElement` を外す。
動かない UI が消えるので、少なくとも利用者が混乱しない。

```qml
ListElement { label: "Geometry"; expandValue: 85; heightValue: 190 }   // ← 削除
```

### B. 本来の機能を実装する（重い・C++ + QML）

1. `observer.h` に `Q_PROPERTY` と getter/setter を 8 項目追加
2. `observer.cpp` で `[Geometery]` を読む（セクション名の誤記も要検討）
3. `Setting.qml:42-49` と `:179-184` のコメントアウトを戻す
4. **`sender.cpp` / `Field.qml` / `VField.qml` の直書きを observer 参照に置き換える**

4 が本体で、ここを直さない限り「設定は変わるが画面と vision は変わらない」状態が続く。

## メモ

SSL はフィールド規格が大会ごとに変わり得るため、実運用で効いてくる箇所。
ただし着手するなら A で一旦塞ぎ、B は設定の一本化（`config.ini` と
`config_v2.ini` の統合）と合わせて計画したほうがよい。
