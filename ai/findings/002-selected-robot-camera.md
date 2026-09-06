# カメラ切替の「Selected Robot」が動作しない

- **状態**: 未修正
- **箇所**: `src/qml/Main.qml:517-521`
- **影響**: 設定パネルでこの視点を選んでも切り替わらない。JavaScript エラーになる
- **発見日**: 2026-09-01
- **ビルド**: 不要（QML のみ）

## 症状

設定パネルの Camera セクションで Main Camera を `Selected Robot` に変更しても、
視点が俯瞰カメラのまま切り替わらない。
他の 3 つ（Overview / Ceiling Left / Ceiling Right）は正常に動く。

## 原因

`Main.qml:517-521` が、**存在しないプロパティを参照している**。

```qml
} else if (selectedCamera == "Selected Robot") {
    if (game_objects.selectedRobotColor === "blue") {
        viewport.camera = game_objects.bBotsCamera[game_objects.botCursorID];   // 518行
    } else if (game_objects.selectedRobotColor === "yellow") {
        viewport.camera = game_objects.yBotsCamera[game_objects.botCursorID];   // 520行
    }
}
```

`bBotsCamera` / `yBotsCamera` はリポジトリ全体で以下の 4 箇所にしか現れない。

| 箇所 | 状態 |
|---|---|
| `src/qml/Main.qml:518` | 参照 |
| `src/qml/Main.qml:520` | 参照 |
| `src/qml/sim/GameObjects.qml:213` | **コメントアウト** (`// bBotsCamera = [];`) |
| `src/qml/sim/GameObjects.qml:343` | **コメントアウト** (`// yBotsCamera = [];`) |

つまり宣言がどこにも無い。未定義プロパティへの添字アクセスとなり、
`onSelectedCameraChanged` ハンドラが TypeError で中断するため
`viewport.camera` への代入に到達しない。

## 修正案

ロボット搭載カメラの実体は `RobotInfo.qml:38` の `cameras` 配列にある。
`Sync.qml:98` が `color.cameras[i]` として使っているのと同じもの。

```qml
} else if (selectedCamera == "Selected Robot") {
    let team = (game_objects.selectedRobotColor === "yellow") ? yellow : blue;
    let cam = team.cameras[game_objects.botCursorID];
    if (cam !== undefined) {
        viewport.camera = cam;
    }
}
```

`cameras` は `property var cameras: []` と空配列で初期化されており、
ロボット生成時に埋まる。台数や生成タイミングによっては `undefined` があり得るため、
上記のように未定義チェックを入れる。

## 確認方法

修正前は、設定パネルで `Selected Robot` を選んだ際に
起動したターミナルへ TypeError が出る。ここが再現の目印になる。

修正後は、ロボットを左クリックで選択してから切り替えると
その機体の搭載カメラ視点になることを確認する。

## 関連

- `game_objects.selectedRobotColor` (`GameObjects.qml:25`) と
  `botCursorID` (`GameObjects.qml:26`) は正しく宣言されており、
  R キーでのリセット操作で更新される。壊れているのはカメラ配列の参照のみ。
- `Main.qml:493-495` の `cameraMain` も未使用の残骸。本件とは無関係だが同種の痕跡。
