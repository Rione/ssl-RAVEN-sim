# FPS 表示が常に 60 になる

- **状態**: 未修正
- **箇所**: `src/qml/Main.qml:212`, `:503`, `:510`
- **影響**: 描画性能が測れない。重くなっても「FPS: 60」と表示され続ける
- **発見日**: 2026-09-01
- **ビルド**: 不要（QML のみ）

## 症状

画面左下の FPS 表示が、実際の描画負荷にかかわらず常に `FPS: 60` を出す。
ロボット台数を増やしても、CCD を有効にしても値が変わらない。

## 原因

`Main.qml:502` の `onUpdateSimulationSignal()` が、実経過時間を計測せずに
定数を代入している。

```qml
function onUpdateSimulationSignal() {
    runTime = fixedFrameTime;      // 503行: 定数 1000.0/60.0 = 16.666...
    ...
    showRunTime = runTime;         // 510行
}
```

`runTime` に代入されるのはここだけで、他の箇所（507-508 行）は
`syncEmptyObjects(runTime)` の引数として渡しているだけ。

その値がそのまま表示式に入る。

```qml
text: "FPS: " + Math.round(1000.0 / showRunTime)   // 212行 → 1000/16.666 = 60
```

## 修正案

`Main.qml:32` に**未使用のまま残っている** `lastTime` プロパティを使い、
`onUpdateSimulationSignal()` で前回呼び出しからの経過時間を測る。

```qml
property real lastTime: 0          // 32行（既存・未使用）

function onUpdateSimulationSignal() {
    let now = Date.now();
    if (lastTime > 0) {
        runTime = now - lastTime;
    }
    lastTime = now;
    ...
    showRunTime = runTime;
}
```

表示のちらつきが気になる場合は、`showRunTime` に移動平均をかける。

影響範囲は `Main.qml` 内で閉じており、C++ を触らないためビルドは不要。

## 関連

- `Main.qml:32` の `lastTime` が宣言のみで未使用なのは、
  この計測を実装しかけて中断した跡と考えられる。
- 別件だが `Observer::setDesiredFps()`（`src/observer.cpp:243`）も
  引数を捨てて常に 60 を設定しており、設定パネルの Desired FPS スライダーは効かない。
  こちらは C++ 側の話なので本件とは独立。

## メモ

GUI 側の最初の改修課題として適している。影響範囲が明確で、
ビルド不要のため試行錯誤のサイクルが速い。
