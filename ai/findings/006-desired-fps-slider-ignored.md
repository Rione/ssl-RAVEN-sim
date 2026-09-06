# Desired FPS スライダーが効かない

- **状態**: 未修正
- **箇所**: `src/observer.cpp:243-248`
- **影響**: 設定パネルの Desired FPS を動かしても常に 60 のまま。動かない UI が表示されている
- **発見日**: 2026-09-01
- **ビルド**: 必要（C++）

## 症状

設定パネルの Physics セクションにある `Desired FPS` スライダーを動かして Save しても、
値が 60 から変わらない。次に設定パネルを開くと 60 に戻っている。

## 原因

setter が**引数を明示的に捨てている**。

```cpp
void Observer::setDesiredFps(int fps) {
    Q_UNUSED(fps);                              // ← 受け取った値を破棄
    desiredFps = 60;                            // ← 常に 60
    config.setValue("Physics/DesiredFps", desiredFps);
    emit settingChanged();
}
```

`Q_UNUSED(fps)` は「使わないことを意図している」というマーカーなので、
**書き忘れではなく意図的に無効化されている**。

UI 側（`List.qml:27`）はスライダーとして定義されており、見た目は動く。

```qml
ListElement { name: "Desired FPS"; ...; slider: true; InitValue: 60.0; MaxValue: 60.0 }
```

`MaxValue` が `60.0` なので、そもそも 60 より上げられない作りにもなっている。

## なぜ無効化されているかの推測

物理ループの駆動元が `PhysicsWorld.onFrameDone`（描画フレーム同期）であり、
`Main.qml:45-46` で刻み幅を 1/60 秒に固定している。

```qml
maximumTimestep: fixedFrameTime      // 1000.0 / 60.0
minimumTimestep: fixedFrameTime
```

`desiredFps` だけ変えても物理側は追従しないため、
中途半端に動くより固定したほうが安全、という判断だったと考えられる。

## 影響範囲

`desiredFps` は他にも参照されている。

| 箇所 | 用途 |
|---|---|
| `src/qml/viz/VObject.qml:20` | ミニマップの更新間隔（`1000.0 / observer.desiredFps`） |
| `src/observer.cpp:88` | ここでは使われず、`1000 / 60` を直書き |

## 修正案

### A. UI から外す（軽い・推奨）

`List.qml:27` の `Desired FPS` の `ListElement` を削除し、
`Setting.qml:58` の Physics セクションの `heightValue` を減らす。
QML のみなのでビルド不要。

動かないスライダーが消えるだけだが、**現状はこれが実態に合っている**。

### B. 本来の機能を実装する（重い）

FPS を可変にするには、`Main.qml` の `fixedFrameTime` を
`observer.desiredFps` から算出する形に変え、物理の刻み幅も追従させる必要がある。

ただしこれは「シミュレーション時間と実時間の混在」という
構造的な問題に直接触れる変更になる。
センサモデル（エンコーダ合成・アクチュエータ遅延）が実時間 dt を使っているため、
FPS を変えると挙動の再現性に影響する。**単独では着手しないほうがよい。**

## 関連

- `src/observer.cpp:88` の `simTimer->start(1000 / 60)` は**整数除算で 16 ms** になるため、
  名目 60 Hz のこのタイマは実際には約 62.5 Hz で回っている。
- FPS 表示自体も常に 60 を出す別の不具合がある:
  `ai/findings/001-fps-always-60.md`
- 同種の「動かない UI」: `ai/findings/005-geometry-settings-dead.md`
