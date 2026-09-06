# 黄チームの姿勢角が正規化されずに vision へ送られる

- **状態**: 未修正
- **箇所**: `src/networks/sender.cpp:109`
- **影響**: 同じ姿勢でも黄チームだけ範囲外の角度が送信され得る。受信側の実装次第で挙動が変わる
- **発見日**: 2026-09-01
- **ビルド**: 必要（C++）

## 症状

出力経路⑤（`SSL_WrapperPacket`）に載せるロボットの姿勢角が、
青チームは `[-π, π]` に収まるが、**黄チームは範囲外の値になり得る**。

## 原因

`Sender::setDetectionInfo()` の中で、青と黄が別々のループで書かれており、
**正規化が青側にしかない**。

### 青（`sender.cpp:92-94`）

```cpp
float tempOrientation = blue_positions[i].z();
tempOrientation = fmod(tempOrientation + 180, 360) - 180;  // Normalize to [-180, 180]
robot->set_orientation(tempOrientation*M_PI/180);
```

### 黄（`sender.cpp:109`）

```cpp
robot->set_orientation(yellow_positions[i].z()*M_PI/180);   // 正規化なし
```

`positions[i].z()` は座標ではなく**見出し角[度]**が入っている
（`QVector3D` を「x, y, 見出し角」のタプルとして流用している箇所）。
この値は `Sync.qml:18` が `mu.radianToDegree()` で作っており、
累積回転によって ±180 度を超えることがある。

## 修正案

青と同じ 2 行を黄側にも入れる。ただし 003 と同様、
**青/黄のループを 1 つに共通化する**のが本筋。

```cpp
// 共通化案：チーム別の add_robots_* だけ切り替え、中身は 1 か所にまとめる
auto fill = [](SSL_DetectionRobot* r, int id, const QVector3D& p) {
    r->set_robot_id(id);
    r->set_confidence(1.0);
    r->set_x(p.x());
    r->set_y(p.y());
    float deg = fmod(p.z() + 180, 360) - 180;
    r->set_orientation(deg * M_PI / 180);
    r->set_pixel_x(0);
    r->set_pixel_y(0);
    r->set_height(0);
};
```

## 確認方法

黄チームのロボットを同一方向に何度も回転させ、
vision を購読して `orientation` が `[-π, π]` を超えないか確認する。
青チームで同じ操作をして値が収まることと比較すると分かりやすい。

## 関連

- 同種の青/黄非対称: `ai/findings/003-yellow-response-missing-camera.md`
- `sender.cpp:86` と `:103` には、カメラ振り分けを試みた形跡が
  コメントアウトで残っている（マルチカメラ対応の名残）。
