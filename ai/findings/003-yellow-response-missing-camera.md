# 黄チームへの応答にカメラ検出結果が入らない

- **状態**: 未修正
- **箇所**: `src/networks/receiver.cpp:150-155`
- **影響**: 黄チームを制御する側から見て、経路③のオンボードカメラ情報が常に空になる
- **発見日**: 2026-09-01
- **ビルド**: 必要（C++）

## 症状

`RobotControl` を受信したときに同じソケットで返す `RobotControlResponse`（出力経路③）に、
青チームはカメラ検出結果が入るが、**黄チームは入らない**。

黄チームを制御している側は、この経路ではボールのカメラ検出を一切受け取れない。

## 原因

青と黄で受信クラスが別々に書かれており、**黄側だけカメラの詰め込みが抜けている**。

### 青（`receiver.cpp:75-85`）

```cpp
RobotControlResponse robotControlResponse;
for (int i = 0; i < botBallContacts.size(); ++i) {
    auto feedback = robotControlResponse.add_feedback();
    feedback->set_id(i);
    feedback->set_dribbler_ball_contact(botBallContacts[i]);
    CameraDetect botCamera;                              // ← 黄には無い
    botCamera.set_is_ball_exist(ballCameraExists[i]);    // ← 黄には無い
    botCamera.set_x(ballCameraPositions[i].x());         // ← 黄には無い
    botCamera.set_y(ballCameraPositions[i].y());         // ← 黄には無い
    *feedback->mutable_camera() = botCamera;             // ← 黄には無い
}
```

### 黄（`receiver.cpp:150-155`）

```cpp
RobotControlResponse robotControlResponse;
for (int i = 0; i < botBallContacts.size(); ++i) {
    auto feedback = robotControlResponse.add_feedback();
    feedback->set_id(i);
    feedback->set_dribbler_ball_contact(botBallContacts[i]);
}
```

**データ自体は届いている。** `ControlYellowReceiver::updateBallContacts()`
（`receiver.cpp:166-179`）が黄チーム用のカメラ情報をきちんとメンバに保存している。

```cpp
this->botBallContacts   = yBotBallContacts;
this->ballCameraExists  = yBallCameraExists;      // 保存はされている
this->ballCameraPositions = yBallCameraPositions; // 保存はされている
```

つまり**使える状態で持っているのに送っていない**だけ。

## 修正案

青の 80-84 行と同じ 5 行を黄側にも追加する。メンバ名は両クラスで同一なので、
コードはそのままコピーで動く。

ただし根本原因は「チーム色ごとにクラスを複製している」構造にある。
`ControlBlueReceiver` と `ControlYellowReceiver` は
`isYellow` フラグ以外ほぼ同一のため、**1 クラスに統合するのが望ましい**。
統合すれば本件と 004 の両方が再発しなくなる。

## 関連

- 同種の青/黄非対称: `ai/findings/004-yellow-orientation-not-normalized.md`
- どちらも「片方を直してもう片方を忘れた」形。片側でしか動作確認されていない可能性が高い。

## メモ

`Encoder/Team` の既定値が `blue`（`config/config_v2.ini:26`）であることから、
普段は青チームで検証していると推測される。黄チーム側の経路は検証が薄い。
