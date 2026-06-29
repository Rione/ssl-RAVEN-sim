# ビジョンのマルチカメラ分割（重なり＋ランダム継ぎ目）

実機 ssl-vision はフィールドを複数カメラで分担し、隣接カメラの視野は中央付近で
重なる。m2-sim もこれを模して、`SSL_DetectionFrame` を **カメラごとに分割**して
送出する。

## 仕様
- **2カメラ**（既定）。ハーフウェイライン（x=0）で左右に分担する。
  - camera 0 … 左半面（`x ≤ split + overlap`）
  - camera 1 … 右半面（`x ≥ split − overlap`）
- **重なり帯**: 分割線まわり `±overlap` の帯は **両カメラが重複して検出**する。
  実カメラの視野オーバーラップ（同一オブジェクトが2台に映る）を再現。
- **ランダム継ぎ目**: 毎フレーム、分割線 `split` を `[−jitter, +jitter]` の一様乱数で
  ずらす。重なり領域が揺らぎ、継ぎ目の受け渡しが不確かになる実環境を再現する。
- 各カメラは個別の `SSL_WrapperPacket`（`camera_id` 0 / 1）として送出。`geometry`
  は重複を避けて camera 0 のパケットにのみ添付し、`calib` も `NumCameras` 台ぶん宣言。

`NumCameras=1` にすると従来どおり全面を1カメラ（id=0）で送る。

## ボールのオクルージョン（ロボット遮蔽）
各カメラは担当ハーフの**天井中央**（X=±3000、高さ `CameraHeightMm`）に設置されて
いるものとし、カメラからボールへの視線が高さ `RobotHeightMm` のロボット（半径90mm）
に遮られると、**そのカメラはボール検出を落とす**。

- カメラ→ボールの3D視線（レイ）が、ロボットの円柱（半径90mm・高さ0〜RobotHeight）を
  ボールより手前で貫く場合に遮蔽と判定（レイ-円柱交差）。
- カメラは高所にあるため視線がロボット高さを横切るのはボール近傍だけだが、**フィールド
  端ほど視野角が浅くなり遮蔽の影（シャドウ）が伸びる**ため、端のボールほど隠れやすい。
- 2カメラの**重なり帯では、片方が遮られてももう片方が見えていればボールは残る**。
  端は1カメラしか覆わないため、遮られると**完全に消える**（実機の挙動と一致）。
- `CameraHeightMm` を下げると遮蔽が強くなる。`OcclusionEnabled=false` で無効化。

## 設定（`config/config_v2.ini`）
```ini
[Vision]
NumCameras=2      ; 1=全面1カメラ(従来) / 2=左右分割
OverlapMm=500     ; 分割線まわりで両カメラが重複検出する帯の半幅[mm]
SplitJitterMm=200 ; 毎フレーム分割線をずらす量[mm](0で固定)
OcclusionEnabled=true ; ロボットによるボール遮蔽の有効/無効
CameraHeightMm=4000   ; カメラ天井高[mm](下げるほど遮蔽が強い)
RobotHeightMm=150     ; ロボット高さ[mm](SSL上限)
```

## 実装
- `src/networks/sender.cpp::send` … 毎フレーム `split` を乱択し、`NumCameras` 台ぶん
  ループしてカメラ別フレームを送出。
- `src/networks/sender.cpp::setDetectionInfo` … `inCamera(x)` 述語で各オブジェクト
  （ボール・各ロボット）の所属カメラを判定。重なり帯は両方に入れる。
- `src/networks/sender.cpp::isBallOccluded` … カメラ→ボール視線とロボット円柱の
  交差判定。遮蔽時はそのカメラのボール検出を抑止。
- `src/networks/sender.cpp::setGeometryInfo` … `calib` を `NumCameras` 台、各カメラを
  担当ハーフの中央（tx=±3000）・高さ `CameraHeightMm` に配置。
- `src/observer.cpp` … `[Vision]` を読んで `Sender::setCameraSplit` /
  `Sender::setOcclusion` に渡す。
- `test/vision_split_test.cpp` … 左/右/中央のオブジェクトが正しいカメラに入り、
  中央は両カメラに重複することを実コードで検証。
- `test/vision_occlusion_test.cpp` … 端のボールがロボット背後で消え、中央のボールは
  重なりで生き残ることを実コードで検証。
