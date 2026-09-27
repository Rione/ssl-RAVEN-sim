#ifndef ROBOTMODEL_H
#define ROBOTMODEL_H

#include <QSettings>
#include <QString>

// 実機 1 台ぶんの同定済みプラントモデル。
//
// RAVEN の app/config/system_model_real_<mac>_ID<n>.yaml の `robot:` ブロックと
// 1 対 1 に対応する。RAVEN の MPC (control.yaml の optimal_control) は
// 「指令を出してから input_dead_time_sec 遅れて効きはじめ、時定数 tau_v* で立ち上がり、
// 定常では gain_v* 倍にしかならない」台を前提に最適化している。sim が指令どおりに
// 即座に動いてしまうと、その補償ぶんだけ MPC が行き過ぎる — 実機で詰めた設定のまま
// sim が変な挙動になる、の正体がこれ。ここは「sim の台を実機に合わせる」ための器。
//
// 単位は RAVEN の yaml と同じ (秒 / mm / mm/s^2 / rad)。
struct RobotMotionModel {
    // 一次遅れの時定数 [s] (yaml: tau_vx / tau_vy / tau_omega)。
    float tauVxSec = 0.0f;
    float tauVySec = 0.0f;
    float tauOmegaSec = 0.0f;

    // むだ時間 [s] (yaml: input_dead_time_sec)。指令が効きはじめるまでの輸送遅れで、
    // 実機では vision → 判断 → 無線 → モータドライバまでの一巡ぶんを含む同定値。
    // RAVEN は自分の閉ループを丸ごと同定しているので、sim でも同じ値を再現するのが
    // MPC から見て正しい台になる。
    float deadTimeSec = 0.0f;

    // 定常ゲイン行列 v = G u (yaml: gain_vx / gain_vy / gain_vx_from_uy / gain_vy_from_ux)。
    //   vx = GainVx * ux + GainVxFromUy * uy
    //   vy = GainVy * uy + GainVyFromUx * ux
    // 非対角項は「前に出せと言うと少し横に流れる」実機の癖。yaml に無い機体は 0。
    float gainVx = 1.0f;
    float gainVy = 1.0f;
    float gainVxFromUy = 0.0f;
    float gainVyFromUx = 0.0f;
    // 角速度のゲイン。RAVEN の yaml には対応する項が無いので既定 1.0。
    float gainOmega = 1.0f;

    // 機体軸ごとの牽引 (トラクション) 限界 [mm/s^2]
    // (yaml: traction_accel_x / traction_accel_y / traction_decel_x / traction_decel_y)。
    // x は前後、y は横。実機は横が明確に弱く、ID4 で前後 3971 に対し横 1715 と 2 倍以上違う。
    // sim の MotionControl は等方の 1 つの上限しか持っていなかったので、この非対称が出なかった。
    float tractionAccelXMmS2 = 3000.0f;
    float tractionAccelYMmS2 = 3000.0f;
    float tractionDecelXMmS2 = 3000.0f;
    float tractionDecelYMmS2 = 3000.0f;

    // 角速度の限界 (RAVEN の app/config/physics.yaml: max_angular_velocity / _acceleration)。
    float maxAngularVelRadS = 10.0f;
    float maxAngularAccelRadS2 = 35.0f;

    // 並進と旋回を合わせた車輪周速の予算 [mm/s] (yaml: wheel_rim_speed_budget_mm_s)。
    // 全速で走りながら全速で回ることはできない、という実機の当たり前をここで効かせる。
    float wheelRimSpeedBudgetMmS = 2250.0f;

    // 車輪配置 (周速の計算用)。既定は sim の [Encoder] と同じ幾何。
    float robotRadiusMm = 90.0f;
    float wheelAngleRad[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    // group (例 "RobotModel" や "RobotModel.4") にあるキーだけを base から上書きして返す。
    // 無いキーは base のまま — [RobotModel] を既定値、[RobotModel.<id>] を差分として書ける。
    static RobotMotionModel fromSettings(QSettings &cfg, const QString &group,
                                         const RobotMotionModel &base);
};

#endif // ROBOTMODEL_H
