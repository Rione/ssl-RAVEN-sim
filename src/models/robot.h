#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <deque>
#include <QObject>
#include <QVector3D>
#include <QVariantList>
#include <QElapsedTimer>
#include <QRandomGenerator>
#include <QDebug>

#include "mocSim_Commands.pb.h"
#include "ssl_simulation_robot_control.pb.h"
#include "ssl_simulation_robot_feedback.pb.h"
#include "robotModel.h"

using namespace std;

class Robot : public QObject {
    Q_OBJECT

    Q_PROPERTY(uint32_t id READ getId)
    Q_PROPERTY(float kickspeedx READ getKickspeedx)
    Q_PROPERTY(float kickspeedz READ getKickspeedz)
    Q_PROPERTY(float veltangent READ getVeltangent)
    Q_PROPERTY(float velnormal READ getVelnormal)
    Q_PROPERTY(float velangular READ getVelangular)
    Q_PROPERTY(float spinner READ getSpinner)
    Q_PROPERTY(bool wheelsspeed READ getWheelsspeed)
    Q_PROPERTY(float wheel1 READ getWheel1)
    Q_PROPERTY(float wheel2 READ getWheel2)
    Q_PROPERTY(float wheel3 READ getWheel3)
    Q_PROPERTY(float wheel4 READ getWheel4)

public:
    explicit Robot(QObject *parent = nullptr);
    ~Robot();

    void visionUpdate(mocSim_Robot_Command robotCommand);
    void controlUpdate(RobotCommand robotCommand);

    // 実機の同定モデルをこの台に載せる。むだ時間・一次遅れ・定常ゲイン・軸別の
    // 牽引限界・車輪周速の予算を、指令から実際の機体速度までの間に効かせる。
    // 上限値 (牽引限界・角速度・車輪周速) は 0 を「上限なし」として扱うので、
    // すべて 0・ゲイン 1・tau = むだ時間 = 0 のモデルを渡せば完全な素通しになる。
    void setMotionModel(const RobotMotionModel &model);
    const RobotMotionModel &motionModel() const { return model; }
    // 同定モデルを通す前の生指令 [mm/s, rad/s]。診断 ([Diag] RobotCsvPath) が
    // 「RAVEN が何を出したか」と「台が実際に何を出したか」を並べるために読む。
    float getCmdTangent() const { return cmdTangent; }
    float getCmdNormal() const { return cmdNormal; }
    float getCmdAngular() const { return cmdAngular; }
    // Advance applied velocity one tick; getVel* then return the applied value.
    void advanceActuation(float dtSec);

    // Stops all latched motion/kick/dribble commands and their actuation-delay
    // pipeline immediately (no ramp-down). Called when this robot is teleported
    // (Replacement): without this, the previous velocity command stays latched
    // and the very next simulation tick re-applies it, driving the robot away
    // from the spot it was just placed at.
    void resetMotion();

    uint32_t getId() const;
    float getKickspeedx() const;
    float getKickspeedz() const;
    float getVeltangent() const;
    float getVelnormal() const;
    float getVelangular() const;
    float getSpinner() const;
    bool getWheelsspeed() const;
    float getWheel1() const;
    float getWheel2() const;
    float getWheel3() const;
    float getWheel4() const;

private:
    void processMoveCommand(const RobotMoveCommand &moveCommand);

    uint32_t id;
    float kickspeedx;
    float kickspeedz;
    float veltangent;
    float velnormal;
    float velangular;

    float spinner;
    bool wheelsspeed;

    float wheel1;
    float wheel2;
    float wheel3;
    float wheel4;

    // --- Actuation delay model state ---
    // Raw command targets (set by vision/control update); veltangent/velnormal/
    // velangular hold the *applied* values that QML reads.
    float cmdTangent = 0.0f;
    float cmdNormal = 0.0f;
    float cmdAngular = 0.0f;
    float appliedTangent = 0.0f;
    float appliedNormal = 0.0f;
    float appliedAngular = 0.0f;
    RobotMotionModel model;
    std::deque<float> delayBufTangent;
    std::deque<float> delayBufNormal;
    std::deque<float> delayBufAngular;

    // 指令を buf に積み、deadTimeSec ぶん前の値を返す (輸送遅れ)。
    static float delayed(std::deque<float> &buf, float cmd, float deadTimeSec, float dtSec);
    // applied から target へ 1 tick 進める: 時定数 tauSec の一次遅れをかけ、
    // 加速側 / 減速側で別々の上限 (mm/s^2 または rad/s^2) で頭を押さえる。
    static float advanceAxis(float applied, float target, float tauSec,
                             float accelLimit, float decelLimit, float dtSec);
    // 並進 + 旋回の車輪周速が予算を超えるぶんだけ twist 全体を縮める。
    void applyWheelSpeedBudget(float &vx, float &vy, float &vw) const;
};

#endif // ROBOT_H