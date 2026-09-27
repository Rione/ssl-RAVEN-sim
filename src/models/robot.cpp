#include "robot.h"
#include <algorithm>
#include <cmath>

Robot::Robot(QObject *parent)
    : QObject(parent),
      id(0),
      kickspeedx(0.0f),
      kickspeedz(0.0f),
      veltangent(0.0f),
      velnormal(0.0f),
      velangular(0.0f),
      spinner(false),
      wheelsspeed(false),
      wheel1(0.0f),
      wheel2(0.0f),
      wheel3(0.0f),
      wheel4(0.0f) {}

Robot::~Robot() = default;

void Robot::visionUpdate(mocSim_Robot_Command robotCommand) {
    id = robotCommand.id();
    kickspeedx = robotCommand.kickspeedx()*1000.0;
    kickspeedz = robotCommand.kickspeedz()*1000.0;
    // Velocity goes through the actuation delay model (advanceActuation), not
    // straight to veltangent/velnormal/velangular which hold the applied value.
    cmdTangent = robotCommand.veltangent()*1000.0;
    cmdNormal = robotCommand.velnormal()*1000.0;
    cmdAngular = robotCommand.velangular();
    spinner = robotCommand.spinner() ? 1.0 : 0.0;
    wheelsspeed = robotCommand.wheelsspeed();
    wheel1 = robotCommand.wheel1();
    wheel2 = robotCommand.wheel2();
    wheel3 = robotCommand.wheel3();
    wheel4 = robotCommand.wheel4();
}

void Robot::controlUpdate(RobotCommand robotCommand) {
    id = robotCommand.id();

    if (robotCommand.has_kick_speed() && robotCommand.kick_speed() > 0) {
        double kickSpeed = robotCommand.kick_speed();
        double limit = robotCommand.kick_angle() > 0 ? 10000 : 10001;
        kickSpeed = kickSpeed * 1000.0;
        if (kickSpeed > limit) {
            kickSpeed = limit;
        }
        double kickAngle = robotCommand.kick_angle() * M_PI / 180.0;
        double length = cos(kickAngle) * kickSpeed;
        double z = sin(kickAngle) * kickSpeed;
        
        kickspeedx = length;
        kickspeedz = z;
    } else {
        kickspeedx = 0;
        kickspeedz = 0;
    }

    spinner = 0.0;
    if (robotCommand.has_dribbler_speed()) {
        spinner = robotCommand.dribbler_speed();
    }

    if (robotCommand.has_move_command()) {
        processMoveCommand(robotCommand.move_command());
    }
}

void Robot::processMoveCommand(const RobotMoveCommand &moveCommand) {
    if (moveCommand.has_wheel_velocity()) {
        // auto &wheelVel = moveCommand.wheel_velocity();
        // robot->setSpeed(0, wheelVel.front_right());
        // robot->setSpeed(1, wheelVel.back_right());
        // robot->setSpeed(2, wheelVel.back_left());
        // robot->setSpeed(3, wheelVel.front_left());
    } else if (moveCommand.has_local_velocity()) {
        auto &vel = moveCommand.local_velocity();
        cmdNormal = vel.left()*1000.0;
        cmdTangent = vel.forward()*1000.0;
        cmdAngular = vel.angular();
    } else if(moveCommand.has_global_velocity()) {
        // auto &vel = moveCommand.global_velocity();
        // dReal orientation = -robot->getDir() * M_PI / 180.0;
        // dReal vx = (vel.x() * cos(orientation)) - (vel.y() * sin(orientation));
        // dReal vy = (vel.y() * cos(orientation)) + (vel.x() * sin(orientation));
        // robot->setSpeed(vx, vy, vel.angular());
    }  else {
        // SimulatorError *pError = robotControlResponse.add_errors();
        // pError->set_code("GRSIM_UNSUPPORTED_MOVE_COMMAND");
        // pError->set_message("Unsupported move command");
    }
}

void Robot::setMotionModel(const RobotMotionModel &m) {
    model = m;
}

// 指令を遅延線に積み、deadTimeSec ぶん前の値を取り出す。
// deadTimeSec = 0 なら押した値がそのまま返る。
float Robot::delayed(std::deque<float> &buf, float cmd, float deadTimeSec, float dtSec) {
    int delaySteps = (deadTimeSec > 0.0f && dtSec > 0.0f)
                         ? static_cast<int>(std::lround(deadTimeSec / dtSec))
                         : 0;
    buf.push_back(cmd);
    while (static_cast<int>(buf.size()) > delaySteps + 1) {
        buf.pop_front();
    }
    // 履歴が足りないあいだ (起動直後、resetMotion() 直後、むだ時間を伸ばした直後) は
    // 前に 0 を詰める。詰めないと front() が push したばかりの指令になってしまい、
    // そこから delaySteps tick のあいだだけ「むだ時間 0 の台」になる — teleport のたびに
    // MPC から見た台が入れ替わることになるので、ここは必ず埋める。
    while (static_cast<int>(buf.size()) < delaySteps + 1) {
        buf.push_front(0.0f);
    }
    return buf.front();  // command from ~delaySteps ticks ago
}

// 一次遅れ + 加減速の頭打ち。tau = 0 なら 1 tick で target に届こうとするが、
// そのぶん加減速の上限が効く — 実機で「指令が飛んでも台はついてこない」ぶん。
float Robot::advanceAxis(float applied, float target, float tauSec,
                         float accelLimit, float decelLimit, float dtSec) {
    const float alpha = (tauSec > 0.0f) ? (1.0f - std::exp(-dtSec / tauSec)) : 1.0f;
    float next = applied + alpha * (target - applied);

    // 速さが増える向きなら加速側、減る向きなら減速側の上限。符号が反転する
    // 場合は「まず減速」なので減速側で押さえる。
    const float limit = (std::fabs(next) > std::fabs(applied)) ? accelLimit : decelLimit;
    if (limit > 0.0f) {
        const float maxDelta = limit * dtSec;
        const float delta = next - applied;
        if (std::fabs(delta) > maxDelta) {
            next = applied + std::copysign(maxDelta, delta);
        }
    }
    return next;
}

// 車輪 k の周速 v_k = sin(α_k)·vx − cos(α_k)·vy − R·ω [mm/s]
// (Observer::emitEncoderFeedback と同じ順運動学)。いちばん速い車輪が予算を
// 超えるなら、その比で twist を丸ごと縮める。全速で走りながら全速で回れない、
// という実機の制約がこれで入る。
void Robot::applyWheelSpeedBudget(float &vx, float &vy, float &vw) const {
    const float budget = model.wheelRimSpeedBudgetMmS;
    if (budget <= 0.0f) {
        return;
    }
    float peak = 0.0f;
    for (int k = 0; k < 4; ++k) {
        const float v = std::sin(model.wheelAngleRad[k]) * vx
                      - std::cos(model.wheelAngleRad[k]) * vy
                      - model.robotRadiusMm * vw;
        peak = std::max(peak, std::fabs(v));
    }
    if (peak > budget) {
        const float scale = budget / peak;
        vx *= scale;
        vy *= scale;
        vw *= scale;
    }
}

void Robot::resetMotion() {
    kickspeedx = 0.0f;
    kickspeedz = 0.0f;
    spinner = 0.0f;

    cmdTangent = 0.0f;
    cmdNormal = 0.0f;
    cmdAngular = 0.0f;
    appliedTangent = 0.0f;
    appliedNormal = 0.0f;
    appliedAngular = 0.0f;
    veltangent = 0.0f;
    velnormal = 0.0f;
    velangular = 0.0f;

    // Drop anything sitting in the transport-delay pipeline so a previously
    // latched command can't re-emerge a few ticks later.
    delayBufTangent.clear();
    delayBufNormal.clear();
    delayBufAngular.clear();
}

// 指令 (cmd*) から実際に台へ与える速度 (veltangent/velnormal/velangular) までを
// 1 tick 進める。実機の同定モデルの順で効かせる:
//   むだ時間 → 定常ゲイン → 角速度上限 → 車輪周速の予算 → 一次遅れ → 軸別の加減速上限
// 既定のモデル (設定なし) では素通しになる。
void Robot::advanceActuation(float dtSec) {
    if (dtSec <= 0.0f) {
        return;
    }

    // むだ時間: いま効くのは deadTimeSec 前に出された指令。
    const float ux = delayed(delayBufTangent, cmdTangent, model.deadTimeSec, dtSec);
    const float uy = delayed(delayBufNormal, cmdNormal, model.deadTimeSec, dtSec);
    const float uw = delayed(delayBufAngular, cmdAngular, model.deadTimeSec, dtSec);

    // 定常ゲイン: 指令どおりの速さは出ないし、前進指令が少し横に漏れる。
    float targetX = model.gainVx * ux + model.gainVxFromUy * uy;
    float targetY = model.gainVy * uy + model.gainVyFromUx * ux;
    float targetW = model.gainOmega * uw;

    if (model.maxAngularVelRadS > 0.0f && std::fabs(targetW) > model.maxAngularVelRadS) {
        targetW = std::copysign(model.maxAngularVelRadS, targetW);
    }
    applyWheelSpeedBudget(targetX, targetY, targetW);

    appliedTangent = advanceAxis(appliedTangent, targetX, model.tauVxSec,
                                 model.tractionAccelXMmS2, model.tractionDecelXMmS2, dtSec);
    appliedNormal = advanceAxis(appliedNormal, targetY, model.tauVySec,
                                model.tractionAccelYMmS2, model.tractionDecelYMmS2, dtSec);
    appliedAngular = advanceAxis(appliedAngular, targetW, model.tauOmegaSec,
                                 model.maxAngularAccelRadS2, model.maxAngularAccelRadS2, dtSec);

    veltangent = appliedTangent;
    velnormal = appliedNormal;
    velangular = appliedAngular;
}

uint32_t Robot::getId() const { return id; }
float Robot::getKickspeedx() const { return kickspeedx; }
float Robot::getKickspeedz() const { return kickspeedz; }
float Robot::getVeltangent() const { return veltangent; }
float Robot::getVelnormal() const { return velnormal; }
float Robot::getVelangular() const { return velangular; }
float Robot::getSpinner() const { return spinner; }
bool Robot::getWheelsspeed() const { return wheelsspeed; }
float Robot::getWheel1() const { return wheel1; }
float Robot::getWheel2() const { return wheel2; }
float Robot::getWheel3() const { return wheel3; }
float Robot::getWheel4() const { return wheel4; }
