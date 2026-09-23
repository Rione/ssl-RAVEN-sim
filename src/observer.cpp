#include "observer.h"

#include <cmath>

Observer::Observer(QObject *parent) : QObject(parent), config("../config/config_v2.ini", QSettings::IniFormat) {
    visionMulticastAddress = config.value("Network/visionMulticastAddress", "127.0.0.1").toString();
    visionMulticastPort = config.value("Network/visionMulticastPort", 10020).toInt();
    commandListenPort = config.value("Network/commandListenPort", 20011).toInt();
    blueTeamControlPort = config.value("Network/blueTeamControlPort", 10301).toInt();
    yellowTeamControlPort = config.value("Network/yellowTeamControlPort", 10302).toInt();
    
    forceDebugDrawMode = config.value("Display/ForceDebugDrawMode", false).toBool();
    lightBlueRobotMode = config.value("LightMode/BlueRobot", true).toBool();
    lightYellowRobotMode = config.value("LightMode/YellowRobot", true).toBool();
    lightStadiumMode = config.value("LightMode/Stadium", true).toBool();
    lightFieldMode = config.value("LightMode/Field", true).toBool();
    ballStaticFriction = config.value("Physics/BallStaticFriction", 0.5).toFloat();
    ballRestitution = config.value("Physics/BallRestitution", 0.5).toFloat();
    kickerFriction = config.value("Physics/KickerFriction", 0.8).toFloat();
    gravity = config.value("Physics/Gravity", 9.81).toFloat();
    desiredFps = 60;
    ccdMode = config.value("Physics/CCD", true).toBool();
    hideBallMode = config.value("Camera/HideBallMode", false).toBool();
    onboardCameraWidth = config.value("Camera/OnboardFrameWidth", 640).toInt();
    onboardCameraHeight = config.value("Camera/OnboardFrameHeight", 480).toInt();

    sender = new Sender(visionMulticastAddress.toStdString(), visionMulticastPort, this);
    visionReceiver = new VisionReceiver(this);
    controlBlueReceiver = new ControlBlueReceiver(this);
    controlYellowReceiver = new ControlYellowReceiver(this);

    visionReceiver->startListening(commandListenPort);
    controlBlueReceiver->startListening(blueTeamControlPort);
    controlYellowReceiver->startListening(yellowTeamControlPort);

    connect(visionReceiver, &VisionReceiver::receivedPacket, this, &Observer::visionReceive);
    connect(controlBlueReceiver, &ControlBlueReceiver::receivedPacket, this, &Observer::controlReceive);
    connect(controlYellowReceiver, &ControlYellowReceiver::receivedPacket, this, &Observer::controlReceive);
    connect(sender, &Sender::packetSent, this, &Observer::visionPacketSent);
    connect(this, &Observer::sendBotBallContacts, controlBlueReceiver, &ControlBlueReceiver::updateBallContacts);
    connect(this, &Observer::sendBotBallContacts, controlYellowReceiver, &ControlYellowReceiver::updateBallContacts);

    for (int i = 0; i < MaxRobots; ++i) {
        blueRobots[i] = new Robot(this);
        yellowRobots[i] = new Robot(this);
    }

    windowWidth = config.value("Display/width", 1100).toInt();
    windowHeight = config.value("Display/height", 720).toInt();

    blueRobotCount = config.value("Robot/blueRobotCount", 11).toInt();
    yellowRobotCount = config.value("Robot/yellowRobotCount", 11).toInt();

    numThreads = config.value("Display/NumThreads", -1).toInt();

    // --- Synthetic wheel-encoder feedback + actuation delay model ---
    encoderEnabled = config.value("Encoder/Enabled", true).toBool();
    encoderTeamYellow = (config.value("Encoder/Team", "blue").toString().toLower() == "yellow");
    wheelRadiusMm = config.value("Encoder/WheelRadiusMm", 26.0).toDouble();
    robotRadiusMm = config.value("Encoder/RobotRadiusMm", 90.0).toDouble();
    wheelAngleRad[0] = config.value("Encoder/WheelAngleFlDeg", 60.0).toDouble() * M_PI / 180.0;
    wheelAngleRad[1] = config.value("Encoder/WheelAngleBlDeg", 135.0).toDouble() * M_PI / 180.0;
    wheelAngleRad[2] = config.value("Encoder/WheelAngleBrDeg", -135.0).toDouble() * M_PI / 180.0;
    wheelAngleRad[3] = config.value("Encoder/WheelAngleFrDeg", -60.0).toDouble() * M_PI / 180.0;
    encoderNoiseSigmaMps = config.value("Encoder/NoiseSigmaMps", 0.0).toDouble();
    encoderBiasMps = config.value("Encoder/BiasMps", 0.0).toDouble();
    encoderQuantMps = config.value("Encoder/QuantizationMps", 0.0).toDouble();
    QString feedbackAddress = config.value("Encoder/FeedbackAddress", "224.5.69.4").toString();
    int feedbackPort = config.value("Encoder/FeedbackPort", 16941).toInt();
    feedbackSender = new FeedbackSender(feedbackAddress.toStdString(),
                                        static_cast<unsigned short>(feedbackPort));

    loadRobotModels();

    // --- 追従診断 ---
    QString diagPath = config.value("Diag/RobotCsvPath", "").toString();
    diagRobotId = config.value("Diag/RobotId", -1).toInt();
    if (!diagPath.isEmpty()) {
        diagCsv.open(diagPath.toStdString(), std::ios::out | std::ios::trunc);
        diagEnabled = diagCsv.is_open();
        if (diagEnabled) {
            // cmd_* = RAVEN が出した生指令、app_* = 同定モデルを通した値 (台に与える速度)、
            // meas_* = vision に出る実際の機体速度 (姿勢差分を機体座標へ)。
            diagCsv << "t,id,cmd_vx,cmd_vy,cmd_w,app_vx,app_vy,app_w,meas_vx,meas_vy,meas_w\n";
        } else {
            qWarning("[Diag] %s を開けなかった", qPrintable(diagPath));
        }
    }

    // --- Ball model (RAVEN の BallSpeedModel と同じ 2 段一定減速) ---
    // RAVEN 側は system_model の ball_model で持っていて、パスの初速逆算も到達時刻の
    // 予測もすべてこの 3 つから引いている (common/physics/BallPhysics)。sim が別の
    // パラメータ化 (摩擦係数) で転がしていると、RAVEN の「ここで受け取れる」が
    // 外れ続ける。既定値は 0917 の実測同定値。
    ballSlideDecelMmS2 = std::fabs(config.value("BallModel/AccSlideMmS2", 2159.324207613644).toFloat());
    ballRollDecelMmS2 = std::fabs(config.value("BallModel/AccRollMmS2", 213.609470182153).toFloat());
    ballSwitchRatio = config.value("BallModel/KSwitch", 2.0 / 3.0).toFloat();
    ballNormalRestitution = config.value("BallModel/DirectKickNormalRestitution", 0.8).toFloat();
    ballTangentRetention = config.value("BallModel/DirectKickTangentRetention", 1.0).toFloat();
    // NOTE: no wall-clock simulation timer. Vision/actuation/feedback are driven
    // from updateObjects() once per physics frame (simulation time) — see the
    // comment there.
}

// config_v2.ini から各 ID のプラントモデルを組み立てて青黄の台に載せる。
//
//   [RobotModel]       … 全 ID の既定値 (Enabled=false でモデルを丸ごと切れる)
//   [RobotModel.<id>]  … その ID の差分。書かれたキーだけが既定値を上書きする。
//
// 値の出どころは RAVEN の app/config/system_model_real_<mac>_ID<n>.yaml。ID 2/4/11 は
// 実機の同定値そのまま、それ以外は 3 台の世代のどれかを母体にしたクローン。
// 詳細は docs/robot_motion_model.md。
void Observer::loadRobotModels() {
    robotModelEnabled = config.value("RobotModel/Enabled", true).toBool();

    RobotMotionModel base;
    // 車輪配置は [Encoder] と同じものを使う。エンコーダの合成と車輪周速の予算が
    // 別々の幾何を見ていると、RAVEN から見て辻褄が合わなくなる。
    base.robotRadiusMm = static_cast<float>(robotRadiusMm);
    for (int k = 0; k < 4; ++k) {
        base.wheelAngleRad[k] = static_cast<float>(wheelAngleRad[k]);
    }

    if (!robotModelEnabled) {
        // 素通し: ゲイン 1、遅れなし、上限なし (0 は「上限なし」)。QML 側も
        // robotModelEnabled が false なら従来の MotionControl 経由に戻る。
        base.tauVxSec = base.tauVySec = base.tauOmegaSec = 0.0f;
        base.deadTimeSec = 0.0f;
        base.tractionAccelXMmS2 = base.tractionAccelYMmS2 = 0.0f;
        base.tractionDecelXMmS2 = base.tractionDecelYMmS2 = 0.0f;
        base.maxAngularVelRadS = base.maxAngularAccelRadS2 = 0.0f;
        base.wheelRimSpeedBudgetMmS = 0.0f;
        for (int i = 0; i < MaxRobots; ++i) {
            blueRobots[i]->setMotionModel(base);
            yellowRobots[i]->setMotionModel(base);
        }
        return;
    }

    base = RobotMotionModel::fromSettings(config, "RobotModel", base);
    for (int i = 0; i < MaxRobots; ++i) {
        const RobotMotionModel m =
            RobotMotionModel::fromSettings(config, QString("RobotModel.%1").arg(i), base);
        blueRobots[i]->setMotionModel(m);
        yellowRobots[i]->setMotionModel(m);
    }
}

void Observer::visionReceive(const mocSim_Packet& packet) {
    bool isYellow = packet.commands().isteamyellow();
    for (const auto& command : packet.commands().robot_commands()) {
        int id = command.id();
        if (id < 0 || id >= MaxRobots) continue;
        if (isYellow) {
            yellowRobots[id]->visionUpdate(command);
        } else {
            blueRobots[id]->visionUpdate(command);
        }
    }
    if (isYellow) emit yellowRobotsChanged();
    else emit blueRobotsChanged();

    // Robot/ball placement (Replacement). turnon=false (robot removal) is not
    // handled: this side always sends turnon=true.
    bool blueReplaced = false;
    bool yellowReplaced = false;
    for (const auto& robotReplacement : packet.replacement().robots()) {
        int id = robotReplacement.id();
        if (id < 0 || id >= MaxRobots) continue;
        float sceneX = robotReplacement.x() * 1000.0f;
        float sceneZ = -robotReplacement.y() * 1000.0f;
        float sceneRotYDeg = robotReplacement.dir() * 180.0 / M_PI - 90.0;
        // Stop any velocity/kick/dribble command that was in flight for this robot
        // *before* the teleport. Without this the previous command is still latched
        // and gets re-applied on the very next tick, so the robot immediately drives
        // off again and the teleport looks like it never took effect.
        if (robotReplacement.yellowteam()) {
            yellowRobots[id]->resetMotion();
            yellowReplaced = true;
        } else {
            blueRobots[id]->resetMotion();
            blueReplaced = true;
        }
        emit robotReplacementRequested(id, robotReplacement.yellowteam(), sceneX, sceneZ, sceneRotYDeg);
    }
    // Push the zeroed motion state to QML now (rather than waiting for the next
    // incoming commands packet for that team), so botMovement() doesn't have a
    // window where it still reads the stale, pre-reset velocity.
    if (blueReplaced) emit blueRobotsChanged();
    if (yellowReplaced) emit yellowRobotsChanged();

    if (packet.replacement().has_ball()) {
        const auto& ballReplacement = packet.replacement().ball();
        if (ballReplacement.has_x() && ballReplacement.has_y()) {
            float sceneX = ballReplacement.x() * 1000.0f;
            float sceneZ = -ballReplacement.y() * 1000.0f;
            bool hasVelocity = ballReplacement.has_vx() || ballReplacement.has_vy();
            float sceneVx = ballReplacement.has_vx() ? static_cast<float>(ballReplacement.vx() * 1000.0) : 0.0f;
            float sceneVz = ballReplacement.has_vy() ? static_cast<float>(-ballReplacement.vy() * 1000.0) : 0.0f;
            emit ballReplacementRequested(sceneX, sceneZ, hasVelocity, sceneVx, sceneVz);
        }
    }
}

void Observer::controlReceive(const RobotControl& packet, bool isYellow) {
    int receive_count = 0;
    for (const auto& robotCommand : packet.robot_commands()) {
        int id = robotCommand.id();
        if (id < 0 || id >= MaxRobots) continue;
        if (!robotCommand.has_move_command()) continue;
        if (isYellow) {
            yellowRobots[id]->controlUpdate(robotCommand);
        } else {
            blueRobots[id]->controlUpdate(robotCommand);
        }
        receive_count++;
    }
    if (receive_count == 0) return;
    if (isYellow) emit yellowRobotsChanged();
    else emit blueRobotsChanged();
}

void Observer::setWindowWidth(int width) { 
    windowWidth = width; 
    config.setValue("Display/width", width);
    emit settingChanged(); 
}
void Observer::setWindowHeight(int height) { 
    windowHeight = height; 
    config.setValue("Display/height", height);
    emit settingChanged(); 
}
void Observer::setVisionMulticastPort(int port) { 
    visionMulticastPort = port; 
    config.setValue("Network/visionMulticastPort", port);
    sender->setPort(visionMulticastAddress.toStdString(), visionMulticastPort);
    emit settingChanged(); 
}
void Observer::setVisionMulticastAddress(const QString &address) {
    visionMulticastAddress = address;
    config.setValue("Network/visionMulticastAddress", QString::fromStdString(visionMulticastAddress.toStdString()));
    sender->setPort(visionMulticastAddress.toStdString() , visionMulticastPort);
    emit settingChanged();
}
void Observer::setCommandListenPort(int port) {
    commandListenPort = port;
    config.setValue("Network/commandListenPort", port);
    visionReceiver->setPort(commandListenPort);
    emit settingChanged();
}
void Observer::setBlueTeamControlPort(int port) {
    blueTeamControlPort = port;
    config.setValue("Network/blueTeamControlPort", port);
    controlBlueReceiver->setPort(blueTeamControlPort);
    emit settingChanged();
}
void Observer::setYellowTeamControlPort(int port) {
    yellowTeamControlPort = port;
    config.setValue("Network/yellowTeamControlPort", port);
    controlYellowReceiver->setPort(yellowTeamControlPort);
    emit settingChanged();
}
void Observer::setForceDebugDrawMode(bool mode) {
    forceDebugDrawMode = mode;
    config.setValue("Display/ForceDebugDrawMode", mode);
    emit settingChanged();
}
void Observer::setLightBlueRobotMode(bool mode) {
    lightBlueRobotMode = mode;
    config.setValue("LightMode/BlueRobot", mode);
    emit settingChanged();
}
void Observer::setLightYellowRobotMode(bool mode) {
    lightYellowRobotMode = mode;
    config.setValue("LightMode/YellowRobot", mode);
    emit settingChanged();
}
void Observer::setLightStadiumMode(bool mode) {
    lightStadiumMode = mode;
    config.setValue("LightMode/Stadium", mode);
    emit settingChanged();
}
void Observer::setLightFieldMode(bool mode) {
    lightFieldMode = mode;
    config.setValue("LightMode/Field", mode);
    emit settingChanged();
}
void Observer::setBlueRobotCount(int count) {
    blueRobotCount = count;
    config.setValue("Robot/blueRobotCount", count);
    emit settingChanged();
}
void Observer::setYellowRobotCount(int count) {
    yellowRobotCount = count;
    config.setValue("Robot/yellowRobotCount", count);
    emit settingChanged();
}
void Observer::setBallRestitution(float restitution) {
    ballRestitution = restitution;
    config.setValue("Physics/BallRestitution", qRound(restitution*100)/100.0);
    emit settingChanged();
}
void Observer::setBallSlideDecelMmS2(float decel) {
    ballSlideDecelMmS2 = std::fabs(decel);
    config.setValue("BallModel/AccSlideMmS2", -ballSlideDecelMmS2);
    emit settingChanged();
}
void Observer::setBallRollDecelMmS2(float decel) {
    ballRollDecelMmS2 = std::fabs(decel);
    config.setValue("BallModel/AccRollMmS2", -ballRollDecelMmS2);
    emit settingChanged();
}
void Observer::setKickerFriction(float friction) {
    kickerFriction = friction;
    config.setValue("Physics/KickerFriction", qRound(friction*100)/100.0);
    emit settingChanged();
}
void Observer::setGravity(float gravity) {
    this->gravity = gravity;
    config.setValue("Physics/Gravity", qRound(gravity*100)/100.0);
    emit settingChanged();
}
void Observer::setDesiredFps(int fps) {
    Q_UNUSED(fps);
    desiredFps = 60;
    config.setValue("Physics/DesiredFps", desiredFps);
    emit settingChanged();
}
void Observer::setCcdMode(bool mode) {
    ccdMode = mode;
    config.setValue("Physics/CCD", mode);
    emit settingChanged();
}
void Observer::setNumThreads(int threads) {
    numThreads = threads;
    config.setValue("Display/NumThreads", numThreads);
    emit settingChanged();
}
void Observer::setHideBallMode(bool mode) {
    hideBallMode = mode;
    config.setValue("Camera/HideBallMode", mode);
    emit settingChanged();
}

void Observer::updateObjects(
    QList<QVector3D> blue_positions, 
    QList<QVector3D> yellow_positions,
    QList<QVector2D> blueBallPixels,
    QList<QVector2D> yellowBallPixels,
    QList<bool> blueBallCameraExists,
    QList<bool> yellowBallCameraExists,
    QList<bool> bBotBallContacts, 
    QList<bool> yBotBallContacts,
    QVector3D ball_position,
    bool isFoundBall
) {
    bluePositions = blue_positions.mid(0, blueRobotCount);
    yellowPositions = yellow_positions.mid(0, yellowRobotCount);

    // updateObjects() runs once per PHYSICS frame (syncGameObjects <-
    // PhysicsWorld::onFrameDone), each advancing exactly 1/60 s of simulation.
    // Everything time-based below therefore uses the fixed simulation step, and
    // vision is emitted from here — one packet per physics frame — instead of a
    // wall-clock QTimer. With the old 60 Hz wall timer the vision rate and physics
    // rate diverged whenever the render loop ran off 60 fps (headless/offscreen:
    // ~50 fps → ~17% duplicate-pose frames + all speeds scaled by the ratio;
    // occluded window: physics frozen while vision streamed the stale world).
    const float simDtSec = 1.0f / 60.0f;

    // Advance the actuation delay model before QML reads applied velocities next
    // frame (first-order lag + dead time are simulation dynamics — sim time).
    for (int i = 0; i < MaxRobots; ++i) {
        blueRobots[i]->advanceActuation(simDtSec);
        yellowRobots[i]->advanceActuation(simDtSec);
    }
    // 適用速度が動いたことを QML に伝える。これが無いと QML 側のキャッシュは
    // 指令パケットが届いた瞬間しか更新されず、むだ時間と一次遅れの立ち上がりが
    // パケットの到着間隔ぶん飛び飛びになる。
    emit actuationAdvanced();

    // Synthesize RACOON-Pi feedback (wheel encoders + onboard camera + sensors)
    // for the team RAVEN controls. Differentiation dt = simulation step (the pose
    // delta it differentiates is exactly one physics frame apart).
    writeRobotDiag(encoderTeamYellow ? yellowPositions : bluePositions, simDtSec);

    emitEncoderFeedback(encoderTeamYellow ? yellowPositions : bluePositions,
                        encoderTeamYellow ? yellowBallCameraExists : blueBallCameraExists,
                        encoderTeamYellow ? yellowBallPixels : blueBallPixels,
                        encoderTeamYellow ? yBotBallContacts : bBotBallContacts,
                        simDtSec);

    if (isFoundBall)
        this->ballPosition = ball_position;
    emit sendBotBallContacts(bBotBallContacts, yBotBallContacts, blueBallCameraExists, yellowBallCameraExists, blueBallPixels, yellowBallPixels);

    emit updateSimulationSignal();
    sender->send(1, ballPosition, bluePositions, yellowPositions);
}

// RAVEN の指令・同定モデルの出力・実際の機体速度を 1 行に並べて書く。
// 実速度は vision と同じ姿勢 (positions[i] = X[mm], Y[mm], heading[deg]) の差分を
// 機体座標へ回したもの — RAVEN から見える速度そのもの。
void Observer::writeRobotDiag(const QList<QVector3D> &positions, float dtSec) {
    if (!diagEnabled || dtSec <= 0.0f) {
        return;
    }
    const int n = positions.size();
    if (prevEncoderPositions.size() != n) {
        return;  // 差分が取れるのは 2 フレーム目から (prevEncoderPositions は下の合成が更新する)
    }
    auto *team = encoderTeamYellow ? yellowRobots.data() : blueRobots.data();
    for (int i = 0; i < n; ++i) {
        if (diagRobotId >= 0 && i != diagRobotId) {
            continue;
        }
        const QVector3D &p = positions[i];
        const QVector3D &pp = prevEncoderPositions[i];
        const double wvx = (p.x() - pp.x()) / dtSec;
        const double wvy = (p.y() - pp.y()) / dtSec;
        const double dthDeg = std::fmod(p.z() - pp.z() + 540.0, 360.0) - 180.0;
        const double measW = (dthDeg * M_PI / 180.0) / dtSec;
        const double w = p.z() * M_PI / 180.0;
        const double measVx = wvx * std::cos(w) + wvy * std::sin(w);
        const double measVy = -wvx * std::sin(w) + wvy * std::cos(w);
        const Robot *r = team[i];
        diagCsv << diagTimeSec << ',' << i << ','
                << r->getCmdTangent() << ',' << r->getCmdNormal() << ',' << r->getCmdAngular() << ','
                << r->getVeltangent() << ',' << r->getVelnormal() << ',' << r->getVelangular() << ','
                << measVx << ',' << measVy << ',' << measW << '\n';
    }
    diagTimeSec += dtSec;
}

void Observer::emitEncoderFeedback(const QList<QVector3D> &positions,
                                   const QList<bool> &ballCameraExists,
                                   const QList<QVector2D> &ballCameraPixels,
                                   const QList<bool> &ballContacts,
                                   float dtSec) {
    if (!encoderEnabled || feedbackSender == nullptr) {
        return;
    }
    const int n = positions.size();
    if (prevEncoderPositions.size() != n || dtSec <= 0.0f || dtSec > 0.5f) {
        prevEncoderPositions = positions;  // (re)seed; need two frames to differentiate
        return;
    }
    // Onboard-camera frame center: pixels from QML are top-left origin; RACOON-Pi
    // reports center-origin, x right / y up (camera/transport/encoder.py).
    const float halfW = onboardCameraWidth * 0.5f;
    const float halfH = onboardCameraHeight * 0.5f;
    std::normal_distribution<double> gauss(0.0, encoderNoiseSigmaMps > 0.0 ? encoderNoiseSigmaMps : 1.0);
    for (int i = 0; i < n; ++i) {
        // positions[i] = (X = frame.x [mm], Y = -frame.z [mm], heading [deg]) —
        // the same pose RAVEN receives on vision, so the encoder agrees with it.
        const QVector3D &p = positions[i];
        const QVector3D &pp = prevEncoderPositions[i];
        double wvx = (p.x() - pp.x()) / dtSec;  // world vx [mm/s]
        double wvy = (p.y() - pp.y()) / dtSec;  // world vy [mm/s]
        double dthDeg = std::fmod(p.z() - pp.z() + 540.0, 360.0) - 180.0;  // wrapped [deg]
        double omega = (dthDeg * M_PI / 180.0) / dtSec;  // [rad/s]
        double w = p.z() * M_PI / 180.0;                 // heading [rad]
        // Rotate world velocity into the body frame (+x heading, +y left).
        double forward = wvx * std::cos(w) + wvy * std::sin(w);   // body vx [mm/s]
        double left = -wvx * std::sin(w) + wvy * std::cos(w);     // body vy [mm/s]

        float wheelMps[4];
        for (int k = 0; k < 4; ++k) {
            // v_k = sin(α)·vx − cos(α)·vy − R·ω  [mm/s] (matches RAVEN forward kin.)
            double vMm = std::sin(wheelAngleRad[k]) * forward
                       - std::cos(wheelAngleRad[k]) * left
                       - robotRadiusMm * omega;
            double vMps = vMm / 1000.0 + encoderBiasMps;
            if (encoderNoiseSigmaMps > 0.0) {
                vMps += gauss(encoderRng);
            }
            if (encoderQuantMps > 0.0) {
                vMps = std::round(vMps / encoderQuantMps) * encoderQuantMps;
            }
            wheelMps[k] = static_cast<float>(vMps);
        }

        FeedbackSender::RobotFeedback fb;
        fb.flMps = wheelMps[0];
        fb.blMps = wheelMps[1];
        fb.brMps = wheelMps[2];
        fb.frMps = wheelMps[3];

        // Onboard-camera ball detection (RACOON-Pi Ball_Status).
        const bool ballSeen = i < ballCameraExists.size() && ballCameraExists[i];
        fb.ballExists = ballSeen;
        if (ballSeen && i < ballCameraPixels.size()) {
            const QVector2D &px = ballCameraPixels[i];  // top-left pixel origin
            fb.ballCamX = px.x() - halfW;               // +x right
            fb.ballCamY = halfH - px.y();               // +y up
        }

        // Kicker photo / dribbler sensors. The sim exposes a single "ball at the
        // mouth" contact (holds[i]); RACOON-Pi has both an IR kicker sensor and a
        // dribbler sensor that are effectively co-asserted when the ball is held.
        const bool ballHeld = i < ballContacts.size() && ballContacts[i];
        fb.photoSensor = ballHeld;
        fb.dribblerSensor = ballHeld;

        feedbackSender->sendRobotFeedback(i, fb);
    }
    prevEncoderPositions = positions;
}