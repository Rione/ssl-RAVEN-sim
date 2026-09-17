#ifndef OBSERVER_H
#define OBSERVER_H

#include <QObject>
#include <QThread>
#include <QUdpSocket>
#include <QHostAddress>
#include <QSettings>
#include <QTimer>
#include <QElapsedTimer>

#include <fstream>
#include <random>

#include "networks/receiver.h"
#include "networks/sender.h"
#include "networks/feedbackSender.h"
#include "models/robot.h"
#include "models/robotModel.h"
#include "mocSim_Packet.pb.h"
#include "ssl_simulation_robot_control.pb.h"

class Observer : public QObject {
    Q_OBJECT
    Q_PROPERTY(QList<QObject*> blue_robots READ getBlueRobots NOTIFY blueRobotsChanged)
    Q_PROPERTY(QList<QObject*> yellow_robots READ getYellowRobots NOTIFY yellowRobotsChanged)
    Q_PROPERTY(int windowWidth READ getWindowWidth WRITE setWindowWidth NOTIFY settingChanged)
    Q_PROPERTY(int windowHeight READ getWindowHeight WRITE setWindowHeight NOTIFY settingChanged)
    Q_PROPERTY(QString visionMulticastAddress READ getVisionMulticastAddress WRITE setVisionMulticastAddress NOTIFY settingChanged)
    Q_PROPERTY(int visionMulticastPort READ getVisionMulticastPort WRITE setVisionMulticastPort NOTIFY settingChanged)
    Q_PROPERTY(int commandListenPort READ getCommandListenPort WRITE setCommandListenPort NOTIFY settingChanged)
    Q_PROPERTY(int blueTeamControlPort READ getBlueTeamControlPort WRITE setBlueTeamControlPort NOTIFY settingChanged)
    Q_PROPERTY(int yellowTeamControlPort READ getYellowTeamControlPort WRITE setYellowTeamControlPort NOTIFY settingChanged)
    Q_PROPERTY(bool forceDebugDrawMode READ getForceDebugDrawMode WRITE setForceDebugDrawMode NOTIFY settingChanged)
    Q_PROPERTY(bool lightBlueRobotMode READ getLightBlueRobotMode WRITE setLightBlueRobotMode NOTIFY settingChanged)
    Q_PROPERTY(bool lightYellowRobotMode READ getLightYellowRobotMode WRITE setLightYellowRobotMode NOTIFY settingChanged)
    Q_PROPERTY(bool lightStadiumMode READ getLightStadiumMode WRITE setLightStadiumMode NOTIFY settingChanged)
    Q_PROPERTY(bool lightFieldMode READ getLightFieldMode WRITE setLightFieldMode NOTIFY settingChanged)
    Q_PROPERTY(int blueRobotCount READ getBlueRobotCount WRITE setBlueRobotCount NOTIFY settingChanged)
    Q_PROPERTY(int yellowRobotCount READ getYellowRobotCount WRITE setYellowRobotCount NOTIFY settingChanged)
    Q_PROPERTY(float ballRestitution READ getBallRestitution WRITE setBallRestitution NOTIFY settingChanged)
    Q_PROPERTY(float kickerFriction READ getKickerFriction WRITE setKickerFriction NOTIFY settingChanged)
    Q_PROPERTY(float gravity READ getGravity WRITE setGravity NOTIFY settingChanged)
    Q_PROPERTY(int desiredFps READ getDesiredFps WRITE setDesiredFps NOTIFY settingChanged)
    Q_PROPERTY(bool ccdMode READ getCcdMode WRITE setCcdMode NOTIFY settingChanged)
    Q_PROPERTY(int numThreads READ getNumThreads WRITE setNumThreads NOTIFY settingChanged)
    Q_PROPERTY(bool hideBallMode READ getHideBallMode WRITE setHideBallMode NOTIFY settingChanged)
    // Onboard (RACOON-Pi) camera frame size used when projecting the ball into
    // each robot's camera. QML reads these so the pixel scale matches the
    // center-origin conversion done for PiToMw.
    Q_PROPERTY(int onboardCameraWidth READ getOnboardCameraWidth CONSTANT)
    Q_PROPERTY(int onboardCameraHeight READ getOnboardCameraHeight CONSTANT)
    // 実機同定モデル ([RobotModel]) が有効か。有効なら QML は Robot が出す
    // 適用速度をそのまま台に与える (MotionControl の等方な加減速制限は通さない)。
    Q_PROPERTY(bool robotModelEnabled READ getRobotModelEnabled CONSTANT)
    // RAVEN のボールモデル (BallSpeedModel / system_model の ball_model) と同じ 3 つ。
    // 係数 (μ) ではなく mm/s^2 の減速度そのものなので、RAVEN の到達時刻・到達速度の
    // 予測と sim の実際が一致する。
    Q_PROPERTY(float ballSlideDecelMmS2 READ getBallSlideDecelMmS2 WRITE setBallSlideDecelMmS2 NOTIFY settingChanged)
    Q_PROPERTY(float ballRollDecelMmS2 READ getBallRollDecelMmS2 WRITE setBallRollDecelMmS2 NOTIFY settingChanged)
    Q_PROPERTY(float ballSwitchRatio READ getBallSwitchRatio CONSTANT)
    // ロボットに当たったときの跳ね返り (ball_model.direct_kick)。
    Q_PROPERTY(float ballNormalRestitution READ getBallNormalRestitution CONSTANT)
    Q_PROPERTY(float ballTangentRetention READ getBallTangentRetention CONSTANT)

public:
    static constexpr int MaxRobots = 16;
    explicit Observer(QObject *parent = nullptr);

    Q_INVOKABLE void updateObjects(
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
    );

    void start(quint16 port);
    void stop();

    void visionReceive(const mocSim_Packet& packet);
    void controlReceive(const RobotControl& packet, bool isYellow);

    QList<QObject*> getBlueRobots() const {
        QList<QObject*> blueList;
        for (int i = 0; i < blueRobotCount; ++i) {
            blueList.append(blueRobots[i]);
        }
        return blueList;
    }
    QList<QObject*> getYellowRobots() const {
        QList<QObject*> yellowList;
        for (int i = 0; i < yellowRobotCount; ++i) {
            yellowList.append(yellowRobots[i]);
        }
        return yellowList;
    }
    int getWindowWidth() const { return windowWidth; }
    int getWindowHeight() const { return windowHeight; }
    QString getVisionMulticastAddress() const { return visionMulticastAddress; }
    int getVisionMulticastPort() const { return visionMulticastPort; }
    int getCommandListenPort() const { return commandListenPort; }
    int getBlueTeamControlPort() const { return blueTeamControlPort; }
    int getYellowTeamControlPort() const { return yellowTeamControlPort; }
    bool getForceDebugDrawMode() const { return forceDebugDrawMode; }
    bool getLightBlueRobotMode() const { return lightBlueRobotMode; }
    bool getLightYellowRobotMode() const { return lightYellowRobotMode; }
    bool getLightStadiumMode() const { return lightStadiumMode; }
    bool getLightFieldMode() const { return lightFieldMode; }
    int getBlueRobotCount() const { return blueRobotCount; }
    int getYellowRobotCount() const { return yellowRobotCount; }
    float getBallRestitution() const { return ballRestitution; }
    float getKickerFriction() const { return kickerFriction; }
    float getGravity() const { return gravity; }
    int getDesiredFps() const { return desiredFps; }
    bool getCcdMode() const { return ccdMode; }
    int getNumThreads() const { return numThreads; }
    bool getHideBallMode() const { return hideBallMode; }
    int getOnboardCameraWidth() const { return onboardCameraWidth; }
    int getOnboardCameraHeight() const { return onboardCameraHeight; }
    bool getRobotModelEnabled() const { return robotModelEnabled; }
    float getBallSlideDecelMmS2() const { return ballSlideDecelMmS2; }
    float getBallRollDecelMmS2() const { return ballRollDecelMmS2; }
    float getBallSwitchRatio() const { return ballSwitchRatio; }
    float getBallNormalRestitution() const { return ballNormalRestitution; }
    float getBallTangentRetention() const { return ballTangentRetention; }

    void setWindowWidth(int width);
    void setWindowHeight(int height);
    void setVisionMulticastAddress(const QString &address);
    void setVisionMulticastPort(int port);
    void setCommandListenPort(int port);
    void setBlueTeamControlPort(int port);
    void setYellowTeamControlPort(int port);
    void setForceDebugDrawMode(bool mode);
    void setLightBlueRobotMode(bool mode);
    void setLightYellowRobotMode(bool mode);
    void setLightStadiumMode(bool mode);
    void setLightFieldMode(bool mode);
    void setBlueRobotCount(int count);
    void setYellowRobotCount(int count);
    void setBallRestitution(float restitution);
    void setBallSlideDecelMmS2(float decel);
    void setBallRollDecelMmS2(float decel);
    void setKickerFriction(float friction);
    void setGravity(float gravity);
    void setDesiredFps(int fps);
    void setCcdMode(bool mode);
    void setNumThreads(int threads);
    void setHideBallMode(bool mode);
    
signals:
    void blueRobotsChanged();
    void yellowRobotsChanged();
    void settingChanged();
    void sendBotBallContacts(
        const QList<bool>& bBotBallContacts, 
        const QList<bool>& yBotBallContacts,
        const QList<bool>& bBallCameraExists,
        const QList<bool>& yBallCameraExists,
        const QList<QVector2D>& bBallCameraPositions,
        const QList<QVector2D>& yBallCameraPositions
    );
    void updateSenderData(QVector3D ball, QList<QVector3D> blue, QList<QVector3D> yellow);
    void updateSimulationSignal();
    // 1 物理フレームぶん同定モデルを進めたあと。QML はこれを受けて各台の適用速度を
    // 読み直す。blueRobotsChanged と違って kick/dribble の指令は触らないので、
    // 毎フレーム鳴らしてもキックを取りこぼしたり二度撃ちしたりしない。
    void actuationAdvanced();
    void robotReplacementRequested(int id, bool isYellow, float sceneX, float sceneZ, float sceneRotYDeg);
    // hasVelocity is false when the Replacement didn't set vx/vy (they are optional
    // in mocSim_BallReplacement); sceneVx/sceneVz are only meaningful when true.
    void ballReplacementRequested(float sceneX, float sceneZ, bool hasVelocity, float sceneVx, float sceneVz);

private:
    QSettings config;

    VisionReceiver *visionReceiver;
    ControlBlueReceiver *controlBlueReceiver;
    ControlYellowReceiver *controlYellowReceiver;

    Sender *sender;

    std::array<Robot*, MaxRobots> blueRobots;
    std::array<Robot*, MaxRobots> yellowRobots;

    int windowWidth;
    int windowHeight;
    int numThreads;

    QString visionMulticastAddress;
    int visionMulticastPort;
    int commandListenPort;
    int blueTeamControlPort;
    int yellowTeamControlPort;

    bool forceDebugDrawMode;
    bool lightBlueRobotMode;
    bool lightYellowRobotMode;
    bool lightStadiumMode;
    bool lightFieldMode;

    int blueRobotCount;
    int yellowRobotCount;

    float ballStaticFriction;
    float ballRestitution;
    float kickerFriction;
    float gravity;
    int desiredFps;
    bool ccdMode;
    bool hideBallMode;
    int onboardCameraWidth = 640;
    int onboardCameraHeight = 480;

    QList<QVector3D> bluePositions;
    QList<QVector3D> yellowPositions;
    QVector3D ballPosition;

    RobotControlResponse robotControlResponse;

    // --- Synthetic wheel-encoder feedback (PiToMw to RAVEN) ---
    // Derives each robot's body twist from the physics-reported pose (the same
    // pose RAVEN sees on vision), maps it to wheel speeds via inverse omni
    // kinematics, injects configurable sensor noise, and emits PiToMw.
    // Also carries the per-robot onboard-camera ball detection and dribbler/
    // photo sensor state so the PiToMw RAVEN receives matches what RACOON-Pi
    // would report (camera coords + sensors), not just wheel speeds.
    void emitEncoderFeedback(const QList<QVector3D> &positions,
                             const QList<bool> &ballCameraExists,
                             const QList<QVector2D> &ballCameraPixels,
                             const QList<bool> &ballContacts,
                             float dtSec);

    FeedbackSender *feedbackSender = nullptr;
    QList<QVector3D> prevEncoderPositions;

    bool encoderEnabled = false;
    bool encoderTeamYellow = false;  // which team RAVEN controls
    double wheelRadiusMm = 26.0;
    double robotRadiusMm = 90.0;
    double wheelAngleRad[4] = {0, 0, 0, 0};  // FL, BL, BR, FR
    double encoderNoiseSigmaMps = 0.0;
    double encoderBiasMps = 0.0;
    double encoderQuantMps = 0.0;
    std::mt19937 encoderRng{12345u};

    // 実機同定モデル。config_v2.ini の [RobotModel] を既定、[RobotModel.<id>] を
    // 各 ID の差分として読む。青黄ともに同じ ID には同じ台を割り当てる。
    bool robotModelEnabled = true;
    void loadRobotModels();

    // --- 追従診断 ([Diag] RobotCsvPath) ---
    // 1 物理フレーム 1 行で「RAVEN の指令 → 同定モデルの出力 → 実際の機体速度」を並べる。
    // 3 つが揃っているかどうかが、RAVEN の速度指令に sim が追従できているかそのもの。
    std::ofstream diagCsv;
    bool diagEnabled = false;
    int diagRobotId = -1;   // -1 = RAVEN が操作するチームの全機体
    double diagTimeSec = 0.0;
    void writeRobotDiag(const QList<QVector3D> &positions, float dtSec);

    float ballSlideDecelMmS2 = 0.0f;
    float ballRollDecelMmS2 = 0.0f;
    float ballSwitchRatio = 2.0f / 3.0f;
    float ballNormalRestitution = 0.8f;
    float ballTangentRetention = 1.0f;
};

#endif // OBSERVER_H
