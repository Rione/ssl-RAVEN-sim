#include "models/robot.h"
#include <QCoreApplication>
#include <QJSEngine>
#include <QFile>
#include <QTemporaryDir>
#include <cmath>
#include <cstdlib>

void check(bool ok, const char *message) {
    if (!ok) { qCritical("%s", message); std::exit(1); }
}
void command(Robot &r, float x, float y) {
    mocSim_Robot_Command c;
    c.set_veltangent(x / 1000); c.set_velnormal(y / 1000);
    r.visionUpdate(c);
}
float speed(Robot &r) { return std::hypot(r.getVeltangent(), r.getVelnormal()); }
int main(int argc, char **argv) {
    QCoreApplication app(argc, argv);
    RobotMotionModel m;
    m.wheelRimSpeedBudgetMmS = 0;
    m.maxLinearVelMmS = 4000;
    m.maxLinearAccelMmS2 = 3500;
    m.maxLinearDecelMmS2 = 6000;
    const float dt = 1.f / 60;
    for (float angle : {0.f, 0.785398163f, 1.570796327f}) {
        Robot r; r.setMotionModel(m);
        command(r, 6000 * std::cos(angle), 6000 * std::sin(angle));
        float before = 0;
        for (int i = 0; i < 100; ++i) {
            r.advanceActuation(dt);
            check(speed(r) <= 4000.01, "linear speed limit");
            check((speed(r) - before) / dt <= 3500.1, "vector acceleration limit");
            before = speed(r);
        }
        check(std::abs(speed(r) - 4000) < .01, "reach top speed");
        command(r, 0, 0);
        for (int i = 0; i < 50; ++i) {
            r.advanceActuation(dt);
            check((before - speed(r)) / dt <= 6000.1, "vector braking limit");
            before = speed(r);
        }
        check(speed(r) < .01, "stop");
    }
    Robot r; r.setMotionModel(m);
    command(r, 20, 0); r.advanceActuation(dt);
    command(r, -4000, 0); r.advanceActuation(dt);
    const float expected = -3500 * (dt - 20.f / 6000);
    check(std::abs(r.getVeltangent() - expected) < .01, "split reversal brake/accel time");
    r.resetMotion(); command(r, 4000, 0);
    for (int i = 0; i < 100; ++i) r.advanceActuation(dt);
    command(r, 0, 4000);
    for (int i = 0; i < 100; ++i) {
        float x = r.getVeltangent(), y = r.getVelnormal();
        r.advanceActuation(dt);
        check(std::hypot(r.getVeltangent() - x, r.getVelnormal() - y) <= 6000 * dt + .01,
              "turn vector delta limit");
        check(speed(r) <= 4000.01, "turn speed limit");
    }
    m.maxLinearAccelMmS2 = m.maxLinearDecelMmS2 = 0;
    m.tractionAccelXMmS2 = 4269; m.tractionAccelYMmS2 = 3740;
    r.resetMotion(); r.setMotionModel(m); command(r, 2000, 2000); r.advanceActuation(dt);
    check(std::abs(r.getVeltangent() - 4269 * dt) < .01 &&
          std::abs(r.getVelnormal() - 3740 * dt) < .01, "legacy axis limits unchanged");
    QTemporaryDir tmp;
    QSettings cfg(tmp.filePath("model.ini"), QSettings::IniFormat);
    cfg.setValue("RobotModel/MaxLinearAccelMmS2", 3500);
    cfg.setValue("RobotModel/MaxLinearDecelMmS2", 6000);
    auto base = RobotMotionModel::fromSettings(cfg, "RobotModel", RobotMotionModel{});
    auto child = RobotMotionModel::fromSettings(cfg, "RobotModel.2", base);
    check(child.maxLinearAccelMmS2 == 3500 && child.maxLinearDecelMmS2 == 6000, "settings inheritance");
    QJSEngine js;
    QFile f(QStringLiteral(SOURCE_ROOT "/src/qml/sim/BallContact.js"));
    check(f.open(QIODevice::ReadOnly), "read contact helpers");
    check(!js.evaluate(QString::fromUtf8(f.readAll())).isError(), "contact JS parses");
    auto result = js.evaluate(R"(
        var newest = {x:-2,z:0}, older = {x:-2.09,z:0}, outgoing = {x:1,z:0};
        incomingVelocity([newest,older],0,0,1,0,120) === newest &&
        incomingVelocity([outgoing,newest,older],0,0,1,0,120) === newest &&
        incomingVelocity([outgoing],0,0,1,0) === outgoing &&
        incomingVelocity([{x:-.5,z:0},newest,older],0,0,1,0,120) === newest &&
        canRecover(150,0,false) && canRecover(200,-100,false) &&
        !canRecover(100,0,false) && !canRecover(200,100,false) && !canRecover(200,0,true)
    )");
    check(!result.isError() && result.toBool(), "incoming speed and soft-kick recovery");
    qInfo("PR20 motion/contact regression checks passed");
}
