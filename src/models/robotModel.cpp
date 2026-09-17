#include "robotModel.h"

namespace {
// group/key があればその値、無ければ fallback。QSettings::value() は存在しないキーに
// 既定値を返すので、「書かれていないキーは base のまま」をこれ 1 本で表せる。
float pick(QSettings &cfg, const QString &group, const char *key, float fallback) {
    return cfg.value(group + "/" + QLatin1String(key), fallback).toFloat();
}
}  // namespace

RobotMotionModel RobotMotionModel::fromSettings(QSettings &cfg, const QString &group,
                                                const RobotMotionModel &base) {
    RobotMotionModel m = base;
    m.tauVxSec = pick(cfg, group, "TauVxSec", base.tauVxSec);
    m.tauVySec = pick(cfg, group, "TauVySec", base.tauVySec);
    m.tauOmegaSec = pick(cfg, group, "TauOmegaSec", base.tauOmegaSec);
    m.deadTimeSec = pick(cfg, group, "DeadTimeSec", base.deadTimeSec);

    m.gainVx = pick(cfg, group, "GainVx", base.gainVx);
    m.gainVy = pick(cfg, group, "GainVy", base.gainVy);
    m.gainVxFromUy = pick(cfg, group, "GainVxFromUy", base.gainVxFromUy);
    m.gainVyFromUx = pick(cfg, group, "GainVyFromUx", base.gainVyFromUx);
    m.gainOmega = pick(cfg, group, "GainOmega", base.gainOmega);

    m.tractionAccelXMmS2 = pick(cfg, group, "TractionAccelXMmS2", base.tractionAccelXMmS2);
    m.tractionAccelYMmS2 = pick(cfg, group, "TractionAccelYMmS2", base.tractionAccelYMmS2);
    m.tractionDecelXMmS2 = pick(cfg, group, "TractionDecelXMmS2", base.tractionDecelXMmS2);
    m.tractionDecelYMmS2 = pick(cfg, group, "TractionDecelYMmS2", base.tractionDecelYMmS2);

    m.maxAngularVelRadS = pick(cfg, group, "MaxAngularVelRadS", base.maxAngularVelRadS);
    m.maxAngularAccelRadS2 = pick(cfg, group, "MaxAngularAccelRadS2", base.maxAngularAccelRadS2);
    m.wheelRimSpeedBudgetMmS = pick(cfg, group, "WheelRimSpeedBudgetMmS", base.wheelRimSpeedBudgetMmS);
    return m;
}
