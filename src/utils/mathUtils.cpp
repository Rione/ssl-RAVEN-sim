#include "mathUtils.h"

MathUtils::MathUtils(QObject *parent)
    : QObject(parent), config("../config/config.ini", QSettings::IniFormat) {
}

float MathUtils::normalizeRadian(float radian) {
    while (radian > M_PI) {
        radian -= 2 * M_PI;
    }
    while (radian < -M_PI) {
        radian += 2 * M_PI;
    }
    return radian;
}

float MathUtils::radianToDegree(float radian) {
    return radian * (180.0f / M_PI);
}

float MathUtils::degreeToRadian(float degree) {
    return degree * (M_PI / 180.0f);
}

float MathUtils::vector3dLength(QVector4D vec) {
    return std::sqrt(vec.x() * vec.x() + vec.y() * vec.y() + vec.z() * vec.z());
}

// 1 刻みの姿勢の差分から速度を返す。x/y/z はシーンの軸のまま回さない (y は高さ)。
// w は向きの変化率。呼び出し側は球との相対速度のように世界の軸で使うので、機体の軸の
// 速度が要るときは呼び出し側で向きを使って回す。
QVector4D MathUtils::calcVelocity(QVector4D pose, QVector4D prePose, float deltaTime) {
    return QVector4D(
        (pose.x() - prePose.x()) / deltaTime,
        (pose.y() - prePose.y()) / deltaTime,
        (pose.z() - prePose.z()) / deltaTime,
        normalizeRadian(pose.w() - prePose.w()) / deltaTime
    );
}
