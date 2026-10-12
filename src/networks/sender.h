#ifndef SENDER_H
#define SENDER_H

#include <QObject>
#include <iostream>
#include <thread>
#include <chrono>
#include <boost/asio.hpp>
#include <QVector3D>
#include <QList>
#include <QDebug>
#include <QTimer>
#include <cmath>
#include "ssl_vision_wrapper.pb.h"
#include "ssl_simulation_robot_feedback.pb.h"

using namespace std;

class Sender : public QObject{
    Q_OBJECT
public:
    explicit Sender(const string address, quint16 port, QObject *parent = nullptr);
    ~Sender();
    void send(int camera_id, QVector3D ball_position, QList<QVector3D> blue_positions, QList<QVector3D> yellow_positions, double timestepSec);
    void setPort(string address, quint16 newPort);
    void setDetectionInfo(SSL_DetectionFrame &detection, int camera_num, QVector3D ball_position, QList<QVector3D> blue_positions, QList<QVector3D> yellow_positions);
    SSL_GeometryData setGeometryInfo();
    // 場の座標 [mm] がこの世界の中か。外 (持った球の置き場・場から出した台の置き場) の物は vision に出さない。
    static bool inWorld(float xMm, float yMm) { return std::abs(xMm) < 20000.0f && std::abs(yMm) < 20000.0f; }

signals:
    void packetSent();

private:
    boost::asio::io_context ioContext_;
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint endpoint_;

    int captureCount;
    int geometryCount;
    double t_capture;
    double start_time;
    double loop_time;

    quint16 port;
    string address;
};

#endif // SENDER_H
