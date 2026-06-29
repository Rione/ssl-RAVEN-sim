#ifndef SENDER_H
#define SENDER_H

#include <QObject>
#include <iostream>
#include <thread>
#include <chrono>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <QVector3D>
#include <QList>
#include <QDebug>
#include <QTimer>
#include <random>
#include "ssl_vision_wrapper.pb.h"
#include "ssl_simulation_robot_feedback.pb.h"

using namespace std;

class Sender : public QObject{
    Q_OBJECT
public:
    explicit Sender(const string address, quint16 port, QObject *parent = nullptr);
    ~Sender();
    void send(int camera_id, QVector3D ball_position, QList<QVector3D> blue_positions, QList<QVector3D> yellow_positions);
    void setPort(string address, quint16 newPort);
    // Multi-camera coverage split (default: 2 cameras along the halfway line).
    //  numCameras    1 ⇒ one camera sees the whole field (legacy behaviour).
    //                2 ⇒ camera 0 covers x≲split, camera 1 covers x≳split.
    //  overlapMm     half-width of the band around the split line that BOTH
    //                cameras report, reproducing real overlapping FOVs.
    //  splitJitterMm per-frame random shift of the split line (0 ⇒ static).
    void setCameraSplit(int numCameras, double overlapMm, double splitJitterMm);
    void setDetectionInfo(SSL_DetectionFrame &detection, int camera_id, int numCameras,
                          double splitX, double overlapMm, QVector3D ball_position,
                          QList<QVector3D> blue_positions, QList<QVector3D> yellow_positions);
    SSL_GeometryData setGeometryInfo();

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

    // Camera coverage split configuration.
    int numCameras_ = 2;
    double overlapMm_ = 500.0;
    double splitJitterMm_ = 200.0;
    std::mt19937 splitRng_{20260630u};
};

#endif // SENDER_H
