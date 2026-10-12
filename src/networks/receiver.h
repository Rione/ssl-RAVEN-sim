#ifndef RECEIVER_H
#define RECEIVER_H

#include <QObject>
#include <QThread>
#include <QUdpSocket>
#include <QHostAddress>
#include <QNetworkDatagram>
#include <QVector2D>
#include <QVector3D>
#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include "mocSim_Packet.pb.h"
#include "ssl_simulation_robot_control.pb.h"
#include "ssl_simulation_synchronous.pb.h"
#include "ssl_simulation_robot_feedback.pb.h"
#include "ssl_simulation_control.pb.h"

using namespace std;

class VisionReceiver : public QObject
{
    Q_OBJECT

public:
    explicit VisionReceiver(QObject *parent = nullptr);
    ~VisionReceiver();

    void startListening(quint16 port);
    void stopListening();
    void setPort(quint16 newPort);

signals:
    void receivedPacket(const mocSim_Packet packet);

private slots:
    void receive();

private:
    QUdpSocket *udpSocket;
    int currentPort;

};

class ControlBlueReceiver : public QObject
{
    Q_OBJECT

public:
    explicit ControlBlueReceiver(QObject *parent = nullptr);
    ~ControlBlueReceiver();

signals:
    void receivedPacket(const RobotControl packet, bool isYellow);

public slots:
    void startListening(quint16 port);
    void setPort(quint16 newPort);
    void receive();
    void updateBallContacts(
        const QList<bool> &bBotBallContacts, 
        const QList<bool> &yBotBallContacts,
        const QList<bool> &bBallCameraExists,
        const QList<bool> &yBallCameraExists,
        const QList<QVector2D> &bBallCameraPositions,
        const QList<QVector2D> &yBallCameraPositions
    );
    void stopListening();

private:
    QUdpSocket *udpSocket;
    QList<bool> botBallContacts;

    QList<bool> ballCameraExists;
    QList<QVector2D> ballCameraPositions;

    int currentPort;
};

class ControlYellowReceiver : public QObject
{
    Q_OBJECT

public:
    explicit ControlYellowReceiver(QObject *parent = nullptr);
    ~ControlYellowReceiver();

signals:
    void receivedPacket(const RobotControl packet, bool isYellow);

public slots:
    void startListening(quint16 port);
    void setPort(quint16 newPort);
    void receive();
    void stopListening();
    void updateBallContacts(
        const QList<bool> &bBotBallContacts, 
        const QList<bool> &yBotBallContacts,
        const QList<bool> &bBallCameraExists,
        const QList<bool> &yBallCameraExists,
        const QList<QVector2D> &bBallCameraPositions,
        const QList<QVector2D> &yBallCameraPositions
    );

private:
    QUdpSocket *udpSocket;
    QList<bool> botBallContacts;

    QList<bool> ballCameraExists;
    QList<QVector2D> ballCameraPositions;
    
    int currentPort;
};

// sim を外から操る口 (ssl-simulation-protocol の SimulatorCommand、既定は 10300)。審判の道具や試合のハーネスが
// 球と台を置き直し、台を場から出し入れするのに使う。受けた命令は Observer に渡し、Observer が作った返事
// (SimulatorResponse) を送り主へ返す。返事は誤りが無くても返す (空の SimulatorResponse)。
class SimulationControlReceiver : public QObject
{
    Q_OBJECT

public:
    explicit SimulationControlReceiver(QObject *parent = nullptr);
    ~SimulationControlReceiver();

    void startListening(quint16 port);
    void stopListening();
    void sendResponse(const SimulatorResponse &response, const QHostAddress &address, quint16 port);

signals:
    void receivedCommand(const SimulatorCommand &command, const QHostAddress &sender, quint16 senderPort);

private slots:
    void receive();

private:
    QUdpSocket *udpSocket;
};

#endif // receiver.h