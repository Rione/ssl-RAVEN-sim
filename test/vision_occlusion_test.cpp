// Verifies ball occlusion: a robot blocking a camera's line of sight to the
// ball removes the ball from that camera's detection. With 2 overlapping
// cameras the ball survives near center (the other camera still sees it) but
// vanishes near the field edge where only one camera covers it.
#include <boost/asio.hpp>
#include <cassert>
#include <iostream>
#include <string>

#include "networks/sender.h"
#include "ssl_vision_wrapper.pb.h"

namespace {
constexpr char kAddr[] = "127.0.0.1";

struct Result { bool cam0Ball = false, cam1Ball = false; };

// Run one frame and report whether each camera reported the ball.
Result run(unsigned short port, QVector3D ball,
           QList<QVector3D> blue, QList<QVector3D> yellow, bool occlusion) {
    using boost::asio::ip::udp;
    boost::asio::io_context io;
    udp::socket rx(io);
    rx.open(udp::v4());
    rx.set_option(udp::socket::reuse_address(true));
    rx.bind(udp::endpoint(boost::asio::ip::make_address(kAddr), port));

    Sender sender(kAddr, port);
    sender.setCameraSplit(2, /*overlap=*/500.0, /*jitter=*/0.0);
    sender.setOcclusion(occlusion, /*cameraHeight=*/4000.0, /*robotHeight=*/150.0);
    sender.send(0, ball, blue, yellow);

    Result res;
    char buf[4096];
    udp::endpoint from;
    for (int n = 0; n < 2; ++n) {
        std::size_t len = rx.receive_from(boost::asio::buffer(buf), from);
        SSL_WrapperPacket pkt;
        assert(pkt.ParseFromArray(buf, static_cast<int>(len)));
        bool hasBall = pkt.detection().balls_size() > 0;
        if (pkt.detection().camera_id() == 0) res.cam0Ball = hasBall;
        else res.cam1Ball = hasBall;
    }
    return res;
}
}  // namespace

int main() {
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    const QVector3D edgeBall(5500, 21, 0);    // right edge: only camera 1 covers it
    const QVector3D centerBall(0, 21, 0);     // seam: both cameras cover it
    const QList<QVector3D> none;

    // (1) Edge ball, no robots ⇒ camera 1 sees it, camera 0 does not cover it.
    {
        Result r = run(17801, edgeBall, none, none, true);
        assert(r.cam1Ball && !r.cam0Ball);
    }

    // (2) Edge ball with a robot between camera 1 and the ball ⇒ camera 1 loses
    //     it; no other camera covers the edge ⇒ ball disappears entirely.
    {
        QList<QVector3D> blue = {QVector3D(5460, 0, 0)};  // on the cam1→ball line
        Result r = run(17802, edgeBall, blue, none, true);
        assert(!r.cam1Ball && !r.cam0Ball && "edge ball must vanish when occluded");
    }

    // (3) Same geometry but occlusion disabled ⇒ ball is back (proves it was the
    //     occlusion, not the split, that removed it).
    {
        QList<QVector3D> blue = {QVector3D(5460, 0, 0)};
        Result r = run(17803, edgeBall, blue, none, false);
        assert(r.cam1Ball && "ball must return with occlusion off");
    }

    // (4) Robot beside the line of sight (not between) ⇒ no occlusion.
    {
        QList<QVector3D> blue = {QVector3D(5460, 400, 0)};  // 400mm off the Y=0 ray
        Result r = run(17804, edgeBall, blue, none, true);
        assert(r.cam1Ball && "robot beside the sightline must not occlude");
    }

    // (5) Center ball in the overlap, robot blocks ONLY camera 1 ⇒ camera 0 still
    //     sees it ⇒ ball survives (the edge-vs-center difference).
    {
        QList<QVector3D> blue = {QVector3D(50, 0, 0)};  // blocks cam1 (+x side) only
        Result r = run(17805, centerBall, blue, none, true);
        assert(!r.cam1Ball && r.cam0Ball && "center ball must survive via the other camera");
    }

    std::cout << "OK: ball occlusion verified "
                 "(edge ball vanishes behind a robot, center ball survives via overlap)\n";
    google::protobuf::ShutdownProtobufLibrary();
    return 0;
}
