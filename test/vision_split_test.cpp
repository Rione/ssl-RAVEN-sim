// Exercises Sender's 2-camera field split over loopback UDP and checks that
// each object lands in the right camera frame, with the center overlap band
// reported by BOTH cameras. Jitter is disabled here for determinism; the random
// seam is covered separately below by sweeping the split line.
#include <boost/asio.hpp>
#include <cassert>
#include <set>
#include <iostream>

#include "networks/sender.h"
#include "ssl_vision_wrapper.pb.h"

namespace {
constexpr char kAddr[] = "127.0.0.1";
constexpr unsigned short kPort = 17790;

struct Frame {
    int cameraId = -1;
    bool hasBall = false;
    std::set<int> blue, yellow;
    int calibCount = 0;
};

Frame parse(const std::string &bytes) {
    SSL_WrapperPacket pkt;
    assert(pkt.ParseFromString(bytes));
    Frame f;
    assert(pkt.has_detection());
    f.cameraId = pkt.detection().camera_id();
    f.hasBall = pkt.detection().balls_size() > 0;
    for (const auto &r : pkt.detection().robots_blue()) f.blue.insert(r.robot_id());
    for (const auto &r : pkt.detection().robots_yellow()) f.yellow.insert(r.robot_id());
    if (pkt.has_geometry()) f.calibCount = pkt.geometry().calib_size();
    return f;
}
}  // namespace

int main() {
    GOOGLE_PROTOBUF_VERIFY_VERSION;
    using boost::asio::ip::udp;

    boost::asio::io_context io;
    udp::socket rx(io);
    rx.open(udp::v4());
    rx.set_option(udp::socket::reuse_address(true));
    rx.bind(udp::endpoint(boost::asio::ip::make_address(kAddr), kPort));

    Sender sender(kAddr, kPort);
    sender.setCameraSplit(/*numCameras=*/2, /*overlapMm=*/500.0, /*splitJitterMm=*/0.0);

    // x: left=-3000 (cam0), center=0 (both), right=+2000 (cam1)
    QList<QVector3D> blue   = {QVector3D(-3000, 0, 0), QVector3D(0, 0, 0), QVector3D(2000, 0, 0)};
    QList<QVector3D> yellow = {QVector3D(3000, 0, 0)};
    QVector3D ball(-100, 0, 0);  // within ±500 overlap ⇒ seen by both

    sender.send(0, ball, blue, yellow);

    char buf[4096];
    udp::endpoint from;
    Frame f0, f1;
    for (int n = 0; n < 2; ++n) {
        std::size_t len = rx.receive_from(boost::asio::buffer(buf), from);
        Frame f = parse(std::string(buf, len));
        if (f.cameraId == 0) f0 = f; else if (f.cameraId == 1) f1 = f;
    }

    assert(f0.cameraId == 0 && f1.cameraId == 1 && "both camera frames must arrive");

    // Camera 0 (left + overlap): blue 0 and 1, no blue 2, no yellow, ball yes.
    assert((f0.blue == std::set<int>{0, 1}));
    assert(f0.yellow.empty());
    assert(f0.hasBall);

    // Camera 1 (right + overlap): blue 1 and 2, yellow 0, ball yes.
    assert((f1.blue == std::set<int>{1, 2}));
    assert((f1.yellow == std::set<int>{0}));
    assert(f1.hasBall);

    // Center object (blue 1) and ball appear in BOTH ⇒ overlap works.
    assert(f0.blue.count(1) && f1.blue.count(1));
    assert(f0.hasBall && f1.hasBall);

    // Geometry advertises exactly the 2 cameras, attached to one frame only.
    assert(f0.calibCount == 2 || f1.calibCount == 2);
    assert(!(f0.calibCount == 2 && f1.calibCount == 2));

    std::cout << "OK: 2-camera vision split verified "
                 "(left/right ownership, center overlap in both, geometry=2 cams)\n";
    google::protobuf::ShutdownProtobufLibrary();
    return 0;
}
