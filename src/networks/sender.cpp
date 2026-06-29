#include "sender.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

Sender::Sender(const string address, quint16 port, QObject *parent) :
    ioContext_(),
    socket_(ioContext_),
    endpoint_(boost::asio::ip::make_address(address), port),
    address(address),
    port(port),
    captureCount(0),
    geometryCount(0),
    t_capture(0),
    start_time(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()),
    loop_time(0)
{
    socket_.open(boost::asio::ip::udp::v4());

    captureCount = 0;
    geometryCount = 0;

    loop_time = 0;
    t_capture = 0;
}

Sender::~Sender() {
    socket_.close();
}

void Sender::setPort(string address, quint16 newPort) {
    port = newPort;
    endpoint_ = boost::asio::ip::udp::endpoint(boost::asio::ip::make_address(address), port);
}

void Sender::setCameraSplit(int numCameras, double overlapMm, double splitJitterMm) {
    numCameras_ = (numCameras == 1) ? 1 : 2;  // only 1 or 2 supported
    overlapMm_ = overlapMm < 0.0 ? 0.0 : overlapMm;
    splitJitterMm_ = splitJitterMm < 0.0 ? 0.0 : splitJitterMm;
}

void Sender::setOcclusion(bool enabled, double cameraHeightMm, double robotHeightMm) {
    occlusionEnabled_ = enabled;
    if (cameraHeightMm > 0.0) cameraHeightMm_ = cameraHeightMm;
    if (robotHeightMm > 0.0) robotHeightMm_ = robotHeightMm;
}

bool Sender::isBallOccluded(int camera_id, const QVector3D &ball_position,
                            const QList<QVector3D> &blue_positions,
                            const QList<QVector3D> &yellow_positions) const {
    if (!occlusionEnabled_) return false;

    // Camera mount in field coords (ceiling above the center of its half).
    const double Cx = (numCameras_ == 2) ? (camera_id == 0 ? -kHalfCenterX : kHalfCenterX) : 0.0;
    const double Cy = 0.0;
    const double Cz = cameraHeightMm_;

    // Field coords: X = world x, Y = -world z, Z(height) = world y.
    const double Bx = ball_position.x();
    const double By = -ball_position.z();
    double Bz = ball_position.y();
    if (Bz < 0.0) Bz = 0.0;
    if (Cz <= Bz) return false;  // camera must be above the ball

    // Horizontal direction camera→ball.
    const double vx = Bx - Cx;
    const double vy = By - Cy;
    const double a = vx * vx + vy * vy;
    if (a < 1e-6) return false;  // ball straight under camera ⇒ nothing beside it blocks

    // The ray drops below the robot top (height robotHeightMm_) only for t ≥ t0;
    // a robot can only intersect the line of sight within its own height band.
    double t0 = (Cz - robotHeightMm_) / (Cz - Bz);
    if (t0 < 0.0) t0 = 0.0;
    if (t0 >= 1.0) return false;

    const double rr = robotRadiusMm_;
    auto blockedBy = [&](const QList<QVector3D> &team) {
        for (const auto &r : team) {
            const double Rx = r.x();
            const double Ry = r.y();
            // Ray horizontal P(t) = C + t·v. Solve |P(t) − R| = rr (cylinder).
            const double fx = Cx - Rx;
            const double fy = Cy - Ry;
            const double b = 2.0 * (fx * vx + fy * vy);
            const double c = fx * fx + fy * fy - rr * rr;
            const double disc = b * b - 4.0 * a * c;
            if (disc <= 0.0) continue;  // ray misses the robot footprint
            const double sq = std::sqrt(disc);
            const double tEnter = (-b - sq) / (2.0 * a);
            const double tExit  = (-b + sq) / (2.0 * a);
            // Inside cylinder for t∈[tEnter,tExit]; below robot top for t≥t0;
            // between camera and ball for t∈(0,1). The 0.98 cap excludes a robot
            // sitting essentially at the ball (e.g. its own dribbler).
            const double lo = std::max(tEnter, t0);
            const double hi = std::min(tExit, 1.0);
            if (lo < hi && lo < 0.98) return true;
        }
        return false;
    };
    return blockedBy(blue_positions) || blockedBy(yellow_positions);
}

void Sender::send(int camera_num, QVector3D ball_position, QList<QVector3D> blue_positions, QList<QVector3D> yellow_positions) {
    t_capture = (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - start_time)/1000.0;
    // t_capture += 1/60.0;

    // Randomly shift the split line each frame so the overlap band wanders, the
    // way real adjacent cameras hand objects off across an imperfect seam.
    double splitX = 0.0;
    if (numCameras_ == 2 && splitJitterMm_ > 0.0) {
        std::uniform_real_distribution<double> jitter(-splitJitterMm_, splitJitterMm_);
        splitX = jitter(splitRng_);
    }

    for (int i = 0; i < numCameras_; i++) {
        SSL_WrapperPacket packet;

        SSL_DetectionFrame detection;
        detection.set_frame_number(captureCount);
        detection.set_t_capture(t_capture);
        detection.set_t_sent(t_capture);
        detection.set_camera_id(i);
        setDetectionInfo(detection, i, numCameras_, splitX, overlapMm_,
                         ball_position, blue_positions, yellow_positions);

        packet.mutable_detection()->CopyFrom(detection);

        // Attach geometry on one camera only to avoid duplicate field data.
        if (i == 0 && geometryCount % 1000 == 0) {
            packet.mutable_geometry()->CopyFrom(setGeometryInfo());
        }

        std::string serializedData;
        if (!packet.SerializeToString(&serializedData)) {
            std::cerr << "Failed to serialize command." << std::endl;
            return;
        }
        socket_.send_to(boost::asio::buffer(serializedData), endpoint_);
    }
    geometryCount++;
    captureCount++;
}

void Sender::setDetectionInfo(SSL_DetectionFrame &detection, int camera_id, int numCameras,
                              double splitX, double overlapMm, QVector3D ball_position,
                              QList<QVector3D> blue_positions, QList<QVector3D> yellow_positions) {
    // Whether an object at world-x belongs to this camera's coverage. Camera 0
    // owns the left half (x ≤ split), camera 1 the right half (x ≥ split); the
    // ±overlap band around the split line is reported by BOTH cameras.
    auto inCamera = [&](double x) -> bool {
        if (numCameras <= 1) return true;
        if (camera_id == 0) return x <= splitX + overlapMm;
        return x >= splitX - overlapMm;
    };

    // Do not emit a ball detection with off-field/garbage coordinates
    // (e.g. the dribble "park" sentinel at ~100000). Mirrors ssl-Raven's CamFilter guard.
    // Also drop it when a robot occludes this camera's line of sight to the ball,
    // so a ball hidden behind a robot disappears from that camera (and entirely
    // when no other camera can see it — typically near the field edges).
    if (std::abs(ball_position.x()) < 20000.0f && std::abs(ball_position.z()) < 20000.0f
            && inCamera(ball_position.x())
            && !isBallOccluded(camera_id, ball_position, blue_positions, yellow_positions)) {
        SSL_DetectionBall* ball = detection.add_balls();
        ball->set_confidence(1.0);
        ball->set_x(ball_position.x());
        ball->set_y(-ball_position.z());
        ball->set_z(ball_position.y());
        ball->set_pixel_x(0);
        ball->set_pixel_y(0);
    }

    for (int i = 0; i < blue_positions.size(); ++i) {
        if (!inCamera(blue_positions[i].x())) continue;
        SSL_DetectionRobot* robot = detection.add_robots_blue();
        robot->set_robot_id(i);
        robot->set_confidence(1.0);
        robot->set_x(blue_positions[i].x());
        robot->set_y(blue_positions[i].y());
        float tempOrientation = blue_positions[i].z();
        tempOrientation = fmod(tempOrientation + 180, 360) - 180; // Normalize to [-180, 180]
        robot->set_orientation(tempOrientation*M_PI/180);
        robot->set_pixel_x(0);
        robot->set_pixel_y(0);
        robot->set_height(0);
    }

    for (int i = 0; i < yellow_positions.size(); ++i) {
        if (!inCamera(yellow_positions[i].x())) continue;
        SSL_DetectionRobot* robot = detection.add_robots_yellow();
        robot->set_robot_id(i);
        robot->set_confidence(1.0);
        robot->set_x(yellow_positions[i].x());
        robot->set_y(yellow_positions[i].y());
        robot->set_orientation(yellow_positions[i].z()*M_PI/180);
        robot->set_pixel_x(0);
        robot->set_pixel_y(0);
        robot->set_height(0);
    }
}

SSL_GeometryData Sender::setGeometryInfo() {
    SSL_GeometryData geometry;

    SSL_GeometryFieldSize* field = geometry.mutable_field();
    field->set_field_length(12000);
    field->set_field_width(9000);
    field->set_goal_width(1800);
    field->set_goal_depth(180);
    field->set_boundary_width(300);
    SSL_FieldLineSegment* topTouchLine = field->add_field_lines();
    topTouchLine->set_name("TopTouchLine");
    Vector2f point;
    point.set_x(-6000);
    point.set_y(4500);
    topTouchLine->mutable_p1()->CopyFrom(point);
    point.set_x(6000);
    point.set_y(4500);
    topTouchLine->mutable_p2()->CopyFrom(point);
    topTouchLine->set_thickness(10);
    SSL_FieldLineSegment* bottomTouchLine = field->add_field_lines();
    bottomTouchLine->set_name("BottomTouchLine");
    point.set_x(-6000);
    point.set_y(-4500);
    bottomTouchLine->mutable_p1()->CopyFrom(point);
    point.set_x(6000);
    point.set_y(-4500);
    bottomTouchLine->mutable_p2()->CopyFrom(point);
    bottomTouchLine->set_thickness(10);
    SSL_FieldLineSegment* leftGoalLine = field->add_field_lines();
    leftGoalLine->set_name("LeftGoalLine");
    point.set_x(-6000);
    point.set_y(-4500);
    leftGoalLine->mutable_p1()->CopyFrom(point);
    point.set_x(-6000);
    point.set_y(4500);
    leftGoalLine->mutable_p2()->CopyFrom(point);
    leftGoalLine->set_thickness(10);
    SSL_FieldLineSegment* rightGoalLine = field->add_field_lines();
    rightGoalLine->set_name("RightGoalLine");
    point.set_x(6000);
    point.set_y(-4500);
    rightGoalLine->mutable_p1()->CopyFrom(point);
    point.set_x(6000);
    point.set_y(4500);
    rightGoalLine->mutable_p2()->CopyFrom(point);
    rightGoalLine->set_thickness(10);
    SSL_FieldLineSegment* halfWayLine = field->add_field_lines();
    halfWayLine->set_name("HalfWayLine");
    point.set_x(0);
    point.set_y(-4500);
    halfWayLine->mutable_p1()->CopyFrom(point);
    point.set_x(0);
    point.set_y(4500);
    halfWayLine->mutable_p2()->CopyFrom(point);
    halfWayLine->set_thickness(10);
    SSL_FieldCircularArc* centerCircle = field->add_field_arcs();
    centerCircle->set_name("CenterCircle");
    Vector2f* center = centerCircle->mutable_center();
    center->set_x(0);
    center->set_y(0);
    centerCircle->set_radius(500);
    centerCircle->set_a1(0);
    centerCircle->set_a2(6.28319);
    centerCircle->set_thickness(10);
    
    SSL_FieldLineSegment* leftPenaltyStretch = field->add_field_lines();
    leftPenaltyStretch->set_name("LeftPenaltyStretch");
    point.set_x(-4200);
    point.set_y(-1800);
    leftPenaltyStretch->mutable_p1()->CopyFrom(point);
    point.set_x(-4200);
    point.set_y(1800);
    leftPenaltyStretch->mutable_p2()->CopyFrom(point);
    leftPenaltyStretch->set_thickness(10);
    SSL_FieldLineSegment* rightPenaltyStretch = field->add_field_lines();
    rightPenaltyStretch->set_name("RightPenaltyStretch");
    point.set_x(4200);
    point.set_y(-1800);
    rightPenaltyStretch->mutable_p1()->CopyFrom(point);
    point.set_x(4200);
    point.set_y(1800);
    rightPenaltyStretch->mutable_p2()->CopyFrom(point);
    rightPenaltyStretch->set_thickness(10);

    SSL_FieldLineSegment* leftFieldLeftPenaltyStretch = field->add_field_lines();
    leftFieldLeftPenaltyStretch->set_name("LeftFieldLeftPenaltyStretch");
    point.set_x(-6000);
    point.set_y(-1800);
    leftFieldLeftPenaltyStretch->mutable_p1()->CopyFrom(point);
    point.set_x(-4200);
    point.set_y(-1800);
    leftFieldLeftPenaltyStretch->mutable_p2()->CopyFrom(point);
    leftFieldLeftPenaltyStretch->set_thickness(10);
    SSL_FieldLineSegment* leftFieldRightPenaltyStretch = field->add_field_lines();
    leftFieldRightPenaltyStretch->set_name("LeftFieldRightPenaltyStretch");
    point.set_x(-6000);
    point.set_y(1800);
    leftFieldRightPenaltyStretch->mutable_p1()->CopyFrom(point);
    point.set_x(-4200);
    point.set_y(1800);
    leftFieldRightPenaltyStretch->mutable_p2()->CopyFrom(point);
    leftFieldRightPenaltyStretch->set_thickness(10);

    SSL_FieldLineSegment* rightFieldRightPenaltyStretch = field->add_field_lines();
    rightFieldRightPenaltyStretch->set_name("RightFieldRightPenaltyStretch");
    point.set_x(6000);
    point.set_y(-1800);
    rightFieldRightPenaltyStretch->mutable_p1()->CopyFrom(point);
    point.set_x(4200);
    point.set_y(-1800);
    rightFieldRightPenaltyStretch->mutable_p2()->CopyFrom(point);
    rightFieldRightPenaltyStretch->set_thickness(10);
    SSL_FieldLineSegment* rightFieldLeftPenaltyStretch = field->add_field_lines();
    rightFieldLeftPenaltyStretch->set_name("RightFieldLeftPenaltyStretch");
    point.set_x(6000);
    point.set_y(1800);
    rightFieldLeftPenaltyStretch->mutable_p1()->CopyFrom(point);
    point.set_x(4200);
    point.set_y(1800);
    rightFieldLeftPenaltyStretch->mutable_p2()->CopyFrom(point);
    rightFieldLeftPenaltyStretch->set_thickness(10);

    for (int i = 0; i < numCameras_; i++) {
        SSL_GeometryCameraCalibration* camera = geometry.add_calib();
        camera->set_camera_id(i);
        camera->set_focal_length(500.0);
        camera->set_principal_point_x(390.0);
        camera->set_principal_point_y(290.0);
        camera->set_distortion(0.0);
        camera->set_q0(0.7);
        camera->set_q1(-0.7);
        camera->set_q2(0.0);
        camera->set_q3(0.0);
        // Place each camera over the center of the half it covers (left/right),
        // so the calibration matches the detection split (single cam ⇒ center).
        double camTx = 0.0;
        if (numCameras_ == 2) camTx = (i == 0) ? -kHalfCenterX : kHalfCenterX;
        camera->set_tx(camTx);
        camera->set_ty(1250);
        camera->set_tz(cameraHeightMm_);
        camera->set_derived_camera_world_tx(camTx);
        camera->set_derived_camera_world_ty(0);
        camera->set_derived_camera_world_tz(0);
        camera->set_pixel_image_width(780);
        camera->set_pixel_image_height(580);
    }


    return geometry;
}
