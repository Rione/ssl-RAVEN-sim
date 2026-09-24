import QtQuick

QtObject {
    function kick(color, frame, i, radian, ballVelocity) {
        color.holds[i] = false;
        
        frame.collisionShapes[5].position = Qt.vector3d(0, 5000, 0);
        if (ball.position.x > 50000) {
            ball.reset(Qt.vector3d(frame.position.x + (95 * Math.cos(-radian)), 25, (frame.position.z + (95 * Math.sin(-radian)))), Qt.vector3d(0, 0, 0));
        }
        dribbleInfo.id = -1;

        color.kickspeeds[i].x *= observer.kickerFriction;
        color.kickspeeds[i].y *= observer.kickerFriction;
        // Defer the launch: store the velocity and let updateGameObjects() apply it only
        // after the ball.reset() above has actually moved the ball back to the mouth
        // (next physics step). Applying it now would launch the ball from the off-field
        // park position (x~100000) and send it flying into the void.
        pendingKickVelocity = Qt.vector3d(
            color.kickspeeds[i].x * Math.cos(radian),
            color.kickspeeds[i].y,
            -color.kickspeeds[i].x * Math.sin(radian)
        );
    }

    // The dribbler was switched off while this robot held the ball: put the real ball back in front of the mouth
    // (with the robot's own velocity, so it is left behind rather than teleported) and retract the mouth collider.
    // Without this the only way to let go of the ball was to kick it: the holder's mouth test is forced to pass
    // (botDistanceBall = 95) so the release branch could never run, and a robot that stopped its dribbler kept
    // carrying the ball. Ball placement could never finish (2026-09-17: released at the target, then dragged 850 mm).
    function release(frame, isYellow, i, color) {
        if (dribbleInfo.id != i || dribbleInfo.isYellow != isYellow) {
            return;
        }
        dribbleInfo.id = -1;
        let w = color.poses[i].w;
        if (ball.position.x > 50000) {
            ball.reset(Qt.vector3d(frame.position.x + (95 * Math.cos(-w)), 25, (frame.position.z + (95 * Math.sin(-w)))),
                       Qt.vector3d(color.velocities[i].x, 0, color.velocities[i].z));
        }
        frame.collisionShapes[5].position = Qt.vector3d(0, 5000, 0);
        // holds[i] (RAVEN の ballCatched = 口のセンサ) はここでは触らない。実物の光センサはドリブラを回して
        // いなくても口の球を見る。幾何の判定 (GameObjects の口の窓) が次のフレームから本当の球の位置で決める。
        // 触ると RAVEN が「持っていない」と思い込み、拾う指示のまま球を抱えて動かなくなる (0917 sim の対戦: 蹴り 0 本)
    }

    function dribble(frame, isYellow, i, botRadianBall, botDistanceBall, color) {
        dribbleInfo.id = i;
        dribbleInfo.isYellow = isYellow;
        dribbleInfo.radianBall = botRadianBall;
        dribbleInfo.distanceBall = 95;
        // ball.simulationEnabled = false;
        ball.reset(Qt.vector3d(100000, 0, 100000), Qt.vector3d(0, 0, 0));
        
        // ball.reset(Qt.vector3d(frame.position.x + (95 * Math.cos(-color.poses[i].w)), 25, (frame.position.z + (95 * Math.sin(-color.poses[i].w)))), Qt.vector3d(0, 0, 0));
        frame.collisionShapes[5].position = Qt.vector3d(0, 25, -95*Math.cos(botRadianBall));
    }
}