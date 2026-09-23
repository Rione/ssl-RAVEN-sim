import QtQuick

QtObject {
    function kick(color, frame, i, radian, ballVelocity) {
        color.holds[i] = false;

        // 持っていた球 (口に隠して場外に置いてある) は速度 0 で板の前から出る。転がって来た球を止めずに撃つときは、
        // 実物と同じく板の反発が蹴り出しに乗る: 法線は −e·v_n (来る球だけ)、接線は r·v_t。RAVEN の一発の模型
        // (Strike.oneTouchKick / reboundLocal) がこの式で蹴る強さを引いているので、sim が反発を乗せないと
        // 止めずに蹴った球だけが e·v_n ぶん遅く出る (0924 RAVEN passdrill: 1.4 m/s で来た球を 2.5 m/s 相当で
        // 撃ったのに 0.66 m/s で角に届いた)
        let held = ball.position.x > 50000;
        // 反発を乗せるのは本当に転がって来た球だけ: 口の前で体に押されている球の位置の差分は当たり判定の押し出しで
        // 跳ね (1 フレームで数 m/s)、そのまま使うと蹴り出しの向きが暴れる。台との相対速度が 500 mm/s を超え、
        // 蹴りで出せる速さの内 (8 m/s) のときだけ来た球と見る
        let vin = Qt.vector3d(0, 0, 0);
        if (!held) {
            let bx = ballVelocity.x * 1000.0, bz = ballVelocity.z * 1000.0;   // m/s -> mm/s
            let rx = color.velocities[i].x * 1000.0, rz = color.velocities[i].z * 1000.0;
            let rel = Math.hypot(bx - rx, bz - rz);
            if (rel > 500.0 && Math.hypot(bx, bz) < 8000.0) {
                vin = Qt.vector3d(bx, 0, bz);
            }
        }

        frame.collisionShapes[5].position = Qt.vector3d(0, 5000, 0);
        if (held) {
            ball.reset(Qt.vector3d(frame.position.x + (95 * Math.cos(-radian)), 25, (frame.position.z + (95 * Math.sin(-radian)))), Qt.vector3d(0, 0, 0));
        }
        dribbleInfo.id = -1;

        color.kickspeeds[i].x *= observer.kickerFriction;
        color.kickspeeds[i].y *= observer.kickerFriction;
        let nx = Math.cos(radian), nz = -Math.sin(radian);   // 板の法線 = 蹴る向き
        let tx = -nz, tz = nx;                                // 接線
        let vn = vin.x * nx + vin.z * nz;                     // 来る球は負
        let vt = vin.x * tx + vin.z * tz;
        let outN = color.kickspeeds[i].x + (vn < 0 ? -observer.ballNormalRestitution * vn : 0);
        let outT = observer.ballTangentRetention * vt;
        // Defer the launch: store the velocity and let updateGameObjects() apply it only
        // after the ball.reset() above has actually moved the ball back to the mouth
        // (next physics step). Applying it now would launch the ball from the off-field
        // park position (x~100000) and send it flying into the void.
        pendingKickVelocity = Qt.vector3d(
            outN * nx + outT * tx,
            color.kickspeeds[i].y,
            outN * nz + outT * tz
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