import QtQuick
import QtQuick3D
import QtQuick3D.Physics
import Qt.labs.folderlistmodel
import M2

import "../../../assets/models/bot/Rione/viz" as BlueBody
import "../../../assets/models/bot/Rione/rigid_body" as BlueLightBody
import "../../../assets/models/bot/Rione/viz" as YellowBody
import "../../../assets/models/bot/Rione/rigid_body" as YellowLightBody
import "../../../assets/models/ball/"
import "../../../assets/models/circle/ballMarker/"

Node {
    id: robotNode
    Sync {
        id: sync
    }
    Control {
        id: control
    }
    property real colorHeight: 0.3

    property real radianOffset: -Math.atan(350.0/547.72)
    property var selectedRobotColor: "blue"
    property real botCursorID: 0

    // --- Grab & carry state (mouse drag pick-up of a robot) ---
    property bool isGrabbingBot: false
    property string grabbedColor: "blue"
    property int grabbedId: -1
    property real grabOffsetX: 0
    property real grabOffsetZ: 0
    property real grabLiftHeight: 80

    // Per-robot kicker recharge, in physics frames (1 s at 60 Hz). A kick used to raise ONE global flag that
    // blocked every robot's kick AND dribbling for 1 s: with an opponent that kicks often, the other team could
    // hardly ever kick or hold the ball. The kicker capacitor is per robot, and the dribbler is independent of it.
    property int kickRechargeFrames: 60
    property var kickCooldown: ({})
    // A dribbler cannot catch a ball that passes it faster than this (mm/s, relative to the robot). Without
    // this limit a robot that kicks with its dribbler running re-catches the ball in the launch frame and the
    // kick is swallowed (the ball is 95 mm in front of it and still inside the hold cone).
    property real dribbleCatchMaxSpeedMmS: 1500
    // Who may act on the ball this frame ({isYellow, id} or null). The ball can sit in several mouth cones at once
    // (a face-to-face contest): a robot asking to kick wins (a kick is instantaneous), otherwise the nearest mouth.
    // If the winner is not the current holder, the hold is released so the winner can dribble or kick the ball.
    // Before, whoever registered the dribble first kept the ball for good and the other robot's kicks were
    // silently refused although its sensor reported "holding".
    property var ballContestWinner: null
    // A challenger that only dribbles must be this much closer to the ball than the current holder to take it
    // (mm). Without it two facing dribblers swap the ball every frame.
    property real contestTakeoverMarginMm: 30
    property var pendingKickVelocity: null
    property var preBallPosition: Qt.vector4d(0, 0, 0, 0)
    property var ballAngularVelocity: Qt.vector4d(0, 0, 0, 0)
    property var preBallAngularPosition: Qt.vector4d(0, 0, 0, 0)
    property var ballVelocity: Qt.vector4d(0, 0, 0, 0)
    property var ballModelNum: 1
    property var ballReset: false
    // 配置の直後、減速を止めておくフレーム数。ballVelocity は位置の差分なので、
    // 瞬間移動をまたいだ 1 フレームは出鱈目な速さになる。それを摩擦に食わせないための
    // 逃げで、必要なのは差分が綺麗になるまでの 2 フレームだけ。
    // 以前は 30 (= 0.5 s) で、速度つきの配置 (RAVEN のフリーキック・ボール配置) のあいだ
    // 球が完全に無摩擦で転がっていた。0918 実測: 3000 mm/s で置くと 0.45 s / 1350 mm を
    // 一切減速せずに直進し、RAVEN の到達予測がそのぶん丸ごと外れていた。
    property int skipRollingFrictionFrames: 0
    readonly property int placementSettleFrames: 2
    // Ball physical constants (scene length units are mm). 42 mm diameter golf ball.
    // Mass is kilograms — same unit as robot DynamicRigidBody.mass (2.5 kg).
    // Was wrongly 46.0 (=46 kg, ~1000× SSL ball) which made the ball heavier than robots.
    property real ballRadius: 21.0
    property real ballMass: 0.046
    // Angular velocity of the ball we integrate ourselves (rad/s). PhysX does not expose
    // a readable angular velocity, and the field has no contact friction, so the slip/roll
    // friction model owns the ball's spin. See applyBallFriction().
    property var ballSpin: Qt.vector3d(0, 0, 0)
    // Tracked velocity of the previous frame (m/s, scene axes), for impact detection in applyBallFriction().
    property var prevBallVelocity: Qt.vector3d(0, 0, 0)
    // 直近の「蹴り出しの速さ」v0 [mm/s] と、もう転がりに入ったか。RAVEN のボールモデルは
    // 滑走 → 転がりの切り替えを v0 の割合 (k_switch) で決めるので、v0 を覚えておく必要がある。
    // キック・配置・衝突・外から押されたとき、のいずれでも数え直す。
    property real ballLaunchSpeed: 0.0
    property bool ballRolling: true
    property var ballPositions: new Array(ballModelNum).fill(Qt.vector4d(0, 0, 0, 0))
    MotionControl {
        id: motionControl
    }
    MathUtils {
        id: mu
    }
    Connections {
        target: observer
        function onBlueRobotsChanged() {
            for (var i = 0; i < blue.num; i++) {
                blue.velNormals[i] = observer.blue_robots[i].velnormal;
                blue.velTangents[i] = observer.blue_robots[i].veltangent;
                
                blue.velAngulars[i] = observer.blue_robots[i].velangular;
                blue.spinners[i] = observer.blue_robots[i].spinner;
            }
        }
        // Robot::advanceActuation が 1 tick 進むたび。むだ時間と一次遅れの立ち上がりを
        // 取りこぼさないよう、実際に台へ効いている値 (速度と、放電する蹴り) を毎フレーム
        // 読み直す。蹴りをここで読むのは、それが速度と同じむだ時間の線を通って出てくるから:
        // 指令パケットが届いた瞬間に読むと、まだ線の中に居る蹴りを先に撃ってしまう。
        function onActuationAdvanced() {
            for (var i = 0; i < blue.num; i++) {
                blue.velNormals[i] = observer.blue_robots[i].velnormal;
                blue.velTangents[i] = observer.blue_robots[i].veltangent;
                blue.velAngulars[i] = observer.blue_robots[i].velangular;
                blue.kickspeeds[i] = Qt.vector3d(observer.blue_robots[i].kickspeedx, observer.blue_robots[i].kickspeedz, observer.blue_robots[i].kickspeedx);
            }
            for (var j = 0; j < yellow.num; j++) {
                yellow.velNormals[j] = observer.yellow_robots[j].velnormal;
                yellow.velTangents[j] = observer.yellow_robots[j].veltangent;
                yellow.velAngulars[j] = observer.yellow_robots[j].velangular;
                yellow.kickspeeds[j] = Qt.vector3d(observer.yellow_robots[j].kickspeedx, observer.yellow_robots[j].kickspeedz, observer.yellow_robots[j].kickspeedx);
            }
        }
        function onYellowRobotsChanged() {
            for (var i = 0; i < yellow.num; i++) {
                yellow.velNormals[i] = observer.yellow_robots[i].velnormal;
                yellow.velTangents[i] = observer.yellow_robots[i].veltangent;
                yellow.velAngulars[i] = observer.yellow_robots[i].velangular;
                yellow.spinners[i] = observer.yellow_robots[i].spinner;
            }
        }
        function onRobotReplacementRequested(id, isYellow, sceneX, sceneZ, sceneRotYDeg) {
            // reset() (not just assigning position/eulerRotation) is required to actually
            // warp a DynamicRigidBody's physics pose: for a dynamic body, physics owns the
            // transform, so plain property assignment wouldn't move the PhysX actor. This
            // also zeroes the body's linear/angular velocity. The observer has already
            // stopped any latched velocity command for this robot (Robot::resetMotion(),
            // called before this signal), so it stays put instead of immediately driving
            // off again on the next tick.
            let color = isYellow ? yellow : blue;
            let frame = (isYellow ? yBotsFrame : bBotsFrame).children[id];
            // m2 は config の robotCount 分しか Repeater3D を生まない。mirror-kickoff が
            // 退避用に id 11..15 を送ると children[id] が undefined → TypeError だった。
            if (!frame || typeof frame.reset !== "function") {
                return;
            }
            frame.reset(Qt.vector3d(sceneX, 0, sceneZ), Qt.vector3d(0, sceneRotYDeg, 0));
            // botMovement() derives the robot's "current velocity" from the pose delta
            // across one tick (poses[i] vs. prePoses[i]). Left untouched, prePoses[i]
            // still holds the pre-teleport pose, so the position jump reads as a huge
            // one-tick velocity and MotionControl's accel-limiter spends a few frames
            // coasting it back down to the (now zero) commanded speed instead of the
            // robot being still immediately. Seeding prePoses/preVelocities to the
            // just-placed, at-rest pose avoids that phantom delta.
            let headingRad = mu.normalizeRadian((frame.eulerRotation.y + 90) * Math.PI / 180.0);
            color.prePoses[id] = Qt.vector4d(frame.position.x, frame.position.y, frame.position.z, headingRad);
            color.preVelocities[id] = Qt.vector4d(0, 0, 0, 0);
        }
        function onBallReplacementRequested(sceneX, sceneZ, hasVelocity, sceneVx, sceneVz) {
            placeBall(Qt.vector3d(sceneX, 21, sceneZ), hasVelocity ? Qt.vector3d(sceneVx, 0, sceneVz) : null);
        }
    }

    Repeater3D {
        id: bBotsFrame
        model: blue.num
        DynamicRigidBody {
            objectName: "b" + String(index)
            massMode: DynamicRigidBody.MassAndInertiaTensor
            mass: 2.5
            inertiaTensor: Qt.vector3d(5000, 5000, 5000)
            linearAxisLock: DynamicRigidBody.LockY
            sendContactReports: true
            physicsMaterial: botMaterial
            position: Qt.vector4d(blue.poses[index].x, 0, blue.poses[index].z, blue.poses[index].w)
            collisionShapes: [
                ConvexMeshShape {
                    source: "../../../assets/models/bot/Rione/rigid_body/meshes/body.cooked.cvx"
                    eulerRotation: Qt.vector3d(-90, 0, 0)
                },
                ConvexMeshShape { 
                    source: "../../../assets/models/bot/Rione/rigid_body/meshes/centerLeft.cooked.cvx" 
                    eulerRotation: Qt.vector3d(-90, 0, 0)
                },
                ConvexMeshShape { 
                    source: "../../../assets/models/bot/Rione/rigid_body/meshes/centerRight.cooked.cvx" 
                    eulerRotation: Qt.vector3d(-90, 0, 0)
                },
                ConvexMeshShape { 
                    source: "../../../assets/models/bot/Rione/rigid_body/meshes/dribbler.cooked.cvx" 
                    eulerRotation: Qt.vector3d(-90, 0, 0)
                },
                ConvexMeshShape {
                    source: "../../../assets/models/bot/Rione/rigid_body/meshes/chip.cooked.cvx" 
                    eulerRotation: Qt.vector3d(-90, 0, 0)
                },
                ConvexMeshShape {
                    source: "../../../assets/models/ball/meshes/ball.cooked.cvx"
                    position: Qt.vector3d(0, 5000, 0)
                }
            ]
            BlueBody.Visualize {
                visible: !observer.lightBlueRobotMode
                eulerRotation: Qt.vector3d(-90, 0, 0)
                position: Qt.vector3d(0, 0, 0)
            }
            BlueLightBody.Frame {
                visible: observer.lightBlueRobotMode
                eulerRotation: Qt.vector3d(-90, 0, 0)
                position: Qt.vector3d(0, 0, 0)
            }
            PerspectiveCamera {
                id: bRobotCamera
                position: Qt.vector3d(0, 90, -70)
                clipFar: 20000
                clipNear: 1
                fieldOfView: 60
                eulerRotation: Qt.vector3d(-35, 0, 0)
                Component.onCompleted: {
                    blue.cameras.push(bRobotCamera);
                }
            }
            Model {
                source: "../../../assets/models/bot/Rione/viz/meshes/visualize.mesh"
                pickable: true
                objectName: "b"+String(index)
                eulerRotation: Qt.vector3d(-90, 0, 0)
                
            }
            Model {
                source: "#Cylinder"
                scale: Qt.vector3d(0.5, colorHeight, 0.5)
                position: Qt.vector3d(0, 122, 0)
                materials: [
                    DefaultMaterial {
                        diffuseColor: "blue"
                    }
                ]
            }
            Repeater3D {
                model: 4
                delegate: Model {
                    source: "#Cylinder"
                    scale: Qt.vector3d(0.4, colorHeight, 0.4)
                    position: {
                        var offsets = [
                            Qt.vector3d(65*Math.cos(Math.PI-radianOffset), 0, 65*Math.sin(Math.PI-radianOffset)),  // Left Up
                            Qt.vector3d(65*Math.cos(Math.PI/2.0-radianOffset), 0, 65*Math.sin(Math.PI/2.0-radianOffset)), // Left Down
                            Qt.vector3d(65*Math.cos(Math.PI/2.0+radianOffset), 0, 65*Math.sin(Math.PI/2.0+radianOffset)), // Right Down
                            Qt.vector3d(65*Math.cos(radianOffset), 0, 65*Math.sin(radianOffset))   // Right Up
                        ];
                        return Qt.vector3d(
                            offsets[index].x, 122, offsets[index].z
                        );
                    }
                    materials: [
                        DefaultMaterial {
                            diffuseColor: {
                                var colors = ["#EA3EF7", "#75FA4C", "#EA3EF7", "#75FA4C"];
                                return colors[index];
                            }
                        }
                    ]
                }
            }
            // Glowing "grabbed" ring: only visible on the robot currently
            // being picked up/carried, so it's obvious at a glance which one
            // has been grabbed.
            Model {
                source: "#Cylinder"
                visible: isGrabbingBot && grabbedColor === "blue" && grabbedId === index
                scale: Qt.vector3d(0.62, 0.015, 0.62)
                position: Qt.vector3d(0, 4, 0)
                opacity: 0.6
                materials: [
                    DefaultMaterial {
                        diffuseColor: "#00E5FF"
                        lighting: DefaultMaterial.NoLighting
                    }
                ]
                SequentialAnimation on opacity {
                    loops: Animation.Infinite
                    NumberAnimation { to: 0.9; duration: 450; easing.type: Easing.InOutQuad }
                    NumberAnimation { to: 0.25; duration: 450; easing.type: Easing.InOutQuad }
                }
            }
        }
    }
    // onBBotNumChanged: {
    //     bBotsCamera = [];
    // }

    Repeater3D {
        id: yBotsFrame
        model: yellow.num
        DynamicRigidBody {
            objectName: "y" + String(index)
            massMode: DynamicRigidBody.MassAndInertiaTensor
            mass: 2.5
            inertiaTensor: Qt.vector3d(5000, 5000, 5000)
            linearAxisLock: DynamicRigidBody.LockY
            sendContactReports: true
            physicsMaterial: botMaterial
            position: Qt.vector4d(yellow.poses[index].x, 0, yellow.poses[index].z, yellow.poses[index].w)
            collisionShapes: [
                ConvexMeshShape {
                    source: "../../../assets/models/bot/Rione/rigid_body/meshes/body.cooked.cvx"
                    eulerRotation: Qt.vector3d(-90, 0, 0)
                },
                ConvexMeshShape { 
                    source: "../../../assets/models/bot/Rione/rigid_body/meshes/centerLeft.cooked.cvx" 
                    eulerRotation: Qt.vector3d(-90, 0, 0)
                },
                ConvexMeshShape { 
                    source: "../../../assets/models/bot/Rione/rigid_body/meshes/centerRight.cooked.cvx" 
                    eulerRotation: Qt.vector3d(-90, 0, 0)
                },
                ConvexMeshShape { 
                    source: "../../../assets/models/bot/Rione/rigid_body/meshes/dribbler.cooked.cvx" 
                    eulerRotation: Qt.vector3d(-90, 0, 0)
                },
                ConvexMeshShape {
                    source: "../../../assets/models/bot/Rione/rigid_body/meshes/chip.cooked.cvx" 
                    eulerRotation: Qt.vector3d(-90, 0, 0)
                },
                ConvexMeshShape {
                    source: "../../../assets/models/ball/meshes/ball.cooked.cvx"
                    position: Qt.vector3d(0, 5000, 0)
                }
            ]
            YellowBody.Visualize {
                visible: !observer.lightYellowRobotMode
                eulerRotation: Qt.vector3d(-90, 0, 0)
                position: Qt.vector3d(0, 0, 0)
            }
            YellowLightBody.Frame {
                visible: observer.lightYellowRobotMode
                eulerRotation: Qt.vector3d(-90, 0, 0)
                position: Qt.vector3d(0, 0, 0)
            }
            PerspectiveCamera {
                id: yRobotCamera
                position: Qt.vector3d(0, 90, -70)
                clipFar: 20000
                clipNear: 1
                fieldOfView: 60
                eulerRotation: Qt.vector3d(-35, 0, 0)
                Component.onCompleted: {
                    yellow.cameras.push(yRobotCamera);
                }
            }
            Model {
                source: "../../../assets/models/bot/Rione/viz/meshes/visualize.mesh"
                pickable: true
                objectName: "y"+String(index)
                eulerRotation: Qt.vector3d(-90, 0, 0)
            }
            Model {
                source: "#Cylinder"
                scale: Qt.vector3d(0.5, colorHeight, 0.5)
                position: Qt.vector3d(0, 122, 0)
                materials: [
                    DefaultMaterial {
                        diffuseColor: "yellow"
                    }
                ]
            }

            Repeater3D {
                model: 4
                delegate: Model {
                    source: "#Cylinder"
                    scale: Qt.vector3d(0.4, colorHeight, 0.4)
                    position: {
                        var offsets = [
                            Qt.vector3d(65*Math.cos(Math.PI-radianOffset), 0, 65*Math.sin(Math.PI-radianOffset)), // Left Up
                            Qt.vector3d(65*Math.cos(Math.PI/2.0-radianOffset), 0, 65*Math.sin(Math.PI/2.0-radianOffset)), // Left Down
                            Qt.vector3d(65*Math.cos(Math.PI/2.0+radianOffset), 0, 65*Math.sin(Math.PI/2.0+radianOffset)), // Right Down
                            Qt.vector3d(65*Math.cos(radianOffset), 0, 65*Math.sin(radianOffset))   // Right Up
                        ];
                        return Qt.vector3d(
                            offsets[index].x,
                            122,
                            offsets[index].z
                        );
                    }
                    materials: [
                        DefaultMaterial {
                            diffuseColor: {
                                var colors = ["#EA3EF7", "#75FA4C", "#EA3EF7", "#75FA4C"];
                                return colors[index];
                            }
                        }
                    ]
                }
            }
            // Glowing "grabbed" ring: only visible on the robot currently
            // being picked up/carried, so it's obvious at a glance which one
            // has been grabbed.
            Model {
                source: "#Cylinder"
                visible: isGrabbingBot && grabbedColor === "yellow" && grabbedId === index
                scale: Qt.vector3d(0.62, 0.015, 0.62)
                position: Qt.vector3d(0, 4, 0)
                opacity: 0.6
                materials: [
                    DefaultMaterial {
                        diffuseColor: "#00E5FF"
                        lighting: DefaultMaterial.NoLighting
                    }
                ]
                SequentialAnimation on opacity {
                    loops: Animation.Infinite
                    NumberAnimation { to: 0.9; duration: 450; easing.type: Easing.InOutQuad }
                    NumberAnimation { to: 0.25; duration: 450; easing.type: Easing.InOutQuad }
                }
            }
        }
    }
    // onYBotNumChanged: {
    //     yBotsCamera = [];
    // }

    PhysicsMaterial {
        id: ballMaterial
        // 接地の摩擦は 0。地面との接線力は applyBallFriction() が丸ごと持っているので、
        // PhysX 側にも摩擦があると滑走相が二重に減速する (PhysX は 2 つの材質を平均するため、
        // 既定の 0.5 のままだと地面接触に μ = 0.25 が上乗せされていた)。
        staticFriction: 0.0
        dynamicFriction: 0.0
        // 壁・ゴール枠との跳ね返り。ロボットとの跳ね返りは botMaterial 側で決める。
        restitution: observer.ballRestitution
    }

    // ロボットの外装。RAVEN の ball_model.direct_kick を再現するための材質。
    //   tangent_retention = 1.0 … 接線方向は落ちない → 摩擦 0
    //   normal_restitution    … 法線方向の反発。PhysX は接触する 2 つの材質の平均を取るので、
    //                           球もロボットも同じ値を持たせて平均を normal_restitution にする。
    // 以前は ball 側を壁向けの値 (0.6) のままにして、その平均が 0.8 になる値をここに入れて
    // いた。式としては合うが、ロボット同士の反発が 1.0 (完全弾性) になる副作用があった。
    // 壁の跳ね返りは Field.qml の wallMaterial 側で調整する。
    PhysicsMaterial {
        id: botMaterial
        staticFriction: 0.0
        dynamicFriction: 0.0
        restitution: observer.ballNormalRestitution
    }

    DynamicRigidBody {
        id: ball
        objectName: "ball"
        position: Qt.vector3d(0, 0, 0)
        sendContactReports: true
        physicsMaterial: ballMaterial
        // Explicit mass/inertia so the friction model's impulses (J = m*dv) produce the
        // intended deceleration exactly, independent of the engine's default density.
        // Solid-sphere inertia I = (2/5) m R^2 about every axis.
        massMode: DynamicRigidBody.MassAndInertiaTensor
        mass: ballMass
        inertiaTensor: Qt.vector3d(0.4 * ballMass * ballRadius * ballRadius,
                                   0.4 * ballMass * ballRadius * ballRadius,
                                   0.4 * ballMass * ballRadius * ballRadius)
        collisionShapes: [
            SphereShape {
                diameter: 42
            }
        ]
        Ball {
        }
    }

    // Repeater3D {
    //     id: ballModels
    //     model: ballModelNum
    //     Ball {
    //     }
    // }
    Ball {
        id: tempBallModel
        
    }
    BallMarker {
        id: ballMarker
        eulerRotation: Qt.vector3d(0, 0, 0)
        scale: Qt.vector3d(0.8, 0.01, 0.8)
    }

    // Scene position of the ball: the physics body, or the holder's mouth while it is held (the body is parked).
    function heldBallScenePosition() {
        if (dribbleInfo.id == -1) {
            return Qt.vector3d(ballPosition.x, ballPosition.y, ballPosition.z);
        }
        let frame = (dribbleInfo.isYellow ? yBotsFrame : bBotsFrame).children[dribbleInfo.id];
        let w = mu.normalizeRadian((frame.eulerRotation.y + 90) * Math.PI / 180.0);
        return Qt.vector3d(frame.position.x + 95 * Math.cos(-w), 25, frame.position.z + 95 * Math.sin(-w));
    }

    function resolveBallContest() {
        let bp = heldBallScenePosition();
        let best = null;
        let bestKick = false;
        let bestDist = 1e9;
        for (let team = 0; team < 2; team++) {
            let isYellow = team == 1;
            let color = isYellow ? yellow : blue;
            let frames = isYellow ? yBotsFrame : bBotsFrame;
            for (let i = 0; i < color.num; i++) {
                let frame = frames.children[i];
                if (!frame) {
                    continue;
                }
                let w = mu.normalizeRadian((frame.eulerRotation.y + 90) * Math.PI / 180.0);
                let d = Math.hypot(frame.position.x - bp.x, frame.position.z - bp.z);
                let rad = mu.normalizeRadian(Math.atan2(frame.position.z - bp.z, frame.position.x - bp.x) - Math.PI + w);
                let inCone = d < 110 * Math.cos(Math.abs(rad)) && Math.abs(rad) < Math.PI / 15.0 && bp.y < 40;
                if (!inCone) {
                    continue;
                }
                let key = (isYellow ? "y" : "b") + i;
                let recharged = !(key in kickCooldown) || kickCooldown[key] <= 0;
                let wantsKick = recharged && (color.kickspeeds[i].x != 0 || color.kickspeeds[i].y != 0);
                if (!wantsKick && !(color.spinners[i] > 0 && recharged)) {
                    continue;   // a robot that has just kicked neither kicks nor catches until it recharges
                }
                let isHolder = dribbleInfo.id == i && dribbleInfo.isYellow == isYellow;
                let effective = isHolder ? d - contestTakeoverMarginMm : d;   // the holder keeps a small edge
                if (best === null || (wantsKick && !bestKick) || (wantsKick == bestKick && effective < bestDist)) {
                    best = { isYellow: isYellow, id: i };
                    bestKick = wantsKick;
                    bestDist = effective;
                }
            }
        }
        ballContestWinner = best;
        if (dribbleInfo.id != -1 && best !== null && (best.id != dribbleInfo.id || best.isYellow != dribbleInfo.isYellow)) {
            // The holder loses the ball: the body goes back to the mouth as a free ball for this frame's botMovement.
            let holderColor = dribbleInfo.isYellow ? yellow : blue;
            let holderFrame = (dribbleInfo.isYellow ? yBotsFrame : bBotsFrame).children[dribbleInfo.id];
            holderFrame.collisionShapes[5].position = Qt.vector3d(0, 5000, 0);
            holderColor.holds[dribbleInfo.id] = false;
            ball.reset(bp, Qt.vector3d(0, 0, 0));
            ballPosition = Qt.vector4d(bp.x, bp.y, bp.z, 0);
            dribbleInfo.id = -1;
        }
    }

    function botMovement(color, timestep, isYellow=false) {
        let botFrame = isYellow ? yBotsFrame : bBotsFrame;

        for (let i = 0; i < color.num; i++) {
            let frame = botFrame.children[i];
            let tempBallFrame = frame.collisionShapes[5];
            color.poses[i] = Qt.vector4d(frame.position.x, frame.position.y, frame.position.z, mu.normalizeRadian((frame.eulerRotation.y+90) * Math.PI / 180.0));

            color.velocities[i] = mu.calcVelocity(color.poses[i], color.prePoses[i], timestep);
            // velTangents/velNormals/velAngulars は Robot::advanceActuation が実機の同定モデル
            // (むだ時間・一次遅れ・定常ゲイン・軸別の牽引限界・車輪周速の予算) を通したあとの
            // 「実際に出る速度」。MotionControl の等方な加減速制限を上から重ねると、軸ごとに
            // 2 倍以上違う牽引限界も 1 未満の定常ゲインも潰れて実機と別物になるので通さない。
            // [RobotModel] Enabled=false のときだけ従来の経路に戻る。
            let newVelocity = observer.robotModelEnabled
                ? Qt.vector3d(color.velTangents[i], color.velNormals[i], color.velAngulars[i])
                : motionControl.calcSpeed(Qt.vector3d(color.velTangents[i], color.velNormals[i], color.velAngulars[i]), color.velocities[i], color.preVelocities[i], timestep, color.poses[i].w);

            color.prePoses[i] = color.poses[i];
            color.preVelocities[i] = Qt.vector4d(newVelocity.x, newVelocity.y, newVelocity.z, newVelocity.w);

            frame.setLinearVelocity(Qt.vector3d(newVelocity.x*Math.cos(color.poses[i].w) - newVelocity.y*Math.sin(color.poses[i].w), 0, -newVelocity.x*Math.sin(color.poses[i].w) - newVelocity.y*Math.cos(color.poses[i].w)));
            frame.setAngularVelocity(Qt.vector3d(0, newVelocity.z, 0));

            let botDistanceBall = Math.sqrt(Math.pow(frame.position.x - ballPosition.x, 2) + Math.pow(frame.position.z - ballPosition.z, 2));
            let botRadianBall = mu.normalizeRadian(Math.atan2(frame.position.z - ballPosition.z, frame.position.x - ballPosition.x) - Math.PI + color.poses[i].w);
            // A robot that switched its dribbler off lets go of the ball. The holder's mouth test below is forced to
            // pass, so without this the release branch can never run and the ball is carried for ever (the only way
            // out was a kick). Keep holding while it asks to kick: the kick needs the ball in the mouth.
            let asksKickNow = color.kickspeeds[i].x != 0 || color.kickspeeds[i].y != 0;
            if (!(color.spinners[i] > 0) && !asksKickNow) {
                control.release(frame, isYellow, i, color);
            }
            if (dribbleInfo.id != -1) {
                if (isYellow == dribbleInfo.isYellow && i == dribbleInfo.id) {
                    botDistanceBall = 95;
                    botRadianBall = 0;
                }
            }
            if (botDistanceBall < 110 * Math.cos(Math.abs(botRadianBall)) && Math.abs(botRadianBall) < Math.PI/15.0 && ballPosition.y < 40) {
                let kickKey = (isYellow ? "y" : "b") + i;
                let asksKick = color.kickspeeds[i].x != 0 || color.kickspeeds[i].y != 0;
                if (ballContestWinner !== null && (ballContestWinner.id != i || ballContestWinner.isYellow != isYellow)) {
                    kickDiag(kickKey, asksKick, "contest winner is " + (ballContestWinner.isYellow ? "y" : "b") + ballContestWinner.id);
                    // The other robot has the ball: our mouth sensor must read empty. Leaving holds[i] stale kept RAVEN's
                    // "touching" true and it fired into nothing for 0.5 s at a time (m33: 146 kick frames, 6 launches).
                    color.holds[i] = false;
                    continue;   // another robot has the ball this frame (resolveBallContest)
                }
                if (dribbleInfo.id != -1 && (dribbleInfo.id != i || isYellow != dribbleInfo.isYellow)) {
                    kickDiag(kickKey, asksKick, "held by " + (dribbleInfo.isYellow ? "y" : "b") + dribbleInfo.id);
                    color.holds[i] = false;
                    continue;
                }
                color.holds[i] = true;
                let recharged = !(kickKey in kickCooldown) || kickCooldown[kickKey] <= 0;
                if (asksKick && !recharged) {
                    kickDiag(kickKey, true, "not recharged (" + kickCooldown[kickKey] + " frames left)");
                }
                let relVx = (ballVelocity.x - color.velocities[i].x) * 1000.0;   // m/s -> mm/s
                let relVz = (ballVelocity.z - color.velocities[i].z) * 1000.0;
                let catchable = Math.hypot(relVx, relVz) < dribbleCatchMaxSpeedMmS
                        || (dribbleInfo.id == i && dribbleInfo.isYellow == isYellow);   // already held: keep it
                if (recharged && asksKick) {
                    kickCooldown[kickKey] = kickRechargeFrames;
                    // wall-clock ms so the line can be matched to RAVEN's MCAP (log_time) without guessing from positions
                    console.log("[kick] " + kickKey + " fires " + Math.round(color.kickspeeds[i].x) + "/" + Math.round(color.kickspeeds[i].y) + " mm/s at ball ("
                            + Math.round(ballPosition.x) + ", " + Math.round(ballPosition.z) + ") t=" + Date.now());
                    control.kick(color, frame, i, color.poses[i].w, ballVelocity);
                } else if (color.spinners[i] > 0 && catchable && recharged) {
                    // recharged: a robot that has just kicked does not re-catch the ball it launched (the body's reset
                    // lands one frame later and the mouth test passes meanwhile; the ball is gone for real).

                    control.dribble(frame, isYellow, i, botRadianBall, botDistanceBall, color);
                }
            } else {
                if (color.holds[i] == true) {
                    dribbleInfo.id = -1;
                    if (ball.position.x > 50000) {
                        ball.reset(Qt.vector3d(frame.position.x + (95 * Math.cos(-color.poses[i].w)), 25, (frame.position.z + (95 * Math.sin(-color.poses[i].w)))), Qt.vector3d(0, 0, 0));
                    }
                }
                if (frame.collisionShapes[5].position.y < 5000) {
                    frame.collisionShapes[5].position = Qt.vector3d(0, 5000, 0);
                }
                color.holds[i] = false;
            }
        }
    }

    // Diagnostics: a robot with the ball in its mouth asked to kick but could not. Logged at most once per second per robot.
    property var kickDiagLast: ({})
    property int diagFrame: 0
    function kickDiag(kickKey, asksKick, why) {
        if (!asksKick) {
            return;
        }
        let now = diagFrame;
        if (!(kickKey in kickDiagLast) || now - kickDiagLast[kickKey] >= 60) {
            kickDiagLast[kickKey] = now;
            console.log("[kick] " + kickKey + " asks to kick but cannot: " + why);
        }
    }

    function updateGameObjects(timestep) 
    {
        diagFrame++;
        for (let key in kickCooldown) {
            if (kickCooldown[key] > 0) {
                kickCooldown[key]--;
            }
        }
        ballVelocity = mu.calcVelocity(ballPosition, preBallPosition, timestep);
        ballAngularVelocity = mu.calcVelocity(ball.eulerRotation, preBallAngularPosition, timestep);
        let teleopSpeed = Math.sqrt(teleopVelocity.x * teleopVelocity.x
                                    + teleopVelocity.y * teleopVelocity.y
                                    + teleopVelocity.z * teleopVelocity.z);
        let teleopActive = teleopSpeed > 1.0;
        // Apply a deferred kick only once the ball has actually returned to the mouth.
        // kick() teleports the parked ball (x~100000) back to the dribbler via the
        // deferred ball.reset(), then stores the launch velocity here. Waiting until the
        // ball is back on-field prevents it from being launched from the off-field park
        // position and flying off into the void on the kick frame.
        if (pendingKickVelocity !== null
                && Math.abs(ball.position.x) < 50000
                && Math.abs(ball.position.z) < 50000) {
            ball.setLinearVelocity(pendingKickVelocity);
            // ここが RAVEN のモデルで言う蹴り出し: v0 を取り直して滑走からやり直す。
            ballLaunchSpeed = Math.sqrt(pendingKickVelocity.x * pendingKickVelocity.x
                                        + pendingKickVelocity.z * pendingKickVelocity.z);
            ballRolling = false;
            // The ball leaves the kicker with no spin; the slide phase spins it up.
            ballSpin = Qt.vector3d(0, 0, 0);
            ball.setAngularVelocity(Qt.vector3d(0, 0, 0));
            pendingKickVelocity = null;
        }
        if (skipRollingFrictionFrames > 0)
            skipRollingFrictionFrames--;
        // Friction: a no-spin kick slides (kinetic friction decelerates + spins it up),
        // then rolls (rolling resistance slowly bleeds off the rest), so it doesn't roll
        // forever off the field (and escape past the boundary walls into huge vision
        // coordinates).
        if (!teleopActive && skipRollingFrictionFrames == 0)
            applyBallFriction(ball, ballVelocity, timestep);
        preBallPosition = ballPosition;
        preBallAngularPosition = ball.eulerRotation;
        ballReset = true;
        
        resolveBallContest();
        botMovement(blue, timestep);
        botMovement(yellow, timestep, true);

        ball2DPosition = Qt.vector2d(ballPosition.x, ballPosition.z);
        if (teleopActive){
            ball.setLinearVelocity(Qt.vector3d(teleopVelocity.x, teleopVelocity.y, teleopVelocity.z));
            let ballFriction = 0.99;
            teleopVelocity = Qt.vector3d(teleopVelocity.x * ballFriction, teleopVelocity.y * ballFriction, teleopVelocity.z * ballFriction);
        } else {
            teleopVelocity = Qt.vector3d(0, 0, 0);
        }
    }

    // RAVEN のボールモデル (common/physics/BallSpeedModel + system_model の ball_model) を
    // そのまま持ち込んだ、滑走 → 転がりの 2 段「一定減速」モデル。
    //
    //   滑走 (slide): 蹴り出しの速さ v0 から k_switch·v0 まで、一定の acc_slide で落ちる
    //   転がり (roll): そこから先は一定の acc_roll で落ちる
    //
    // RAVEN はパスの初速逆算 (BallSpeedModel.initialSpeedFor)、到達時刻 (BallPhysics.arrivalTime)、
    // 到達速度 (speedAfterTravel) をすべてこの 3 つの数から引いている。sim が「摩擦係数 × g」で
    // 別の落ち方をしていると、RAVEN の「ここで受け取れる」が毎回外れる。だから係数ではなく
    // mm/s^2 の減速度そのものを合わせる。値は [BallModel] (observer 経由)。
    //
    // 切り替えの基準が蹴り出しの速さ v0 なのがこのモデルの肝で、物理的な転がり条件
    // (剛球なら 5/7·v0) とは別物。RAVEN が 2/3 で同定しているので sim もそれに従う。
    // v0 は ballLaunchSpeed で持ち、キック・配置・衝突のたびに数え直す。
    //
    // 球の回転は PhysX から読み出せないので ballSpin が唯一の真値。滑走のあいだに
    // 転がりの回転まで線形に持ち上げて、見た目も滑走 → 転がりになるようにしている。
    function applyBallFriction(ballBody, linearVelocity, timestep)
    {
        let aSlide = observer.ballSlideDecelMmS2;   // 滑走の減速度 [mm/s^2]
        let aRoll = observer.ballRollDecelMmS2;     // 転がりの減速度 [mm/s^2]
        let kSwitch = observer.ballSwitchRatio;     // v_switch = kSwitch * v0

        // Skip when both effects are off, on a bad dt, or while the ball is parked
        // off-field during dribbling (x ~ 100000): never disturb the park sentinel.
        if ((aSlide <= 0 && aRoll <= 0) || timestep <= 0 || ballBody.position.x > 50000) {
            return;
        }
        // A chip in the air gets no ground friction until it lands.
        if (ballBody.position.y > 30) {
            return;
        }

        let dt = timestep > 1.0 ? timestep / 1000.0 : timestep;   // ms -> s (fixed 1/60 s)
        let R = ballRadius;

        // calcVelocity() reports mm-per-(frame ms), which is numerically m/s; convert to
        // the scene's mm/s so it is consistent with R (mm) and the decelerations (mm/s^2).
        let vx = linearVelocity.x * 1000.0;
        let vz = linearVelocity.z * 1000.0;
        let vx0 = vx;
        let vz0 = vz;
        let speed0 = Math.sqrt(vx * vx + vz * vz);

        // Impact (wall, goal, robot): the velocity direction flipped or the speed jumped between two
        // frames. The finite-difference velocity spans the impact, so decelerating here would bend the
        // rebound. Skip this frame's impulse and treat the rebound as a fresh launch — which is also
        // what RAVEN's model says happens after a robot contact (direct_kick).
        let pvx = prevBallVelocity.x * 1000.0;
        let pvz = prevBallVelocity.z * 1000.0;
        let prevSpeed = Math.sqrt(pvx * pvx + pvz * pvz);
        let impact = prevSpeed > 200.0 && speed0 > 200.0
                && (vx * pvx + vz * pvz < 0.0 || speed0 > 1.5 * prevSpeed);
        prevBallVelocity = linearVelocity;
        if (impact) {
            ballLaunchSpeed = speed0;
            ballRolling = false;
            ballSpin = Qt.vector3d(0, ballSpin.y, 0);
            ballBody.setAngularVelocity(ballSpin);
            return;
        }
        // Fully stop a crawling ball (there is no PhysX floor friction; the deceleration is
        // modelled entirely here, so nothing else would ever bring it to rest).
        if (speed0 < 20.0) {
            if (speed0 > 0.0) {
                ballBody.applyCentralImpulse(Qt.vector3d(-ballMass * vx, 0, -ballMass * vz));
            }
            ballLaunchSpeed = 0.0;
            ballRolling = true;
            ballSpin = Qt.vector3d(0, 0, 0);
            ballBody.setAngularVelocity(Qt.vector3d(0, 0, 0));
            return;
        }

        // 速くなったなら外から力が入った (押された・蹴られた)。衝突として拾えなかった
        // ぶんもここで新しい蹴り出しとして数え直す。+5 は差分速度の揺れの逃げ。
        if (speed0 > ballLaunchSpeed + 5.0) {
            ballLaunchSpeed = speed0;
            ballRolling = false;
        }

        let speed = speed0;
        let remaining = dt;
        const vSwitch = kSwitch * ballLaunchSpeed;

        if (!ballRolling && aSlide > 0) {
            // --- 滑走相。switch をまたぐフレームは、またぐところで刻んで残りを転がりに回す。 ---
            if (speed > vSwitch) {
                let tSlide = Math.min(remaining, (speed - vSwitch) / aSlide);
                speed -= aSlide * tSlide;
                remaining -= tSlide;
            }
            if (speed <= vSwitch) {
                ballRolling = true;
            }
        }
        if (remaining > 0 && aRoll > 0) {
            // --- 転がり相 ---
            speed = Math.max(0.0, speed - aRoll * remaining);
        }

        // 向きは変えずに大きさだけ落とす。
        const scale = speed / speed0;
        vx *= scale;
        vz *= scale;

        // 見た目の回転。転がりに入ったら転がり条件 (接地点が止まる) そのもの、滑走の
        // あいだは v0 から v_switch へ進んだ割合ぶんだけ、そこへ線形に持ち上げる。
        let rollWx = vz / R;
        let rollWz = -vx / R;
        let spinRatio = 1.0;
        if (!ballRolling && ballLaunchSpeed > vSwitch) {
            spinRatio = (ballLaunchSpeed - speed) / (ballLaunchSpeed - vSwitch);
            spinRatio = Math.max(0.0, Math.min(1.0, spinRatio));
        }

        // Apply the linear change as an impulse (J = m*dv) so PhysX still owns collision
        // response; drive the visible spin directly from our tracked angular velocity.
        ballBody.applyCentralImpulse(Qt.vector3d(ballMass * (vx - vx0), 0, ballMass * (vz - vz0)));
        ballSpin = Qt.vector3d(rollWx * spinRatio, ballSpin.y, rollWz * spinRatio);
        ballBody.setAngularVelocity(ballSpin);
    }

    function syncGameObjects() {
        let blueBotData = sync.updateBot(blue, false);
        let yellowBotData = sync.updateBot(yellow, true);
        sync.updateBall();
        observer.updateObjects(
            blueBotData.positions, 
            yellowBotData.positions, 
            blueBotData.pixels,
            yellowBotData.pixels,
            blueBotData.cameraExists,
            yellowBotData.cameraExists, 
            blueBotData.ballContacts, 
            yellowBotData.ballContacts,
            ballPosition,
            isFoundBall
        );
    }

    // Places the ball's PHYSICS body at scenePosition (grounded, y=21) and, if velocity
    // is non-null, sets its linear velocity too; otherwise it is left at rest (reset()
    // already zeroes it). Clears every piece of ball state that could otherwise fight
    // the placement on a later tick (a deferred kick landing, stale tracked spin/velocity,
    // an in-progress mouse-drag teleop throw), and un-parks any robot's ball-holding
    // marker so dribble state doesn't linger against the newly placed ball. Shared by the
    // mouse "place ball" shortcut and network Replacement so both behave identically.
    function placeBall(scenePosition, velocity) {
        teleopVelocity = Qt.vector4d(0, 0, 0, 0);
        ballVelocity = Qt.vector4d(0, 0, 0, 0);
        prevBallVelocity = Qt.vector3d(0, 0, 0);
        ballSpin = Qt.vector3d(0, 0, 0);
        pendingKickVelocity = null;
        ball.reset(scenePosition, Qt.vector3d(0, 0, 0));
        if (velocity !== null) {
            ball.setLinearVelocity(velocity);
            // 速度つきの配置は蹴り出しと同じ扱い: v0 を取り直して滑走からやり直す。
            ballLaunchSpeed = Math.sqrt(velocity.x * velocity.x + velocity.z * velocity.z);
            ballRolling = false;
        } else {
            ballLaunchSpeed = 0.0;
            ballRolling = true;
        }
        ball.setAngularVelocity(Qt.vector3d(0, 0, 0));
        ballPosition = Qt.vector4d(ball.position.x, ball.position.y, ball.position.z, 0);
        preBallPosition = ballPosition;
        skipRollingFrictionFrames = placementSettleFrames;
        // Release the dribbler hold too: while dribbleInfo points at a robot, botMovement() forces
        // that robot's ball distance/angle to "held" and the next dribble() would snap the ball
        // back onto its dribbler, so a placement could never take the ball away from a holder.
        dribbleInfo.id = -1;
        for (let i = 0; i < blue.num; i++) {
            bBotsFrame.children[i].collisionShapes[5].position = Qt.vector3d(0, 5000, 0);
            blue.holds[i] = false;
        }
        for (let i = 0; i < yellow.num; i++) {
            yBotsFrame.children[i].collisionShapes[5].position = Qt.vector3d(0, 5000, 0);
            yellow.holds[i] = false;
        }
    }

    function resetPosition(target, result) {
        if (target == "ball") {
            placeBall(result.scenePosition, null);
        } else if (target == "bot") {
            if (selectedRobotColor == "blue") {
                bBotsFrame.children[botCursorID].reset(result.scenePosition, Qt.vector3d(0, -90, 0));
            } else if (selectedRobotColor == "yellow") {
                yBotsFrame.children[botCursorID].reset(result.scenePosition, Qt.vector3d(0, 90, 0));
            }
        }
    }

    // Finds the field-plane hit point in a pickAll() result list, or null if the
    // cursor isn't over the field this frame (e.g. it slid off the pitch).
    function findFieldHit(results) {
        for (let i = 0; i < results.length; i++) {
            if (results[i].objectHit.objectName == "field") {
                return results[i].scenePosition;
            }
        }
        return null;
    }

    // Picks whichever robot is under the cursor (if any) out of a pickAll() result list.
    function findBotHit(results) {
        let hit = { color: null, id: -1 };
        for (let i = 0; i < results.length; i++) {
            let name = results[i].objectHit.objectName;
            if (name.startsWith("b")) {
                hit.color = "blue";
                hit.id = parseInt(name.slice(1));
            } else if (name.startsWith("y")) {
                hit.color = "yellow";
                hit.id = parseInt(name.slice(1));
            }
        }
        return hit;
    }

    // --- Grab & carry -------------------------------------------------------
    // Press-down on a robot: "picks it up" (lifts it off the pitch) and remembers
    // the offset between the robot's origin and the click point, so the robot
    // keeps its position relative to the cursor instead of snapping its center
    // onto the pointer. Returns true if a robot was actually grabbed.
    function beginGrabBot(results) {
        let fieldPos = findFieldHit(results);
        let hit = findBotHit(results);
        if (fieldPos === null || hit.color === null) return false;

        let frame = (hit.color == "blue" ? bBotsFrame : yBotsFrame).children[hit.id];

        grabbedColor = hit.color;
        grabbedId = hit.id;
        grabOffsetX = frame.position.x - fieldPos.x;
        grabOffsetZ = frame.position.z - fieldPos.z;
        isGrabbingBot = true;

        // Keep the existing "selected robot" concept (used by the R-key reset
        // shortcut) in sync with whatever we just grabbed.
        selectedRobotColor = hit.color;
        botCursorID = hit.id;

        frame.reset(Qt.vector3d(frame.position.x, grabLiftHeight, frame.position.z), frame.eulerRotation);
        return true;
    }

    // Called while the mouse moves with the button held: carries the grabbed
    // robot to the new cursor position (offset-corrected, lifted, and clamped
    // to stay on the pitch), keeping its current heading untouched.
    function updateGrabBot(results) {
        if (!isGrabbingBot) return;
        let fieldPos = findFieldHit(results);
        if (fieldPos === null) return;

        let frame = (grabbedColor == "blue" ? bBotsFrame : yBotsFrame).children[grabbedId];
        let targetX = Math.max(-6200, Math.min(6200, fieldPos.x + grabOffsetX));
        let targetZ = Math.max(-4700, Math.min(4700, fieldPos.z + grabOffsetZ));
        frame.reset(Qt.vector3d(targetX, grabLiftHeight, targetZ), frame.eulerRotation);
    }

    // Mouse released: sets the carried robot back down where it currently is.
    function endGrabBot() {
        if (!isGrabbingBot) return;
        let frame = (grabbedColor == "blue" ? bBotsFrame : yBotsFrame).children[grabbedId];
        frame.reset(Qt.vector3d(frame.position.x, 0, frame.position.z), frame.eulerRotation);
        isGrabbingBot = false;
        grabbedId = -1;
    }

    function updateBallModel() {
        for (let i = ballModelNum - 1; i > 0; i--) {
            ballPositions[i] = ballPositions[i - 1];
            ballModels.children[i].position = Qt.vector4d(ballPositions[i].x, ballPositions[i].y, ballPositions[i].z, ballPositions[i].w);
        }
        ballPositions[0] = Qt.vector4d(ball.position.x, ball.position.y, ball.position.z, 0);
    }

    Component.onCompleted: {
        
        for (let i = 0; i < observer.blueRobotCount; i++) {
            let frame = bBotsFrame.children[i];
            frame.reset(Qt.vector3d(frame.position.x, 0, frame.position.z), Qt.vector3d(0, blue.poses[i].w, 0));
        }
        for (let i = 0; i < observer.yellowRobotCount; i++) {
            let frame = yBotsFrame.children[i];
            frame.reset(Qt.vector3d(frame.position.x, 0, frame.position.z), Qt.vector3d(0, yellow.poses[i].w, 0));
        }
        for (let i = 1; i < ballModelNum; i++) {
            ballModels.children[i].children[0].materials[0].opacity = 0.11;
        }
        ballMarker.children[0].materials[0].diffuseColor= "#EB392A";
        ballMarker.children[0].materials[0].opacity = 0.4;
    }
    QtObject {
        id: dribbleInfo
        property var id: -1
        property var isYellow: false
        property real radianBall: 0.0
        property real distanceBall: 0.0
    }
    function test() {
        let i = 1;
        let dx,dy,dz;
        let vx,vy,vz;
        let zf = blue.poses[i].kickspeedz;
        dx = Math.cos(blue.poses[i].w);
        dy = Math.sin(blue.poses[i].w);
        
        let dlen = Math.sqrt(dx*dx+dy*dy);
        vx = dx*blue.poses[i].kickspeedx/dlen;
        vy = dy*blue.poses[i].kickspeedx/dlen;
        vz = zf;
        let vballx = ballVelocity.x;
        let vbally = ballVelocity.z;
        let vn = -(vballx*dx + vbally*dy);
        let vt = -(vballx*dy - vbally*dx);
        vx += vn * dx - vt * dy;
        vy += vn * dy + vt * dx; 

    }
    function placeClothLineBall() {
        ballSpin = Qt.vector3d(0, 0, 0);
        ball.setAngularVelocity(Qt.vector3d(0, 0, 0));
        if (Math.abs(ball.position.x) > 5500 && Math.abs(ball.position.z) > 4000) {
            ball.reset(Qt.vector3d(5500 * Math.sign(ball.position.x), 21, 4000 * Math.sign(ball.position.z)), Qt.vector3d(0, 0, 0));
            return;
        }

        if (Math.abs(ball.position.x) > 5500) {
            ball.reset(Qt.vector3d(5500 * Math.sign(ball.position.x), 21, ball.position.z), Qt.vector3d(0, 0, 0));
            return;
        }

        if (ball.position.z < 0) {
            ball.reset(Qt.vector3d(ball.position.x, 21, -4000), Qt.vector3d(0, 0, 0));
        } else {
            ball.reset(Qt.vector3d(ball.position.x, 21, 4000), Qt.vector3d(0, 0, 0));
        }
    }
}
