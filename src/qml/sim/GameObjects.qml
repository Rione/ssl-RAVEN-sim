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
    property int skipRollingFrictionFrames: 0
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
                blue.kickspeeds[i] = Qt.vector3d(observer.blue_robots[i].kickspeedx, observer.blue_robots[i].kickspeedz, observer.blue_robots[i].kickspeedx);
                blue.spinners[i] = observer.blue_robots[i].spinner;
            }
        }
        function onYellowRobotsChanged() {
            for (var i = 0; i < yellow.num; i++) {
                yellow.velNormals[i] = observer.yellow_robots[i].velnormal;
                yellow.velTangents[i] = observer.yellow_robots[i].veltangent;
                yellow.velAngulars[i] = observer.yellow_robots[i].velangular;
                yellow.kickspeeds[i] = Qt.vector3d(observer.yellow_robots[i].kickspeedx, observer.yellow_robots[i].kickspeedz, observer.yellow_robots[i].kickspeedx);
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
        restitution: observer.ballRestitution
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
            let newVelocity = motionControl.calcSpeed(Qt.vector3d(color.velTangents[i], color.velNormals[i], color.velAngulars[i]), color.velocities[i], color.preVelocities[i], timestep, color.poses[i].w);

            color.prePoses[i] = color.poses[i];
            color.preVelocities[i] = Qt.vector4d(newVelocity.x, newVelocity.y, newVelocity.z, newVelocity.w);

            frame.setLinearVelocity(Qt.vector3d(newVelocity.x*Math.cos(color.poses[i].w) - newVelocity.y*Math.sin(color.poses[i].w), 0, -newVelocity.x*Math.sin(color.poses[i].w) - newVelocity.y*Math.cos(color.poses[i].w)));
            frame.setAngularVelocity(Qt.vector3d(0, newVelocity.z, 0));

            let botDistanceBall = Math.sqrt(Math.pow(frame.position.x - ballPosition.x, 2) + Math.pow(frame.position.z - ballPosition.z, 2));
            let botRadianBall = mu.normalizeRadian(Math.atan2(frame.position.z - ballPosition.z, frame.position.x - ballPosition.x) - Math.PI + color.poses[i].w);
            if (dribbleInfo.id != -1) {
                if (isYellow == dribbleInfo.isYellow && i == dribbleInfo.id) {
                    botDistanceBall = 95;
                    botRadianBall = 0;
                }
            }
            if (botDistanceBall < 110 * Math.cos(Math.abs(botRadianBall)) && Math.abs(botRadianBall) < Math.PI/15.0 && ballPosition.y < 40) {
                if (ballContestWinner !== null && (ballContestWinner.id != i || ballContestWinner.isYellow != isYellow)) {
                    continue;   // another robot has the ball this frame (resolveBallContest)
                }
                if (dribbleInfo.id != -1 && (dribbleInfo.id != i || isYellow != dribbleInfo.isYellow)) {
                    continue;
                }
                color.holds[i] = true;
                let kickKey = (isYellow ? "y" : "b") + i;
                let recharged = !(kickKey in kickCooldown) || kickCooldown[kickKey] <= 0;
                let relVx = (ballVelocity.x - color.velocities[i].x) * 1000.0;   // m/s -> mm/s
                let relVz = (ballVelocity.z - color.velocities[i].z) * 1000.0;
                let catchable = Math.hypot(relVx, relVz) < dribbleCatchMaxSpeedMmS
                        || (dribbleInfo.id == i && dribbleInfo.isYellow == isYellow);   // already held: keep it
                if (recharged && (color.kickspeeds[i].x != 0 || color.kickspeeds[i].y != 0)) {
                    kickCooldown[kickKey] = kickRechargeFrames;
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

    function updateGameObjects(timestep) 
    {
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
            // The ball leaves the kicker with no spin; the slip-friction phase spins it up.
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

    // Slip/roll ball-friction model.
    //
    // A ball launched without spin first SLIDES: kinetic (dynamic) friction acts at the
    // contact point opposite the slip velocity, decelerating the ball AND applying a
    // torque that spins it up. The slip shrinks until the contact point stops moving
    // (rolling without slipping), after which only the much smaller ROLLING RESISTANCE
    // decelerates it. The ball therefore transitions slip -> roll while slowing down.
    //
    // Contact point (ball bottom, offset (0,-R,0) from the centre) velocity:
    //   u = v + omega x (0,-R,0) = (vx + R*wz, vz - R*wx)
    // Rolling-without-slipping means u = 0, i.e. wz = -vx/R, wx = vz/R.
    // For a solid sphere (I = 2/5 m R^2) the slip magnitude decays at rate (7/2)*muK*g,
    // giving the classic result that a no-spin launch reaches rolling at 5/7 of its
    // launch speed.
    //
    // Parameters live in [Physics] of the config:
    //   BallDynamicFriction (muK)  - kinetic/dynamic friction coefficient (slip phase)
    //   RollingFriction     (cRoll) - rolling resistance coefficient (roll phase)
    function applyBallFriction(ballBody, linearVelocity, timestep)
    {
        let muK = observer.ballDynamicFriction;   // 動摩擦係数
        let cRoll = observer.rollingFriction;     // 転がり抵抗係数

        // Skip when both effects are off, on a bad dt, or while the ball is parked
        // off-field during dribbling (x ~ 100000): never disturb the park sentinel.
        if ((muK <= 0 && cRoll <= 0) || timestep <= 0 || ballBody.position.x > 50000) {
            return;
        }
        // A chip in the air gets no ground friction until it lands.
        if (ballBody.position.y > 30) {
            return;
        }

        let dt = timestep > 1.0 ? timestep / 1000.0 : timestep;   // ms -> s (fixed 1/60 s)
        let R = ballRadius;
        let g = observer.gravity * 1000.0;   // m/s^2 -> mm/s^2, matching PhysicsWorld.gravity

        // calcVelocity() reports mm-per-(frame ms), which is numerically m/s; convert to
        // the scene's mm/s so it is consistent with R (mm) and g (mm/s^2).
        let vx = linearVelocity.x * 1000.0;
        let vz = linearVelocity.z * 1000.0;
        let wx = ballSpin.x;
        let wz = ballSpin.z;
        let vx0 = vx;
        let vz0 = vz;

        let speed = Math.sqrt(vx * vx + vz * vz);

        // Impact (wall, goal, robot): the velocity direction flipped or the speed jumped between two
        // frames. The tracked spin still belongs to the motion BEFORE the impact, and the finite-
        // difference velocity spans the impact, so applying slip friction here would drag the ball
        // along its old spin and bend the rebound. Re-sync the spin to rolling with the new velocity
        // and let this frame pass without a friction impulse.
        let pvx = prevBallVelocity.x * 1000.0;
        let pvz = prevBallVelocity.z * 1000.0;
        let prevSpeed = Math.sqrt(pvx * pvx + pvz * pvz);
        let impact = prevSpeed > 200.0 && speed > 200.0
                && (vx * pvx + vz * pvz < 0.0 || speed > 1.5 * prevSpeed);
        prevBallVelocity = linearVelocity;
        if (impact) {
            ballSpin = Qt.vector3d(vz / R, ballSpin.y, -vx / R);
            ballBody.setAngularVelocity(ballSpin);
            return;
        }
        // Fully stop a crawling ball (there is no PhysX floor friction; friction is
        // modelled entirely here, so nothing else would ever bring it to rest).
        if (speed < 20.0) {
            if (speed > 0.0) {
                ballBody.applyCentralImpulse(Qt.vector3d(-ballMass * vx, 0, -ballMass * vz));
            }
            ballSpin = Qt.vector3d(0, 0, 0);
            ballBody.setAngularVelocity(Qt.vector3d(0, 0, 0));
            return;
        }

        // Slip velocity at the contact point.
        let ux = vx + R * wz;
        let uz = vz - R * wx;
        let slip = Math.sqrt(ux * ux + uz * uz);

        let remaining = dt;
        const slipEps = 1.0;   // mm/s: below this the contact point is effectively rolling

        if (slip > slipEps && muK > 0) {
            // --- Slip phase (sub-stepped so we switch to rolling exactly when u -> 0). ---
            let aK = muK * g;                 // linear deceleration magnitude
            let slipDecayRate = 3.5 * aK;     // d|u|/dt for a solid sphere
            let tRoll = slip / slipDecayRate; // time until rolling without slipping
            let tSlip = Math.min(tRoll, remaining);

            let nux = ux / slip;
            let nuz = uz / slip;
            // Linear: kinetic friction opposes the slip direction.
            vx -= aK * tSlip * nux;
            vz -= aK * tSlip * nuz;
            // Angular: the same contact force spins the ball up.
            //   dwx/dt = (5 aK)/(2R) * nuz,  dwz/dt = -(5 aK)/(2R) * nux
            let angK = (2.5 * aK) / R;
            wx += angK * tSlip * nuz;
            wz -= angK * tSlip * nux;

            remaining -= tSlip;
        }

        if (remaining > 0) {
            // --- Roll phase: enforce the rolling constraint, then rolling resistance. ---
            wz = -vx / R;
            wx = vz / R;

            let vmag = Math.sqrt(vx * vx + vz * vz);
            if (vmag > 0.0 && cRoll > 0) {
                let dv = Math.min(cRoll * g * remaining, vmag);
                vx -= dv * vx / vmag;
                vz -= dv * vz / vmag;
                wz = -vx / R;   // keep spin consistent with the reduced linear speed
                wx = vz / R;
            }
        }

        // Apply the linear change as an impulse (J = m*dv) so PhysX still owns collision
        // response; drive the visible spin directly from our tracked angular velocity.
        ballBody.applyCentralImpulse(Qt.vector3d(ballMass * (vx - vx0), 0, ballMass * (vz - vz0)));
        ballSpin = Qt.vector3d(wx, ballSpin.y, wz);
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
        }
        ball.setAngularVelocity(Qt.vector3d(0, 0, 0));
        ballPosition = Qt.vector4d(ball.position.x, ball.position.y, ball.position.z, 0);
        preBallPosition = ballPosition;
        skipRollingFrictionFrames = 30;
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
