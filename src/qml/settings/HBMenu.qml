import QtQuick

Item {
    id: menuContainer
    x: windowWidth - 60
    y: 48
    width: 52
    height: 52

    Rectangle {
        id: menuBackground
        anchors.fill: parent
        radius: 8
        color: "#1A1A1A"
        opacity: 0.85
    }

    Rectangle {
        id: upLine
        x: 11
        y: 16
        width: 30
        height: 5
        radius: 2.5
        color: "white"
        opacity: isMenuRunning ? 0.0 : 1.0
    }

    Rectangle {
        id: centerLine
        x: 11
        y: 24
        width: 30
        height: 5
        radius: 2.5
        color: "white"
        opacity: isMenuRunning ? 0.0 : 1.0
    }

    Rectangle {
        id: downLine
        x: 11
        y: 32
        width: 30
        height: 5
        radius: 2.5
        color: "white"
        opacity: isMenuRunning ? 0.0 : 1.0
    }

    MouseArea {
        anchors.fill: parent
        onClicked: {
            if (leftShadowRect.x === 0) {
                leftAnim.running = false;
                rightAnim.running = false;
                leftReverseAnim.running = true;
                rightReverseAnim.running = true;
            } else {
                leftAnim.running = true;
                rightAnim.running = true;
                leftReverseAnim.running = false;
                rightReverseAnim.running = false;
            }
        }
    }
}