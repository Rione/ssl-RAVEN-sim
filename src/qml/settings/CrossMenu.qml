import QtQuick

Item {
    x: rightShadowRect.width - 60
    y: 48
    width: 52
    height: 52

    Canvas {
        anchors.fill: parent

        onPaint: {
            var ctx = getContext("2d");
            ctx.clearRect(0, 0, width, height);

            ctx.strokeStyle = "rgba(255, 255, 255, 0.9)";
            ctx.lineWidth = 4;
            ctx.lineCap = "round"; // 丸い端

            ctx.beginPath();
            ctx.moveTo(16, 16);
            ctx.lineTo(width - 16, height - 16);
            ctx.moveTo(width - 16, 16);
            ctx.lineTo(16, height - 16);
            ctx.stroke();
        }
    }

    MouseArea {
        anchors.fill: parent
        onClicked: {
            leftAnim.running = false;
            rightAnim.running = false;
            leftReverseAnim.running = true;
            rightReverseAnim.running = true;
        }
    }
}