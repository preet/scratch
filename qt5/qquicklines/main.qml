import QtQuick 2.1
import QtQuickLines 1.0

Rectangle
{
    id: root;
    width:  800;
    height: 480;
    color: "#333333";

    Line {
        id: line;
        anchors.fill: parent;
        p1: Qt.point(drag_p1.x+drag_p1.width*0.5,
                     drag_p1.y+drag_p1.height*0.5);

        p2: Qt.point(drag_p2.x+drag_p2.width*0.5,
                     drag_p2.y+drag_p2.height*0.5);
        thickness: 10;
        color: "#71C837";
    }

    SequentialAnimation {
        running: true;
        loops: Animation.Infinite;
        // Why doesn't ColorAnimation work here?
        PropertyAnimation {
            target: line;
            property: "color";
            from: "#71C837";
            to: "#5FD3BC";
            duration: 500;
        }
        PropertyAnimation {
            target: line;
            property: "color";
            from: "#5FD3BC";
            to: "#71C837";
            duration: 500;
        }
    }

    Rectangle {
        id: drag_p1;
        x: 200; y: 100;
        width:  25;
        height: 25;
        color: "#666666";
        MouseArea {
            anchors.fill: parent;
            drag.target: parent;
            drag.axis: Drag.XandYAxis;
            drag.minimumX: 0;
            drag.maximumX: root.width-parent.width;
            drag.minimumY: 0;
            drag.maximumY: root.height-parent.height;
        }
    }

    Rectangle {
        id: drag_p2;
        x: 600; y: 300;
        width:  25;
        height: 25;
        color: "#666666";
        MouseArea {
            anchors.fill: parent;
            drag.target: parent;
            drag.axis: Drag.XandYAxis;
            drag.minimumX: 0;
            drag.maximumX: root.width-parent.width;
            drag.minimumY: 0;
            drag.maximumY: root.height-parent.height;
        }
    }
}
