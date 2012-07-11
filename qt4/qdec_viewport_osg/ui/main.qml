import QtQuick 1.1
import ViewportItems 1.0
import TouchItems 1.0

Rectangle
{
    width: 1024;
    height: 600;
    color: "gray";

    Rectangle
    {
        id: redtangle;
        width: parent.width*0.2;
        height: parent.height*0.4;
        color: "red";

        anchors.verticalCenter: parent.verticalCenter;
        anchors.left:parent.left;

//        RotationAnimation on rotation {
//                 loops: Animation.Infinite
//                 from: 0
//                 to: 360
//                 duration: 3000;
//             }
    }

    ViewportItem
    {
        id: viewportItem;
        height: parent.height*0.5;
        width: parent.width*0.5;
        anchors.bottom: parent.bottom;
    }

    MouseArea
    {
        id: mouseArea;
        enabled: true;
        height: parent.height*0.5;
        width: parent.width*0.5;
        anchors.bottom: parent.bottom;
        acceptedButtons: Qt.LeftButton | Qt.RightButton | Qt.MiddleButton;

        onPressed:  {
            viewportItem.onMousePressed(mouse.x,mouse.y,mouse.button);
        }

        onPositionChanged:  {
            viewportItem.onMouseMoved(mouse.x,mouse.y,mouse.button);
        }

        onReleased:  {
            viewportItem.onMouseReleased(mouse.x,mouse.y,mouse.button);
        }
    }

    Rectangle
    {
        id: bluetangle;
        width: parent.width*0.2;
        height: parent.height*0.4;
        color: "blue";

        anchors.verticalCenter: parent.verticalCenter;
        anchors.left:redtangle.right;
    }
}


