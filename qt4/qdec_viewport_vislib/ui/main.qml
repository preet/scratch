import QtQuick 1.1
import ViewportItems 1.0
import TouchItems 1.0

Rectangle
{
    width: 1024;
    height: 600;
    color: "black";

    Rectangle
    {
        width: parent.width*0.8;
        height: parent.height*0.8;
        gradient: Gradient {
            GradientStop { position: 0.0; color: "#FF6633" }
            GradientStop { position: 1.0; color: "#405091" }
        }
        anchors.verticalCenter: parent.verticalCenter;
        anchors.horizontalCenter: parent.horizontalCenter;

        ViewportItem
        {
            id: viewportItem;
            height: parent.height*0.6;
            width: parent.width*0.6;
            anchors.verticalCenter: parent.verticalCenter;
            anchors.horizontalCenter: parent.horizontalCenter;
        }
    }
}


