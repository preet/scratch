import QtQuick 1.1
import ViewportItems 1.0

Rectangle
{
    width: 1024
    height: 600
    color: "gray";

    Rectangle
    {
        height: parent.height*0.8;
        width: parent.width*0.8;
        color: "brown";
        opacity: 0.5;
        anchors.horizontalCenter: parent.horizontalCenter;
        anchors.verticalCenter: parent.verticalCenter;

        ViewportItem
        {
            height: 512;
            width: 512;
            anchors.verticalCenter: parent.verticalCenter;
            anchors.horizontalCenter: parent.horizontalCenter;
        }
    }
}
