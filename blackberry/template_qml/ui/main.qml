import QtQuick 1.1
import Qt3D 1.0

Viewport {
    width: 1024;
    height: 600;
    Item3D {
        id: teapot;
        mesh: Mesh { source:"teapot.bez"; }
        effect: Effect {}
    }
}

//Rectangle
//{
//    id: layer_base;
//    width: 1024;
//    height: 600;
//    color: "#333333";   // dark gray

//    Column {
//        height:parent.height;
//        width: parent.width;
//        Item {
//            height: parent.width*0.2;
//            width: parent.width;
//            Text {
//                anchors.horizontalCenter: parent.horizontalCenter;
//                anchors.verticalCenter: parent.verticalCenter;
//                horizontalAlignment: Text.AlignHCenter;
//                text: "Hello World This is a QML Template!";
//                color:"#72E84A";    // lime green
//                font.pointSize: 24;
//                font.bold: true;
//            }
//        }
//        Item {
//            height: parent.width*0.8;
//            width: parent.width;
//        }
//    }
//}
