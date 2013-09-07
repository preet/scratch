

import QtQuick 2.0

Rectangle {
    id:root;
    width: 1024;
    height: 600;
    color: "pink";

    Row   {
        id: row;
        anchors.fill: parent;
        Rectangle   {
            id: blue;
            width:parent.width/2;
            height: parent.height;
            color: "steelblue";

        }
        Rectangle   {
            id: purple;
            width:parent.width/2;
            height: parent.height;
            gradient: Gradient {
                    GradientStop { position: 0.0; color: "purple" }
                    GradientStop { position: 1.0; color: "orange" }
            }

            ShaderEffect {
                id: sh;
                height: 300;
                width: parent.width;
                anchors.verticalCenter: parent.verticalCenter;
                property variant source: sfx;

//                fragmentShader:
//                    "uniform sampler2D source;"+
//                    "varying highp vec2 qt_TexCoord0;"+
//                    "void main() {"+
//                    "    gl_FragColor = texture2D(source,qt_TexCoord0);"+
//                    "}";

                property real amplitude: 0.01;
                property real frequency: 6;
                property real time: 0
                NumberAnimation on time { loops: Animation.Infinite; from: 0; to: Math.PI * 2; duration: 600 }
                fragmentShader:
                    "uniform lowp float qt_Opacity;" +
                    "uniform highp float amplitude;" +
                    "uniform highp float frequency;" +
                    "uniform highp float time;" +
                    "uniform sampler2D source;" +
                    "varying highp vec2 qt_TexCoord0;" +
                    "void main() {" +
                    "    highp vec2 p = sin(time + frequency * qt_TexCoord0);" +
                    "    gl_FragColor = texture2D(source, qt_TexCoord0 + amplitude * vec2(p.y, -p.x)) * qt_Opacity;" +
                    "}"
            }
        }
    }

    Rectangle   {
        id: button;
        width: 200;
        height: 100;
        color: "yellow";

        MouseArea   {
            anchors.fill: parent;
            onClicked:   {
                sfx.scheduleUpdate();
            }
        }
    }

    ShaderEffectSource   {
        anchors.fill: parent;
        id: sfx;
        visible: false;
        mipmap: false;
        live: false;
        recursive: true;
        sourceItem: root;
    }


}
