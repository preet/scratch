//import QtQuick 2.0

//Rectangle {
//    id: root;
//    width: 1024;
//    height: 600;
//    color: "blue";

//    Rectangle   {
//        id: rect;
//        width: parent.width-100;
//        height: parent.height;
//        gradient: Gradient {
//            GradientStop { position: 0; color: "black" }
//            GradientStop { position: 1; color: "orange" }
//        }

//        Component.onCompleted: {
//            sfs.sourceItem = rect;
//            sfs.scheduleUpdate();
//        }
//    }

//    Rectangle   {
//        x: rect.width;
//        y: 100;
//        width: 100; height: 100
//        color: "purple";
//        MouseArea   {
//            anchors.fill: parent;
//            onClicked:   {
//                //console.log("start");
//                pageAnimation.start();
//            }
//        }
//    }

//    ShaderEffectSource {
//        id: sfs;
//        x: rect.width;
//        width: 100; height: 100
////        visible: false;
//        live: false;
//        recursive: false;

//        onScheduledUpdateCompleted:   {
//            ///console.log("done");
//            //sfs.sourceItem = null;
//        }
//    }

//    Rectangle   {
//        id: snrub;
//        width: parent.width-100;
//        height: parent.height;
//        opacity: 1;
//        gradient: Gradient {
//            GradientStop { position: 0; color: "cyan" }
//            GradientStop { position: 1; color: "red" }
//        }
////        visible: false;
//    }

//    ShaderEffect   {
//        id: pageTransition;
//        width: parent.width-100;
//        height: parent.height;
//        visible: false;

//        //
//        property variant source: sfs;
//        property real dist: 1;

//        vertexShader: "
//            uniform highp mat4 qt_Matrix;
//            uniform highp float width;
//            uniform highp float dist;
//            attribute highp vec4 qt_Vertex;
//            attribute highp vec2 qt_MultiTexCoord0;
//            varying highp vec2 texcoord;
//            varying lowp float opacity;
//            void main() {
//                opacity = 1.0-dist;
//                texcoord = qt_MultiTexCoord0;
//                highp vec4 xlated_Vertex = vec4(dist*width,0,0,0)+qt_Vertex;
//                gl_Position = qt_Matrix * xlated_Vertex;
//            }";

//        fragmentShader: "
//            uniform sampler2D source;
//            varying highp vec2 texcoord;
//            varying lowp float opacity;
//            void main() {
//                // note! explicitly specify float precision
//                lowp vec4 color = texture2D(source,texcoord)*(opacity);
//                gl_FragColor = color;
//            }";
//    }

//    SequentialAnimation   {
//        id: pageAnimation;
//        running: false;
//        alwaysRunToEnd: true;
//        PropertyAction   {
//            target: pageTransition;
//            property: "visible";
//            value: true;
//        }
//        PropertyAnimation   {
//            target: pageTransition;
//            property: "dist";
//            from: 1; to: 0; duration: 500;
//            easing.type: Easing.InOutQuad;
//        }
////        PropertyAction   {
////            target: pageTransition;
////            property: "visible";
////            value: false;
////        }
//    }
//}


import QtQuick 2.0

import ViewportItems 1.0

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
            ViewportItem {
                id: renderer
                width: parent.width;
                height: 400;
                anchors.verticalCenter: parent.verticalCenter;
            }
        }
        Rectangle   {
            id: purple;
            width:parent.width/2;
            height: parent.height;
            color: "purple";

            Image   {
                id: placeholder;
                height: 300;
                width: parent.width;
                source: "snrub.png";
                anchors.verticalCenter: parent.verticalCenter;
            }

            ShaderEffect {
                id:sh;
                height: 300;
                width: parent.width;
                anchors.verticalCenter: parent.verticalCenter;
                property variant source: placeholder;

//                fragmentShader:
//                    "uniform sampler2D source;"+
//                    "varying highp vec2 qt_TexCoord0;"+
//                    "void main() {"+
//                    "    gl_FragColor = texture2D(source,qt_TexCoord0);"+
//                    "}";

                property real amplitude: 0.01;
                property real frequency: 12;
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
                sh.source = sfx;
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
