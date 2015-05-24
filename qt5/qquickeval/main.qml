import QtQuick 2.0

Rectangle   {
    id: root;
    width:  320;
    height: 480;
    gradient: Gradient {
        GradientStop { position: 0.0; color: "#212729" }
        GradientStop { position: 1.0; color: "#111414" }
    }

    property real margins: 5;
    property real fontsize: 12;

    FontLoader { id: robotomed; source: "fonts/FiraSans-Medium.ttf"; }

    Connections   {
        target: Helper;
        onTestComplete:   {
            result.text = message;
        }
    }

    Rectangle   {
        id: main_rect;
        color: "#00000000";
        border.width: 1;
        border.color: "#444f51";
        height: parent.height - 4*margins;
        width:  parent.width - 4*margins;
        anchors.horizontalCenter: parent.horizontalCenter;
        anchors.verticalCenter: parent.verticalCenter;
        clip: true;

        Flow   {
            id: content;
            height: parent.height-4*margins;
            width:  parent.width-4*margins;
            anchors.horizontalCenter: parent.horizontalCenter;
            anchors.verticalCenter: parent.verticalCenter;

            TextButton   {
                id: i;
                width: parent.width-4*margins;
                label.text: "Run Test";
                onClicked:   {
                    Helper.runTest();
                }
            }

            Item   {
                id: spacer;
                width: parent.width-4*margins;
                height: 20;
            }

            Text   {
                id: result;
                width: parent.width-4*margins;
                height: contentHeight;
                color: "#666d70";
                font.weight: Font.Light;
                font.family: robotomed.name;
                font.pointSize: root.fontsize;
                verticalAlignment: Text.AlignVCenter;
                wrapMode: Text.Wrap;
                text: "result:";
            }
        }
    }
}
