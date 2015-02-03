import QtQuick 2.0

Item   {
    id: r;
    property alias label: label;
    signal clicked;

    height: label.height;

    Rectangle   {
        id: button;
        height: parent.height;
        width:  parent.width;
        border.width: 1;
        border.color: label.color;
        color: "#00000000";
        anchors.verticalCenter: parent.verticalCenter;
        anchors.horizontalCenter: parent.horizontalCenter;
    }

    Text   {
        id: label;
        width: parent.width;
        height: contentHeight*2;
        color: "#444f51";
        font.family: robotomed.name;
        font.weight: Font.DemiBold;
        font.pointSize: root.fontsize;
        verticalAlignment: Text.AlignVCenter;
        horizontalAlignment: Text.AlignHCenter;
        text: "Messages";
    }

    MouseArea
    {
        id: mousearea;
        anchors.fill: parent;
        enabled: true;
        onClicked:
        {  parent.clicked();  }
    }

    states:
    [
        State   {
            name: "onPressed";
            when: mousearea.pressed;
            PropertyChanges   {
                target: r;
//                color: qs.color_selected;
            }
            PropertyChanges   {
                target: label;
                color: "white";
            }
            PropertyChanges   {
                target: r;
                z: 5;
            }
        }
    ]
}
