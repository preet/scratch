import QtQuick 2.0

Rectangle   {
    id: root;
    width:  320;
    height: 480;
    gradient: Gradient {
        GradientStop { position: 0.0; color: "#212729" }
        GradientStop { position: 1.0; color: "#111414" }
    }

    property real margins:  20;
    property real fontsize: 12;

    FontLoader { id: robotomed; source: "fonts/Roboto-Medium.ttf"; }

    Connections   {
        target: Helper;
    }

    Component.onCompleted: {
        console.log("QAbstractListModel property:"+Helper.getSourceModel().count);
        console.log("Some other QObject property:"+Helper.message);
    }

    Rectangle   {
        color: "#00000000";
        border.width: 1;
        border.color: "#444f51";
        height: parent.height-margins;
        width:  parent.width-margins;
        anchors.centerIn: parent;
        clip: true;

        Column   {
            width:  parent.width-margins;
            height: parent.height-margins;
            spacing: margins/2;
            anchors.centerIn: parent;

            Text   {
                id: searchLabel;
                width: parent.width-margins;
                height: contentHeight;
                color: "#666d70";
                font.weight: Font.Light;
                font.family: robotomed.name;
                font.pointSize: root.fontsize;
                verticalAlignment: Text.AlignVCenter;
                wrapMode: Text.Wrap;
                text: "Input Search Text";
            }

            Rectangle   {
                id: searchTextInputBg;
                color:  "#77000000";
                width: parent.width-margins;
                height: searchLabel.height*2;
                radius: height/4;

                TextInput   {
                    id: searchTextInput;
                    width: parent.width-margins;
                    height: parent.height;
                    verticalAlignment: Text.AlignVCenter;
                    clip: true;

                    font.family: robotomed.name;
                    font.weight: Font.DemiBold;
                    font.pointSize: root.fontsize;

                    color: "#60dd49";
                    selectedTextColor: "#FFFFFF";
                    selectionColor:color;

                    anchors.horizontalCenter: parent.horizontalCenter;

                    onTextChanged:   {
                        Helper.searchFilterChanged(text);
                    }
                }
            }

            ListView   {
                id: searchResultsView;
                height: parent.height-searchLabel.height-searchTextInputBg.height;
                width:  parent.width-margins;
                model: SearchResultsModel;
                spacing: margins/2;
                clip: true;
                delegate: searchResult;

                Component   {
                    id: searchResult;
                    Rectangle   {
                        height: root.height/10;
                        width:  searchResultsView.width-margins;
                        color: "#00000000";
                        border.width: 1;
                        border.color: "#60dd49";
                        Text   {
                            x:margins;
                            id: resultName;
                            width: parent.width;
                            height: parent.height;
                            color: "#60dd49";
                            font.weight: Font.Light;
                            font.family: robotomed.name;
                            font.pointSize: root.fontsize*0.85;
                            verticalAlignment: Text.AlignVCenter;
                            wrapMode: Text.Wrap;
                            text: index + ": " + name + ": " + Math.round(dist_km) + "km";
                        }
                    }
                }
            }
        }
    }

    ListModel   {
        id: tempModel;
        ListElement   {
            name: "Hello World";
            dist_km: 100; // km
        }
    }
}
