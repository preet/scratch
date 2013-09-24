import QtQuick 2.0

Rectangle
{
    id: root;
    color: "#333333";

    width:  500;
    height: 500;

//    Image  {
//        id: i1;
//        width:  parent.width * 0.75;
//        height: width;

//        anchors.horizontalCenter: parent.horizontalCenter;
//        anchors.verticalCenter: parent.verticalCenter;

//        source: "qrc:/img.png";
//    }

    Connections   {
//        target: GeoLocation;

//        onLocationChanged:   {

//            var pTime = utc_time;
//            var pLon = lon;
//            var pLat = lat;

//            tlocation.text =
//                    pTime + "," +
//                    pLon.toString() + "," +
//                    pLat.toString();
//        }

//        onStatusChanged:   {
//            tstatus.text = status;
//        }

//        onProviderEnabled:   {
//            tproviderEnabled.text = provider + " was enabled";
//        }

//        onProviderDisabled:   {
//            tproviderDisabled.text = provider + " was disabled";
//        }

        target: BluetoothSerial;

        onBluetoothAdapterStatusChanged:   {
            if(status)   {
                tbluetoothstatus.text = "ON";
            }
            else   {
                tbluetoothstatus.text = "OFF";
            }
        }
    }

    Column   {
        y:20;
        width:  parent.width;
        height: parent.height-20;
        spacing: 20;

        property real fontpixelsize: root.height/40;
        property real textwidth: root.width-40;

        Text {
            id: tbluetoothstatus;
            x:20;
            width: parent.textwidth;
            wrapMode: Text.WrapAnywhere;
            font.pixelSize: parent.fontpixelsize;
            font.bold: true;
            color: "#1eb53a";
            text: "unknown";
        }

//        Text {
//            id: tlocation;
//            x:20;
//            width: parent.textwidth;
//            wrapMode: Text.WrapAnywhere;
//            font.pixelSize: parent.fontpixelsize;
//            font.bold: true;
//            color: "#1eb53a";
//            text: "location";
//        }
//        Text {
//            id: tstatus;
//            x:20;
//            width: parent.textwidth;
//            wrapMode: Text.WrapAnywhere;
//            font.pixelSize: parent.fontpixelsize;
//            font.bold: true;
//            color: "#1eb53a";
//            text: "status";
//        }
//        Text {
//            id: tproviderEnabled;
//            x:20;
//            width: parent.textwidth;
//            wrapMode: Text.WrapAnywhere;
//            font.pixelSize: parent.fontpixelsize;
//            font.bold: true;
//            color: "#1eb53a";
//            text: "providerEnabled";
//        }
//        Text {
//            id: tproviderDisabled;
//            x:20;
//            width: parent.textwidth;
//            wrapMode: Text.WrapAnywhere;
//            font.pixelSize: parent.fontpixelsize;
//            font.bold: true;
//            color: "#1eb53a";
//            text: "providerDisabled";
//        }
//        Text {
//            x:20;
//            width: parent.textwidth;
//            wrapMode: Text.WrapAnywhere;
//            font.pixelSize: parent.fontpixelsize;
//            font.bold: true;
//            color: "#5bbf21";
//            text: Helper.getPathUi();
//        }
//        Text {
//            x:20;
//            width: parent.textwidth;
//            wrapMode: Text.WrapAnywhere;
//            font.pixelSize: parent.fontpixelsize;
//            font.bold: true;
//            color: "#8cd600";
//            text: Helper.getPathPlugins();
//        }
//        Text {
//            x:20;
//            width: parent.textwidth;
//            wrapMode: Text.WrapAnywhere;
//            font.pixelSize: parent.fontpixelsize;
//            font.bold: true;
//            color: "#bad80a";
//            text: Helper.getPathData();
//        }
//        Text {
//            x:20;
//            width: parent.textwidth;
//            wrapMode: Text.WrapAnywhere;
//            font.pixelSize: parent.fontpixelsize;
//            font.bold: true;
//            color: "#cee007";
//            text: Helper.getPathLogs();
//        }
//        Text {
//            x:20;
//            width: parent.textwidth;
//            wrapMode: Text.WrapAnywhere;
//            font.pixelSize: parent.fontpixelsize;
//            font.bold: true;
//            color: "#e0e20c";
//            text: Helper.getPathSettings();
//        }
//        Text {
//            x:20;
//            width: parent.textwidth;
//            wrapMode: Text.WrapAnywhere;
//            font.pixelSize: parent.fontpixelsize;
//            font.bold: true;
//            color: "#e8dd11";
//            text: Helper.getPathUser();
//        }
//        Text {
//            id: quote;
//            x:20;
//            width: parent.textwidth;
//            wrapMode: Text.WrapAnywhere;
//            font.pixelSize: parent.fontpixelsize;
//            font.bold: true;
//            color: "#00c993";
//        }
//        Rectangle   {
//            x: 20;
//            height: root.height/10;
//            width:  height;
//            color: "white";
//            MouseArea   {
//                anchors.fill: parent;
//                onClicked:    {
//                    quote.text = Helper.getQuote();
//                }
//            }
//        }
    }
}
