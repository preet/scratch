import QtQuick 2.0
import ViewportItems 1.0

Rectangle {
    id:root;
    width: 854;
    height: 480;
    color: "black";

    ViewportItem {
        id: renderer
        width: parent.width;
        height: parent.height;
    }
}
