import QtQuick 2.0
import ViewportItems 1.0

Rectangle {
    id:root;
    width: 1024;
    height: 600;
    color: "black";

    ViewportItem {
        id: renderer
        width: parent.width;
        height: parent.height;
    }
}
