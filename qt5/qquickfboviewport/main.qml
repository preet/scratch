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

        // flip-y
        // BUG: disable in Qt 5.1+
        transform: Scale   {
            origin.y: renderer.height/2;
            yScale: -1.0;
        }
    }
}
