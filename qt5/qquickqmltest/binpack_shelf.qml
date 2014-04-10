import QtQuick 2.2

Rectangle
{
    id: root;
    width:  600;
    height: 600;
    color: "#333333";

    Component {
        id: rect_component;
        Rectangle {
            width: 10;
            height: 20;
            color: "#666666";
        }
    }

    Rectangle {
        id: binpack_shelf;
        width: 512;
        height: 512;
        color: "#444444";
        anchors.centerIn: parent;

        Rectangle {
            id: binpack_placer;
            width:  5;
            height: 5;
            color: "#60c659";
            x: binpack_shelf.m_place_x;
            y: binpack_shelf.height-binpack_shelf.m_place_y-height;
            z: binpack_shelf.z+5;
        }

        Rectangle {
            id: binpack_shelfline;
            width: binpack_shelf.width;
            height: 2;
            color: "#008ed6";
            x: 0;
            y: binpack_shelf.height-binpack_shelf.m_shelf_y-height;
            z: binpack_shelf.z+5;
        }

        // The origin (0,0) is defined as the
        // bottom left corner of the bin
        property int m_place_x: 0;
        property int m_place_y: 0;
        property int m_shelf_y: 0;
        property int m_spacing: 1;

        property int m_bin_top: height;
        property int m_bin_right: width;

        function addRectangle(r_width, r_height)
        {
            var rect = {
                x: m_place_x+m_spacing,
                y: m_place_y+m_spacing,
                right: m_place_x+m_spacing+r_width,
                top: m_place_y+m_spacing+r_height
            };

            if(rect.top < m_bin_top) {
                if(rect.right < m_bin_right) {
                    // draw rectangle
                    drawRectangle(rect);

                    // advance place
                    m_place_x = rect.right;

                    // adjust shelf
                    if(m_shelf_y < rect.top) {
                        m_shelf_y = rect.top;
                    }
                }
                else { // try jumping up a shelf
                    m_place_x = 0;
                    m_place_y = m_shelf_y;
                    rect.x = m_place_x+m_spacing; // left
                    rect.y = m_place_y+m_spacing; // bottom
                    rect.right = rect.x+r_width;
                    rect.top = rect.y+r_height;

                    if(rect.top < m_bin_top) {
                        if(rect.right < m_bin_right) {
                            // draw rectangle
                            drawRectangle(rect);

                            // advance place
                            m_place_x = rect.right;

                            // adjust shelf
                            if(m_shelf_y < rect.top) {
                                m_shelf_y = rect.top;
                            }
                        }
                    }
                }
            }
            return false;
        }

        function drawRectangle(rect)
        {
            var rect_width = rect.right-rect.x;
            var rect_height = rect.top-rect.y;

            // adjust y so that the origin (0,0)
            // is at the top left of binpack_shelf
            rect.y = m_bin_top-rect.top;

            var rect_obj = rect_component.createObject(binpack_shelf);
            rect_obj.x = rect.x;
            rect_obj.y = rect.y;
            rect_obj.width = rect_width;
            rect_obj.height = rect_height;
        }

        MouseArea {
            anchors.fill: parent;
            enabled: true;
            onClicked: {
                var w = Math.floor((Math.random()*60)+30);
                var h = Math.floor((Math.random()*60)+30);
                binpack_shelf.addRectangle(w,h);
            }
        }
    }
}
