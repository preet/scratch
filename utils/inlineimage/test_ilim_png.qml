/*
   Copyright (C) 2014 Preet Desai (preet.desai@gmail.com)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

import QtQuick 2.2

Rectangle
{
    id: root;
    width:  600;
    height: 600;
    color: "#333333";

    Grid {
        width: 580;
        height: 580;

        columns: 4;
        spacing: 0;

        anchors.centerIn: parent;

        // ============================================================= //

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/1bitpaletted.png_r8";
                anchors.centerIn: parent;
            }
        }

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/1bitpaletted.png_rgb8";
                anchors.centerIn: parent;
            }
        }

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/1bitpaletted.png_rgba8";
                anchors.centerIn: parent;
            }
        }

        Item {
            height: 40;
            width: childrenRect.width;
            Text   {
                color: "white";
                height: parent.height;
                font.pointSize: 10;
                verticalAlignment: Text.AlignVCenter;
                text: "  <R8> <RGB8> <RGBA8>: 1 bit paletted";
            }
        }

        // ============================================================= //

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/2bitgrayscale.png_r8";
                anchors.centerIn: parent;
            }
        }

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/2bitgrayscale.png_rgb8";
                anchors.centerIn: parent;
            }
        }

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/2bitgrayscale.png_rgba8";
                anchors.centerIn: parent;
            }
        }

        Item {
            height: 40;
            width: childrenRect.width;
            Text   {
                color: "white";
                height: parent.height;
                font.pointSize: 10;
                verticalAlignment: Text.AlignVCenter;
                text: "  <R8> <RGB8> <RGBA8>: 2 bit grayscale";
            }
        }

        // ============================================================= //

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/2bitpaletted.png_r8";
                anchors.centerIn: parent;
            }
        }

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/2bitpaletted.png_rgb8";
                anchors.centerIn: parent;
            }
        }

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/2bitpaletted.png_rgba8";
                anchors.centerIn: parent;
            }
        }

        Item {
            height: 40;
            width: childrenRect.width;
            Text   {
                color: "white";
                height: parent.height;
                font.pointSize: 10;
                verticalAlignment: Text.AlignVCenter;
                text: "  <R8> <RGB8> <RGBA8>: 2bitpaletted";
            }
        }

        // ============================================================= //

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/4bitgrayscale.png_r8";
                anchors.centerIn: parent;
            }
        }

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/4bitgrayscale.png_rgb8";
                anchors.centerIn: parent;
            }
        }

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/4bitgrayscale.png_rgba8";
                anchors.centerIn: parent;
            }
        }

        Item {
            height: 40;
            width: childrenRect.width;
            Text   {
                color: "white";
                height: parent.height;
                font.pointSize: 10;
                verticalAlignment: Text.AlignVCenter;
                text: "  <R8> <RGB8> <RGBA8>: 4bitgrayscale";
            }
        }

        // ============================================================= //

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/4bitpaletted.png_r8";
                anchors.centerIn: parent;
            }
        }

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/4bitpaletted.png_rgb8";
                anchors.centerIn: parent;
            }
        }

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/4bitpaletted.png_rgba8";
                anchors.centerIn: parent;
            }
        }

        Item {
            height: 40;
            width: childrenRect.width;
            Text   {
                color: "white";
                height: parent.height;
                font.pointSize: 10;
                verticalAlignment: Text.AlignVCenter;
                text: "  <R8> <RGB8> <RGBA8>: 4bitpaletted";
            }
        }

        // ============================================================= //

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/8bitgrayscale.png_r8";
                anchors.centerIn: parent;
            }
        }

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/8bitgrayscale.png_rgb8";
                anchors.centerIn: parent;
            }
        }

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/8bitgrayscale.png_rgba8";
                anchors.centerIn: parent;
            }
        }

        Item {
            height: 40;
            width: childrenRect.width;
            Text   {
                color: "white";
                height: parent.height;
                font.pointSize: 10;
                verticalAlignment: Text.AlignVCenter;
                text: "  <R8> <RGB8> <RGBA8>: 8bitgrayscale";
            }
        }

        // ============================================================= //

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/8bitgrayscale_8bitalpha.png_r8";
                anchors.centerIn: parent;
            }
        }

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/8bitgrayscale_8bitalpha.png_rgb8";
                anchors.centerIn: parent;
            }
        }

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/8bitgrayscale_8bitalpha.png_rgba8";
                anchors.centerIn: parent;
            }
        }

        Item {
            height: 40;
            width: childrenRect.width;
            Text   {
                color: "white";
                height: parent.height;
                font.pointSize: 10;
                verticalAlignment: Text.AlignVCenter;
                text: "  <R8> <RGB8> <RGBA8>: 8bitgrayscale_8bitalpha";
            }
        }

        // ============================================================= //

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/8bitpaletted.png_r8";
                anchors.centerIn: parent;
            }
        }

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/8bitpaletted.png_rgb8";
                anchors.centerIn: parent;
            }
        }

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/8bitpaletted.png_rgba8";
                anchors.centerIn: parent;
            }
        }

        Item {
            height: 40;
            width: childrenRect.width;
            Text   {
                color: "white";
                height: parent.height;
                font.pointSize: 10;
                verticalAlignment: Text.AlignVCenter;
                text: "  <R8> <RGB8> <RGBA8>: 8bitpaletted";
            }
        }

        // ============================================================= //

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/8bitrgb.png_r8";
                anchors.centerIn: parent;
            }
        }

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/8bitrgb.png_rgb8";
                anchors.centerIn: parent;
            }
        }

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/8bitrgb.png_rgba8";
                anchors.centerIn: parent;
            }
        }

        Item {
            height: 40;
            width: childrenRect.width;
            Text   {
                color: "white";
                height: parent.height;
                font.pointSize: 10;
                verticalAlignment: Text.AlignVCenter;
                text: "  <R8> <RGB8> <RGBA8>: 8bitrgb";
            }
        }

        // ============================================================= //

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/8bitrgba.png_r8";
                anchors.centerIn: parent;
            }
        }

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/8bitrgba.png_rgb8";
                anchors.centerIn: parent;
            }
        }

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/8bitrgba.png_rgba8";
                anchors.centerIn: parent;
            }
        }

        Item {
            height: 40;
            width: childrenRect.width;
            Text   {
                color: "white";
                height: parent.height;
                font.pointSize: 10;
                verticalAlignment: Text.AlignVCenter;
                text: "  <R8> <RGB8> <RGBA8>: 8bitrgba";
            }
        }

        // ============================================================= //

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/16bitgrayscale.png_r8";
                anchors.centerIn: parent;
            }
        }

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/16bitgrayscale.png_rgb8";
                anchors.centerIn: parent;
            }
        }

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/16bitgrayscale.png_rgba8";
                anchors.centerIn: parent;
            }
        }

        Item {
            height: 40;
            width: childrenRect.width;
            Text   {
                color: "white";
                height: parent.height;
                font.pointSize: 10;
                verticalAlignment: Text.AlignVCenter;
                text: "  <R8> <RGB8> <RGBA8>: 16bitgrayscale";
            }
        }

        // ============================================================= //

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/16bitgrayscale_16bitalpha.png_r8";
                anchors.centerIn: parent;
            }
        }

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/16bitgrayscale_16bitalpha.png_rgb8";
                anchors.centerIn: parent;
            }
        }

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/16bitgrayscale_16bitalpha.png_rgba8";
                anchors.centerIn: parent;
            }
        }

        Item {
            height: 40;
            width: childrenRect.width;
            Text   {
                color: "white";
                height: parent.height;
                font.pointSize: 10;
                verticalAlignment: Text.AlignVCenter;
                text: "  <R8> <RGB8> <RGBA8>: 16bitgrayscale_16bitalpha";
            }
        }

        // ============================================================= //

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/16bitrgb.png_r8";
                anchors.centerIn: parent;
            }
        }

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/16bitrgb.png_rgb8";
                anchors.centerIn: parent;
            }
        }

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/16bitrgb.png_rgba8";
                anchors.centerIn: parent;
            }
        }

        Item {
            height: 40;
            width: childrenRect.width;
            Text   {
                color: "white";
                height: parent.height;
                font.pointSize: 10;
                verticalAlignment: Text.AlignVCenter;
                text: "  <R8> <RGB8> <RGBA8>: 16bitrgb";
            }
        }

        // ============================================================= //

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/16bitrgba.png_r8";
                anchors.centerIn: parent;
            }
        }

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/16bitrgba.png_rgb8";
                anchors.centerIn: parent;
            }
        }

        Rectangle {
            width: 40;
            height: 40;
            color: "#444444";

            Image {
                width: 32;
                height: 32;
                source: "image://test_png/16bitrgba.png_rgba8";
                anchors.centerIn: parent;
            }
        }

        Item {
            height: 40;
            width: childrenRect.width;
            Text   {
                color: "white";
                height: parent.height;
                font.pointSize: 10;
                verticalAlignment: Text.AlignVCenter;
                text: "  <R8> <RGB8> <RGBA8>: 16bitrgba";
            }
        }

        // ============================================================= //
    }
}
