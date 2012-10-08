import QtQuick 1.1
import ViewportItems 1.0
import TouchItems 1.0

Rectangle
{
    width: 1024;
    height: 600;
    gradient: Gradient {
        GradientStop { position: 0.0; color: "#FF3366" }
        GradientStop { position: 1.0; color: "#405091" }
    }

    ViewportItem
    {
        id: viewportItem;
        height: parent.height*0.5;
        width: parent.width*0.5;
        anchors.fill: parent;
    }

    MouseArea
    {
        height:viewportItem;
        width:viewportItem;
        anchors.fill: parent;
    }

/*
    TouchArea {
        id: touchArea;
        height: parent.height*0.5;
        width: parent.width*0.5;
        anchors.bottom: parent.bottom;

        onScaleFactorChanged:  {
//            myRect.scale = totalScaleFactor;
            viewportItem.onCalcViewZoom(diffScaleFactor);
        }

        onRotationAngleChanged: {

        }

        onTouchPointPressed:  {
            if(touchPoint0.isActive && !touchPoint2.isActive && !touchPoint3.isActive)  {
                if(touchPoint1.isActive)  {
                    var av_x = (touchPoint0.x+touchPoint1.x)/2.0;
                    var av_y = (touchPoint0.y+touchPoint1.y)/2.0;

                    // pan (2 finger drag)
                    viewportItem.onBeginViewPan(av_x,av_y);
                }
                else  {
                    // rotate (1 finger drag)
                    viewportItem.onBeginViewRotate(touchPoint0.x,touchPoint0.y);
                }
            }
        }

        onTouchPointMoved:  {
            if(touchPoint0.isActive && !touchPoint2.isActive && !touchPoint3.isActive)  {
                if(touchPoint1.isActive)  {
                    var av_x = (touchPoint0.x+touchPoint1.x)/2.0;
                    var av_y = (touchPoint0.y+touchPoint1.y)/2.0;

                    // pan (2 finger drag)
                    viewportItem.onCalcViewPan(av_x,av_y);
                }
                else  {
                    // rotate (1 finger drag)
                    viewportItem.onCalcViewRotate(touchPoint0.x,touchPoint0.y);
                }
            }
        }
    }
        */
}
