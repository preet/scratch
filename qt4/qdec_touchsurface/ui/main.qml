import QtQuick 1.1
import TouchItems 1.0

Rectangle
{
    id: layer_base;
    width: 1024;
    height: 600;
    color: "#333333";   // dark gray

    property real touchScale: 1.0;
    property real touchAngle: 0.0;

    Rectangle  {
        id: cRect;
        anchors.horizontalCenter: parent.horizontalCenter;
        anchors.verticalCenter: parent.verticalCenter;
        color: "#555555";
        height: 200 * touchScale;
        width: height;
        rotation: touchAngle;
    }

    TouchArea {
        id:touchArea
        anchors.fill: parent

        onScaleFactorChanged: {
            layer_base.touchScale = totalScaleFactor;
//            console.log("Current scale factor:",scaleFactor);
        }

        onRotationAngleChanged: {
            layer_base.touchAngle = totalRotationAngle;
//            console.log("Current rotation angle:",rotationAngle);
        }

        onTouchPointPressed: {
            if(touchPtId == 0)  {
                rect0.x = touchPoint0.x - rect0.width/2;
                rect0.y = touchPoint0.y - rect0.height/2;
                rect0.opacity = 1.0;
            }
            else if(touchPtId == 1)  {
                rect1.x = touchPoint1.x - rect1.width/2;
                rect1.y = touchPoint1.y - rect1.height/2;
                rect1.opacity = 1.0;
            }
            else if(touchPtId == 2)  {
                rect2.x = touchPoint2.x - rect2.width/2;
                rect2.y = touchPoint2.y - rect2.height/2;
                rect2.opacity = 1.0;
            }
            else if(touchPtId == 3)  {
                rect3.x = touchPoint3.x - rect3.width/2;
                rect3.y = touchPoint3.y - rect3.height/2;
                rect3.opacity = 1.0;
            }
        }

        onTouchPointMoved: {
            if(touchPtId == 0)  {
                rect0.x = touchPoint0.x - rect0.width/2;
                rect0.y = touchPoint0.y - rect0.height/2;
                rect0.opacity = 1.0;
            }
            else if(touchPtId == 1)  {
                rect1.x = touchPoint1.x - rect1.width/2;
                rect1.y = touchPoint1.y - rect1.height/2;
                rect1.opacity = 1.0;
            }
            else if(touchPtId == 2)  {
                rect2.x = touchPoint2.x - rect2.width/2;
                rect2.y = touchPoint2.y - rect2.height/2;
                rect2.opacity = 1.0;
            }
            else if(touchPtId == 3)  {
                rect3.x = touchPoint3.x - rect3.width/2;
                rect3.y = touchPoint3.y - rect3.height/2;
                rect3.opacity = 1.0;
            }
        }

        onTouchPointReleased: {
            if(touchPtId == 0)  {
                rect0.x = touchPoint0.x - rect0.width/2;
                rect0.y = touchPoint0.y - rect0.height/2;
                rect0.opacity = 0.0;
            }
            else if(touchPtId == 1)  {
                rect1.x = touchPoint1.x - rect1.width/2;
                rect1.y = touchPoint1.y - rect1.height/2;
                rect1.opacity = 0.0;
            }
            else if(touchPtId == 2)  {
                rect2.x = touchPoint2.x - rect2.width/2;
                rect2.y = touchPoint2.y - rect2.height/2;
                rect2.opacity = 0.0;
            }
            else if(touchPtId == 3)  {
                rect3.x = touchPoint3.x - rect3.width/2;
                rect3.y = touchPoint3.y - rect3.height/2;
                rect3.opacity = 0.0;
            }
        }

//        onTouchPointChanged: {

//            if(touchPtId == 0)  {
////                console.log("Id:",touchPtId,"Active:",touchPoint0.isActive,"Pos:",touchPoint0.x,",",touchPoint0.y);
//                if(touchPoint0.isActive)  {
//                    rect0.x = touchPoint0.x - rect0.width/2;
//                    rect0.y = touchPoint0.y - rect0.height/2;
//                    rect0.opacity = 1.0;
//                }
//                else {
//                    rect0.opacity = 0.0;
//                }
//            }
//            else if(touchPtId == 1)  {
////                console.log("Id:",touchPtId,"Active:",touchPoint1.isActive,"Pos:",touchPoint1.x,",",touchPoint1.y);
//                if(touchPoint1.isActive)  {
//                    rect1.x = touchPoint1.x - rect1.width/2;
//                    rect1.y = touchPoint1.y - rect1.height/2;
//                    rect1.opacity = 1.0;
//                }
//                else {
//                    rect1.opacity = 0.0;
//                }
//            }
//            else if(touchPtId == 2)  {
////                console.log("Id:",touchPtId,"Active:",touchPoint2.isActive,"Pos:",touchPoint2.x,",",touchPoint2.y);
//                if(touchPoint2.isActive)  {
//                    rect2.x = touchPoint2.x - rect2.width/2;
//                    rect2.y = touchPoint2.y - rect2.height/2;
//                    rect2.opacity = 1.0;
//                }
//                else {
//                    rect2.opacity = 0.0;
//                }
//            }
//            else if(touchPtId == 3)  {
////                console.log("Id:",touchPtId,"Active:",touchPoint3.isActive,"Pos:",touchPoint3.x,",",touchPoint3.y);
//                if(touchPoint3.isActive)  {
//                    rect3.x = touchPoint3.x - rect3.width/2;
//                    rect3.y = touchPoint3.y - rect3.height/2;
//                    rect3.opacity = 1.0;
//                }
//                else {
//                    rect3.opacity = 0.0;
//                }
//            }
//            else if(touchPtId == 4)  {
////                console.log("Id:",touchPtId,"Active:",touchPoint4.isActive,"Pos:",touchPoint4.x,",",touchPoint4.y);
//                if(touchPoint4.isActive)  {
//                    rect4.x = touchPoint4.x - rect4.width/2;
//                    rect4.y = touchPoint4.y - rect4.height/2;
//                    rect4.opacity = 1.0;
//                }
//                else {
//                    rect4.opacity = 0.0;
//                }
//            }
//            else if(touchPtId == 5)  {
////                console.log("Id:",touchPtId,"Active:",touchPoint5.isActive,"Pos:",touchPoint5.x,",",touchPoint5.y);
//                if(touchPoint5.isActive)  {
//                    rect5.x = touchPoint5.x - rect5.width/2;
//                    rect5.y = touchPoint5.y - rect5.height/2;
//                    rect5.opacity = 1.0;
//                }
//                else {
//                    rect5.opacity = 0.0;
//                }
//            }
//            else if(touchPtId == 6)  {
////                console.log("Id:",touchPtId,"Active:",touchPoint6.isActive,"Pos:",touchPoint6.x,",",touchPoint6.y);
//                if(touchPoint6.isActive)  {
//                    rect6.x = touchPoint6.x - rect6.width/2;
//                    rect6.y = touchPoint6.y - rect6.height/2;
//                    rect6.opacity = 1.0;
//                }
//                else {
//                    rect6.opacity = 0.0;
//                }
//            }
//            else if(touchPtId == 7)  {
////                console.log("Id:",touchPtId,"Active:",touchPoint7.isActive,"Pos:",touchPoint7.x,",",touchPoint7.y);
//                if(touchPoint7.isActive)  {
//                    rect7.x = touchPoint7.x - rect7.width/2;
//                    rect7.y = touchPoint7.y - rect7.height/2;
//                    rect7.opacity = 1.0;
//                }
//                else {
//                    rect7.opacity = 0.0;
//                }
//            }
//            else if(touchPtId == 8)  {
////                console.log("Id:",touchPtId,"Active:",touchPoint8.isActive,"Pos:",touchPoint8.x,",",touchPoint8.y);
//                if(touchPoint8.isActive)  {
//                    rect8.x = touchPoint8.x - rect8.width/2;
//                    rect8.y = touchPoint8.y - rect8.height/2;
//                    rect8.opacity = 1.0;
//                }
//                else {
//                    rect8.opacity = 0.0;
//                }
//            }
//            else if(touchPtId == 9)  {
////                console.log("Id:",touchPtId,"Active:",touchPoint9.isActive,"Pos:",touchPoint9.x,",",touchPoint9.y);
//                if(touchPoint9.isActive)  {
//                    rect9.x = touchPoint9.x - rect9.width/2;
//                    rect9.y = touchPoint9.y - rect9.height/2;
//                    rect9.opacity = 1.0;
//                }
//                else {
//                    rect9.opacity = 0.0;
//                }
//            }
//        }
    }

    Rectangle {
        id: rect0;

        opacity: 0;
        height: 100;
        width: 100;
        color: "#AC58FA";
    }

    Rectangle {
        id: rect1;

        opacity: 0;
        height: 100;
        width: 100;
        color: "#5858FA";
    }

    Rectangle {
        id: rect2;

        opacity: 0;
        height: 100;
        width: 100;
        color: "#58ACFA";
    }

    Rectangle {
        id: rect3;

        opacity: 0;
        height: 100;
        width: 100;
        color: "#58FAF4";
    }

//    Rectangle {
//        id: rect4;

//        opacity: 0;
//        height: 100;
//        width: 100;
//        color: "#58FAAC";
//    }

//    Rectangle {
//        id: rect5;

//        opacity: 0;
//        height: 100;
//        width: 100;
//        color: "#58FA58";
//    }

//    Rectangle {
//        id: rect7;

//        opacity: 0;
//        height: 100;
//        width: 100;
//        color: "#ACFA58";
//    }

//    Rectangle {
//        id: rect8;

//        opacity: 0;
//        height: 100;
//        width: 100;
//        color: "#F4FA58";
//    }

//    Rectangle {
//        id: rect9;

//        opacity: 0;
//        height: 100;
//        width: 100;
//        color: "#FAAC58";
//    }

//    Rectangle {
//        id: rect10;

//        opacity: 0;
//        height: 100;
//        width: 100;
//        color: "#FA5858";
//    }
}
