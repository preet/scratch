/*
   Copyright 2013 Preet Desai
 
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

import QtQuick 2.0

Rectangle
{
    id: root;
    color: "#333333";

    width: 1024;
    height: 600;

    MultiPointTouchArea {

        id: touchArea;
        anchors.fill: parent;

        minimumTouchPoints: 0;
        maximumTouchPoints: 4;

        touchPoints:
        [
            TouchPoint { id: p0; },
            TouchPoint { id: p1; },
            TouchPoint { id: p2; },
            TouchPoint { id: p3; }
        ]

        property real panY: 0;
        property real relScale: 1.0;
        property real absScale: 1.0;
        property real relAngle: 0.0;
        property real absAngle: 0.0;
        property int touchCount: Number(p0.pressed) +
                                 Number(p1.pressed) +
                                 Number(p2.pressed) +
                                 Number(p3.pressed);




        function dot(x1,y1,x2,y2)
        {
            return ((x1*x2)+(y1*y2));
        }

        function perpdot(x1,y1,x2,y2)
        {   // perpendicular dot product operator
            // http://mathworld.wolfram.com/PerpDotProduct.html
            // http://johnblackburne.blogspot.ca/2012/02/...
            // ...perp-dot-product.html
            return ((x1*y2)-(y1*x2));
        }

        function angleBetween(x1,y1,x2,y2)
        {   // returns the min angle between this
            // vector and otherVec (range is +/- pi)
            // http://johnblackburne.blogspot.ca/2012/01/...
            // ...angle-between-two-vectors.html

            // the angle is positive for CCW
            var ndot = dot(x1,y1,x2,y2);
            var pdot = perpdot(x1,y1,x2,y2);
            return Math.atan2(pdot,ndot) * 180.0/Math.PI;
        }

        // for swipe detection
        property var p0YSamples: [];
        property var p0XSamples: [];
        property var p1YSamples: [];
        property var p1XSamples: [];
        property bool isSwipe: false;
        property int swipeNumSamples: 10;
        property real swipeAngleSideMax: 30;
        property real swipeAngleThreshold: 15;

        function updateScaleAndRotation()
        {
            var deltaPrevX = p0.previousX-p1.previousX;
            var deltaPrevY = p0.previousY-p1.previousY;
            var deltaNextX = p0.x-p1.x;
            var deltaNextY = p0.y-p1.y;

            // update scale
            var distPrev = Math.sqrt((deltaPrevX*deltaPrevX)+
                                     (deltaPrevY*deltaPrevY));

            var distNext = Math.sqrt((deltaNextX*deltaNextX)+
                                     (deltaNextY*deltaNextY));

            if(distPrev != 0 && distNext != 0)   {
                var scale = (distNext/distPrev) - 1.0;

                // dampen jitter on scaling
                if((scale)*(relScale-1.0) < 0)
                {   // scaling up v scaling down
                    scale *= 0.01;
                }
                relScale = scale+1;

                // clamp spurious results
                if(relScale < 0.95)        {   relScale = 0.95;   }
                else if(relScale > 1.05)   {   relScale = 1.05;   }

                absScale = absScale * relScale;
            }

            // update angle
            var angle = angleBetween(deltaPrevX,deltaPrevY,
                                     deltaNextX,deltaNextY);

            // dampen jitter on rotation
            if(angle*relAngle < 0)
            {   // one is positive/other is negative
                angle *=0.01;
            }

            relAngle = angle;
            absAngle += relAngle;

            if(absAngle > 360)       {   absAngle -= 360;   }
            else if(absAngle < -360) {   absAngle += 360;   }
        }

        onPressed:   {
            if(touchCount === 2)   {
                relScale = 1.0;
                p0XSamples.push(p0.x);
                p1XSamples.push(p1.x);
                p0YSamples.push(p0.y);
                p1YSamples.push(p1.y);
            }
        }

        onUpdated:   {
            if(touchCount === 2)   {

                if(p0YSamples.length < swipeNumSamples)
                {
                    p0XSamples.push(p0.x);
                    p1XSamples.push(p1.x);
                    p0YSamples.push(p0.y);
                    p1YSamples.push(p1.y);
                    return;
                }
                else if(p0YSamples.length === swipeNumSamples)
                {
                    p0XSamples.push(p0.x);
                    p1XSamples.push(p1.x);
                    p0YSamples.push(p0.y);
                    p1YSamples.push(p1.y);

                    // determine if this gesture is a swipe

                    // 1. ensure that the end of the gesture
                    //    shows movement in the same vertical
                    //    direction
                    var deltaDiffY0 = p0YSamples[0]-p0YSamples[swipeNumSamples];
                    var deltaDiffY1 = p1YSamples[0]-p1YSamples[swipeNumSamples];
                    if(deltaDiffY0*deltaDiffY1 < 0)   {
                        // if deltaDiff0 and deltaDiff1 have different
                        // signs, assume this isn't a swipe
                        isSwipe = false;
                        return;
                    }

                    // 2. ensure that the gesture has two touch
                    //    points that are roughly side by side
                    var deltaX0 = p0XSamples[0]-p1XSamples[0];
                    var deltaY0 = p0YSamples[0]-p1YSamples[0];
                    var length0 = Math.sqrt((deltaX0*deltaX0) + (deltaY0*deltaY0));
                    var angle0 = angleBetween(deltaX0,deltaY0,length0,0.0);
                    if(angle0 < 0) { angle0+= 360; }

                    var lim1 = swipeAngleSideMax;
                    var lim2 = 180-lim1;
                    var lim3 = 180+lim1;
                    var lim4 = 360-lim1;

                    if(((angle0 > lim1) && (angle0 < lim2)) ||
                       ((angle0 > lim3) && (angle0 < lim4)))
                    {   // the angle between the two input
                        // points is out of range
                        isSwipe = false;
                        return;
                    }

                    // 3. check that the angle between the first and
                    //    last set of corresponding points is below
                    //    a certain threshold
                    for(var i=1; i < p0YSamples.length; i++)   {
                        var deltaX = p0XSamples[i]-p1XSamples[i];
                        var deltaY = p0YSamples[i]-p1YSamples[i];
                        var diffAngle = angleBetween(deltaX0,deltaY0,deltaX,deltaY);
                        debugText1.text = "!" + (diffAngle);
                        if(Math.abs(diffAngle) > swipeAngleThreshold)   {
                            isSwipe = false;
                            return;
                        }
                    }
                    isSwipe = true;
                }
                else
                {
                    if(isSwipe)   {
                        // update y
                        panY += ((p0.y-p0.previousY) + (p1.y-p1.previousY))/2.0;
                    }
                    else   {
                        updateScaleAndRotation();
                    }
                }
            }
        }

        onReleased: {
            debugText0.text = "tp0: ";
            debugText1.text = "tp1: ";

            p0XSamples = [];
            p1XSamples = [];
            p0YSamples = [];
            p1YSamples = [];
        }
    }

    Text {
        id: debugTextCount;
        height:50;
        width: parent.width;
        font.pixelSize: 35;
        text: "isSwipe: " + touchArea.isSwipe;
        color: "white";
    }
    Text {
        id: debugText0;
        height:50;
        width: parent.width;
        font.pixelSize: 35;
        text: "tp0";
        y: 50;
        color: "white";
    }
    Text {
        id: debugText1;
        height:50;
        width: parent.width;
        font.pixelSize: 35;
        y: 100;
        text: "tp1";
        color: "white";
    }
    Text {
        id: debugTextAbsScale;
        height:50;
        width: parent.width;
        font.pixelSize: 35;
        y: 150;
        text: "abs scale: " + Math.round(touchArea.absScale*100)/100;
        color: "white";
    }
    Text {
        id: debugTextRelScale;
        height:50;
        width: parent.width;
        font.pixelSize: 35;
        y: 200;
        text: "rel scale: " + Math.round(touchArea.relScale*100)/100;
        color: "white";
    }
    Text {
        id: debugTextAbsAngle;
        height:50;
        width: parent.width;
        font.pixelSize: 35;
        y: 250;
        text: "abs angle: " + Math.round(touchArea.absAngle*100)/100;
        color: "white";
    }
    Text {
        id: debugTextRelAngle;
        height:50;
        width: parent.width;
        font.pixelSize: 35;
        y: 300;
        text: "rel angle: " + Math.round(touchArea.relAngle*100)/100;
        color: "white";
    }
    Text {
        id: debugTextPanY;
        height:50;
        width: parent.width;
        font.pixelSize: 35;
        y: 350;
        text: "pan y: " + Math.round(touchArea.panY*100)/100;
        color: "white";
    }
    Rectangle {
        id: rect0;
        height: 200 * touchArea.absScale;
        width: 200 * touchArea.absScale;
        color: "#AC58FA";
//        x: point0.x-width/2;
//        y: point0.y-height/2;
        x: (parent.width-rect0.width)/2;
        y: (parent.height-rect0.height)/2 + touchArea.panY;
        rotation: touchArea.absAngle;
    }
}
