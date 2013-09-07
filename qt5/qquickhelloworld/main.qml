import QtQuick 2.0

Rectangle
{
    id: root;
    color: "#333333";

//    width: 500;
//    height: 500;

    property int time: 0;

    Timer {
        id: master_timer;
        interval: 16;
        running: true;
        repeat: true;

        onTriggered: {
            time += interval;
        }
    }

    function updateOutput()
    {
        var tnow = outputTimer.t;

        var t,s,t0,t1,s0,s1;

        // iA
        t0 = iA.t0; s0 = iA.s0;
        t1 = iA.t1; s1 = iA.s1;

        if(t0 >= 0) {
            t = (tnow-t1) + t0;
//            if(tnow-t1 < 0) { t = t0; }

            s = interpolate(t0,s0,t1,s1,t);
            output.x = s;
        }


        // iB
        t0 = iB.t0; s0 = iB.s0;
        t1 = iB.t1; s1 = iB.s1;

        if(t0 >= 0) {
            t = (tnow-t1) + t0;
            s = interpolate(t0,s0,t1,s1,t);
            output.rotation = s;
        }
    }

    function interpolate(t0,s0,t1,s1,t)
    {
        // WARN should I check (t1-t0 != 0) here?
        if(t > t1) {
            console.log("OVERSHOOT!");
            t = t1;
        }

        var s = (((s1-s0)/(t1-t0))*(t-t0)) + s0;


        return s;
    }

    Timer {
        id: outputTimer;
        interval: period;
        running: true;

        property int t: 0;        // elapsed ms
        property int period: 60;  // period in ms
        property int error: 0;    // max period error in ms

        onTriggered: {
            t = time;
            updateOutput();
            start();
        }
    }

    // INPUT A
    Rectangle {
        id: iA;
        height: 30;
        width:  30;
        color: "#aadd6d";
        y: 150;

        property real xmin: 50;
        property real xmax: root.width - 50;

        property int t0: -1; property real s0: 0;
        property int t1: -1; property real s1: 0;

        Timer {
            id: iATimer;
            interval: period;
            running: true;

            property int period: 1000;
            property int error: 0;

            onTriggered: {
                iA.s0 = iA.s1;
                iA.s1 = iA.xmin + (Math.random() * (iA.xmax-iA.xmin));
                iA.x = iA.s1;

                iA.t0 = iA.t1;
                iA.t1 = time;
//                var it = period-500;
//                if(it > 500) { period = it; }

                start();
            }
        }
    }

    // PREVIOUS INPUT A
    Rectangle {
        id: prev_iA;
        height: 30;
        width: 30;
        color: "#aadd6d";
        opacity: 0.25;
        y: iA.y;
        x: iA.s0;
    }

    // INPUT B
    Rectangle {
        id: iB;
        height: 30;
        width:  30;
        color: "#9e4fa5";
        x: iA.s1;
        y: iA.y + 50;

        property int t0: -1; property real s0: 0;
        property int t1: -1; property real s1: 0;

        Timer {
            id: iBTimer;
            interval: period;
            running: true;

            property int period: 60;
            property int error: 0;

            onTriggered: {
                iB.s0 = iB.s1;
                iB.s1 += 5.0;
                iB.rotation = iB.s1;

                iB.t0 = iB.t1;
                iB.t1 = time;
                start();
            }
        }
    }

    // OUTPUT
    Rectangle {
        id: output;
        height: 30;
        width:  30;
        color: "#00c993";
        x: (root.width-width)/2;
        y: iB.y + 50;
    }


    Text {
        id: debugText0;
        x:10;
        y:10;
        height:40;
        width: parent.width;
        font.pixelSize: 20;
        font.family: "Droid Sans";
        color: "#00a3dd";
        text: "time: " + time;
    }
//    Text {
//        id: debugText1;
//        x:10;
//        y:40+10;
//        height:40;
//        width: parent.width;
//        font.pixelSize: 20;
//        font.family: "Droid Sans";
//        text: "input A interval: " + iATimer.interval;
//        color: "white";
//    }
}
