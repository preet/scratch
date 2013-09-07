#ifndef RINGZOOM_H
#define RINGZOOM_H

#include <QWidget>
#include <QAbstractSlider>
#include <QRegion>
#include <QRectF>


class RingZoom : public QAbstractSlider
{
    Q_OBJECT

public:
    RingZoom(QWidget *parent = 0);

protected:
    void resizeEvent(QResizeEvent *);
    void paintEvent(QPaintEvent *);
    void mouseReleaseEvent(QMouseEvent *);
    void mouseMoveEvent (QMouseEvent * event);

private:

    int _doInit;
    int _sliderMin;
    int _sliderMax;

    int _zradiusf;                   // zoom radius fixed
    int _zoradiusf;                  // zoom outer radius fixed
    int _ziradiusf;                  // zoom inner radius fixed

    int _zradius;                   // zoom radius
    int _zoradius;                  // zoom outer radius
    int _ziradius;                  // zoom inner radius
    int _zdist;

    int _zcx;                       // dial center x coordinate
    int _zcy;                       // dial center y coordinate

    int _zstatic;                   //remembers the zoom level when
                                    //the user releases mouse

    int _lkmx;                      // last known mouse x
    int _lkmy;                      // last known mouse y

    bool _draggedMarker;            // flag for if user dragged the marker
};




#endif // RINGZOOM_H
