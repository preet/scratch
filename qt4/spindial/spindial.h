#ifndef SPINDIAL_H
#define SPINDIAL_H

#include <QWidget>
#include <QAbstractSlider>
#include <QRectF>


class SpinDial : public QAbstractSlider
{
    Q_OBJECT

public:
    SpinDial(QWidget *parent = 0);

protected:
    void paintEvent(QPaintEvent *);
    void mouseMoveEvent (QMouseEvent * event);

private:

    int _sliderMin;
    int _sliderMax;

    int _doradius;                  // dial outer radius
    int _diradius;                  // dial inner radius
    int _dmradius;                  // dial middle radius
    int _dcx;                       // dial center x coordinate
    int _dcy;                       // dial center y coordinate

    double _mangle;                 // marker angle
    int _mradius;                   // marker radius
    double _mcx;                    // marker center x coordinate
    double _mcy;                    // marker center y coordinate

    int _lkmx;                      // last known mouse x
    int _lkmy;                      // last known mouse y

    bool _draggedMarker;            // flag for if user dragged the marker


};




#endif // SPINDIAL_H
