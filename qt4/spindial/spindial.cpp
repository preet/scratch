#include <QtGui>
#include <iostream>
#include "spindial.h"


SpinDial::SpinDial(QWidget *parent)
    : QAbstractSlider(parent)
{
    //setPalette(Qt::transparent);
    _sliderMin = 0;
    _sliderMax = 250;

    this->setRange(_sliderMin, _sliderMax);
}

void SpinDial::paintEvent(QPaintEvent *)
{

    // grab dial sizes from the widget frame size
    // assume that the widget frame is square (lock this later)
    _doradius = (int)(((double)height())/2*0.9);
    _diradius = (int)(0.85*(double)_doradius);
    _dcx = (width())/2;
    _dcy = (height())/2;

    // draw the dial
    QPainter painter(this);
    QPoint center(_dcx,_dcy);
    //painter.drawEllipse(center,_doradius,_doradius);
    //painter.drawEllipse(center,_diradius,_diradius);

    // draw the marker
    _dmradius = (_doradius+_diradius)/2;
    _mangle = (((double)this->sliderPosition()/(double)_sliderMax) * 3.14159265 * -2);
    _mcx = (_dmradius*sin(_mangle)) + _dcx;
    _mcy = (_dmradius*cos(_mangle)) + _dcy;
    _mradius = _dmradius / 5;
    QPoint mcenter(_mcx,_mcy);

    painter.drawEllipse(mcenter,_mradius,_mradius);

    // set the masked region
    QRegion maskedRegion(_mcx-_mradius*1.25,_mcy-_mradius*1.25,_mradius*2.5,_mradius*2.5,QRegion::Ellipse);
    setMask(maskedRegion);

}


void SpinDial::mouseMoveEvent (QMouseEvent * event)
{

    // verify the mouse click occured in the marker
    // (approximated by a bounding box right now), else do nothing
    _draggedMarker = false;
    _lkmx = event->x();
    _lkmy = event->y();



    if ((event->buttons() & Qt::LeftButton))
    {
        if (((_lkmx > (_mcx-_mradius)) && (_lkmx < (_mcx+_mradius))) &&
                ((_lkmy > (_mcy-_mradius)) && (_lkmy < (_mcy+_mradius))))
        {


           // resolve the dragged mouse position into an angle around dial center
           double temp = atan(double(_dcx-_lkmx)/double(_dcy-_lkmy));

           // four quadrants for atan... probably a better way to do this
           if ((_lkmx < _dcx) && (_lkmy > _dcy))
           {
               temp = -1*temp;
               this->setSliderPosition((int)(temp/(6.2831853)*_sliderMax));
           }

           else if ((_lkmx < _dcx) && (_lkmy < _dcy))
           {
               temp = (6.2831853/2) - temp;
               this->setSliderPosition((int)(temp/(6.2831853)*_sliderMax));
           }

           else if ((_lkmx > _dcx) && (_lkmy < _dcy))
           {
               temp = (6.2831853/2) - temp;
               this->setSliderPosition((int)(temp/(6.2831853)*_sliderMax));
           }

           else if ((_lkmx > _dcx) && (_lkmy > _dcy))
           {
               temp = (6.2831853) - temp;
               this->setSliderPosition((int)(temp/(6.2831853)*_sliderMax));
           }
        }
    }

    // add the x and y components of the drag to the market center point
    // and compare it to to the dial center point to get a new angle

    // resolve the new angle into a slider position and update paintevent

    // more ideas include an 'infinite' wheel without a marker... spinning the
    // wheel forward increases slider position, and rotating it back reduces it
    // but changing the slider value has no effect on the wheel
    // so rotating the wheel CW forever works, but wont do anything once the
    // slider limit is reached... disadvantage is no direct seeking, advantange
    // is that you can be more accurate and adjust sensitivity!


}

