#include <QtGui>
#include <iostream>
#include "ringzoom.h"


RingZoom::RingZoom(QWidget *parent)
    : QAbstractSlider(parent)
{
    _sliderMin = -500;
    _sliderMax = 500;   //% zoom in out

    this->setRange(_sliderMin, _sliderMax);

    _doInit = 1;



}


void RingZoom::paintEvent(QPaintEvent *)
{
    // initial size parameters
    //if (_doInit==1)
    //{


   //     _doInit = 0;
   // }

    // draw the ring
    QPainter painter(this);
    QPoint center(_zcx,_zcy);
    painter.drawEllipse(center,_zoradius,_zoradius);
    painter.drawEllipse(center,_ziradius,_ziradius);

    // set the masked region
    QRegion oRegion(_zcx-_zoradius*1.15,_zcy-_zoradius*1.15,_zoradius*2.3,_zoradius*2.3,QRegion::Ellipse);
    QRegion iRegion(_zcx-_ziradius*0.85,_zcy-_ziradius*0.85,_ziradius*1.7,_ziradius*1.7,QRegion::Ellipse);
    //setMask(oRegion-iRegion);

}

void RingZoom::resizeEvent(QResizeEvent * event)
{
    _zoradiusf = (int)(((double)this->size().height())/2*0.9);
    _ziradiusf = (int)(0.85*(double)_zoradiusf);
    _zradiusf = (_zoradiusf+_ziradiusf)/2;
    _zdist = (_zoradiusf-_ziradiusf);
    _zstatic = 0;
    _zradius = _zradiusf;
    _zoradius = _zoradiusf;
    _ziradius = _ziradiusf;

    _zcx = (this->size().width())/2;
    _zcy = (this->size().height())/2;
}

void RingZoom::mouseReleaseEvent (QMouseEvent * event)
{

    // reset zoom ring (but not slider!)
    _zoradiusf = (int)(((double)this->size().height())/2*0.9);
    _ziradiusf = (int)(0.85*(double)_zoradiusf);
    _zradiusf = (_zoradiusf+_ziradiusf)/2;
    _zdist = (_zoradiusf-_ziradiusf);
    _zstatic = this->sliderPosition();

    // reset zoom ring (but not slider!)
    _zradius = _zradiusf;
    _zoradius = _zoradiusf;
    _ziradius = _ziradiusf;

    _zcx = (this->size().width())/2;
    _zcy = (this->size().height())/2;

    this->repaint();
}

void RingZoom::mouseMoveEvent (QMouseEvent * event)
{

    // verify the mouse click occured in the ring
    // else do nothing
    _draggedMarker = false;
    _lkmx = event->x();
    _lkmy = event->y();

    if ((event->buttons() & Qt::LeftButton))
    {
        // find the distance from the center point
        double dis = sqrt(pow((_zcx-_lkmx),2) + pow((_zcy-_lkmy),2));
        double mindis = _zradiusf * 0.25;
        double maxdis = _zradiusf * 1.75;

        if (((dis > _ziradius) && (dis < _zoradius)))
        {
            if ((dis < maxdis) && (dis > mindis))
            {
                double temp = _zstatic + (((1-(dis/_zradiusf))/0.75)*500);
                this->setSliderPosition((int)temp);

                //std::cerr << temp;

                // scale zoom ring with mouse input
                _zradius = dis;
                _zoradius = _zradius + _zdist/2;
                _ziradius = _zradius - _zdist/2;

                //
            }
        }

        //std::cerr << "Slider Position: " << this->sliderPosition() << std::endl;
    }

}

