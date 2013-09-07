#include "imageviewer.h"

RingZoom::RingZoom(QGraphicsItem* parent) : QGraphicsItem(parent)
{
    // define geometry
    _oradius = 320;
    _thickness = 30;

    // setup initial shape
    _ringPath = new QPainterPath;
    _ringPath->addEllipse(QPointF(0,0),_oradius,_oradius);
    _ringPath->addEllipse(QPointF(0,0),(_oradius-_thickness),(_oradius-_thickness));
    _zoomsaved = 1;
    _zoomfactor = 1;
    _zoomfinal = 1;
}

RingZoom::~RingZoom()
{


}

QPainterPath RingZoom::shape() const
 {
     return (*_ringPath);
 }

QRectF RingZoom::boundingRect() const
{
    // return the bounding rectangle
    // with respect to item coordinate system

    return QRectF(-2*_oradius,-2*_oradius,4*_oradius,4*_oradius);
}

void RingZoom::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    // paint
    painter->setRenderHint(QPainter::Antialiasing);
    painter->setBrush(Qt::black);
    painter->drawPath(*_ringPath);
}

void RingZoom::setScale(qreal factor)
{
    // new scaled path... we overload setScale because
    // we want to maintain the thickness of the item

    // keeps QGraphicsScene's index up to date (required!)
    this->prepareGeometryChange();

    _radius = (_oradius*factor - _thickness/2);
    _ringPath = new QPainterPath;
    _ringPath->addEllipse(QPointF(0,0),_oradius*factor,_oradius*factor);
    _ringPath->addEllipse(QPointF(0,0),(_oradius*factor-_thickness),(_oradius*factor-_thickness));

}

void RingZoom::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
  //qDebug() << "Mouse button clicked at position: "
   //        << event->pos();
}

void RingZoom::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    double lkmx = event->scenePos().x();
    double lkmy = event->scenePos().y();
    double cen_dist = sqrt(pow(lkmx,2) + pow(lkmy,2));
    _zoomfactor = cen_dist/(_oradius-(_thickness/2));
    _zoomfinal = _zoomfactor*_zoomsaved;
    //qDebug() << "The image zoom level is: " << _zoomfinal;

    // update ring scale
    this->setScale(_zoomfactor);

    // update image scale
    emit imageZoomLevel(_zoomfinal);
}

void RingZoom::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    this->setScale(1);
    _zoomsaved *= _zoomfactor;
    //qDebug() << "The saved zoom level is: " << _zoomsaved;
}




RingSeek::RingSeek(int numImgs, QGraphicsItem* parent) : QGraphicsItem(parent)
{
    // define geometry
    _sradius = 320;
    _thickness = 5;

    _mradius = 30;
    _mcx = 0;
    _mcy = _sradius;

    // setup initial shape
    _ringPath = new QPainterPath;
    _ringPath->addEllipse(QPointF(0,0),_sradius,_sradius);
    _ringPath->addEllipse(QPointF(0,0),(_sradius-_thickness),(_sradius-_thickness));

    _markerPath = new QPainterPath;
    _markerPath->addEllipse(QPointF(_mcx,_mcy),_mradius,_mradius);

    // setup slider
    _seriesSeek = new QAbstractSlider;
    _seriesSeek->setRange(0,numImgs);


}

RingSeek::~RingSeek()
{


}

QPainterPath RingSeek::shape() const
 {
     return (*_markerPath);
 }

QRectF RingSeek::boundingRect() const
{
    // return the bounding rectangle
    // with respect to item coordinate system

    return QRectF(-2*_sradius,-2*_sradius,4*_sradius,4*_sradius);
}

void RingSeek::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    // paint
    painter->setRenderHint(QPainter::Antialiasing);
    painter->setBrush(Qt::black);
    painter->drawPath(*_ringPath);
    painter->drawPath(*_markerPath);
}

void RingSeek::mousePressEvent(QGraphicsSceneMouseEvent *event)
{ }

void RingSeek::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    double lkmx = event->scenePos().x();
    double lkmy = event->scenePos().y();

    //qDebug() << "LKM Position was: " << event->scenePos();

    // resolve mouse position into angle around ring center
    double angle = atan(lkmx/lkmy);

    // get angle from four quadrants... probably better way to do this
    if ((lkmx < 0) && (lkmy > 0))
    {   angle = angle * -1; }

    else if ((lkmx < 0) && (lkmy < 0))
    {   angle = (6.2831853/2) - angle;  }

    else if ((lkmx > 0) && (lkmy < 0))
    {   angle = (6.2831853/2) - angle;  }

    else if ((lkmx > 0) && (lkmy > 0))
    {   angle = (6.2831853) - angle;  }

    else
    {  }

    double seekTo = (angle/6.2831853)*(double(_seriesSeek->maximum()));
    emit imageSeek(int(seekTo));

    this->setRotation(angle*360/6.2831853);



    //qDebug() << "Rotated by: " << angle*360/6.2831853;

}

void RingSeek::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{ }


ImageViewer::ImageViewer(QWidget *parent) : QMainWindow(parent)
{
    // init
    _prevImage = -1;
    _currentImage = 0;
    t_zoom = 1;
    t_xpos = 0;
    t_ypos = 0;

    // set minimum window size
    _MIN_WIN_X = 1000;
    _MIN_WIN_Y = 650;
    this->setMinimumSize(_MIN_WIN_X,_MIN_WIN_Y);

    // GraphicsView setup
    //QGraphicsView::setViewport(new QGLWidget)
    _mainView = new QGraphicsView(this);
    _mainView->setOptimizationFlags(QGraphicsView::DontClipPainter);
    _mainView->setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
    _mainView->setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));

    _mainScene = new QGraphicsScene(double(-1*_MIN_WIN_X)/2.0f,double(-1*_MIN_WIN_Y)/2.0f,_MIN_WIN_X,_MIN_WIN_Y);
    _mainImage = new QGraphicsPixmapItem(0,_mainScene);
    _mainScene->setBackgroundBrush(Qt::black);
    _mainView->setScene(_mainScene);

    // init display
    LoadImageList(QString("blah"));

    // add RingZoom
    _myZoom = new RingZoom;
    _mainScene->addItem(_myZoom);
    QObject::connect(_myZoom, SIGNAL(imageZoomLevel(double)), this, SLOT(UpdateZoom(double)));

    // add RingSeek
    _mySeek = new RingSeek(_imageList.size());
    _mainScene->addItem(_mySeek);
    QObject::connect(_mySeek, SIGNAL(imageSeek(int)), this, SLOT(UpdateCurrent(int)));

    UpdateImage();
    setCentralWidget(_mainView);
}

ImageViewer::~ImageViewer()
{

}


void ImageViewer::LoadImageList(QString path)
{
    // get list of images
    _imagePath = "/home/preet/Pictures/imageset1";
    _imageDir.setPath(_imagePath);
    _imageList = _imageDir.entryList();
    _imageList.removeAt(0);     // remove "." dir listing
    _imageList.removeAt(0);     // remove ".." dir listing
}

void ImageViewer::UpdateImage()
{
    //
    if (_prevImage != _currentImage)
    {
        _image = new QPixmap(QString(_imagePath+"/"+_imageList[_currentImage]));
        _mainImage->setPixmap(*_image);
        _mainImage->setOffset(double(-1*_image->width())/2.0f,double(-1*_image->height())/2.0f);
        _prevImage = _currentImage;
    }

    //_mainImage->translate(t_xpos,t_ypos);
    QMatrix scaleM;
    scaleM.scale(t_zoom,t_zoom);
    _mainImage->setTransform(QTransform(scaleM),false);
}

void ImageViewer::UpdateZoom(double newZoom)
{
    t_zoom = newZoom;
    UpdateImage();
}

void ImageViewer::UpdateCurrent(int newImg)
{
    if (newImg >= 0 && newImg < _imageList.size())
    {
        _currentImage = newImg;
        UpdateImage();
    }
}
