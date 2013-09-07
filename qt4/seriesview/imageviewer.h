#ifndef IMAGEVIEWER_H
#define IMAGEVIEWER_H

#include <iostream>

#include <QtGui/QMainWindow>
#include <QDebug>
#include <QString>
#include <QStringList>
#include <QDir>
#include <QMatrix>
#include <QTransform>
#include <QPainter>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QGraphicsPixmapItem>
#include <QGraphicsSceneMouseEvent>
#include <QAbstractSlider>
#include <QPainterPath>
#include <QtOpenGL>
#include "math.h"

class RingZoom : public QObject, public QGraphicsItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)

public:
    RingZoom(QGraphicsItem* parent=0);
    ~RingZoom();

    QPainterPath shape() const;
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    void setScale(qreal);
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

signals:
    void imageZoomLevel(double);

private:
    // geometry
    QPainterPath* _ringPath;
    double _oradius;
    double _radius;
    double _thickness;

    double _zoomsaved;
    double _zoomfactor;
    double _zoomfinal;

};



class RingSeek : public QObject, public QGraphicsItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)

public:
    RingSeek(int, QGraphicsItem* parent=0);
    ~RingSeek();

    QPainterPath shape() const;
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
    void mousePressEvent(QGraphicsSceneMouseEvent *event);

signals:
    void imageSeek(int);

private:
    // geometry
    QPainterPath* _ringPath;
    QPainterPath* _markerPath;
    double _sradius;
    double _thickness;
    double _mradius;                // marker radius
    double _mcx;                    // marker center x coordinate
    double _mcy;                    // marker center y coordinate

    // slider
    QAbstractSlider* _seriesSeek;

};




class ImageViewer : public QMainWindow
{
    Q_OBJECT

public:
    ImageViewer(QWidget *parent = 0);
    ~ImageViewer();

    void LoadImageList(QString);
    void UpdateImage();

public slots:
    void UpdateZoom(double);
    void UpdateCurrent(int);


private:

    int _MIN_WIN_X;        // minimum window size in x
    int _MIN_WIN_Y;         // minimum window size in y

    QPixmap* _image;

    QGraphicsView* _mainView;
    QGraphicsScene* _mainScene;
    QGraphicsPixmapItem* _mainImage;

    QString _imagePath;
    QStringList _imageList;
    QDir _imageDir;

    RingZoom* _myZoom;
    RingSeek* _mySeek;

    double t_zoom;
    double t_xpos;
    double t_ypos;

    int _prevImage;
    int _currentImage;

};

#endif // IMAGEVIEWER_H
