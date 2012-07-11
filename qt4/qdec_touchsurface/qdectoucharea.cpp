#include "qdectoucharea.h"

QDecTouchArea::QDecTouchArea(QDeclarativeItem *parent) :
    QDeclarativeItem(parent),
    m_diffScaleFactor(1.0),
    m_totalScaleFactor(1.0),
    m_diffRotationAngle(0.0),
    m_totalRotationAngle(0.0)
{
    // have to explicitly accept touch events
    setAcceptTouchEvents(true);

    // we have 10 possible touch points
    for(int i=0; i < 10; i++)  {
        QDecTouchPoint * myTouchPt = new QDecTouchPoint;
        m_listTouchPoints.push_back(myTouchPt);
    }
}

QDecTouchArea::~QDecTouchArea()
{}

bool QDecTouchArea::sceneEvent(QEvent *event)
{
    bool interceptedEvent = false;

    if(event->type() == QEvent::TouchBegin)
    {
        updateTouchData(event);

        // must be accepted to receive following events
        // (QEvent::TouchUpdate and QEvent::TouchEnd)
        event->accept();
        interceptedEvent = true;
    }
    else if(event->type() == QEvent::TouchUpdate)
    {
        updateTouchData(event);
        interceptedEvent = true;
    }
    else if(event->type() == QEvent::TouchEnd)
    {
        updateTouchData(event);
        interceptedEvent = true;
    }

    return interceptedEvent;
}

void QDecTouchArea::updateTouchData(QEvent *event)
{  
    QTouchEvent *e = static_cast<QTouchEvent*>(event);
    QList<QTouchEvent::TouchPoint> listTouchPts = e->touchPoints();
    for(int i=0; i < listTouchPts.count(); i++)
    {
        int touchPtId = listTouchPts[i].id();
        Qt::TouchPointState touchPtState = listTouchPts[i].state();
        QPointF touchPtPos = listTouchPts[i].pos();

        QDecTouchPoint * qDecTouchPt = m_listTouchPoints[touchPtId];

        // set touch point active/inactive
        if(touchPtState & Qt::TouchPointPressed)  {
            qDecTouchPt->setActive(true);
            qDecTouchPt->setX(touchPtPos.x());
            qDecTouchPt->setY(touchPtPos.y());
            emit touchPointPressed(touchPtId);
        }
        else if(touchPtState & Qt::TouchPointReleased)  {
            qDecTouchPt->setActive(false);
            qDecTouchPt->setX(touchPtPos.x());
            qDecTouchPt->setY(touchPtPos.y());
            emit touchPointReleased(touchPtId);
        }

        // if point moved, set new position
        if(touchPtState & Qt::TouchPointMoved)  {
            qDecTouchPt->setX(touchPtPos.x());
            qDecTouchPt->setY(touchPtPos.y());
            emit touchPointMoved(touchPtId);
        }
    }

    // handle two touch inputs
    if(listTouchPts.count() == 2)
    {
        QTouchEvent::TouchPoint tp1 = listTouchPts.at(0);
        QTouchEvent::TouchPoint tp2 = listTouchPts.at(1);

        QPointF tp1LastPos = tp1.lastPos();
        QPointF tp2LastPos = tp2.lastPos();
        QPointF tp1Pos = tp1.pos();
        QPointF tp2Pos = tp2.pos();

        QPointF deltaA = tp1LastPos - tp2LastPos;
        QPointF deltaB = tp1Pos - tp2Pos;

        qreal distanceA = sqrt(pow(deltaA.x(),2.0)+pow(deltaA.y(),2.0));
        qreal distanceB = sqrt(pow(deltaB.x(),2.0)+pow(deltaB.y(),2.0));

        if (distanceA != 0 && distanceB != 0) {
            m_diffScaleFactor = (distanceB/distanceA);
            m_totalScaleFactor *= m_diffScaleFactor;
            emit(scaleFactorChanged());
        }

        QLineF lineA(tp1LastPos, tp2LastPos);
        QLineF lineB(tp1Pos,tp2Pos);
        m_diffRotationAngle = lineA.angleTo(lineB);
        m_totalRotationAngle -= m_diffRotationAngle;

        if(m_totalRotationAngle > 360)  {
            m_totalRotationAngle -= 360;
        }
        else if(m_totalRotationAngle < -360)  {
            m_totalRotationAngle += 360;
        }

        emit(rotationAngleChanged());
    }
}
