// Qt libs
#include <QCoreApplication>
#include <QStringList>
#include <QDebug>
#include <QImage>
#include <QPainter>
#include <QDir>

// shapelib libs
#include "shapelib/shapefil.h"

struct Vec2d
{
    Vec2d(double sx=0,double sy=0,QColor cy=QColor())
    {   x = sx;   y = sy;   c = cy;   }

    double x;
    double y;
    QColor c;
};

int main(int argc, char *argv[])
{
    QCoreApplication myApp(argc, argv);

    // check input args
    QStringList inputArgs = myApp.arguments();

    if(inputArgs.size() < 2)   {
        qDebug() << "Error: No shapefile directory: ";
        qDebug() << "Pass the shapefile directory as an argument: ";
        qDebug() << "./paint_shapefile /my/shapefiledir";
        return -1;
    }

    // filter shapefile types
    QStringList shFilterList;
    shFilterList << "*.shp" << "*.shx" << "*.dbf" << "*.prj";

    // check for all files
    QDir shDir = inputArgs[1];
    QStringList shDirList = shDir.entryList(shFilterList,QDir::Files);

    // get all file paths
    QString fileShx,fileShp,fileDbf,filePrj;
    for(int i=0; i < shDirList.size(); i++)   {
        if(shDirList[i].contains(".shp"))   {
            fileShp = shDir.absoluteFilePath(shDirList[i]);
        }
        else if(shDirList[i].contains(".shx"))   {
            fileShx = shDir.absoluteFilePath(shDirList[i]);
        }
        else if(shDirList[i].contains(".dbf"))   {
            fileDbf = shDir.absoluteFilePath(shDirList[i]);
        }
        else if(shDirList[i].contains(".prj"))   {
            filePrj = shDir.absoluteFilePath(shDirList[i]);
        }
    }

    // open the shape file
    SHPHandle hSHP = SHPOpen(fileShp.toLocal8Bit().data(),"rb");
    if(hSHP == NULL)   {
        qDebug() << "Error: Could not open shape file";
        return -1;
    }

    if(hSHP->nShapeType != SHPT_POLYGON)   {
        qDebug() << "Error: Wrong shape file type:";
        qDebug() << "Expect POLYGON";
        return -1;
    }

    size_t nRecords = hSHP->nRecords;
    double xMax = hSHP->adBoundsMax[0];
    double yMax = hSHP->adBoundsMax[1];
    double xMin = hSHP->adBoundsMin[0];
    double yMin = hSHP->adBoundsMin[1];
    qDebug() << "Info: Bounds: x: " << xMin << "<>" << xMax;
    qDebug() << "Info: Bounds: y: " << yMin << "<>" << yMax;
    qDebug() << "Info: Found " << nRecords << "POLYGONS";
    qDebug() << "Info: Reading in data...";

    qDebug() << "Info: Creating color list...";
    QStringList listColors;
    for(size_t i=0; i < nRecords; i++)   {
        QString strColor = QString::number((i+1),16);
        while(strColor.length() < 6)   {    // pad with zeros
            strColor.prepend("0");
        }
        strColor.prepend("#");
        listColors.push_back(strColor);
    }

    QList<QList<Vec2d> > listPolygons;
    SHPObject * pSHPObj;

    for(size_t i=0; i < nRecords; i++)
    {   // for each object
        pSHPObj = SHPReadObject(hSHP,i);
        size_t nParts = pSHPObj->nParts;

        // build a list of start and end pts
        QList<int> listStartPts;
        QList<int> listEndB4Pts;
        for(size_t j=0; j < nParts-1; j++)   {
            listStartPts.push_back(pSHPObj->panPartStart[j]);
            listEndB4Pts.push_back(pSHPObj->panPartStart[j+1]);
        }
        listStartPts.push_back(pSHPObj->panPartStart[nParts-1]);
        listEndB4Pts.push_back(pSHPObj->nVertices);

        // build polys from start/end pts
        for(size_t j=0; j < listStartPts.size(); j++)
        {
            QList<Vec2d> listPts;
            size_t sIx = listStartPts[j];
            size_t eIx = listEndB4Pts[j];

            for(size_t k=sIx; k < eIx; k++)
            {
                listPts.push_back(Vec2d(pSHPObj->padfX[k]+180,
                                       (pSHPObj->padfY[k]-90)*-1,
                                        QColor(listColors[i])));
            }
            listPolygons.push_back(listPts);
        }
        SHPDestroyObject(pSHPObj);
    }
    SHPClose(hSHP);

    qDebug() << "Info: Rendering shape file to images...";
    int kSzMult=100;

    QImage shImage1(180*kSzMult,180*kSzMult,
                   QImage::Format_RGB888);

    QImage shImage2(180*kSzMult,180*kSzMult,
                   QImage::Format_RGB888);

    shImage1.fill(Qt::white);
    shImage2.fill(Qt::white);

    // setup painter
    QPainter shPainter;
    QBrush shBrush(Qt::blue);

    // draw polys on image 1
    shPainter.begin(&shImage1);
    shPainter.setPen(Qt::NoPen);
    for(size_t i=0; i < listPolygons.size(); i++)
    {
        QPainterPath pPath;
        pPath.setFillRule(Qt::WindingFill);
        pPath.moveTo(listPolygons[i][0].x*kSzMult,
                     listPolygons[i][0].y*kSzMult);

        for(size_t j=0; j < listPolygons[i].size(); j++)   {
            pPath.lineTo(listPolygons[i][j].x*kSzMult,
                         listPolygons[i][j].y*kSzMult);
        }
        pPath.closeSubpath();

        shBrush.setColor(listPolygons[i][0].c);
        shPainter.setBrush(shBrush);
        shPainter.drawPath(pPath);
    }
    shPainter.end();

    // draw polys on image2
    shPainter.begin(&shImage2);
    shPainter.setPen(Qt::NoPen);
    for(size_t i=0; i < listPolygons.size(); i++)
    {
        QPainterPath pPath;
        pPath.setFillRule(Qt::WindingFill);
        pPath.moveTo(listPolygons[i][0].x*kSzMult - 180*kSzMult,
                     listPolygons[i][0].y*kSzMult);

        for(size_t j=0; j < listPolygons[i].size(); j++)   {
            pPath.lineTo(listPolygons[i][j].x*kSzMult - 180*kSzMult,
                         listPolygons[i][j].y*kSzMult);
        }
        pPath.closeSubpath();

        shBrush.setColor(listPolygons[i][0].c);
        shPainter.setBrush(shBrush);
        shPainter.drawPath(pPath);
    }
    shPainter.end();


    if(shImage1.save("img1.png") && shImage2.save("img2.png"))   {
        qDebug() << "Info: Saved images as img1.png and img2.png!";
    }
    else   {
        qDebug() << "Error: Could not save image";
    }

    return 0;
}
