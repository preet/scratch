// Qt libs
#include <QCoreApplication>
#include <QStringList>
#include <QDebug>
#include <QImage>
#include <QPainter>
#include <QDir>

#include <string>
#include <iostream>

// kompex libs
#include "kompex/KompexSQLiteDatabase.h"
#include "kompex/KompexSQLiteStatement.h"

void GetTilePixel(double mLon,double mLat,bool &isWest,
                  int &tileNum,int &tileX,int&tileY)
{
    if(mLon < 0)   {
        // west tiles
        mLon += 180.0;
        isWest = true;
    }
    else   {
        // east tiles
        isWest = false;
    }

    mLat = (mLat-90)*-1;

    int rowIdx = mLat/10;
    int colIdx = mLon/10;

    // note we assume kSzMult = 100;
    tileNum = rowIdx*18 + colIdx;
    tileX = (mLon*100) - (colIdx*1000);
    tileY = (mLat*100) - (rowIdx*1000);
}

int main(int argc, char *argv[])
{
    QCoreApplication myApp(argc, argv);

    // check input args
    QStringList inputArgs = myApp.arguments();

    if(inputArgs.size() < 3)   {
        qDebug() << "Error: Invalid input:";
        qDebug() << "Pass the required arguments as follows: ";
        qDebug() << "./lonlat2placename <dir_tilesW> <dir_tilesE> <db>";
        return -1;
    }

    QStringList filterList; filterList << "*.png";

    // get list of tiles for west_input_dir (expect 324)
    QDir dirTilesWest = inputArgs[1];
    QStringList listFilesTilesWest =
        dirTilesWest.entryList(filterList,QDir::Files);

    if(listFilesTilesWest.size() != 324)   {
        qDebug() << "Error: Expected 324 West Tiles, got"
                 << listFilesTilesWest.size();
        return -1;
    }

    // get list of tiles for east_input_dir (expect 324)
    QDir dirTilesEast = inputArgs[2];
    QStringList listFilesTilesEast =
        dirTilesEast.entryList(filterList,QDir::Files);

    if(listFilesTilesEast.size() != 324)   {
        qDebug() << "Error: Expected 324 East Tiles, got"
                 << listFilesTilesEast.size();
        return -1;
    }

    // open database
    // returns exception if file dne/invalid
    qDebug() << "Info: Opening Admin Regions Database...";
    QString argDb  = inputArgs[3];

    Kompex::SQLiteDatabase * pDatabase =
            new Kompex::SQLiteDatabase(argDb.toStdString(),
                SQLITE_OPEN_READWRITE,0);

    Kompex::SQLiteStatement * pStmt =
            new Kompex::SQLiteStatement(pDatabase);

    // start main loop
    while(1)   {
        qDebug() << "Enter Coordinates (entering 'n' quits)? [y/n]";

        std::string keepGoingResponse;
        std::cin >> keepGoingResponse;
        if(keepGoingResponse.compare("y") != 0)   {
            qDebug() << "Info: Quitting...";
            break;
        }

        double userLon;
        qDebug() << "Enter Longitude: ";
        std::cin >> userLon;

        double userLat;
        qDebug() << "Enter Latitude: ";
        std::cin >> userLat;

        // get corresponding pixel from image

        QRgb pixelRgb;
        QString tilePath;
        bool isWest = false;
        int tileNum,tileX,tileY;

        GetTilePixel(userLon,userLat,isWest,
                     tileNum,tileX,tileY);

        if(isWest)   {
            tilePath = dirTilesWest.canonicalPath() +
                    QDir::separator();
        }
        else   {
            tilePath = dirTilesEast.canonicalPath() +
                    QDir::separator();
        }

        qDebug() << "Info: Search in dir" << tilePath;
        qDebug() << "Info: Search in Tile" << tileNum;
        qDebug() << "Info:" << tileX << "," << tileY;

        QString tileName = QString::number(tileNum,10);
        tileName.prepend("tile_");
        tileName.append(".png");

        QImage tileImg(tilePath + tileName);
        if(tileImg.isNull())   {
            qDebug() << "Error: Could not open tile";
            return -1;
        }

        pixelRgb = tileImg.pixel(tileX,tileY);

        // convert color into id
        // id #abc000 -> (abc) -> 2748 (ignore trailing zeros)
        QColor pixelColor(pixelRgb);

        QString colorName = pixelColor.name();          // #RRGGBB
        colorName.remove(0,1);                          // remove '#'
//        while(colorName[colorName.size()-1] == '0')
//        {   colorName.remove(colorName.size()-1,1);   } // remove trailing zeros

        // smarter to create a lookup table for this
        // when its actually implemented
        bool opOk; int dbIdx = colorName.toInt(&opOk,16);
        QString regionId = QString::number(dbIdx,10);

        qDebug() << "Info: Color String: " << pixelColor.name();
        qDebug() << "Info: Color Index:" << dbIdx;

        // lookup name using database
        QString sqlQuery = "SELECT name FROM data WHERE data.regionid = " + regionId;
        pStmt->Sql(sqlQuery.toStdString());
        if(pStmt->FetchRow())   {
            QString placeName = QString::fromStdString(pStmt->GetColumnString(0));
            qDebug() << "Found" << placeName << " at ("
                     << userLon << "," << userLat << ")";
        }
        else   {
            qDebug() << "Couldn't find anything at location!";
        }
        pStmt->FreeQuery();
    }

    // clean up database
    delete pStmt;
    delete pDatabase;

    return 0;
}
