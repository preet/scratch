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


int main(int argc, char *argv[])
{
    QCoreApplication myApp(argc, argv);

    // check input args
    QStringList inputArgs = myApp.arguments();

    if(inputArgs.size() < 3)   {
        qDebug() << "Error: Invalid input:";
        qDebug() << "Pass the required arguments as follows: ";
        qDebug() << "./lonlat2placename <bitmapW> <bitmapE> <db>";
        return -1;
    }

    // interpret args
    QString argImgW = inputArgs[1];
    QString argImgE = inputArgs[2];
    QString argDb  = inputArgs[3];

    qDebug() << "Info: Opening Admin Regions East image...";
    QImage adminImgE(argImgE);
    if(adminImgE.isNull())   {
        qDebug() << "Error: Could not open image file: " << argImgE;
        qDebug() << "Is the file path correct?";   return -1;
    }

    return -1;

    // open image files
    qDebug() << "Info: Opening Admin Regions West image...";
    QImage adminImgW(argImgW);
    if(adminImgW.isNull())   {
        qDebug() << "Error: Could not open image file:" << argImgW;
        qDebug() << "Is the file path correct?";   return -1;
    }

    // open database
    // returns exception if file dne/invalid
    qDebug() << "Info: Opening Admin Regions Database...";

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

        // get corresponding pixel from image, assume kMult of 100
        QRgb pixelRgb;
        int kSzMult = 100;
        if(userLon < 0)   {
            // search west image
            int pixelLon = (userLon+180)*kSzMult;
            int pixelLat = (userLat-90)*-1*kSzMult;
            pixelRgb = adminImgW.pixel(pixelLon,pixelLat);
        }
        else   {
            // search east image
            int pixelLon = (userLon)*kSzMult;
            int pixelLat = (userLat-90)*-1*kSzMult;
            pixelRgb = adminImgW.pixel(pixelLon,pixelLat);
        }

        // convert color into id
        // id #abc000 -> (abc) -> 2748 (ignore trailing zeros)
        QColor pixelColor(pixelRgb);

        QString colorName = pixelColor.name();          // #RRGGBB
        colorName.remove(0,1);                          // remove '#'
        while(colorName[colorName.size()-1] == '0')
        {   colorName.remove(colorName.size()-1,1);   } // remove trailing zeros

        // smarter to create a lookup table for this
        // when its actually implemented
        bool opOk; int dbIdx = colorName.toInt(&opOk,16);
        QString regionId = QString::number(dbIdx,10);

        // lookup name using database
        QString sqlQuery = "SELECT name FROM data WHERE data.regionid = " + regionId;
        pStmt->Sql(sqlQuery.toStdString());
        if(pStmt->FetchRow())   {
            QString placeName = QString::fromStdString(pStmt->GetColumnString(0));
            qDebug() << "Found \"" << placeName << "\" at ("
                     << userLon << "," << userLat << ")";
        }
    }

    // clean up database
    delete pStmt;
    delete pDatabase;

    return 0;
}
