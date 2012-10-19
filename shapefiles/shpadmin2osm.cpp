// Qt libs
#include <QCoreApplication>
#include <QStringList>
#include <QDebug>
#include <QImage>
#include <QPainter>
#include <QDir>

#include <iostream>

// shapelib libs
#include "shapelib/shapefil.h"

// pugixml
#include <pugixml.hpp>

int g_node_id = 0;
pugi::xml_document osm_file;
pugi::xml_node osm_meta;

struct xml_string_writer: pugi::xml_writer
{
    std::string result;

    virtual void write(const void *data, size_t size)
    {   result += std::string(static_cast<const char*>(data), size);   }
};

struct Vec2d
{
    Vec2d(double sx=0,double sy=0,QColor cy=QColor())
    {   x = sx;   y = sy;   c = cy;   }

    double x;
    double y;
    QColor c;
};

bool BuildNameList(QString const &fileDbf,
                   QStringList &listAdminNames,
                   QString const &fieldName)
{
    // open the dbf file
    DBFHandle hDBF = DBFOpen(fileDbf.toLocal8Bit().data(),"rb");
    if(hDBF == NULL)   {
        qDebug() << "Error: Could not open dbf file" << fileDbf;
        return false;
    }

    // get number of fields in db
    size_t nDbfRecords = DBFGetRecordCount(hDBF);
    size_t idx_field = DBFGetFieldIndex(hDBF,fieldName.toLocal8Bit().data());
    qDebug() << "Info: Found" << nDbfRecords << "Records";

    listAdminNames.clear();
    for(size_t i=0; i < nDbfRecords; i++)   {
        QString record_val(DBFReadStringAttribute(hDBF,i,idx_field));
        listAdminNames.push_back(record_val);
    }
    DBFClose(hDBF);

    return true;
}

bool BuildCoordList(QString const &fileShp,
                    QList<Vec2d> &listAdminCoords)
{
    // open the shape file
    SHPHandle hSHP = SHPOpen(fileShp.toLocal8Bit().data(),"rb");
    if(hSHP == NULL)   {
        qDebug() << "Error: Could not open shp file" << fileShp;
        return false;
    }

    if(hSHP->nShapeType != SHPT_POLYGON)   {
        qDebug() << "Error: Wrong shape file type:";
        qDebug() << "Expect POLYGON";
        return false;
    }

    size_t nRecords = hSHP->nRecords;
    double xMax = hSHP->adBoundsMax[0];
    double yMax = hSHP->adBoundsMax[1];
    double xMin = hSHP->adBoundsMin[0];
    double yMin = hSHP->adBoundsMin[1];
    qDebug() << "Info: Bounds: x: " << xMin << "<>" << xMax;
    qDebug() << "Info: Bounds: y: " << yMin << "<>" << yMax;
    qDebug() << "Info: Found " << nRecords << "POLYGONS";
    qDebug() << "Info: Creating Average Lon/Lat entry for each POLYGON";

    // build polygon list
    SHPObject * pSHPObj;
    listAdminCoords.clear();
    for(size_t i=0; i < nRecords; i++)
    {   // for each object
        pSHPObj = SHPReadObject(hSHP,i);
        size_t nParts = pSHPObj->nParts;

        // build a list of start and end pts
        QList<QList<Vec2d> > listPolygons;
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
                listPts.push_back(Vec2d(pSHPObj->padfX[k],
                                        pSHPObj->padfY[k]));
            }
            listPolygons.push_back(listPts);
        }
        SHPDestroyObject(pSHPObj);

        // average all polys for this entry
        double avLon=0; double avLat=0;
        for(size_t j=0; j < listPolygons.size(); j++)
        {
            for(size_t k=0; k < listPolygons[j].size(); k++)   {
                avLon += listPolygons[j][k].x;
                avLat += listPolygons[j][k].y;
            }
            avLon /= listPolygons[j].size();
            avLat /= listPolygons[j].size();
        }
        listAdminCoords.push_back(Vec2d(avLon,avLat));
    }
    SHPClose(hSHP);

    return true;
}

bool WriteOSMData(QList<Vec2d> const &listAdminCoords,
                  QStringList const &listAdminNames,
                  QString const &tagNameKey,
                  QString &outputStr)
{
    for(size_t i=0; i < listAdminCoords.size(); i++)
    {   //<osm version="0.6">
        //  <node id="" lat="" lon ="">
        //      <tag k="tagNameKey" v=""/>
        //      <tag k="name" v="" />
        //  </node>
        //</osm>
        pugi::xml_node osm_node = osm_meta.append_child("node");
        osm_node.append_attribute("id") = g_node_id;
        osm_node.append_attribute("lat") = listAdminCoords[i].y;
        osm_node.append_attribute("lon") = listAdminCoords[i].x;

        pugi::xml_node osm_mma_tag = osm_node.append_child("tag");
        osm_mma_tag.append_attribute("k") = tagNameKey.toLocal8Bit().data();
        osm_mma_tag.append_attribute("v") = "-";

        pugi::xml_node osm_name_tag = osm_node.append_child("tag");
        osm_name_tag.append_attribute("k") = "name";
        osm_name_tag.append_attribute("v") = listAdminNames[i].toLocal8Bit().data();

        g_node_id++;
    }
}

int main(int argc, char *argv[])
{
    QCoreApplication myApp(argc, argv);

    // check input args
    QStringList inputArgs = myApp.arguments();

    if(inputArgs.size() != 4)   {
        qDebug() << "Error: Insufficient arguments: ";
        qDebug() << "## Pass in the shapefile directories (input) and osmfile (output): ";
        qDebug() << "## ./shpadmin2osm /my/admin0shapefiledir /my/admin1shapefiledir /my/osmfile";
        qDebug() << "## Admin0 and Admin1 shapefiles can be obtained from naturalearthdata.com";
        return -1;
    }

    // filter shapefile types
    QStringList shFilterList;
    shFilterList << "*.shp" << "*.dbf";

    // get admin0 file paths
    QDir shDir = inputArgs[1];
    QStringList shDirList = shDir.entryList(shFilterList,QDir::Files);
    QString adm0fileShp,adm0fileDbf;
    for(int i=0; i < shDirList.size(); i++)   {
        if(shDirList[i].contains(".shp"))   {
            adm0fileShp = shDir.absoluteFilePath(shDirList[i]);
        }
        else if(shDirList[i].contains(".dbf"))   {
            adm0fileDbf = shDir.absoluteFilePath(shDirList[i]);
        }
    }

    // get admin1 file paths
    shDir = inputArgs[2];
    shDirList = shDir.entryList(shFilterList,QDir::Files);
    QString adm1fileShp,adm1fileDbf;
    for(int i=0; i < shDirList.size(); i++)   {
        if(shDirList[i].contains(".shp"))   {
            adm1fileShp = shDir.absoluteFilePath(shDirList[i]);
        }
        else if(shDirList[i].contains(".dbf"))   {
            adm1fileDbf = shDir.absoluteFilePath(shDirList[i]);
        }
    }

    // build admin0 names and coords
    QStringList listAdmin0Names;
    if(!BuildNameList(adm0fileDbf,listAdmin0Names,"ADMIN"))
    {   return -1;   }

    QList<Vec2d> listAdmin0Coords;
    if(!BuildCoordList(adm0fileShp,listAdmin0Coords))
    {   return -1;   }

    // build admin1 names and coords
    QStringList listAdmin1Names;
    if(!BuildNameList(adm1fileDbf,listAdmin1Names,"NAME_1"))
    {   return -1;   }

    QList<Vec2d> listAdmin1Coords;
    if(!BuildCoordList(adm1fileShp,listAdmin1Coords))
    {   return -1;   }

    // enclose the xml file with osm tags
    osm_meta = osm_file.append_child("osm");
    osm_meta.append_attribute("version") = 0.6;

    QString osmAdm0Data;
    WriteOSMData(listAdmin0Coords,
                 listAdmin0Names,
                 QString("mapmix_adm0"),
                 osmAdm0Data);

    QString osmAdm1Data;
    WriteOSMData(listAdmin1Coords,
                 listAdmin1Names,
                 QString("mapmix_adm1"),
                 osmAdm1Data);

    xml_string_writer osm_writer;
    osm_file.print(osm_writer);
    QString writeOutStr = QString::fromStdString(osm_writer.result);

    QFile outFile(inputArgs[3]);
    if(!outFile.open(QIODevice::ReadWrite))
    {   qDebug() << "Could not open output file!"; return -1;   }

    QTextStream outXml(&outFile);
    outXml << writeOutStr;
    outFile.close();

    return 0;
}
