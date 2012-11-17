#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <unordered_map>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// osg includes
#include <osg/Geometry>
#include <osg/PolygonMode>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osgViewer/Viewer>

// libosmscout
#include <osmscout/Database.h>

typedef std::vector<osmscout::GroundTile*> ListTiles;


struct Vec2
{
    double x;
    double y;
};

size_t intlog2(size_t val)
{   // warn:
    // returns 0 for an
    // input of 0!
    size_t ret = 0;
    while(val != 0)   {
        val >>= 1;
        ret++;
    }
    return ret;
}

size_t genCellId(size_t xAbs, size_t yAbs,
                 size_t viewMag)
{
    // list of magnification ranges we allow:
    //    magState     =               32, //  5
    //    magStateOver =               64, //  6
    //    magCounty    =              128, //  7
    //    magRegion    =              256, //  8
    //    magProximity =              512, //  9
    //    magCityOver  =             1024, // 10
    //    magCity      =           2*1024, // 11
    //    magSuburb    =         2*2*1014, // 12
    //    magDetail    =       2*2*2*1024, // 13

    // we build a unique tile id as follows:
    // [XXXX][YYYY]
    // XXXX = 4 digits for abs x cell (max 8192)
    // YYYY = 4 digits for abs y cell (max 8192)

    // note: max val for uint32_t
    // 4,294,967,295

    // zoom
    size_t tileId = xAbs*10000 + yAbs;
    return tileId;
}

double randomIntensity(size_t seed)
{
    srand(seed);
    double myNum = (rand()%10 + 1);
    myNum/=10.0;
    return myNum;
}

int main(int argc, char *argv[])
{
    if(argc != 2)   {
            std::cerr << "ERROR: Invalid number of arguments:" << std::endl;
            std::cerr << "Pass the osmscout map data dir as an argument:" << std::endl;
            std::cerr << "./osmscout_coast /my/mapdata" << std::endl;
            return -1;
        }

    osg::ref_ptr<osg::Geode> gdCoast = new osg::Geode;
    osg::ref_ptr<osg::Group> groupRoot = new osg::Group;
    groupRoot->addChild(gdCoast);

    // open up database
    bool opOk = false;

    std::string dataPath(argv[1]);
    osmscout::DatabaseParameter databaseParam;
    osmscout::Database database(databaseParam);

    opOk = database.Open(dataPath);
    if(!opOk)  {   std::cerr << "ERROR: Could not open database\n";    return -1;   }

    // get dataset bounds
    double minLat,minLon,maxLat,maxLon;
    opOk = database.GetBoundingBox(minLat,minLon,maxLat,maxLon);
    if(!opOk)  {   std::cerr << "ERROR: Could not get data bounds\n";  return -1;   }

    // list of magnifications:
    //    magWorld     =                1, //  0
    //    magContinent =               16, //  4
    //    magState     =               32, //  5
    //    magStateOver =               64, //  6
    //    magCounty    =              128, //  7
    //    magRegion    =              256, //  8
    //    magProximity =              512, //  9
    //    magCityOver  =             1024, // 10
    //    magCity      =           2*1024, // 11
    //    magSuburb    =         2*2*1014, // 12
    //    magDetail    =       2*2*2*1024, // 13
    //    magClose     =     2*2*2*2*1024, // 14
    //    magVeryClose =   2*2*2*2*2*1024, // 15
    //    magBlock     = 2*2*2*2*2*2*1024  // 16
    osmscout::Mag mapMag = osmscout::magRegion;

    // get tiles
    std::list<osmscout::GroundTile> listTiles;
    opOk = database.GetGroundTiles(minLon-1.3,minLat-1.3,maxLon+1.3,maxLat+1.3,mapMag,listTiles);
    if(!opOk)  {   std::cerr << "ERROR: Could not get ground tiles\n"; return -1;   }

    std::cerr << "INFO: Found " << listTiles.size()
              << " tiles" << std::endl;

    // note:
    // there can be 100s of tiles for each cell (xAbs,yAbs) and this
    // makes it difficult to generate uids and impractical for generating
    // geometry, so we merge all tiles belonging to a single cell

    std::unordered_map<size_t,ListTiles> listTilesByCell;
    std::unordered_map<size_t,ListTiles>::iterator cellIt,findIt;

    std::list<osmscout::GroundTile>::iterator tileIt;
    for(tileIt = listTiles.begin();
        tileIt != listTiles.end(); ++tileIt)
    {
        if(tileIt->coords.size() == 0)
        {   continue;   }

        size_t cellId = genCellId(tileIt->xAbs,tileIt->yAbs,mapMag);
        findIt = listTilesByCell.find(cellId);
        if(findIt == listTilesByCell.end())     // dne
        {
            ListTiles listTilePtrs;
            listTilePtrs.push_back(&(*tileIt));

            std::pair<size_t,ListTiles> insData;
            insData.first = cellId;
            insData.second = listTilePtrs;
            listTilesByCell.insert(insData);
        }
        else
        {   findIt->second.push_back(&(*tileIt));   }
    }

    osg::Vec4 tileColor(0.3,0.3,0.3,1.0);
    osg::ref_ptr<osg::Vec4Array> listTileCx = new osg::Vec4Array;
    listTileCx->push_back(tileColor);

    // vars to randomize color
    size_t k=0; size_t p=90000000;

    double kMinLat,kMaxLat,kMinLon,kMaxLon;
    for(cellIt = listTilesByCell.begin();
        cellIt != listTilesByCell.end(); ++cellIt)
    {   // for every cell

        k++; p--;
        osg::ref_ptr<osg::Vec3dArray> gmCoastVx = new osg::Vec3dArray;
        gmCoastVx->push_back(osg::Vec3d(0,0,0));
        osg::ref_ptr<osg::Vec4Array> gmCoastCx = new osg::Vec4Array;
        ListTiles &listT = cellIt->second;
        for(size_t i=0; i < listT.size(); i++)
        {   // for every tile
            osmscout::GroundTile * tilePtr = listT[i];
            kMinLat = tilePtr->yAbs*tilePtr->cellHeight-90.0;
            kMaxLat = kMinLat + tilePtr->cellHeight;
            kMinLon = tilePtr->xAbs*tilePtr->cellWidth-180.0;
            kMaxLon = kMinLon + tilePtr->cellWidth;

            // BUILD CELL BORDERS
            osg::ref_ptr<osg::Vec3dArray> gmTileVx = new osg::Vec3dArray;
            gmTileVx->push_back(osg::Vec3d(kMinLon,kMinLat,0));
            gmTileVx->push_back(osg::Vec3d(kMaxLon,kMinLat,0));
            gmTileVx->push_back(osg::Vec3d(kMaxLon,kMaxLat,0));
            gmTileVx->push_back(osg::Vec3d(kMinLon,kMaxLat,0));

            osg::ref_ptr<osg::Geometry> gmTile = new osg::Geometry;
            gmTile->setVertexArray(gmTileVx);
            gmTile->setColorArray(listTileCx);
            gmTile->setColorBinding(osg::Geometry::BIND_OVERALL);
            gmTile->addPrimitiveSet(new osg::DrawArrays(GL_LINE_LOOP,0,gmTileVx->size()));
            gdCoast->addDrawable(gmTile);

            // BUILD TILE COORDS
            size_t lineStart = 0;
            size_t lineEnd;

            while(lineStart < tilePtr->coords.size())
            {
                // seek lineStart to start of coastline segment
                while(lineStart < tilePtr->coords.size() &&
                      !(tilePtr->coords[lineStart].coast))   {
                    lineStart++;
                }

                if(lineStart >= tilePtr->coords.size())   {
                    continue;
                }

                // seek lineEnd to end of coastline segment
                lineEnd = lineStart;
                while(lineEnd < tilePtr->coords.size() &&
                      tilePtr->coords[lineEnd].coast)   {
                    lineEnd++;
                }

                //
                for(size_t n=lineStart; n <= lineEnd; n++)
                {
                    double lon = kMinLon+tilePtr->coords[n].x*tilePtr->cellWidth/
                            osmscout::GroundTile::Coord::CELL_MAX;

                    double lat = kMinLat+tilePtr->coords[n].y*tilePtr->cellHeight/
                            osmscout::GroundTile::Coord::CELL_MAX;

                    gmCoastVx->push_back(osg::Vec3d(lon,lat,0));
                }
                gmCoastVx->push_back(osg::Vec3d(0,0,0));
                lineStart = lineEnd+1;
            }
        }

        // random color
        osg::Vec4 color(randomIntensity(gmCoastVx->size()),
                        randomIntensity(k),
                        randomIntensity(p),1.0);
        gmCoastCx->push_back(color);

        if(gmCoastVx->size() > 2)   {
            osg::ref_ptr<osg::DrawElementsUInt> gmCoastIx =
                    new osg::DrawElementsUInt(GL_LINES);

            for(size_t i=1; i < gmCoastVx->size()-1; i++)
            {
                osg::Vec3d vx = gmCoastVx->at(i);
                if((vx.x() == 0) && (vx.y() == 0) && (vx.z() == 0))
                {   continue;   }

                gmCoastIx->push_back(i);
                gmCoastIx->push_back(i);

                vx = gmCoastVx->at(i-1);
                if((vx.x() == 0) && (vx.y() == 0) && (vx.z() == 0))
                {   gmCoastIx->pop_back();   continue;   }

                vx = gmCoastVx->at(i+1);
                if((vx.x() == 0) && (vx.y() == 0) && (vx.z() == 0))
                {   gmCoastIx->pop_back();   continue;   }
            }

            osg::ref_ptr<osg::Geometry> gmCoastTile = new osg::Geometry;
            gmCoastTile->setVertexArray(gmCoastVx);
            gmCoastTile->setColorArray(gmCoastCx);
            gmCoastTile->setColorBinding(osg::Geometry::BIND_OVERALL);
            gmCoastTile->addPrimitiveSet(gmCoastIx);
            gdCoast->addDrawable(gmCoastTile);
        }
    }

    gdCoast->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

    osgViewer::Viewer viewer;
    viewer.getCamera()->setClearColor(osg::Vec4(0.1,0.1,0.1,1.0));
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    viewer.setUpViewInWindow(100,100,800,480);
    viewer.setSceneData(groupRoot);
    return viewer.run();

    return 0;
}


//    // dump all tile data as dots
//    double kMinLat,kMaxLat,kMinLon,kMaxLon;
//    osg::ref_ptr<osg::Vec3Array> listVx = new osg::Vec3Array;
//    std::list<osmscout::GroundTile>::iterator tileIt;
//    for(tileIt = listTiles.begin();
//        tileIt != listTiles.end(); ++tileIt)
//    {
//        osmscout::GroundTile * tilePtr = &(*tileIt);
//        kMinLat = tilePtr->yAbs*tilePtr->cellHeight-90.0;
//        kMaxLat = kMinLat + tilePtr->cellHeight;
//        kMinLon = tilePtr->xAbs*tilePtr->cellWidth-180.0;
//        kMaxLon = kMinLon + tilePtr->cellWidth;

//        for(size_t n=0; n < tilePtr->coords.size(); n++)
//        {
//            double lon = kMinLon+tilePtr->coords[n].x*tilePtr->cellWidth/
//                    osmscout::GroundTile::Coord::CELL_MAX;

//            double lat = kMinLat+tilePtr->coords[n].y*tilePtr->cellHeight/
//                    osmscout::GroundTile::Coord::CELL_MAX;

//            osg::Vec3 vx(lon,lat,0);
//            listVx->push_back(vx);
//        }
//    }

//    osg::ref_ptr<osg::Geometry> gmCoast = new osg::Geometry;
//    gmCoast->setVertexArray(listVx);
//    gmCoast->addPrimitiveSet(new osg::DrawArrays(GL_POINTS,0,listVx->size()));

//    gdCoast->addDrawable(gmCoast);
