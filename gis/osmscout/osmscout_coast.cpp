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
                 osmscout::Mag viewMag)
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
    // [Z][XXXX][YYYY]
    // Z = 1 digit to represent zoom (5-13)
    //     stored as an offset (0-8)
    // XXXX = 4 digits for abs x cell (max 8192)
    // YYYY = 4 digits for abs y cell (max 8192)

    // note: max val for uint32_t
    // 4,294,967,295

    // zoom
    size_t zoom = (intlog2(size_t(viewMag))-5);
    size_t tileId = zoom*100000000;
    tileId += xAbs*10000 + yAbs;
    return tileId;
}

double randomIntensity(size_t seed)
{
    srand(seed);
    double myNum = (rand()%10 + 1);
    myNum/=10.0;
    std::cout << myNum << std::endl;
    return myNum;
}

int main()
{
    osg::ref_ptr<osg::Geode> gdCoast = new osg::Geode;
    osg::ref_ptr<osg::Group> groupRoot = new osg::Group;
    groupRoot->addChild(gdCoast);

    // open up database
    bool opOk = false;

    std::string dataPath("/home/preet/Documents/maps/openstreetmap/ontario");
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
    osmscout::Mag mapMag = osmscout::magState;

    // get tiles
    std::list<osmscout::GroundTile> listTiles;
    opOk = database.GetGroundTiles(minLon,minLat,maxLon,maxLat,mapMag,listTiles);
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

    size_t k=0; size_t p=90000000;
    double kMinLat,kMaxLat,kMinLon,kMaxLon;
    for(cellIt = listTilesByCell.begin();
        cellIt != listTilesByCell.end(); ++cellIt)
    {   // for every cell

        k++; p--;
        osg::ref_ptr<osg::Vec3dArray> gmCoastVx = new osg::Vec3dArray;
        osg::ref_ptr<osg::Vec4Array> gmCoastCx = new osg::Vec4Array;
        ListTiles &listT = cellIt->second;
        for(size_t i=0; i < listT.size(); i++)
        {   // for every tile

            osmscout::GroundTile * tilePtr = listT[i];
            kMinLat = tilePtr->yAbs*tilePtr->cellHeight-90.0;
            kMaxLat = kMinLat + tilePtr->cellHeight;
            kMinLon = tilePtr->xAbs*tilePtr->cellWidth-180.0;
            kMaxLon = kMinLon + tilePtr->cellWidth;

            for(size_t j=0; j < tilePtr->coords.size(); j++)
            {   // for each coord
                double lon = kMinLon+tilePtr->coords[j].x*tilePtr->cellWidth/
                        osmscout::GroundTile::Coord::CELL_MAX;

                double lat = kMinLat+tilePtr->coords[j].y*tilePtr->cellHeight/
                        osmscout::GroundTile::Coord::CELL_MAX;

                gmCoastVx->push_back(osg::Vec3d(lon,lat,0));
            }
//            gmCoastVx->push_back(osg::Vec3d(0,0,0));    // mark the end of a coastline
        }

        std::cout << "ARRAY SIZE: " << gmCoastVx->size() << "\n";

        // random color
        osg::Vec4 color(randomIntensity(gmCoastVx->size()),
                        randomIntensity(k),
                        randomIntensity(p),1.0);
        gmCoastCx->push_back(color);

        if(gmCoastVx->size() > 0)   {
            osg::ref_ptr<osg::Geometry> gmCoastTile = new osg::Geometry;
            gmCoastTile->setVertexArray(gmCoastVx);
            gmCoastTile->setColorArray(gmCoastCx);
            gmCoastTile->setColorBinding(osg::Geometry::BIND_OVERALL);
            gmCoastTile->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP,0,gmCoastVx->size()));
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
