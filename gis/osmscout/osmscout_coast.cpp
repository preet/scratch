#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

// libosmscout
#include <osmscout/Database.h>

int main()
{
    // open up database
    bool opOk = false;

    std::string dataPath("/home/preet/Documents/maps/toronto_default");
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
    osmscout::Mag mapMag = osmscout::magBlock;

    // get tiles
    std::list<osmscout::GroundTile> listTiles;
    opOk = database.GetGroundTiles(minLon,minLat,maxLon,maxLat,mapMag,listTiles);
    if(!opOk)  {   std::cerr << "ERROR: Could not get ground tiles\n"; return -1;   }

    std::list<osmscout::GroundTile>::iterator mIt;
    for(mIt = listTiles.begin();
        mIt != listTiles.end(); ++mIt)
    {
        // type
        std::cout << "Ground Tile Type: ";
        switch(mIt->type)
        {
            case osmscout::GroundTile::unknown:   {
                std::cout << "Unknown";
                break;
            }
            case osmscout::GroundTile::land:      {
                std::cout << "Land";
                break;
            }
            case osmscout::GroundTile::water:     {
                std::cout << "Water";
                break;
            }
            case osmscout::GroundTile::coast:     {
                std::cout << "Coast";
                break;
            }
            default:
                break;
        }
        std::cout << std::endl;

//        std::cout << "Absolute Coords: " << mIt->xAbs << "," << mIt->yAbs << std::endl;
//        std::cout << "Relative Coords: " << mIt->xRel << "," << mIt->yRel << std::endl;
        std::cout << "Cell Width: "  << 360.0/mIt->cellWidth << std::endl;
        std::cout << "Cell Height: " << 180.0/mIt->cellHeight << std::endl;


//        std::cout << "Coordinates: " << std::endl;
//        for(size_t i=0; i < mIt->coords.size(); i++)   {
//            std::cout << mIt->coords[i].x << ","
//                      << mIt->coords[i].y << ", isCoast "
//                      << mIt->coords[i].coast << std::endl;
//        }

    }
    return 0;
}
