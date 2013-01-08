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

    std::string dataPath("/home/preet/Documents/maps/openstreetmap/london_render");
    osmscout::DatabaseParameter databaseParam;
    databaseParam.SetDebugPerformance(true);
    // default areaAreaIndexCache           1000
    // default areaNodeIndexCache           1000
    // default nodeIndexCacheSize           1000
    // default nodeCacheSize                1000
    // default wayIndexCacheSize            2000
    // default wayCacheSize                 8000
    // default relationIndexCacheSize       1000
    // default realtionCacheSize            1000

    osmscout::Database database(databaseParam);

    opOk = database.Open(dataPath);
    if(!opOk)  {   std::cerr << "ERROR: Could not open database\n";    return -1;   }

    // get dataset bounds
    double minLat,minLon,maxLat,maxLon;
    opOk = database.GetBoundingBox(minLat,minLon,maxLat,maxLon);
    if(!opOk)  {   std::cerr << "ERROR: Could not get data bounds\n";  return -1;   }

    osmscout::TypeConfig * typeConfig = database.GetTypeConfig();

    // set types; nodes and areas are empty
    osmscout::TypeSet nodeSet,waySet,areaSet;
    std::vector<osmscout::TypeInfo> listTypeInfo = typeConfig->GetTypes();

    osmscout::TypeId type_hwy_motorway      = typeConfig->GetTypeId("highway_motorway");
    osmscout::TypeId type_hwy_motorway_link = typeConfig->GetTypeId("highway_motorway_link");
    osmscout::TypeId type_hwy_trunk         = typeConfig->GetTypeId("highway_trunk");
    osmscout::TypeId type_hwy_trunk_link    = typeConfig->GetTypeId("highway_trunk_link");

    waySet.SetType(type_hwy_motorway);
    waySet.SetType(type_hwy_motorway_link);
    waySet.SetType(type_hwy_trunk);
    waySet.SetType(type_hwy_trunk_link);

    return 0;
}
