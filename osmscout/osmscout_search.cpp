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

    std::string dataPath("/home/preet/Documents/maps/toronto");
    osmscout::DatabaseParameter databaseParam;
    osmscout::Database database(databaseParam);

    opOk = database.Open(dataPath);

    if(!opOk)  {   std::cerr << "ERROR: Could not open database\n";    return 1;   }
    else       {   std::cerr << "INFO: Opened database Successfully\n";          }


    // look for administrative regions
    size_t searchLimit = 10000;
    bool limitReached = false;
    std::string regionName = "";
    std::list<osmscout::AdminRegion> listRegions;

    opOk = database.GetMatchingAdminRegions(regionName,listRegions,searchLimit,
                                            limitReached,false);
    // startsWith determines whether the regionName token matches the only the starting
    // portion of the region name or if it can match anywhere within the name

    if(!opOk)   {   std::cerr << "ERROR: Could not look up regions\n";   return 1;   }
    else        {   std::cerr << "INFO:  Looked up regions...\n";   }

    if(listRegions.size() == 0)
    {   std::cerr << "INFO: Didn't find any matching regions!\n";   return 1;   }
    else
    {   std::cerr << "INFO: Found " << listRegions.size() << " results!\n";   }

    // print data
    size_t rIdx = 0;    // iterator index
    size_t printLimit = std::min(searchLimit,listRegions.size());
    std::list<osmscout::AdminRegion>::iterator rIt;
    for(rIt = listRegions.begin();
        rIt != listRegions.end(); ++rIt)
    {
        std::cout << "INFO: Region Name: " << rIt->name;

        std::list<std::string>::iterator sIt;
        for(sIt = rIt->path.begin();
            sIt != rIt->path.end(); ++sIt)
        {   std::cout << "," << *(sIt);   }

        std::cout << std::endl;

        rIdx++;

        if(rIdx >= printLimit)   {
            break;
        }
    }

    return 0;
}
