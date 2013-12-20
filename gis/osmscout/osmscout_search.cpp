#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

// libosmscout
#include <osmscout/Database.h>
#include <osmscout/DebugDatabase.h>
#include <osmscout/ObjectRef.h>

using namespace std;

int main()
{
    // open up database
    bool opOk = false;

    std::string path = "/home/preet/Dev/build/osmscout_json_import/ontario";
    std::string str1 = "Hello";

    osmscout::DatabaseParameter databaseParam;
    osmscout::Database database(databaseParam);

    opOk = database.Open(path);

    if(!opOk)  {   std::cerr << "ERROR: Could not open database\n";    return 1;   }
    else       {   std::cerr << "INFO: Opened database Successfully\n";         }


    // look for administrative regions
    size_t searchLimit = 10000;
    bool limitReached = false;
    std::string regionName = "Sarn";
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

    //     print data
    std::set<osmscout::ObjectFileRef> setOffsets;
    std::list<osmscout::AdminRegion>::iterator rIt;
    std::vector<std::string> listNames;
    std::vector<osmscout::FileOffset> listOffsets;
    for(rIt = listRegions.begin();
        rIt != listRegions.end(); ++rIt)
    {
        std::cout << "INFO: Region Id: " << rIt->reference.GetFileOffset()
                  << " / Name: " << rIt->name << std::endl;

        osmscout::ObjectFileRef fileRef(rIt->reference.GetFileOffset(),
                                        rIt->reference.GetType());
        listNames.push_back(rIt->name);
        listOffsets.push_back(rIt->reference.GetFileOffset());

        setOffsets.insert(fileRef);
    }

    //
    osmscout::DebugDatabaseParameter debugParam;
    osmscout::DebugDatabase debugDb(debugParam);

    opOk = debugDb.Open(path);

    if(!opOk)  {   std::cerr << "ERROR: Could not open debug db";    return 1;   }
    else       {   std::cerr << "INFO: Opened debug db Successfully\n";         }

    std::set<osmscout::ObjectOSMRef> setIds;
    std::map<osmscout::ObjectOSMRef,osmscout::ObjectFileRef> idFileOffsetMap;
    std::map<osmscout::ObjectFileRef,osmscout::ObjectOSMRef> fileOffsetIdMap;
    opOk = debugDb.ResolveReferences(setIds,setOffsets,idFileOffsetMap,fileOffsetIdMap);

    if(!opOk)  {   std::cerr << "ERROR: Could not resolve refs";    return 1;   }
    else       {   std::cerr << "INFO: resolved refs\n";         }

    size_t k=0;
    std::map<osmscout::ObjectFileRef,osmscout::ObjectOSMRef>::iterator mIt;
    for(mIt = fileOffsetIdMap.begin();
        mIt != fileOffsetIdMap.end(); ++mIt)   {

        std::cerr << "INFO: Region Name: " << listNames[k] <<", "
//                  << "FileOffset: " << listOffsets[k] << ", "
                  << "OSMId: " << mIt->second.GetId() << std::endl;
        k++;
    }



    std::cout << "Got Here" << std::endl;


    return 0;
}
