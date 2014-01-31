#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>

// jansson
#include <jansson.h>

// libosmscout
#include <osmscout/import/Import.h>

std::string convIntToString(int myInt)
{
    std::stringstream ss;
    std::string str;
    ss << myInt;
    ss >> str;
    return str;
}

bool GetImportParamAsString(json_t * importObj,
                            std::string const &pName,
                            std::string &pResult)
{
    json_t * jString = json_object_get(importObj,pName.c_str());
    if(json_string_value(jString) == NULL)   {
        std::cerr << "ERROR: Could not get " << pName << std::endl;
        return false;
    }
    else   {
        pResult = std::string(json_string_value(jString));
        std::cerr << "INFO: " << pName << ":" << pResult << std::endl;
        return true;
    }
}

bool GetImportParamAsBool(json_t * importObj,
                          std::string const &pName,
                          bool &pResult)
{
    json_t * jBool = json_object_get(importObj,pName.c_str());
    if(!json_is_boolean(jBool))   {
        std::cerr << "ERROR: Could not get " << pName << std::endl;
        return false;
    }
    else   {
        pResult = json_is_true(jBool);
        std::cerr << "INFO: " << pName << ":" << pResult << std::endl;
        return true;
    }
}

bool GetImportParamAsInt(json_t * importObj,
                         std::string const &pName,
                         size_t &pResult)
{
    json_t * jInt = json_object_get(importObj,pName.c_str());
    if(!json_is_integer(jInt))   {
        std::cerr << "ERROR: Could not get " << pName << std::endl;
        return false;
    }
    else   {
        pResult = json_integer_value(jInt);
        std::cerr << "INFO: " << pName << ":" << pResult << std::endl;
        return true;
    }
}

bool GetImportParamAsDouble(json_t * importObj,
                            std::string const &pName,
                            double &pResult)
{
    json_t * jDouble = json_object_get(importObj,pName.c_str());
    if(!json_is_real(jDouble))   {
        std::cerr << "ERROR: Could not get " << pName << std::endl;
        return false;
    }
    else   {
        pResult = json_real_value(jDouble);
        std::cerr << "INFO: " << pName << ":" << pResult << std::endl;
        return true;
    }
}

int main(int argc, char *argv[])
{
    if(argc != 2)   {
        std::cerr << "ERROR: Invalid number of arguments:" << std::endl;
        std::cerr << "Pass the JSON import config file as an argument:" << std::endl;
        std::cerr << "./import_json /my/importconfig.json" << std::endl;
        return -1;
    }

    // json: ROOT
    json_error_t jError;
    json_t * jRoot = json_load_file(argv[1],0,&jError);
    if(!jRoot)   {
        std::string errorText(jError.text);
        std::string errorLine(convIntToString(jError.line));
        std::string errorPos(convIntToString(jError.position));
        std::cerr << "ERROR: Parsing JSON: Text: " << errorText << std::endl;
        std::cerr << "ERROR: Parsing JSON: Line: " << errorLine << std::endl;
        std::cerr << "ERROR: Parsing JSON: Pos: "  << errorPos << std::endl;
    }

    // json: IMPORTCONFIG
    json_t * j = json_object_get(jRoot,"IMPORTCONFIG");
    if(j == NULL)   {
        std::cerr << "ERROR: Could not find IMPORTCONFIG!" << std::endl;
        return -1;
    }

    // read in params from file
    std::string                  mapfile;                  //! Name of the file containing the map (either *.osm or *.osm.pbf)
    std::string                  typefile;                 //! Name and path ff type definition file (map.ost.xml)
    std::string                  destinationDirectory;     //! Name of the destination directory
    size_t                       startStep;                //! Starting step for import
    size_t                       endStep;                  //! End step for import

    bool                         strictAreas;              //! Assure that areas conform to "simple" definition

    bool                         sortObjects;              //! Sort all objects
    size_t                       sortBlockSize;            //! Number of entries loaded in one sort iteration
    size_t                       sortTileMag;              //! Zoom level for individual sorting cells

    size_t                       numericIndexPageSize;     //! Size of an numeric index page in bytes

    bool                         coordDataMemoryMaped;     //! Use memory mapping for coord data file access

    bool                         rawNodeDataMemoryMaped;   //! Use memory mapping for raw node data file access
    size_t                       rawNodeDataCacheSize;     //! Size of the raw node data cache

    bool                         rawWayIndexMemoryMaped;   //! Use memory mapping for raw way index file access
    bool                         rawWayDataMemoryMaped;    //! Use memory mapping for raw way data file access
    size_t                       rawWayDataCacheSize;      //! Size of the raw way data cache
    size_t                       rawWayIndexCacheSize;     //! Size of the raw way index cache
    size_t                       rawWayBlockSize;          //! Number of ways loaded during import until nodes get resolved

    bool                         areaDataMemoryMaped;      //! Use memory mapping for area data file access
    size_t                       areaDataCacheSize;        //! Size of the area data cache

    bool                         wayDataMemoryMaped;       //! Use memory mapping for way data file access
    size_t                       wayDataCacheSize;         //! Size of the way data cache

    size_t                       areaAreaIndexMaxMag;      //! Maximum depth of the index generated

    size_t                       areaWayMinMag;            //! Minimum magnification of index for individual type
    double                       areaWayIndexMinFillRate;  //! Minimum rate of filled cells in index bitmap
    size_t                       areaWayIndexCellSizeAverage; //! Average entries per index cell
    size_t                       areaWayIndexCellSizeMax;  //! Maximum number of entries  per index cell

    size_t                       areaNodeMinMag;           //! Minimum magnification of index for individual type
    double                       areaNodeIndexMinFillRate; //! Minimum rate of filled cells in index bitmap
    size_t                       areaNodeIndexCellSizeAverage; //! Average entries per index cell
    size_t                       areaNodeIndexCellSizeMax; //! Maximum number of entries  per index cell

    size_t                       waterIndexMinMag;         //! Minimum level of the generated water index
    size_t                       waterIndexMaxMag;         //! Maximum level of the generated water index

    size_t                       optimizationMaxWayCount;  //! Maximum number of ways for one iteration
    size_t                       optimizationMaxMag;       //! Maximum magnification for optimization
    size_t                       optimizationMinMag;       //! Minimum magnification of index for individual type
    size_t                       optimizationCellSizeAverage; //! Average entries per index cell
    size_t                       optimizationCellSizeMax;  //! Maximum number of entries  per index cell
    std::string                  optimizationWayMethod;    //! what method to use to optimize ways

    size_t                       routeNodeBlockSize;       //! Number of route nodes loaded during import until ways get resolved

    bool                         assumeLand;               //! During sea/land detection,we either trust coastlines only or make some
                                                           //! assumptions which tiles are sea and which are land.

    bool ok = true;
    std::cout << "========================================" << std::endl;
    std::cout << "INFO: Using following Import parameters:" << std::endl;

    ok = ok && GetImportParamAsString(j,"mapfile",mapfile);
    ok = ok && GetImportParamAsString(j,"typefile",typefile);
    ok = ok && GetImportParamAsString(j,"destinationDirectory",destinationDirectory);
    ok = ok && GetImportParamAsInt(j,"startStep",startStep);
    ok = ok && GetImportParamAsInt(j,"endStep",endStep);

    ok = ok && GetImportParamAsBool(j,"strictAreas",strictAreas);

    ok = ok && GetImportParamAsBool(j,"sortObjects",sortObjects);
    ok = ok && GetImportParamAsInt(j,"sortBlockSize",sortBlockSize);
    ok = ok && GetImportParamAsInt(j,"sortTileMag",sortTileMag);

    ok = ok && GetImportParamAsInt(j,"numericIndexPageSize",numericIndexPageSize);

    ok = ok && GetImportParamAsBool(j,"coordDataMemoryMaped",coordDataMemoryMaped);

    ok = ok && GetImportParamAsBool(j,"rawNodeDataMemoryMaped",rawNodeDataMemoryMaped);
    ok = ok && GetImportParamAsInt(j,"rawNodeDataCacheSize",rawNodeDataCacheSize);

    ok = ok && GetImportParamAsBool(j,"rawWayIndexMemoryMaped",rawWayIndexMemoryMaped);
    ok = ok && GetImportParamAsBool(j,"rawWayDataMemoryMaped",rawWayDataMemoryMaped);
    ok = ok && GetImportParamAsInt(j,"rawWayDataCacheSize",rawWayDataCacheSize);
    ok = ok && GetImportParamAsInt(j,"rawWayIndexCacheSize",rawWayIndexCacheSize);
    ok = ok && GetImportParamAsInt(j,"rawWayBlockSize",rawWayBlockSize);

    ok = ok && GetImportParamAsBool(j,"areaDataMemoryMaped",areaDataMemoryMaped);
    ok = ok && GetImportParamAsInt(j,"areaDataCacheSize",areaDataCacheSize);

    ok = ok && GetImportParamAsBool(j,"wayDataMemoryMaped",wayDataMemoryMaped);
    ok = ok && GetImportParamAsInt(j,"wayDataCacheSize",wayDataCacheSize);

    ok = ok && GetImportParamAsInt(j,"areaAreaIndexMaxMag",areaAreaIndexMaxMag);

    ok = ok && GetImportParamAsInt(j,"areaWayMinMag",areaWayMinMag);
    ok = ok && GetImportParamAsDouble(j,"areaWayIndexMinFillRate",areaWayIndexMinFillRate);
    ok = ok && GetImportParamAsInt(j,"areaWayIndexCellSizeAverage",areaWayIndexCellSizeAverage);
    ok = ok && GetImportParamAsInt(j,"areaWayIndexCellSizeMax",areaWayIndexCellSizeMax);

    ok = ok && GetImportParamAsInt(j,"areaNodeMinMag",areaNodeMinMag);
    ok = ok && GetImportParamAsDouble(j,"areaNodeIndexMinFillRate",areaNodeIndexMinFillRate);
    ok = ok && GetImportParamAsInt(j,"areaNodeIndexCellSizeAverage",areaNodeIndexCellSizeAverage);
    ok = ok && GetImportParamAsInt(j,"areaNodeIndexCellSizeMax",areaNodeIndexCellSizeMax);

    ok = ok && GetImportParamAsInt(j,"waterIndexMinMag",waterIndexMinMag);
    ok = ok && GetImportParamAsInt(j,"waterIndexMaxMag",waterIndexMaxMag);

    ok = ok && GetImportParamAsInt(j,"optimizationMaxWayCount",optimizationMaxWayCount);
    ok = ok && GetImportParamAsInt(j,"optimizationMaxMag",optimizationMaxMag);
    ok = ok && GetImportParamAsInt(j,"optimizationMinMag",optimizationMinMag);
    ok = ok && GetImportParamAsInt(j,"optimizationCellSizeAverage",optimizationCellSizeAverage);
    ok = ok && GetImportParamAsInt(j,"optimizationCellSizeMax",optimizationCellSizeMax);
    ok = ok && GetImportParamAsString(j,"optimizationWayMethod",optimizationWayMethod);

    ok = ok && GetImportParamAsInt(j,"routeNodeBlockSize",routeNodeBlockSize);
    ok = ok && GetImportParamAsBool(j,"assumeLand",assumeLand);


    if(!ok)   {
        std::cerr << "ERROR: There was an error "
                     "reading in a parameter!" << std::endl;
        return -1;
    }

    // set import params
    osmscout::ImportParameter p;
    osmscout::ConsoleProgress progress;

    p.SetMapfile(mapfile);
    p.SetTypefile(typefile);
    p.SetDestinationDirectory(destinationDirectory);
    p.SetSteps(startStep,endStep);

    p.SetStrictAreas(strictAreas);

    p.SetSortObjects(sortObjects);
    p.SetSortBlockSize(sortBlockSize);
    p.SetSortTileMag(sortTileMag);

    p.SetNumericIndexPageSize(numericIndexPageSize);

    p.SetCoordDataMemoryMaped(coordDataMemoryMaped);

    p.SetRawNodeDataMemoryMaped(rawNodeDataMemoryMaped);
    p.SetRawNodeDataCacheSize(rawNodeDataCacheSize);

    p.SetRawWayIndexMemoryMaped(rawWayIndexMemoryMaped);
    p.SetRawWayDataMemoryMaped(rawWayDataMemoryMaped);
    p.SetRawWayDataCacheSize(rawWayDataCacheSize);
    p.SetRawWayIndexCacheSize(rawWayIndexCacheSize);
    p.SetRawWayBlockSize(rawWayBlockSize);

    p.SetAreaDataMemoryMaped(areaDataMemoryMaped);
    p.SetAreaDataCacheSize(areaDataCacheSize);

    p.SetWayDataMemoryMaped(wayDataMemoryMaped);
    p.SetWayDataCacheSize(wayDataCacheSize);

    p.SetAreaAreaIndexMaxMag(areaAreaIndexMaxMag);

    p.SetAreaWayMinMag(areaWayMinMag);
    p.SetAreaWayIndexMinFillRate(areaWayIndexMinFillRate);
    p.SetAreaWayIndexCellSizeAverage(areaWayIndexCellSizeAverage);
    p.SetAreaWayIndexCellSizeMax(areaWayIndexCellSizeMax);

    p.SetAreaNodeMinMag(areaNodeMinMag);
    p.SetAreaNodeIndexMinFillRate(areaNodeIndexMinFillRate);
    p.SetAreaNodeIndexCellSizeAverage(areaNodeIndexCellSizeAverage);
    p.SetAreaNodeIndexCellSizeMax(areaNodeIndexCellSizeMax);

    p.SetWaterIndexMinMag(waterIndexMinMag);
    p.SetWaterIndexMaxMag(waterIndexMaxMag);

    p.SetOptimizationMaxWayCount(optimizationMaxWayCount);
    p.SetOptimizationMaxMag(optimizationMaxMag);
    p.SetOptimizationMinMag(optimizationMinMag);
    p.SetOptimizationCellSizeAverage(optimizationCellSizeAverage);
    p.SetOptimizationCellSizeMax(optimizationCellSizeMax);
    if(optimizationWayMethod == "TransPolygon::none")
    {   p.SetOptimizationWayMethod(osmscout::TransPolygon::none);   }
    else if(optimizationWayMethod == "TransPolygon::fast")
    {   p.SetOptimizationWayMethod(osmscout::TransPolygon::fast);   }
    else if(optimizationWayMethod == "TransPolygon::quality")
    {   p.SetOptimizationWayMethod(osmscout::TransPolygon::quality);   }
    else   {
        std::cerr << "WARN: Unrecognized Way Optimization Method, "
                     "defaulting to none!" << std::endl;
        p.SetOptimizationWayMethod(osmscout::TransPolygon::none);
    }

    p.SetRouteNodeBlockSize(routeNodeBlockSize);
    p.SetAssumeLand(assumeLand);

    // make sure the destination directory exists
    std::string sys_make_path = "mkdir -p "+destinationDirectory;
    system(sys_make_path.c_str());

    std::cout << "========================================" << std::endl;
    std::cout << "Starting Import..."                       << std::endl;
    std::cout << "========================================" << std::endl;

    if (osmscout::Import(p,progress))
    {   std::cerr << "Import OK!" << std::endl;   }
    else
    {   std::cerr << "Import failed!" << std::endl;   }

    std::cout << "========================================" << std::endl;

    while(true)   {
        std::cout << "Separate temp files? Y/N" << std::endl;
        std::string rem_extra_files;
        std::cin >> rem_extra_files;

        if(rem_extra_files.compare("Y")==0 ||
           rem_extra_files.compare("y")==0)   {

            // create a temp directory
            std::string files_path = destinationDirectory;
            if(files_path[files_path.length()-1] != '/')   {
                files_path.append("/");
            }
            sys_make_path = "mkdir -p "+files_path+"temp";
            system(sys_make_path.c_str());

            std::vector<std::string> list_temp_files;
            list_temp_files.push_back(std::string("areas.idmap"));          // osm id map for DebugDatabase
            list_temp_files.push_back(std::string("coord.dat"));

            list_temp_files.push_back(std::string("location.txt"));

            list_temp_files.push_back(std::string("nodes.idmap"));          // osm id map for DebugDatabase
            list_temp_files.push_back(std::string("nodes.tmp"));

            list_temp_files.push_back(std::string("rawcoastline.dat"));
            list_temp_files.push_back(std::string("rawnode.idx"));
            list_temp_files.push_back(std::string("rawnodes.dat"));
            list_temp_files.push_back(std::string("rawrel.idx"));
            list_temp_files.push_back(std::string("rawrels.dat"));
            list_temp_files.push_back(std::string("rawturnrestr.dat"));
            list_temp_files.push_back(std::string("rawway.idx"));
            list_temp_files.push_back(std::string("rawways.dat"));

            list_temp_files.push_back(std::string("relarea.tmp"));

            list_temp_files.push_back(std::string("turnrestr.dat"));       // ?

            list_temp_files.push_back(std::string("wayareablack.dat"));
            list_temp_files.push_back(std::string("wayarea.tmp"));
            list_temp_files.push_back(std::string("ways.idmap"));          // osm id map for DebugDatabase
            list_temp_files.push_back(std::string("wayway.tmp"));

            for(size_t i=0; i < list_temp_files.size(); i++)   {
                std::string prev_file_path = files_path+list_temp_files[i];
                std::string next_file_path = files_path+"temp/"+list_temp_files[i];
                std::string sys_mv_file = "mv "+prev_file_path+" "+next_file_path;
                system(sys_mv_file.c_str());
                std::cout << "-> " << sys_mv_file << std::endl;
            }
            break;
        }
        else if(rem_extra_files.compare("N")==0 ||
                rem_extra_files.compare("n")==0)   {
            break;
        }
        std::cout << "ERROR: Unrecognized input" << std::endl;
    }

    return 0;
}
