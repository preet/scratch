#include <iostream>
#include <vector>
#include <string>
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

    // [ROOT]
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

    // [IMPORTCONFIG]
    bool opOk = true;
    json_t * jImportConfig = json_object_get(jRoot,"IMPORTCONFIG");
    if(jImportConfig == NULL)   {
        std::cerr << "ERROR: Could not find IMPORTCONFIG!" << std::endl;
        return -1;
    }

    std::string mapfile;
    std::string typefile;

    size_t startStep;
    size_t endStep;

    bool strictAreas;

    bool sortObjects;
    size_t sortBlockSize;
    size_t sortTileMag;

    size_t numericIndexPageSize;

    bool coordDataMemoryMapped;

    bool rawNodeIndexMemoryMapped;
    bool rawNodeDataMemoryMapped;
    size_t rawNodeDataCacheSize;
    size_t rawNodeIndexCacheSize;

    bool rawWayIndexMemoryMapped;
    bool rawWayDataMemoryMapped;
    size_t rawWayDataCacheSize;
    size_t rawWayIndexCacheSize;
    size_t rawWayBlockSize;

    bool wayIndexMemoryMapped;
    bool wayDataMemoryMapped;
    size_t wayDataCacheSize;
    size_t wayIndexCacheSize;
    size_t waysLoadSize;

    size_t areaAreaIndexMaxMag;

    size_t areaWayMinMag;
    double areaWayIndexMinFillRate;
    size_t areaWayIndexCellSizeAverage;
    size_t areaWayIndexCellSizeMax;

    size_t areaNodeMinMag;
    double areaNodeIndexMinFillRate;
    size_t areaNodeIndexCellSizeAverage;
    size_t areaNodeIndexCellSizeMax;

    size_t waterIndexMinMag;
    size_t waterIndexMaxMag;

    size_t optimizationMaxWayCount;
    size_t optimizationMaxMag;
    size_t optimizationMinMag;
    size_t optimizationCellSizeAverage;
    size_t optimizationCellSizeMax;
    std::string optimizationWayMethod;

    size_t routeNodeBlockSize;

    bool assumeLand;

    std::cout << "========================================" << std::endl;
    std::cout << "INFO: Using following Import parameters:" << std::endl;

    opOk = opOk && GetImportParamAsString(jImportConfig,"mapfile",mapfile);
    opOk = opOk && GetImportParamAsString(jImportConfig,"typefile",typefile);

    opOk = opOk && GetImportParamAsInt(jImportConfig,"startStep",startStep);
    opOk = opOk && GetImportParamAsInt(jImportConfig,"endStep",endStep);

    opOk = opOk && GetImportParamAsBool(jImportConfig,"strictAreas",strictAreas);

    opOk = opOk && GetImportParamAsBool(jImportConfig,"sortObjects",sortObjects);
    opOk = opOk && GetImportParamAsInt(jImportConfig,"sortBlockSize",sortBlockSize);
    opOk = opOk && GetImportParamAsInt(jImportConfig,"sortTileMag",sortTileMag);

    opOk = opOk && GetImportParamAsInt(jImportConfig,"numericIndexPageSize",numericIndexPageSize);

    opOk = opOk && GetImportParamAsBool(jImportConfig,"coordDataMemoryMapped",coordDataMemoryMapped);

    opOk = opOk && GetImportParamAsBool(jImportConfig,"rawNodeIndexMemoryMapped",rawNodeIndexMemoryMapped);
    opOk = opOk && GetImportParamAsBool(jImportConfig,"rawNodeDataMemoryMapped",rawNodeDataMemoryMapped);
    opOk = opOk && GetImportParamAsInt(jImportConfig,"rawNodeDataCacheSize",rawNodeDataCacheSize);
    opOk = opOk && GetImportParamAsInt(jImportConfig,"rawNodeIndexCacheSize",rawNodeIndexCacheSize);

    opOk = opOk && GetImportParamAsBool(jImportConfig,"rawWayIndexMemoryMapped",rawWayIndexMemoryMapped);
    opOk = opOk && GetImportParamAsBool(jImportConfig,"rawWayDataMemoryMapped",rawWayDataMemoryMapped);
    opOk = opOk && GetImportParamAsInt(jImportConfig,"rawWayDataCacheSize",rawWayDataCacheSize);
    opOk = opOk && GetImportParamAsInt(jImportConfig,"rawWayIndexCacheSize",rawWayIndexCacheSize);
    opOk = opOk && GetImportParamAsInt(jImportConfig,"rawWayBlockSize",rawWayBlockSize);

    opOk = opOk && GetImportParamAsBool(jImportConfig,"wayIndexMemoryMapped",wayIndexMemoryMapped);
    opOk = opOk && GetImportParamAsBool(jImportConfig,"wayDataMemoryMapped",wayDataMemoryMapped);
    opOk = opOk && GetImportParamAsInt(jImportConfig,"wayDataCacheSize",wayDataCacheSize);
    opOk = opOk && GetImportParamAsInt(jImportConfig,"wayIndexCacheSize",wayIndexCacheSize);
    opOk = opOk && GetImportParamAsInt(jImportConfig,"waysLoadSize",waysLoadSize);

    opOk = opOk && GetImportParamAsInt(jImportConfig,"areaAreaIndexMaxMag",areaAreaIndexMaxMag);

    opOk = opOk && GetImportParamAsInt(jImportConfig,"areaWayMinMag",areaWayMinMag);
    opOk = opOk && GetImportParamAsDouble(jImportConfig,"areaWayIndexMinFillRate",areaWayIndexMinFillRate);
    opOk = opOk && GetImportParamAsInt(jImportConfig,"areaWayIndexCellSizeAverage",areaWayIndexCellSizeAverage);
    opOk = opOk && GetImportParamAsInt(jImportConfig,"areaWayIndexCellSizeMax",areaWayIndexCellSizeMax);

    opOk = opOk && GetImportParamAsInt(jImportConfig,"areaNodeMinMag",areaNodeMinMag);
    opOk = opOk && GetImportParamAsDouble(jImportConfig,"areaNodeIndexMinFillRate",areaNodeIndexMinFillRate);
    opOk = opOk && GetImportParamAsInt(jImportConfig,"areaNodeIndexCellSizeAverage",areaNodeIndexCellSizeAverage);
    opOk = opOk && GetImportParamAsInt(jImportConfig,"areaNodeIndexCellSizeMax",areaNodeIndexCellSizeMax);

    opOk = opOk && GetImportParamAsInt(jImportConfig,"waterIndexMinMag",waterIndexMinMag);
    opOk = opOk && GetImportParamAsInt(jImportConfig,"waterIndexMaxMag",waterIndexMaxMag);

    opOk = opOk && GetImportParamAsInt(jImportConfig,"optimizationMaxWayCount",optimizationMaxWayCount);
    opOk = opOk && GetImportParamAsInt(jImportConfig,"optimizationMaxMag",optimizationMaxMag);
    opOk = opOk && GetImportParamAsInt(jImportConfig,"optimizationMinMag",optimizationMinMag);
    opOk = opOk && GetImportParamAsInt(jImportConfig,"optimizationCellSizeAverage",optimizationCellSizeAverage);
    opOk = opOk && GetImportParamAsInt(jImportConfig,"optimizationCellSizeMax",optimizationCellSizeMax);
    opOk = opOk && GetImportParamAsString(jImportConfig,"optimizationWayMethod",optimizationWayMethod);

    opOk = opOk && GetImportParamAsInt(jImportConfig,"routeNodeBlockSize",routeNodeBlockSize);

    opOk = opOk && GetImportParamAsBool(jImportConfig,"assumeLand",assumeLand);

    if(!opOk)   {
        std::cerr << "ERROR: There was an error "
                     "reading in a parameter!" << std::endl;
        return -1;
    }

    // [import parameter]
    osmscout::ImportParameter parameter;
    osmscout::ConsoleProgress progress;

    parameter.SetMapfile(mapfile);
    parameter.SetTypefile(typefile);

    parameter.SetSteps(startStep,endStep);

    parameter.SetStrictAreas(strictAreas);

    parameter.SetSortObjects(sortObjects);
    parameter.SetSortBlockSize(sortBlockSize);
    parameter.SetSortTileMag(sortTileMag);

    parameter.SetNumericIndexPageSize(numericIndexPageSize);

    parameter.SetCoordDataMemoryMaped(coordDataMemoryMapped);

    parameter.SetRawNodeIndexMemoryMaped(rawNodeIndexMemoryMapped);
    parameter.SetRawNodeDataMemoryMaped(rawNodeDataMemoryMapped);
    parameter.SetRawNodeDataCacheSize(rawNodeDataCacheSize);
    parameter.SetRawNodeIndexCacheSize(rawNodeIndexCacheSize);

    parameter.SetRawWayIndexMemoryMaped(rawWayIndexMemoryMapped);
    parameter.SetRawWayDataMemoryMaped(rawWayDataMemoryMapped);
    parameter.SetRawWayDataCacheSize(rawWayDataCacheSize);
    parameter.SetRawWayIndexCacheSize(rawWayIndexCacheSize);
    parameter.SetRawWayBlockSize(rawWayBlockSize);

    parameter.SetWayIndexMemoryMaped(wayIndexMemoryMapped);
    parameter.SetWayDataMemoryMaped(wayDataMemoryMapped);
    parameter.SetWayDataCacheSize(wayDataCacheSize);
    parameter.SetWayIndexCacheSize(wayIndexCacheSize);
    parameter.SetWaysLoadSize(waysLoadSize);

    parameter.SetAreaAreaIndexMaxMag(areaAreaIndexMaxMag);

    parameter.SetAreaWayMinMag(areaWayMinMag);
    parameter.SetAreaNodeIndexMinFillRate(areaNodeIndexMinFillRate);
    parameter.SetAreaNodeIndexCellSizeAverage(areaNodeIndexCellSizeAverage);
    parameter.SetAreaNodeIndexCellSizeMax(areaNodeIndexCellSizeMax);

    parameter.SetWaterIndexMinMag(waterIndexMinMag);
    parameter.SetWaterIndexMaxMag(waterIndexMaxMag);

    parameter.SetOptimizationMaxWayCount(optimizationMaxWayCount);
    parameter.SetOptimizationMaxMag(optimizationMaxMag);
    parameter.SetOptimizationMinMag(optimizationMinMag);
    parameter.SetOptimizationCellSizeAverage(optimizationCellSizeAverage);
    parameter.SetOptimizationCellSizeMax(optimizationCellSizeMax);
    if(optimizationWayMethod == "TransPolygon::none")
    {   parameter.SetOptimizationWayMethod(osmscout::TransPolygon::none);   }
    else if(optimizationWayMethod == "TransPolygon::fast")
    {   parameter.SetOptimizationWayMethod(osmscout::TransPolygon::fast);   }
    else if(optimizationWayMethod == "TransPolygon::quality")
    {   parameter.SetOptimizationWayMethod(osmscout::TransPolygon::quality);   }
    else   {
        std::cerr << "WARN: Unrecognized Way Optimization Method, "
                     "defaulting to none!" << std::endl;
        parameter.SetOptimizationWayMethod(osmscout::TransPolygon::none);
    }

    parameter.SetRouteNodeBlockSize(routeNodeBlockSize);

    parameter.SetAssumeLand(assumeLand);

    std::cout << "========================================" << std::endl;
    std::cout << "Starting Import..." << std::endl;
    std::cout << "========================================" << std::endl;

    if (osmscout::Import(parameter,progress))
    {   std::cerr << "Import OK!" << std::endl;   }
    else
    {   std::cerr << "Import failed!" << std::endl;   }


    return 0;
}
