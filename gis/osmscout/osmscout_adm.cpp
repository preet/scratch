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

    std::string dataPath("/home/preet/Dev/build/osmscout/osmscout_import/mapmixadm");
    osmscout::DatabaseParameter databaseParam;
    osmscout::Database database(databaseParam);

    opOk = database.Open(dataPath);
    if(!opOk)  {   std::cerr << "ERROR: Could not open database\n";    return -1;   }

    // get dataset bounds
    double minLat,minLon,maxLat,maxLon;
    opOk = database.GetBoundingBox(minLat,minLon,maxLat,maxLon);
    if(!opOk)  {   std::cerr << "ERROR: Could not get data bounds\n";  return -1;   }

    //
    osmscout::TypeSet typeSet(*(database.GetTypeConfig()));

    //
    std::vector<std::string> listTypeNames;
    listTypeNames.push_back("mapmix_adm0");
    listTypeNames.push_back("mapmix_adm1");
    for(size_t i=0; i < listTypeNames.size(); i++)
    {
        osmscout::TypeId nodeType =
                database.GetTypeConfig()->GetNodeTypeId(listTypeNames[i]);

        if(nodeType == osmscout::typeIgnore)   {
            std::cerr << "Could not resolve type " << listTypeNames[i] << std::endl;
            return -1;
        }

        typeSet.SetType(nodeType);
    }

    std::vector<osmscout::NodeRef> listNodes;
    std::vector<osmscout::WayRef>  listWays;
    std::vector<osmscout::WayRef>  listAreas;
    std::vector<osmscout::RelationRef> listRelWays;
    std::vector<osmscout::RelationRef> listRelAreas;
    osmscout::TagId tagNameId = database.GetTypeConfig()->GetTagId("name");
    if(!database.GetObjects(minLon,minLat,maxLon,maxLat,
                            typeSet,
                            listNodes,
                            listWays,
                            listAreas,
                            listRelWays,
                            listRelAreas))
    {   std::cerr << "Could not get objects from database\n";  return -1;   }

    std::cerr << "Found " << listNodes.size() << " nodes" << std::endl;
    for(size_t i=0; i < listNodes.size(); i++)
    {
        std::string nodeName;
        for(size_t j=0; j < listNodes[i]->GetTagCount(); j++)   {
            if(listNodes[i]->GetTagKey(j) == tagNameId)   {
                nodeName = listNodes[i]->GetTagValue(j);
                break;
            }
        }

        std::cout << "Node: " << listNodes[i]->GetId() << std::endl;
        std::cout << "Type: "
                  << database.GetTypeConfig()->GetTypeInfo(listNodes[i]->GetType()).GetName()
                  << std::endl;
        std::cout << "Name: " << nodeName << std::endl;
    }


    return 0;
}
