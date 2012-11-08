#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

// libosmscout
#include <osmscout/TypeConfigLoader.h>
#include <osmscout/TypeConfig.h>

int main()
{
    std::string typeFile("/home/preet/Dev/scratch/osmscout/typeconfig.ost");
    osmscout::TypeConfig typeConfig;
    osmscout::LoadTypeConfig(typeFile.c_str(),typeConfig);

    std::vector<osmscout::TypeInfo> listTypeInfo;
    listTypeInfo = typeConfig.GetTypes();

    // list all types
    for(size_t i=0; i < listTypeInfo.size(); i++)   {
        std::cout << "Type Id: " << listTypeInfo[i].GetId() << std::endl;
        std::cout << "Type Name: " << listTypeInfo[i].GetName() << std::endl;
        std::cout << "Type Object: ";
        if(listTypeInfo[i].CanBeNode())   {
            std::cout << "Node";
        }
        else if(listTypeInfo[i].CanBeWay())   {
            std::cout << "Way";
        }
        else if(listTypeInfo[i].CanBeArea())   {
            std::cout << "Area";
        }
        std::cout << std::endl;
    }

    //
    osmscout::TypeId nodeType = typeConfig.GetNodeTypeId("custom_type1");
    std::cout << "TYPEID: " << nodeType << std::endl;

    return 0;
}
