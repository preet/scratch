#include <iostream>
#include <vector>

#include <QtGui/QApplication>
#include <QtOpenGL/QGLWidget>
#include <QtDeclarative/QtDeclarative>
#include <QtDeclarative/QDeclarativeView>

#include <osmscout/Database.h>
#include <osmscout/TypeConfig.h>

int main(int argc, char *argv[])
{
    QApplication app(argc,argv);
    QDeclarativeView mainView;
    mainView.setSource(QString("app/native/ui/main.qml"));

    // do some libosmscout stuff
//    std::string dataPath("app/native/maps/toronto");
//    osmscout::DatabaseParameter databaseParam;
//    osmscout::Database database(databaseParam);
//    if(database.Open(dataPath))
//    {   std::cout << "INFO: Opened Database Successfully" << std::endl;   }
//    else
//    {   std::cout << "ERROR: Could not open database" << std::endl;   }

//    osmscout::TypeSet typeSet;
//    osmscout::TypeConfig * typeConfig = database.GetTypeConfig();
//    std::vector<osmscout::TypeInfo> listTypeInfo = typeConfig->GetTypes();
//    for(int i=0; i < listTypeInfo.size(); i++)
//    {   typeSet.SetType(listTypeInfo[i].GetId());   }

//    osmscout::WayRef wayRef;
//    database.GetWay(7991747,wayRef);

//    std::cout << "Way Id: " << wayRef->GetId() << std::endl;
//    osmscout::TypeInfo typeInfo = typeConfig->GetTypeInfo(wayRef->GetType());
//    std::cout << "Way Type: " << typeInfo.GetName() << std::endl;
//    std::cout << "Way Name: " << wayRef->GetName() << std::endl;
//    std::cout << "Way Nodes: " << std::endl;
//    for(int i=0; i < wayRef->nodes.size(); i++)   {
//        std::cout << i
//                  << ", Id: " << wayRef->nodes[i].GetId()
//                  << ", Lat: " << wayRef->nodes[i].GetLat()
//                  << ", Lon: " << wayRef->nodes[i].GetLon()
//                  << std::endl;
//    }




    mainView.show();

    return app.exec();
}
