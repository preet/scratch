
#include <QApplication>
#include <QObject>
#include <QPluginLoader>

#include "plugin_implementation.h"

int main(int argc, char *argv[])
{
   if(argc < 2)
   {   qDebug() << "Error! Pass in Plugin File Name!"; return -1;   }

   QApplication app(argc,argv);
   QString pluginFilePath = QString(argv[1]);
   QPluginPloader pluginLoader(pluginFilePath);

   return app.exec();
}
