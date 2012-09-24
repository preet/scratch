#ifndef MYPLUGINIMPLEMENTATION_H
#define MYPLUGINIMPLEMENTATION_H

// Qt Base Includes
#include <QtPlugin>
#include <QMutex>
#include <QDebug>
#include <QObject>
#include <QString>

// interface includes
#include "plugin_interface.h"

class MyPluginImp : public QObject,public MyPluginInterface
{
   Q_OBJECT

public:
   MyPluginImp();
   void SomeMethod();

private:
   void anotherMethod();
};

// hehehe
class MyImpFactory : public QObject,public MyPluginFactory
{
   Q_OBJECT
   Q_INTERFACES(MyPluginFactory)

public:
   MyPluginInterface * NewPlugin() = 0;
}

#endif
