#include "plugin_implementation.h"

MyPluginImp::MyPluginImp()
{   qDebug() << "MyPluginImp Constructor";   }

void MyPluginImp::SomeMethod()
{
   qDebug() << "MyPluginImp SomeMethod()";
   this->anotherMethod();
   this->anotherMethod();
}

void MyPluginImp::anotherMethod()
{   qDebug() << "MyPluginImp anotherMethod()";   }



MyPluginInterface * MyImpFactory::NewPlugin()
{
   new MyPluginImp();
}
