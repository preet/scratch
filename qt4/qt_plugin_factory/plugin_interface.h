#ifndef PLUGININTERFACE_H
#define PLUGININTERFACE_H

// Qt Base Includes
#include <QtPlugin>
#include <QMutex>
#include <QDebug>
#include <QObject>
#include <QString>

namespace pris
{
    class PluginHelper;

    class IPluginMain : public QObject
    {
        Q_OBJECT

    public:
        IPluginMain(QObject *parent = 0);
        virtual ~IPluginMain();

        virtual bool Initialize() = 0;
        virtual void OnAllPluginsInitialized() = 0;
        virtual QObject const *GetObject() const = 0;
        virtual QString GetName() const = 0;
        virtual QString GetDesc() const = 0;
        virtual QString GetTag() const = 0;
        virtual QString GetVersion() const = 0;
        virtual QString GetDependencies() const = 0;
        virtual bool GetAccessible() const = 0;
//        virtual void SetPluginHelper(PluginHelper *) = 0;
    };

    class IPluginDisplay : public QObject
    {
        Q_OBJECT

    public:
        IPluginDisplay(QObject *parent=0);
        virtual ~IPluginDisplay();

        virtual void Display() = 0;
        virtual void Hide() = 0;
    };

    class IPlugin
    {
    public:
        virtual ~IPlugin() {}

        // plugin developer must implement these two
        virtual IPluginMain    * GetPluginMain()  = 0;
        virtual IPluginDisplay * GetPluginDisplay() = 0;
    };
}

Q_DECLARE_INTERFACE(pris::IPlugin,"com.prismatic.IPlugin/1.0")

#endif  // PLUGININTERFACE_H
