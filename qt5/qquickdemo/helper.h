#ifndef HELPER_H
#define HELPER_H

#include <QCoreApplication>
#include <QQuickView>
#include <QQmlContext>
#include <QObject>
#include <QDebug>
#include <QThread>
#include <QDir>

#include "QtPlatformSupport/5.1.2/QtPlatformSupport/private/qjniobject_p.h"
#include "QtPlatformSupport/5.1.2/QtPlatformSupport/private/qjnihelpers_p.h"
//#include "androidjnimain.h"

//#include "helloworld.h"

class Helper : public QObject
{
    Q_OBJECT

public:
    explicit Helper(QQuickView * qqview, QObject * parent=0);

    Q_INVOKABLE QString getPathApp();
    Q_INVOKABLE QString getPathUi();
    Q_INVOKABLE QString getPathPlugins();
    Q_INVOKABLE QString getPathData();
    Q_INVOKABLE QString getPathLogs();
    Q_INVOKABLE QString getPathSettings();
    Q_INVOKABLE QString getPathUser();

    Q_INVOKABLE QString getQuote();

private:
    QString path_ext;
    QString path_app;
    QString path_ui;
    QString path_logs;
    QString path_settings;
    QString path_plugins;
    QString path_data;
    QString path_user;
    //HelloWorld m_helloworld;
};

#endif // HELPER_H
