#ifndef GEOLOCATION_H
#define GEOLOCATION_H

#include <QCoreApplication>
#include <QQuickView>
#include <QQmlContext>
#include <QObject>
#include <QDebug>
#include <QThread>
#include <QDir>

#include <jni.h>

class GeoLocation : public QObject
{
    Q_OBJECT

public:
    explicit GeoLocation(QQuickView * qqview,
                         JavaVM * javaVM,
                         jclass j_classid_prisActivity,
                         jclass j_classid_prisLocationListener,
                         jclass j_classid_context,
                         jclass j_classid_activity,
                         jclass j_classid_locationManager,
                         jclass j_classid_handlerThread,
                         jobject j_ref_prisActivityInstance,
                         QObject * parent=0);
public slots:
    void Initialize();

signals:
    void locationChanged(QString utc_time,
                         double lon,
                         double lat);

    void statusChanged(QString status);

    void providerEnabled(QString provider);

    void providerDisabled(QString provider);

private:
    JavaVM * m_javaVM;
    jclass m_j_classid_prisActivity;
    jclass m_j_classid_prisLocationListener;
    jclass m_j_classid_context;
    jclass m_j_classid_activity;
    jclass m_j_classid_locationManager;
    jclass m_j_classid_handlerThread;
    jobject m_j_ref_prisActivityInstance;

    bool m_init;
};


#endif // GEOLOCATION_H
