#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <QCoreApplication>
#include <QQuickView>
#include <QQmlContext>
#include <QObject>
#include <QDebug>
#include <QThread>
#include <QDir>

#include <jni.h>

class BluetoothSerial : public QObject
{
    Q_OBJECT

public:
    explicit BluetoothSerial(QQuickView * qqview,
                             JavaVM * javaVM,
                             jclass j_classid_prisActivity,
                             jclass j_classid_prisLocationListener,
                             jclass j_classid_context,
                             jclass j_classid_activity,
                             jclass j_classid_handlerThread,
                             jclass j_classid_bluetoothAdapter,
                             jclass j_classid_bluetoothDevice,
                             jclass j_classid_prisBluetoothManager,
                             jclass j_classid_prisBluetoothStatusReceiver,
                             jobject j_ref_prisActivityInstance,
                             QObject * parent=0);
public slots:
    void Initialize();

signals:
    void bluetoothAdapterStatusChanged(bool status);

private:
    JavaVM * m_javaVM;
    jclass m_j_classid_prisActivity;
    jclass m_j_classid_prisLocationListener;
    jclass m_j_classid_context;
    jclass m_j_classid_activity;
    jclass m_j_classid_handlerThread;
    jclass m_j_classid_bluetoothAdapter;
    jclass m_j_classid_bluetoothDevice;
    jclass m_j_classid_prisBluetoothStatusReceive;
    jclass m_j_classid_prisBluetoothManager;
    jobject m_j_ref_prisActivityInstance;

    bool m_init;
};


#endif // BLUETOOTH_H
