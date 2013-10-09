#include "bluetoothserial.h"

static BluetoothSerial * g_bluetoothserial = NULL;

static void jni_onBluetoothAdapterStatusChanged(JNIEnv * env,
                                                jobject * obj,
                                                bool status)
{
    Q_UNUSED(env);
    Q_UNUSED(obj);

    if(g_bluetoothserial)   {
        g_bluetoothserial->bluetoothAdapterStatusChanged(status);
    }
}

static JNINativeMethod methods[] =
{
    {"jni_onBluetoothAdapterStatusChanged","(Z)V",(void*)jni_onBluetoothAdapterStatusChanged}
};

BluetoothSerial::BluetoothSerial(QQuickView * qqview,
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
                                 QObject * parent) :
    QObject(parent),
    m_init(false)
{
    m_javaVM                            = javaVM;
    m_j_classid_prisActivity            = j_classid_prisActivity;
    m_j_classid_prisLocationListener    = j_classid_prisLocationListener;
    m_j_classid_context                 = j_classid_context;
    m_j_classid_activity                = j_classid_activity;
    m_j_classid_handlerThread           = j_classid_handlerThread;
    m_j_classid_bluetoothAdapter        = j_classid_bluetoothAdapter;
    m_j_classid_bluetoothDevice         = j_classid_bluetoothDevice;
    m_j_classid_prisBluetoothManager    = j_classid_prisBluetoothManager;
    m_j_classid_prisBluetoothStatusReceive =
            j_classid_prisBluetoothStatusReceiver;
    m_j_ref_prisActivityInstance        = j_ref_prisActivityInstance;

    g_bluetoothserial = this;
    qqview->rootContext()->setContextProperty("BluetoothSerial",this);
}

//void BluetoothSerial::Initialize()
//{
//    if(m_init)   {
//        qDebug() << "####: Initialize() repeat attempt";
//        return;
//    }

//    qDebug() << "####: Initialize(): BEGIN";

//    // Get JNIenv for current thread
//    JNIEnv * env;
//    if(m_javaVM->AttachCurrentThread(&env,NULL) < 0)   {
//        qDebug() << "####: ERROR: Initialize(): "
//                    "Could not attach thread to JavaVM";
//        return;
//    }

//    m_init = true;
//}

void BluetoothSerial::Initialize()
{
    if(m_init)   {
        qDebug() << "####: Initialize() repeat attempt";
        return;
    }

    qDebug() << "####: Initialize(): BEGIN";

    // Get JNIenv for current thread
    JNIEnv * env;
    if(m_javaVM->AttachCurrentThread(&env,NULL) < 0)   {
        qDebug() << "####: ERROR: Initialize(): "
                    "Could not attach thread to JavaVM";
        return;
    }

    // (register methods) ---------------------------
    env->RegisterNatives(m_j_classid_prisBluetoothStatusReceive,
                         methods,
                         sizeof(methods)/sizeof(methods[0]));
    // ----------------------------------------------

    // Create an instance of PrisBluetoothManager

    // should we even be using a PrisBluetoothManager class?

    jmethodID prisBluetoothManagerConstructor =
            env->GetMethodID(m_j_classid_prisBluetoothManager,
                             "<init>","()V");

    jobject prisBluetoothManagerRef =
            env->CallObjectMethod(m_j_classid_prisBluetoothManager,
                                  prisBluetoothManagerConstructor);

    // Initialize



    // Get a reference to the Bluetooth Adapter
//    jmethodID getDefaultAdapter =
//            env->GetStaticMethodID(m_j_classid_bluetoothAdapter,
//                                   "getDefaultAdapter",
//                                   "()Landroid/bluetooth/BluetoothAdapter;");

//    jobject bluetoothAdapterRef =
//            env->CallStaticObjectMethod(m_j_classid_bluetoothAdapter,
//                                        getDefaultAdapter);

//    // Verify a bluetooth adapter exists on the
//    // local device. It may be better to do this
//    // check on the Java side of things.
//    if(!bluetoothAdapterRef)   {
//        qDebug() << "####: ERROR: Bluetooth Adapter not "
//                    "available on this device";
//        return;
//    }

//    // Check if Bluetooth is enabled
//    jmethodID isEnabled =
//            env->GetMethodID(m_j_classid_bluetoothAdapter,
//                             "isEnabled",
//                             "()Z");

//    jboolean btEnabled =
//            env->CallBooleanMethod(bluetoothAdapterRef,
//                                   isEnabled);

//    if(!btEnabled)   {
//        qDebug() << "####: WARN: Bluetooth Adapter not "
//                    "enabled, turn it on!";
//        return;
//    }

    // Ask the bluetooth adapter to show us
    // a list of discoverable devices

    // ...
    m_init = true;
}






























