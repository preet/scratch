#include <QGuiApplication>
#include <QQuickView>
#include <QTimer>
#include <QMutex>
//#include "anjni.h"
//#include "helper.h"

#include <jni.h>
//#include "geolocation.h"
#include "bluetoothserial.h"


JavaVM *    g_java_vm = 0;
jclass      g_myQtActivityClassID = 0;
jobject     g_myQtActivityInstance = 0;


jclass j_classid_prisActivity = 0;
jclass j_classid_context = 0;
jclass j_classid_activity = 0;
jclass j_classid_handlerThread = 0;

jclass j_classid_prisLocationListener = 0;
jclass j_classid_locationManager = 0;

jclass j_classid_bluetoothAdapter=0;
jclass j_classid_bluetoothDevice=0;
jclass j_classid_prisBluetoothManager=0;
jclass j_classid_prisBluetoothStatusReceiver=0;

jobject j_ref_prisActivityInstance = 0;


bool g_activityReady=false;
QMutex g_mutex_activityReady;

// ref:
// android sample "SimpleJNI"

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM * vm,void * reserved)
{
    g_java_vm = vm;
    Q_UNUSED(reserved);

    qDebug() << "####: JNI_onLoad\n\n\n";

    return JNI_VERSION_1_6;
}

jclass FindClass(JNIEnv * env, char const * className)
{
    jclass temp = env->FindClass(className);
    if(temp)   {
        qDebug() << "####: Found class " << QString(className);
       return (jclass)env->NewGlobalRef(temp);
    }
    else   {
        qDebug() << "####: Failed to find class " << QString(className);
        return 0;
    }
}


extern "C" {
    JNIEXPORT void JNICALL Java_ca_predesign_prismatic_PrisActivity_activityReady
    (JNIEnv *env, jobject obj)
    {
        Q_UNUSED(env);
        Q_UNUSED(obj);


        // find classes
        j_classid_prisActivity =
                FindClass(env,"ca/predesign/prismatic/PrisActivity");

        j_classid_prisLocationListener =
                FindClass(env,"ca/predesign/prismatic/PrisLocationListener");

        j_classid_context =
                FindClass(env,"android/content/Context");

        j_classid_activity =
                FindClass(env,"android/app/Activity");

        j_classid_locationManager =
                FindClass(env,"android/location/LocationManager");

        j_classid_handlerThread =
                FindClass(env,"android/os/HandlerThread");

        j_ref_prisActivityInstance = env->NewGlobalRef(obj);


        // bluetooth
        j_classid_bluetoothAdapter =
                FindClass(env,"android/bluetooth/BluetoothAdapter");

        j_classid_bluetoothDevice =
                FindClass(env,"android/bluetooth/BluetoothDevice");

        j_classid_prisBluetoothManager =
                FindClass(env,"ca/predesign/prismatic/PrisBluetoothManager");

        j_classid_prisBluetoothStatusReceiver =
                FindClass(env,"ca/predesign/prismatic/PrisBluetoothStatusReceiver");


        qDebug() << "####: activityReady\n\n\n";

        //
        jclass prisActivityClassId =
                env->FindClass("ca/predesign/prismatic/PrisActivity");

        g_myQtActivityClassID = (jclass)env->NewGlobalRef(prisActivityClassId);

        // obj = MyQtActivity instance; save it
        g_myQtActivityInstance = env->NewGlobalRef(obj);

        // get the activity class
        jclass activityClassId = env->FindClass("android/app/Activity");

        // get the getApplicationContext method id
        jmethodID getApplicationContextMethodId =
                env->GetMethodID(activityClassId,
                                 "getApplicationContext",
                                 "()Landroid/content/Context;");

        // call Activity.getApplicationContextMethodId()
        jobject contextRef =
                env->CallObjectMethod(g_myQtActivityInstance,
                                      getApplicationContextMethodId);

        // get the context class
        jclass contextClassId = env->FindClass("android/content/Context");

        // get the getFilesDir method id
        jmethodID getFilesDirMethodId =
                env->GetMethodID(contextClassId,
                                 "getFilesDir",
                                 "()Ljava/io/File;");

        // call Context.getFilesDir()
        jobject fileRef =
                env->CallObjectMethod(contextRef,
                                      getFilesDirMethodId);

        // get the file class
        jclass fileClassId = env->FindClass("java/io/File");

        // get the getAbsolutePath method id
        jmethodID getAbsolutePathMethodId =
                env->GetMethodID(fileClassId,
                                 "getAbsolutePath",
                                 "()Ljava/lang/String;");

        // call File.getAbsolutePath()
        jobject stringRef =
                env->CallObjectMethod(fileRef,
                                      getAbsolutePathMethodId);

        // convert the absolute path into a string
        // do I need to clean up GetStringUTFChars somehow?
        const char * jstr = env->GetStringUTFChars(jstring(stringRef),NULL);
        qDebug() << "####: Internal Storage Path: "
                 << QString(jstr) << "\n\n\n";

        while(1)   {
            if(g_mutex_activityReady.tryLock())   {
                g_activityReady = true;
                g_mutex_activityReady.unlock();
                break;
            }
        }
    }
}

int main(int argc, char **argv)
{
    qDebug() << "####: main()\n\n\n";

    // this is an extremely evil hack:
    while(1)   {
        if(g_mutex_activityReady.tryLock())   {
            bool activityReady = g_activityReady;
            g_mutex_activityReady.unlock();
            if(activityReady)   {
                break;
            }
        }
    }

    qDebug() << "####: main after activityReady\n\n\n";

    QGuiApplication app(argc, argv);
    QQuickView * view = new QQuickView;
//    GeoLocation * geoloc = new GeoLocation(view,
//                                           g_java_vm,
//                                           j_classid_prisActivity,
//                                           j_classid_prisLocationListener,
//                                           j_classid_context,
//                                           j_classid_activity,
//                                           j_classid_locationManager,
//                                           j_classid_handlerThread,
//                                           j_ref_prisActivityInstance);

    BluetoothSerial * bluetoothserial =
            new BluetoothSerial(view,
                                g_java_vm,
                                j_classid_prisActivity,
                                j_classid_prisLocationListener,
                                j_classid_context,
                                j_classid_activity,
                                j_classid_handlerThread,
                                j_classid_bluetoothAdapter,
                                j_classid_bluetoothDevice,
                                j_classid_prisBluetoothManager,
                                j_classid_prisBluetoothStatusReceiver,
                                j_ref_prisActivityInstance);

    view->setResizeMode(QQuickView::SizeRootObjectToView);
    view->setSource(QUrl("assets:/main.qml"));

//    view->showFullScreen();
//    view->showMaximized();
    view->show();

    QTimer::singleShot(2000,bluetoothserial,SLOT(Initialize()));

    return app.exec();
}
