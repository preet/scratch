#include <QGuiApplication>
#include <QQuickView>
#include <QTimer>
#include <QDebug>

#include <jni.h>

// ========================================================================== //
// ========================================================================== //

static jclass JniHelperFindClassId(JNIEnv * env, char const * className)
{
    jclass localRefClassId = env->FindClass(className);

    if(!localRefClassId)   {
        qDebug() << "JNI: failed to find class id:"<< className;
        return 0;
    }
    jclass globalRefClassId = (jclass)env->NewGlobalRef(localRefClassId);
    env->DeleteLocalRef(localRefClassId);
    return globalRefClassId;
}

static jmethodID JniHelperFindMethodId(JNIEnv * env,
                                       jclass classId,
                                       char const * methodName,
                                       char const * methodSig)
{
    jmethodID methodIdRef = env->GetMethodID(classId,methodName,methodSig);
    if(!methodIdRef)   {
        qDebug() << "JNI: failed to find method id:"<< methodName;
        return 0;
    }
    return methodIdRef;
}

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM * vm, void * reserved)
{
    JNIEnv * env = NULL;
    if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK) {
        qDebug() << "JNI_OnLoad: Can't get environment!";
        return -1;
    }

    // Find classes
    jclass jni_cid_BluetoothAdapter =
            JniHelperFindClassId(env,"android/bluetooth/BluetoothAdapter");

    jclass jni_cid_BluetoothDevice =
            JniHelperFindClassId(env,"android/bluetooth/BluetoothDevice");

    jclass jni_cid_BluetoothSocket =
            JniHelperFindClassId(env,"android/bluetooth/BluetoothSocket");

    jclass jni_cid_UUID =
            JniHelperFindClassId(env,"java/util/UUID");

    jclass jni_cid_InputStream =
            JniHelperFindClassId(env,"java/io/InputStream");

    jclass jni_cid_OutputStream =
            JniHelperFindClassId(env,"java/io/OutputStream");

    jclass jni_cid_Exception =
            JniHelperFindClassId(env,"java/lang/Exception");

    jclass jni_cid_Set =
            JniHelperFindClassId(env,"java/util/Set");


    // Check classes
    QList<jclass> listClassIds;
    listClassIds.push_back(jni_cid_BluetoothAdapter);
    listClassIds.push_back(jni_cid_BluetoothDevice);
    listClassIds.push_back(jni_cid_BluetoothSocket);
    listClassIds.push_back(jni_cid_UUID);
    listClassIds.push_back(jni_cid_InputStream);
    listClassIds.push_back(jni_cid_OutputStream);
    listClassIds.push_back(jni_cid_Exception);

    for(int i=0; i < listClassIds.size(); i++)   {
        if(!listClassIds[i])   {
            return -1;
        }
    }

    jmethodID jni_mid_Exception_getMessage =
            JniHelperFindMethodId(env,jni_cid_Exception,
                                  "getMessage",
                                  "()Ljava/lang/String;");

    // ====================================================================== //

    // Get a reference to the local device's bluetooth adapter
    jmethodID jni_mid_getDefaultAdapter =
            env->GetStaticMethodID(jni_cid_BluetoothAdapter,
                                   "getDefaultAdapter",
                                   "()Landroid/bluetooth/BluetoothAdapter;");

    if(!jni_mid_getDefaultAdapter)
    { qDebug() << "JNI: j_mid_getDefaultAdapter NULL"; return -1; }

    jobject jni_ref_BluetoothAdapter =
            env->CallStaticObjectMethod(jni_cid_BluetoothAdapter,
                                        jni_mid_getDefaultAdapter);
    if(!jni_ref_BluetoothAdapter)
    { qDebug() << "JNI: j_ref_BluetoothAdapter NULL"; return -1; }


    // Get a list of currently paired devices
    // Set<BluetoothDevice> = BluetoothAdapter.getBondedDevices();
    jmethodID jni_mid_getBondedDevices =
            env->GetMethodID(jni_cid_BluetoothAdapter,
                             "getBondedDevices",
                             "()Ljava/util/Set;");
    if(!jni_mid_getBondedDevices)
    { qDebug() << "JNI: jni_mid_getBondedDevices NULL"; return -1; }

    jobject jni_ref_setPairedDevices =
            env->CallObjectMethod(jni_ref_BluetoothAdapter,
                                  jni_mid_getBondedDevices);
    if(!jni_ref_setPairedDevices)
    { qDebug() << "JNI: jni_ref_setPairedDevices"; return -1; }


    // Convert the set to an object array
    // object[] array = Set<BluetoothDevice>.toArray();
    jmethodID jni_mid_toArray =
            env->GetMethodID(jni_cid_Set,
                             "toArray",
                             "()[Ljava/lang/Object;");
    if(!jni_mid_toArray)
    { qDebug() << "JNI: jni_mid_toArray"; return -1; }

    jobjectArray jni_arrayPairedDevices = (jobjectArray)
            env->CallObjectMethod(jni_ref_setPairedDevices,
                                  jni_mid_toArray);
    if(!jni_arrayPairedDevices)
    { qDebug() << "JNI: jni_arrayPairedDevices"; return -1; }

    // Get information from array of devices
    jsize len = env->GetArrayLength(jni_arrayPairedDevices);
    for(jsize i=0; i < len; i++)   {
        // Call BluetoothDevice.getName();
        //      BluetoothDevice.getAddress();

        jobject j_ref_device =
                env->GetObjectArrayElement(jni_arrayPairedDevices,i);

        jmethodID j_mid_getName =
                env->GetMethodID(jni_cid_BluetoothDevice,
                                 "getName",
                                 "()Ljava/lang/String;");
        jmethodID j_mid_getAddress =
                env->GetMethodID(jni_cid_BluetoothDevice,
                                 "getAddress",
                                 "()Ljava/lang/String;");

        jstring j_str_name = (jstring)
                env->CallObjectMethod(j_ref_device,
                                      j_mid_getName);
        if(j_str_name)   {
            jboolean isCopy;
            const char * name = env->GetStringUTFChars(j_str_name,&isCopy);
            qDebug() << "device" << i << "name:"<< QString(name);
            env->ReleaseStringUTFChars(j_str_name,name);
        }

        jstring j_str_addr = (jstring)
                env->CallObjectMethod(j_ref_device,
                                      j_mid_getAddress);
        if(j_str_addr)   {
            jboolean isCopy;
            const char * addr = env->GetStringUTFChars(j_str_addr,&isCopy);
            qDebug() << "device" << i << "addr" << QString(addr);
            env->ReleaseStringUTFChars(j_str_addr,addr);
        }
    }












    return JNI_VERSION_1_6;
}


// ========================================================================== //
// ========================================================================== //

int main(int argc, char **argv)
{
    QGuiApplication app(argc, argv);

    QQuickView * view = new QQuickView;
    view->setResizeMode(QQuickView::SizeRootObjectToView);
    view->setSource(QUrl("qrc:/main.qml"));
    view->showFullScreen();
//    view->showMaximized();
//    view->show();

    return app.exec();
}
