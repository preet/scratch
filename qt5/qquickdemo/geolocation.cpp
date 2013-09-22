#include "geolocation.h"


static GeoLocation * g_geolocation = NULL;


// example ref:
// andoid samples "SimpleJNI"

static void jni_onLocationChanged(JNIEnv * env,
                                  jobject * obj,
                                  long utc_time,
                                  double lon,
                                  double lat,
                                  double alt,
                                  double speed,
                                  double heading,
                                  double err_lonlat,
                                  int status)
{
//    return;

    Q_UNUSED(env);
    Q_UNUSED(obj);
    Q_UNUSED(alt);
    Q_UNUSED(speed);
    Q_UNUSED(heading);
    Q_UNUSED(err_lonlat);
    Q_UNUSED(status);

    if(g_geolocation == NULL)   {
        return;
    }

    QString timestamp = QString::number(utc_time,10);
    qDebug() << "####: jni_onLocationChanged: "
             << timestamp << "," << lon << "," << lat;
    g_geolocation->locationChanged(timestamp,lon,lat);
}

// status
// value:   meaning:
// 0        TEMPORARILY_UNAVAILABLE
// 1        OUT_OF_SERVICE
// 2        AVAILABLE
static void jni_onStatusChanged(JNIEnv * env,
                                jobject * obj,
                                int status)
{
//    return;

    Q_UNUSED(env);
    Q_UNUSED(obj);

    if(g_geolocation == NULL)   {
        return;
    }

    QString status_desc;
    if(status == 0)   {
        status_desc = "TEMPORARILY UNAVAILABLE";
    }
    else if(status == 1)   {
        status_desc = "OUT_OF_SERVICE";
    }
    else if(status == 2)   {
        status_desc = "AVAILABLE";
    }
    else   {
        status_desc = "ERROR";
    }
    qDebug() << "####: jni_onStatusChanged: " << status_desc;
    g_geolocation->statusChanged(status_desc);
}

// provider
// value:   meaning:
// 0        GPS_PROVIDER
// 1        NETWORK_PROVIDER
// 2        PASSIVE_PROVIDER
static void jni_onProviderEnabled(JNIEnv * env,
                                  jobject * obj,
                                  int provider)
{
//    return;

    Q_UNUSED(env);
    Q_UNUSED(obj);

    // Called when the provider is enabled by the user
    if(g_geolocation == NULL)   {
        return;
    }

    QString p_desc;
    if(provider == 0)   {
        p_desc = "GPS";
    }
    else if(provider == 1)   {
        p_desc = "NETWORK";
    }
    else if(provider == 2)   {
        p_desc = "PASSIVE";
    }
    else   {
        p_desc = "ERROR";
    }

    qDebug() << "####: jni_onProviderEnabled: " << p_desc;
    g_geolocation->providerEnabled(p_desc);
}

// provider
// value:   meaning:
// 0        GPS_PROVIDER
// 1        NETWORK_PROVIDER
// 2        PASSIVE_PROVIDER
static void jni_onProviderDisabled(JNIEnv * env,
                                   jobject * obj,
                                   int provider)
{
//    return;

    Q_UNUSED(env);
    Q_UNUSED(obj);

    // Called when the provider is disabled by the user
    if(g_geolocation == NULL)   {
        return;
    }

    QString p_desc;
    if(provider == 0)   {
        p_desc = "GPS";
    }
    else if(provider == 1)   {
        p_desc = "NETWORK";
    }
    else if(provider == 2)   {
        p_desc = "PASSIVE";
    }
    else   {
        p_desc = "ERROR";
    }

    qDebug() << "####: jni_onProviderDisabled: " << p_desc;
    g_geolocation->providerDisabled(p_desc);
}

static JNINativeMethod methods[] =
{
    {"nativeOnLocationChanged","(JDDDDDDI)V",(void*)jni_onLocationChanged},
    {"nativeOnStatusChanged","(I)V",(void*)jni_onStatusChanged},
    {"nativeOnProviderEnabled","(I)V",(void*)jni_onProviderEnabled},
    {"nativeOnProviderDisabled","(I)V",(void*)jni_onProviderDisabled}
};

// ============================================================ //
// ============================================================ //

GeoLocation::GeoLocation(QQuickView * qqview,
                         JavaVM * javaVM,
                         jclass j_classid_prisActivity,
                         jclass j_classid_prisLocationListener,
                         jclass j_classid_context,
                         jclass j_classid_activity,
                         jclass j_classid_locationManager,
                         jclass j_classid_handlerThread,
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
    m_j_classid_locationManager         = j_classid_locationManager;
    m_j_classid_handlerThread           = j_classid_handlerThread;
    m_j_ref_prisActivityInstance        = j_ref_prisActivityInstance;

    g_geolocation = this;
    qqview->rootContext()->setContextProperty("GeoLocation",this);
}

void GeoLocation::Initialize()
{
    if(m_init)   {
        qDebug() << "####: Initialize() already attemped!";
        return;
    }

    qDebug() << "####: Initialize() begin";

    // start geolocation service using context
    JNIEnv * env;
    if(m_javaVM->AttachCurrentThread(&env,NULL) < 0)   {
        qDebug() << "####: ERROR: Initialize(): "
                    "Could not attach thread to JavaVM";
        return;
    }

    // get PrisLocationListener class id and
    // instantiate an object of this class
    jclass prisLocLisClassId = m_j_classid_prisLocationListener;


    // (register methods) ---------------------------
    env->RegisterNatives(prisLocLisClassId,
                         methods,
                         sizeof(methods)/sizeof(methods[0]));
    // ----------------------------------------------

    qDebug() << "####: Initialize() mid";


    // get the Context class id
    jclass contextClassId = m_j_classid_context;

    // get the static field id Context.LOCATION_SERVICE
    jfieldID locationServiceFieldId =
            env->GetStaticFieldID(contextClassId,
                                  "LOCATION_SERVICE",
                                  "Ljava/lang/String;");

    // get the field value
    jstring locationServiceStr =
            (jstring)env->GetStaticObjectField(contextClassId,
                                      locationServiceFieldId);



    // get a reference to the system's LocationManager
    // by calling Activity.getSystemService(Context.LOCATION_SERVICE);

    jmethodID getSystemServiceMethodId =
            env->GetMethodID(m_j_classid_prisActivity,
                             "getSystemService",
                             "(Ljava/lang/String;)Ljava/lang/Object;");


    jobject locationManagerInstance =
            env->CallObjectMethod(m_j_ref_prisActivityInstance,
                                  getSystemServiceMethodId,
                                  locationServiceStr);

    jmethodID prisLocLisConstructorId =
            env->GetMethodID(prisLocLisClassId,
                             "<init>","()V");  // use V for constructors

    jobject prisLocLisInstance =
            env->NewObject(prisLocLisClassId,
                           prisLocLisConstructorId);


    // get LocationManager class id
    jclass locationManagerClassId = m_j_classid_locationManager;

    // get LocationManager.requestLocationUpdates method id
    jmethodID requestLocationUpdMethodId =
            env->GetMethodID(locationManagerClassId,
                             "requestLocationUpdates",
                             "(Ljava/lang/String;JFLandroid/location/LocationListener;Landroid/os/Looper;)V");

    // get LocationManager provider type field
    // (LocationManager.GPS_PROVIDER)
    jfieldID locationProviderGpsFieldId =
            env->GetStaticFieldID(locationManagerClassId,
                                  "GPS_PROVIDER",
                                  "Ljava/lang/String;");

    jstring locationProviderGpsStr =
            (jstring)env->GetStaticObjectField(locationManagerClassId,
                                      locationProviderGpsFieldId);

    // we need to create a seperate thread for the
    // listener that has a message loop

    // create an instance of android.os.HandlerThread
    jmethodID handlerThreadConstructorId =
            env->GetMethodID(m_j_classid_handlerThread,
                             "<init>","(Ljava/lang/String;)V");

    jstring locationThreadName = env->NewStringUTF("LocationThread");

    jobject handlerThreadInstance =
            env->NewObject(m_j_classid_handlerThread,
                           handlerThreadConstructorId,
                           locationThreadName);

    jmethodID handlerThreadStartId =
            env->GetMethodID(m_j_classid_handlerThread,
                             "start","()V");

    jmethodID handlerThreadGetLooperId =
            env->GetMethodID(m_j_classid_handlerThread,
                             "getLooper","()Landroid/os/Looper;");

    // start thread (must be done before we call getLooper)
    env->CallVoidMethod(handlerThreadInstance,
                          handlerThreadStartId);

    // get looper
    jobject looperInstance =
            env->CallObjectMethod(handlerThreadInstance,
                                  handlerThreadGetLooperId);

    // request updates
    env->CallVoidMethod(locationManagerInstance,
                        requestLocationUpdMethodId,
                        locationProviderGpsStr,
                        jlong(1000),
                        jfloat(0),
                        prisLocLisInstance,
                        looperInstance);

    m_javaVM->DetachCurrentThread();

    qDebug() << "####: Initialize() end";
}
