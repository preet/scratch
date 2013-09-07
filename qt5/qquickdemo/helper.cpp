#include "helper.h"

// WARN: WE ASSUME API LEVEL 8

// ref: Qt, qandroidstandardpaths

//static QString GetAbsolutePath(QJNILocalRef<jobject> const &fileRef)
//{
//    QJNIObject file(fileRef.object());
//    QJNILocalRef<jstring> pathFile =
//            file.callObjectMethod<jstring>("getAbsolutePath",
//                                           "()Ljava/lang/String;");

//    if(pathFile.isNull())   {
//        return QString();
//    }

//    return QString(qt_convertJString(pathFile.object())+"/");
//}

//static bool GetExternalStorageWriteable()
//{
//    // MEDIA_MOUNTED: read write to external storage available
//    // MEDIA_MOUNTED_READ_ONLY: read-only to external storate
//    QJNILocalRef<jstring> mediaMountedField =
//            QJNIObject::getStaticObjectField<jstring>("android/os/Environment",
//                                                      "MEDIA_MOUNTED");
//    if(mediaMountedField.isNull())   {
//        return false;
//    }

//    // current external storage state
//    QJNILocalRef<jstring> extStorageState =
//            QJNIObject::callStaticObjectMethod<jstring>("android/os/Environment",
//                                                       "getExternalStorageState",
//                                                       "()Ljava/lang/String;");
//    if(extStorageState.isNull())   {
//        return false;
//    }

//    // check if state == MEDIA_MOUNTED
//    bool const writeable =
//            (qt_convertJString(extStorageState.object()) ==
//             qt_convertJString(mediaMountedField.object()));

//    return writeable;
//}

//static QString GetExternalStoragePath()
//{
//    // ref: http://developer.android.com/reference/android/
//    //      os/Environment.html#getExternalStorageDirectory()

//    if(!GetExternalStorageWriteable())   {
//        return QString();
//    }

//    QJNILocalRef<jobject> fileRef =
//            QJNIObject::callStaticObjectMethod<jobject>("android/os/Environment",
//                                                        "getExternalStorageDirectory",
//                                                        "()Ljava/io/File;");
//    if(fileRef.isNull())   {
//        return QString();
//    }

//    return GetAbsolutePath(fileRef);
//}

//static QString GetInternalStoragePath()
//{
//    QJNIObject activity(QtAndroid::activity());
//}

Helper::Helper(QQuickView *qqview, QObject *parent) :
    QObject(parent)
{

#ifdef ENV_DEV
    path_app        = QCoreApplication::applicationDirPath() + "/";
    path_ui         = path_app + "ui/";
    path_plugins    = path_app + "plugins/";
    path_data       = path_app + "data/";
    path_logs       = path_app + "data/logs/";
    path_settings   = path_app + "data/settings.json";
    path_user       = path_app + "user/";
#endif

#ifdef ENV_ANDROID
    path_app        = QCoreApplication::applicationDirPath() + "/";

//    // we use internal storage (app rw) for
//    // ui,plugins,data,logs,settings

//    // we use external storage (user rw) for
//    // user

    QString path_ext;// = GetExternalStoragePath();
    QString path_int;

    path_ui         = path_ext + "ui/";
    path_plugins    = path_ext + "plugins/";
    path_data       = path_ext + "data/";
    path_logs       = path_ext + "data/logs/";
    path_settings   = path_ext + "data/settings.json";
    path_user       = path_ext + "user/";
#endif

    qqview->rootContext()->setContextProperty("Helper",this);
}

QString Helper::getPathApp()
{
    return path_app;
}

QString Helper::getPathUi()
{
    return path_ui;
}

QString Helper::getPathPlugins()
{
    return path_plugins;
}

QString Helper::getPathData()
{
    return path_data;
}

QString Helper::getPathLogs()
{
    return path_logs;
}

QString Helper::getPathSettings()
{
    return path_settings;
}

QString Helper::getPathUser()
{
    return path_user;
}

QString Helper::getQuote()
{
    QString quote = QString::fromStdString(m_helloworld.getQuote());
    return quote;
}
