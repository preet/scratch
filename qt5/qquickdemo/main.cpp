#include <QGuiApplication>
#include <QQuickView>
#include <QTimer>
#include "anjni.h"
#include "helper.h"


JavaVM *    g_java_vm = 0;
jclass      g_myQtActivityClassID = 0;
jobject     g_myQtActivityInstance = 0;



//
JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM * vm,void * reserved)
{
    Q_UNUSED(vm);
    Q_UNUSED(reserved);

    qDebug() << "####: JNI_onLoad";

    return JNI_VERSION_1_6;
}
extern "C" {
    JNIEXPORT void JNICALL Java_org_qtproject_qt5_android_bindings_MyQtActivity_activityReady
    (JNIEnv *env, jobject obj)
    {
        Q_UNUSED(env);
        Q_UNUSED(obj);

        qDebug() << "####: activityReady";

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
                 << QString(jstr);
    }
}

int main(int argc, char **argv)
{
    qDebug() << "####: main()";

    QGuiApplication app(argc, argv);
    QQuickView * view = new QQuickView;
    Helper * helper = new Helper(view);
    view->setResizeMode(QQuickView::SizeRootObjectToView);
    view->setSource(QUrl("qrc:/main.qml"));

//    view->showFullScreen();
//    view->showMaximized();
    view->show();

    return app.exec();
}
