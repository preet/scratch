#include "helper.h"
#include <QMetaType>

extern const char * g_init;
extern const char * g_script;

static double genHeaderByte()
{
    return 1;
}

static double genDataByte()
{
    return 3;
}

AnotherThread::AnotherThread(QObject *parent) : QThread(parent)
{

}

void AnotherThread::run()
{
    exec();
}

Another::Another(QObject *parent) : QObject(parent)
{

}

void Another::onRxData(AnotherTest at)
{
    double x=at.arg1 + at.arg2;
    x = x*x;
}

void Another::onRxData(double arg1, double arg2, double arg3, double arg4, double arg5, double arg6, double arg7, double arg8)
{
    double x=arg1+arg2;
    x=x*x;
}

void Another::onRxData(QJsonObject at)
{
    double arg1 = at.value("arg1").toDouble();
    double arg2 = at.value("arg2").toDouble();
    double x=arg1+arg2;
    x=x*x;
}

Helper::Helper(QQuickView *qqview, QObject *parent) :
    QObject(parent)
{
    qRegisterMetaType<AnotherTest>("AnotherTest");

    m_view = qqview;
    m_view->rootContext()->setContextProperty("Helper",this);
    m_view->setResizeMode(QQuickView::SizeRootObjectToView);
    m_view->setSource(QUrl("qrc:/main.qml"));
    m_view->show();
}

void Helper::runAnotherTest()
{
    AnotherThread * anotherThread = new AnotherThread();
    Another * another = new Another;
    another->moveToThread(anotherThread);

    connect(this,SIGNAL(txData(AnotherTest)),
            another,SLOT(onRxData(AnotherTest)));

    connect(this,SIGNAL(txData(double,double,double,double,double,double,double,double)),
            another,SLOT(onRxData(double,double,double,double,double,double,double,double)));

    connect(this,SIGNAL(txData(QJsonObject)),
            another,SLOT(onRxData(QJsonObject)));

    AnotherTest at;

    QElapsedTimer timer;
    timer.start();
    for(int i=0; i < 50000; i++)   {
        emit txData(at);
//        emit txData(1,1,1,1,1,1,1,1);

//        m_json_at.insert("arg1",0.1);
//        m_json_at.insert("arg2",0.2);
//        m_json_at.insert("arg3",0.3);
//        m_json_at.insert("arg4",0.4);
//        m_json_at.insert("arg5",0.5);
//        m_json_at.insert("arg6",0.6);
//        m_json_at.insert("arg7",0.7);
//        m_json_at.insert("arg8",0.8);
//        emit txData(m_json_at);
    }
    double average_ms = timer.elapsed()/double(50000);
    emit testComplete(average_ms);

    disconnect(this,SIGNAL(txData(AnotherTest)),
               another,SLOT(onRxData(AnotherTest)));

    disconnect(this,SIGNAL(txData(double,double,double,double,double,double,double,double)),
            another,SLOT(onRxData(double,double,double,double,double,double,double,double)));

    anotherThread->quit();

    another->deleteLater();
    anotherThread->deleteLater();

}

void Helper::runTest()
{
    runAnotherTest();
    return;

    // create js heap and default context
    duk_context * ctx = NULL;
    ctx = duk_create_heap_default();
    if(!ctx) {
        qDebug() << "ERROR: Could not create JS context";
        emit testComplete(-1.0);
        return;
    }

    size_t ITERATIONS=1000;

    // push the global object onto the context's
    // value stack
    duk_push_global_object(ctx);

    // register global functions and objects to
    // the global object
//    duk_eval_string(ctx,g_init);
    duk_eval_file(ctx,"/home/preet/Dev/projects/libobdref/libobdref/globals.js");
    duk_pop(ctx);   // pop result of eval

    // register parser script to global object
    duk_eval_string(ctx,g_script);
    duk_pop(ctx);

    //
    int js_idx_global_obj = duk_normalize_index(ctx,-1);
    duk_get_prop_string(ctx,js_idx_global_obj,"__private__add_msg_data");
    int js_idx_add_msg_data = duk_normalize_index(ctx,-1);

    // push global list num data
    duk_get_prop_string(ctx,-2,"global_list_num_data");         // <g,listN>

    // push global list lit data
    duk_get_prop_string(ctx,-3,"global_list_lit_data");         // <g,listN,listL>

    // push function __private_clear_all_data onto stack
    duk_get_prop_string(ctx,-4,"__private__clear_all_data");    // <g,listN,listL,func1>

    // push function __private__set_data
    duk_get_prop_string(ctx,-5,"__private__add_list_databytes");          // <g,listN,listL,func1,func2>

    QElapsedTimer timer;
    timer.start();
    for(size_t i=0; i < ITERATIONS; i++)   {
        // clear all data
        duk_dup(ctx,-2);    // <..., func1, func2, func1>
        duk_call(ctx,0);    // <..., func1, func2, retval1>
        duk_pop(ctx);
        int list_arr_idx,data_arr_idx;



//        // set databytes
//        duk_dup(ctx,-1);

//        list_arr_idx = duk_push_array(ctx); // listDataBytes
//        data_arr_idx = duk_push_array(ctx); // dataBytes
//        duk_push_number(ctx,genDataByte());
//        duk_put_prop_index(ctx,data_arr_idx,0);
//        duk_push_number(ctx,genDataByte());
//        duk_put_prop_index(ctx,data_arr_idx,1);
//        duk_push_number(ctx,genDataByte());
//        duk_put_prop_index(ctx,data_arr_idx,2);
//        duk_push_number(ctx,genDataByte());
//        duk_put_prop_index(ctx,data_arr_idx,3);
//        duk_push_number(ctx,genDataByte());
//        duk_put_prop_index(ctx,data_arr_idx,4);
//        duk_push_number(ctx,genDataByte());
//        duk_put_prop_index(ctx,data_arr_idx,5);
//        duk_push_number(ctx,genDataByte());
//        duk_put_prop_index(ctx,data_arr_idx,6);
//        duk_push_number(ctx,genDataByte());
//        duk_put_prop_index(ctx,data_arr_idx,7);
//        duk_put_prop_index(ctx,list_arr_idx,0);

//        duk_call(ctx,1);
//        duk_pop(ctx);



        // set msg data
        // function __private_add_msg_data(listHeaderBytes,listDataBytes)
        duk_dup(ctx,js_idx_add_msg_data);
        // (header)
        list_arr_idx = duk_push_array(ctx); // listHeaderBytes
        data_arr_idx = duk_push_array(ctx); // headerBytes
        duk_push_number(ctx,genHeaderByte());
        duk_put_prop_index(ctx,data_arr_idx,0);
        duk_push_number(ctx,genHeaderByte());
        duk_put_prop_index(ctx,data_arr_idx,1);
        duk_put_prop_index(ctx,list_arr_idx,0);

        data_arr_idx = duk_push_array(ctx); // headerBytes
        duk_push_number(ctx,genHeaderByte());
        duk_put_prop_index(ctx,data_arr_idx,0);
        duk_push_number(ctx,genHeaderByte());
        duk_put_prop_index(ctx,data_arr_idx,1);
        duk_put_prop_index(ctx,list_arr_idx,1);


        // (data)
        list_arr_idx = duk_push_array(ctx); // listDataBytes
        data_arr_idx = duk_push_array(ctx); // dataBytes
        duk_push_number(ctx,genDataByte());
        duk_put_prop_index(ctx,data_arr_idx,0);
        duk_push_number(ctx,genDataByte());
        duk_put_prop_index(ctx,data_arr_idx,1);
        duk_push_number(ctx,genDataByte());
        duk_put_prop_index(ctx,data_arr_idx,2);
        duk_put_prop_index(ctx,list_arr_idx,0);

        data_arr_idx = duk_push_array(ctx); // dataBytes
        duk_push_number(ctx,genDataByte());
        duk_put_prop_index(ctx,data_arr_idx,0);
        duk_push_number(ctx,genDataByte());
        duk_put_prop_index(ctx,data_arr_idx,1);
        duk_push_number(ctx,genDataByte());
        duk_put_prop_index(ctx,data_arr_idx,2);
        duk_put_prop_index(ctx,list_arr_idx,1);




        duk_call(ctx,2);                                // <..., func1, func2, retval2>
        duk_pop(ctx);

        // parse the data
        duk_get_prop_string(ctx,-6,"parse");            // <g,listN,listL,func1,func2,func3>
        duk_call(ctx,0);                                // <g,listN,listL,func1,func2,retval3>
        duk_pop(ctx);

        // save the result
        NumericalData numData;
        duk_get_prop_string(ctx,-4,"listData");             // <..., listN.listData>
        duk_get_prop_string(ctx,-1,"length");               // <..., listN.listData, length>
        size_t listNumLength = size_t(duk_get_number(ctx,-1));
        duk_pop(ctx);

        for(size_t j=0; j < listNumLength; j++)   {
            duk_get_prop_index(ctx,-1,j);               // <..., listN.listData, listData[j]>

            duk_get_prop_string(ctx,-1,"units");        // <..., listData[j], .units>
            numData.units = QString(duk_get_string(ctx,-1));
            duk_pop(ctx);

            duk_get_prop_string(ctx,-1,"min");          // <..., listData[j], .min>
            numData.min = duk_get_number(ctx,-1);
            duk_pop(ctx);

            duk_get_prop_string(ctx,-1,"max");          // <..., listData[j], .max>
            numData.max = duk_get_number(ctx,-1);
            duk_pop(ctx);

            duk_get_prop_string(ctx,-1,"value");        // <..., listData[j], .value>
            numData.value = duk_get_number(ctx,-1);
            duk_pop(ctx);

            duk_pop(ctx);                               // <g,listN,listL,func1,func2,listData>
            duk_pop(ctx);                               // <g,listN,listL,func1,func2>
        }

        //qDebug() << numData.value << numData.units << numData.min << numData.max;
    }
    double average_ms = timer.elapsed()/double(ITERATIONS);
    duk_destroy_heap(ctx);


    emit testComplete(average_ms);
}


const char * g_init =
        "/*\n"
        "    Copyright (c) 2012 Preet Desai (preet.desai@gmail.com)\n"
        "\n"
        "    Permission is hereby granted, free of charge, to any person obtaining\n"
        "    a copy of this software and associated documentation files (the\n"
        "    \"Software\"), to deal in the Software without restriction, including\n"
        "    without limitation the rights to use, copy, modify, merge, publish,\n"
        "    distribute, sublicense, and/or sell copies of the Software, and to\n"
        "    permit persons to whom the Software is furnished to do so, subject to\n"
        "    the following conditions:\n"
        "\n"
        "    The above copyright notice and this permission notice shall be\n"
        "    included in all copies or substantial portions of the Software.\n"
        "\n"
        "    THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY KIND,\n"
        "    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF\n"
        "    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND\n"
        "    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE\n"
        "    LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION\n"
        "    OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION\n"
        "    WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.\n"
        "*/\n"
        "\n"
        "// ================================================================ //\n"
        "// ================================================================ //\n"
        "\n"
        "function LiteralDataObj()\n"
        "{\n"
        "    this.value = false;\n"
        "    this.valueIfFalse = \"\";\n"
        "    this.valueIfTrue = \"\";\n"
        "    this.property = \"\";\n"
        "}\n"
        "\n"
        "function NumericalDataObj()\n"
        "{\n"
        "    this.value = 0;\n"
        "    this.min = 0;\n"
        "    this.max = 0;\n"
        "    this.units = \"\";\n"
        "    this.property = \"\";\n"
        "}\n"
        "\n"
        "function ListDataObj()\n"
        "{\n"
        "    this.listData = [];\n"
        "\n"
        "    this.appendData = function(newData)\n"
        "    {   this.listData.push(newData);   }\n"
        "\n"
        "    this.clearData = function()\n"
        "    {   this.listData.length = 0;   }\n"
        "}\n"
        "\n"
        "var global_list_num_data = new ListDataObj();\n"
        "\n"
        "var global_list_lit_data = new ListDataObj();\n"
        "\n"
        "function saveNumericalData(numDataObj)\n"
        "{   global_list_num_data.appendData(numDataObj);   }\n"
        "\n"
        "function saveLiteralData(litDataObj)\n"
        "{   global_list_lit_data.appendData(litDataObj);   }\n"
        "\n"
        "function __private_get_num_data()\n"
        "{\n"
        "   return global_list_num_data.listData;\n"
        "}\n"
        "\n"
        "function __private_get_lit_data()\n"
        "{\n"
        "   return global_list_lit_data.listData;\n"
        "}\n"
        "\n"
        "// ================================================================ //\n"
        "// ================================================================ //\n"
        "\n"
        "function DataBytesObj()\n"
        "{\n"
        "    this.bytes = [];\n"
        "\n"
        "    this.BYTE = function(bytePos)   {\n"
        "        return this.bytes[bytePos];\n"
        "    }\n"
        "\n"
        "    this.BIT = function(bytePos,bitPos)   {\n"
        "        var byteVal = this.bytes[bytePos];\n"
        "        if((byteVal & (1 << bitPos)) > 0)   {\n"
        "            return 1;\n"
        "        }\n"
        "        return 0;\n"
        "    }\n"
        "\n"
        "    this.LENGTH = function()\n"
        "    {   return this.bytes.length;   }\n"
        "}\n"
        "\n"
        "function MessageDataObj()\n"
        "{\n"
        "    this.listHeaderBytes = [];\n"
        "    this.listDataBytes = [];\n"
        "\n"
        "    this.setListHeaderBytes = function(list_headerbytes)   {\n"
        "        for(var i=0; i < list_headerbytes.length; i++)   {\n"
        "            var headerBytes = new DataBytesObj();\n"
        "            headerBytes.bytes = list_headerbytes[i];\n"
        "            this.listHeaderBytes.push(headerBytes);\n"
        "        }\n"
        "    }\n"
        "\n"
        "    this.setListDataBytes = function(list_databytes)   {\n"
        "        for(var i=0; i < list_databytes.length; i++)   {\n"
        "            var dataBytes = new DataBytesObj();\n"
        "            dataBytes.bytes = list_databytes[i];\n"
        "            this.listDataBytes.push(dataBytes);\n"
        "        }\n"
        "    }\n"
        "}\n"
        "\n"
        "function ParameterObj()\n"
        "{\n"
        "    this.listMessageData = [];\n"
        "\n"
        "    this.appendMessageData = function(msg)   {\n"
        "        this.listMessageData.push(msg);\n"
        "    }\n"
        "\n"
        "    this.clearAll = function()   {\n"
        "        this.listMessageData.length = 0;\n"
        "    }\n"
        "}\n"
        "\n"
        "var global_param = new ParameterObj();\n"
        "\n"
        "function REQ(idx)   {\n"
        "    return global_param.listMessageData[idx];\n"
        "}\n"
        "\n"
        "function HEADER(idx)   {\n"
        "    return REQ(0).listHeaderBytes[idx];\n"
        "}\n"
        "\n"
        "function DATA(idx)   {\n"
        "    return REQ(0).listDataBytes[idx];\n"
        "}\n"
        "\n"
        "function BYTE(bytePos)   {\n"
        "    return DATA(0).BYTE(bytePos);\n"
        "}\n"
        "\n"
        "function BIT(bytePos,bitPos)   {\n"
        "    return DATA(0).BIT(bytePos,bitPos);\n"
        "}\n"
        "\n"
        "function LENGTH()   {\n"
        "    return DATA(0).LENGTH();\n"
        "}\n"
        "\n"
        "// ================================================================ //\n"
        "// ================================================================ //\n"
        "\n"
        "// where list_databytes is a 2d array that looks like:\n"
        "// list_databytes[0]: 0xAA 0xBB 0xCC ...\n"
        "// list_databytes[1]: 0xDD 0xEE 0xFF ...\n"
        "// list_databytes[2]: 0x00 0x11 0x22 ... etc\n"
        "function __private__add_list_databytes(list_databytes)\n"
        "{\n"
        "    var msg = new MessageDataObj();\n"
        "    msg.setListDataBytes(list_databytes);\n"
        "    global_param.appendMessageData(msg);\n"
        "}\n"
        "\n"
        "function __private_add_msg_data(list_headerbytes,list_databytes)\n"
        "{\n"
        "    var msg = new MessageDataObj();\n"
        "    msg.setListHeaderBytes(list_headerbytes);\n"
        "    msg.setListDataBytes(list_databytes);\n"
        "    global_param.appendMessageData(msg);\n"
        "}\n"
        "\n"
        "function __private__clear_all_data()\n"
        "{\n"
        "    global_list_num_data.clearData();\n"
        "    global_list_lit_data.clearData();\n"
        "    global_param.clearAll();\n"
        "}\n"
        "\n"
        "// ================================================================ //\n"
        "// ================================================================ //\n"
        "";


const char * g_script =
        "function parse() {\n"
        "for(var i=0; i < NUM_RESP(0); i++) { print(DATA(i).BYTE(0)); }"
        "}";
