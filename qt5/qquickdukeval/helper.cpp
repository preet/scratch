#include "helper.h"

#include <QStandardPaths>

extern const char * g_init;
extern const char * g_script;

static double genByte()
{
    return 3;
}

Helper::Helper(QQuickView *qqview, QObject *parent) :
    QObject(parent)
{
    QStringList list_doc_paths = QStandardPaths::standardLocations(QStandardPaths::DocumentsLocation);
    for(size_t i=0; i < list_doc_paths.size(); i++) {
        qDebug() << "###: Android sucks: " << list_doc_paths[i];
    }

    m_view = qqview;
    m_view->rootContext()->setContextProperty("Helper",this);
    m_view->setResizeMode(QQuickView::SizeRootObjectToView);
    m_view->setSource(QUrl("qrc:/main.qml"));
    m_view->show();
}

void Helper::runTest()
{
    // create js heap and default context
    duk_context * ctx = NULL;
    ctx = duk_create_heap_default();
    if(!ctx) {
        qDebug() << "ERROR: Could not create JS context";
        emit testComplete(-1.0);
        return;
    }

    size_t ITERATIONS=10000;

    // push the global object onto the context's
    // value stack
    duk_push_global_object(ctx);

    // register global functions and objects to
    // the global object
    duk_eval_string(ctx,g_init);
    duk_pop(ctx);   // pop result of eval

    // register parser script to global object
    duk_eval_string(ctx,g_script);
    duk_pop(ctx);

    // push global list num data
    duk_get_prop_string(ctx,-1,"global_list_num_data");         // <g,listN>

    // push global list lit data
    duk_get_prop_string(ctx,-2,"global_list_lit_data");         // <g,listN,listL>

    // push function __private_clear_all_data onto stack
    duk_get_prop_string(ctx,-3,"__private__clear_all_data");    // <g,listN,listL,func1>

    // push function __private__set_data
    duk_get_prop_string(ctx,-4,"__private__set_data");          // <g,listN,listL,func1,func2>

    QElapsedTimer timer;
    timer.start();
    for(size_t i=0; i < ITERATIONS; i++)   {
        // clear all data
        duk_dup(ctx,-2);    // <..., func1, func2, func1>
        duk_call(ctx,0);    // <..., func1, func2, retval1>
        duk_pop(ctx);

        // set new data
        duk_dup(ctx,-1);                                // <..., func1, func2, func2>
        int list_arr_idx = duk_push_array(ctx);         // <..., func2, list[]>
        int data_arr_idx = duk_push_array(ctx);         // <..., func2, list[], data[]>
        duk_push_number(ctx,genByte());                 // <..., func2, list[], data[], num1>
        duk_put_prop_index(ctx,data_arr_idx,0);         // <..., func2, list[], data[num1]>
        duk_push_number(ctx,genByte());                 // <..., func2, list[], data[num1], num2>
        duk_put_prop_index(ctx,data_arr_idx,1);         // <..., func2, list[], data[num1,num2]>
        duk_put_prop_index(ctx,list_arr_idx,0);         // <..., func2, list[data[num1,num2]]>
        duk_call(ctx,1);                                // <..., func1, func2, retval2>
        duk_pop(ctx);

        // parse the data
        duk_get_prop_string(ctx,-5,"parse");            // <g,listN,listL,func1,func2,func3>
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
        "         {   this.listData.push(newData);   }\n"
        "\n"
        "    this.clearData = function()\n"
        "         {   this.listData.length = 0;   }\n"
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
        "function DataBytesObj()\n"
        "{\n"
        "    this.bytes = [];\n"
        "\n"
        "    this.BYTE = function(bytePos)\n"
        "         { return ((bytePos > -1 && bytePos < this.bytes.length) ? this.bytes[bytePos] : 0); }\n"
        "\n"
        "    this.BIT = function(bytePos,bitPos)\n"
        "         {\n"
        "             if(bytePos > -1 && bytePos < this.bytes.length && bitPos > -1 && bitPos < 8)\n"
        "             {\n"
        "                 var byteVal = this.bytes[bytePos];\n"
        "                 if((byteVal & (1 << bitPos)) > 0)\n"
        "                 {   return 1;   }\n"
        "                 else\n"
        "                 {   return 0;   }\n"
        "             }\n"
        "         }\n"
        "         \n"
        "    this.LENGTH = function()\n"
        "         {   return this.bytes.length;   }\n"
        "\n"
        "    this.appendByte = function(byteVal)\n"
        "         {\n"
        "             if(byteVal >= 0)\n"
        "             {   this.bytes.push(byteVal);   }\n"
        "         }\n"
        "\n"
        "    this.clearData = function()\n"
        "         {   this.bytes.length = 0;   }\n"
        "}\n"
        "\n"
        "function ListDataBytesObj()\n"
        "{\n"
        "   this.listDataBytes = [];\n"
        "   \n"
        "   this.appendDataBytes = function(dataBytes)\n"
        "      {\n"
        "         var dataBytesObj = new DataBytesObj();\n"
        "         dataBytesObj.bytes = dataBytes;\n"
        "         this.listDataBytes.push(dataBytesObj);\n"
        "      }\n"
        "      \n"
        "   this.clearDataBytes = function()\n"
        "      {   this.listDataBytes.length = 0;   }\n"
        "}\n"
        "\n"
        "// global list of databytes for when a given parameter\n"
        "// needs to be reconstructed from multiple messages\n"
        "var global_list_databytes = new ListDataBytesObj();\n"
        "\n"
        "function DATA(respNum)\n"
        "{\n"
        "   if(respNum > -1 && respNum < global_list_databytes.listDataBytes.length)\n"
        "   {   return global_list_databytes.listDataBytes[respNum];   }\n"
        "   else\n"
        "   {   return 0;   }\n"
        "}\n"
        "\n"
        "function BYTE(bytePos)\n"
        "{   return DATA(0).BYTE(bytePos);   }\n"
        "\n"
        "function BIT(bytePos,bitPos)\n"
        "{   return DATA(0).BIT(bytePos,bitPos);   }\n"
        "\n"
        "function LENGTH()\n"
        "{   return DATA(0).LENGTH();   }\n"
        "\n"
        "\n"
        "\n"
        "function __private__set_data(list_databytes)\n"
        "{\n"
        "   global_list_databytes.appendDataBytes(list_databytes[0]);\n"
        "   //var i=0;\n"
        "   //for(i=0; i < list_databytes[0].length; i++)   {\n"
        "   //   print(\"byte \"+i+\": \" + list_databytes[0][i]);\n"
        "   //}\n"
        "}\n"
        "\n"
        "function __private__clear_all_data()\n"
        "{\n"
        "   global_list_num_data.clearData();\n"
        "   global_list_lit_data.clearData();\n"
        "   global_list_databytes.clearDataBytes();\n"
        "   //print(\"boop\");\n"
        "}\n"
        "";


const char * g_script =
        "function parse() {\n"
        "var engSpd = new NumericalDataObj();\n"
        "engSpd.units = \"rpm\";\n"
        "engSpd.min = 0;\n"
        "engSpd.max = 16383.75;\n"
        "engSpd.value = (BYTE(0)*256 + BYTE(1))/4;\n"
        "saveNumericalData(engSpd);\n"
        "}";
