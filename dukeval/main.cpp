#include <QCoreApplication>
#include <QElapsedTimer>
#include <QByteArray>
#include <QString>
#include <QDebug>

#include "duktape.h"

static const char * g_script =
        "function parse() {\n"
        "var engSpd = new NumericalDataObj();\n"
        "engSpd.units = \"rpm\";\n"
        "engSpd.min = 0;\n"
        "engSpd.max = 16383.75;\n"
        "engSpd.value = (BYTE(0)*256 + BYTE(1))/4;\n"
        "saveNumericalData(engSpd);\n"
        "}";


//"engSpd.value = (BYTE(0)*256 + BYTE(1))/4;\n"

double genHeaderByte()
{
    return 1;
}

double genDataByte()
{
    return 3;
}

struct NumericalData   {
    double value;
    double min;
    double max;
    QString units;
};

#define duk_context void

int main(int argc, char *argv[])
{
    // create js heap and default context
    duk_context * ctx = NULL;
    ctx = duk_create_heap_default();
    if(!ctx) {
        qDebug() << "ERROR: Could not create JS context";
        return -1;
    }

    size_t ITERATIONS=1000;

    // push the global object onto the context's
    // value stack
    duk_push_global_object(ctx);

    // register global functions and objects to
    // the global object
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
        // (data)
        list_arr_idx = duk_push_array(ctx); // listDataBytes
        data_arr_idx = duk_push_array(ctx); // dataBytes
        duk_push_number(ctx,genDataByte());
        duk_put_prop_index(ctx,data_arr_idx,0);
        duk_push_number(ctx,genDataByte());
        duk_put_prop_index(ctx,data_arr_idx,1);
        duk_push_number(ctx,genDataByte());
        duk_put_prop_index(ctx,data_arr_idx,2);
        duk_push_number(ctx,genDataByte());
        duk_put_prop_index(ctx,data_arr_idx,3);
        duk_push_number(ctx,genDataByte());
        duk_put_prop_index(ctx,data_arr_idx,4);
        duk_push_number(ctx,genDataByte());
        duk_put_prop_index(ctx,data_arr_idx,5);
        duk_push_number(ctx,genDataByte());
        duk_put_prop_index(ctx,data_arr_idx,6);
        duk_push_number(ctx,genDataByte());
        duk_put_prop_index(ctx,data_arr_idx,7);
        duk_put_prop_index(ctx,list_arr_idx,0);

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
    qDebug() << "total milliseconds taken:" << timer.elapsed();
    qDebug() << "took an average of " << timer.elapsed()/double(ITERATIONS) << "milliseconds/parse";

    duk_destroy_heap(ctx);

    qDebug() << "end";

    return 0;
}
