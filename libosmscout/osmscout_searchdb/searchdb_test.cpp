#include <iostream>
#include <exception>
#include <map>

// qt
#include <QCoreApplication>
#include <QString>
#include <QDebug>
#include <QDir>
#include <QFile>

// boost
#include <boost/unordered_map.hpp>

// osmscout
#include <osmscout/Database.h>

// kompex
#include "KompexSQLitePrerequisites.h"
#include "KompexSQLiteDatabase.h"
#include "KompexSQLiteStatement.h"
#include "KompexSQLiteException.h"
#include "KompexSQLiteBlob.h"

//
#include "quadtilehelpers.hpp"


// ============================================================== //

void badInput()
{
    qDebug() << "ERROR: Bad arguments";
    qDebug() << "ex:";
    qDebug() << "./searchdb_test <osmscout_map_dir> <sqlite_db_dir>";
}

// ============================================================== //

int main(int argc, char *argv[])
{
    QCoreApplication app(argc,argv);

    // check input args
//    QStringList inputArgs = app.arguments();
//    if(inputArgs.size() != 3)   {
//        badInput();
//        return -1;
//    }

//    // open osmscout map
//    osmscout::DatabaseParameter map_param;
//    osmscout::Database map(map_param);
//    if(!map.Open(inputArgs[1].toStdString()))   {
//        qDebug() << "ERROR: Failed to open osmscout map";
//        return -1;
//    }

//    // create search database
//    Kompex::SQLiteDatabase * database;
//    Kompex::SQLiteStatement * stmt;

//    try   {
//        database = new Kompex::SQLiteDatabase(inputArgs[2].toStdString(),
//                                              SQLITE_OPEN_READONLY,0);

//        stmt = new Kompex::SQLiteStatement(database);
//    }
//    catch(Kompex::SQLiteException &exception)   {
//        qDebug() << "ERROR: SQLite exception opening database:"
//                 << QString::fromStdString(exception.GetString());
//        return -1;
//    }

    qDebug() << "INFO: Start typing in a search term";
    bool request_quit=false;

    while(!request_quit)   {
        // get input string
        std::string input_string;
        std::cin >> input_string;

        if(input_string.length() < 4)   {
            qDebug() << "INFO: 4 or more characters req'd to actually search";
            continue;
        }

        if(input_string == "quit")   {
            request_quit=true;
            break;
        }

        // lookup search string in name_lookup

        // use name_id to get search results from
        // admin_region, streets and pois

        // sort by distance using quadtiles first
        // and then offset locations (straight line
        // location is probably good enough)
    }

    return 0;
}

// ============================================================== //
// ============================================================== //
