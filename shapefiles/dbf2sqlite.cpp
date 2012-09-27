// Qt libs
#include <QCoreApplication>
#include <QStringList>
#include <QDebug>
#include <QImage>
#include <QPainter>
#include <QDir>

#include <iostream>

// kompex libs
#include "kompex/KompexSQLiteDatabase.h"
#include "kompex/KompexSQLiteStatement.h"

// shapelib libs
#include "shapelib/shapefil.h"

// Natural Earth Data Set Type
#define ADMIN0 0
#define ADMIN1 1

int main(int argc, char *argv[])
{
    QCoreApplication myApp(argc, argv);

    // check input args
    QStringList inputArgs = myApp.arguments();

    if(inputArgs.size() < 2)   {
        qDebug() << "Error: No dbf directory: ";
        qDebug() << "Pass the dbf directory as an argument: ";
        qDebug() << "./dbf2sqlite /my/dbfdir";
        return -1;
    }

    // filter shapefile types
    QStringList dbfFilterList;
    dbfFilterList << "*.dbf";

    // check for all files
    QDir dbfDir = inputArgs[1];
    QStringList dbfDirList = dbfDir.entryList(dbfFilterList,
                                              QDir::Files);

    // get all file paths
    QString fileDbf;
    for(int i=0; i < dbfDirList.size(); i++)   {
        if(dbfDirList[i].contains(".dbf"))   {
            fileDbf = dbfDir.absoluteFilePath(dbfDirList[i]);
        }
    }

    // open the database file
    DBFHandle hDBF = DBFOpen(fileDbf.toLocal8Bit().data(),"rb");
    if(hDBF == NULL)   {
        qDebug() << "Error: Could not open dbf file";
        return -1;
    }

    // set fields to keep based on data type
    QStringList listFieldsToKeep;
    QString dbFileName;

    if(ADMIN0)   {
        dbFileName = "admin0.sqlite";
        listFieldsToKeep
                << "ADMIN"          // administrative name of country
                << "ADM0_A3";       // 3 letter abbreviatesion of admin name
    }

    if(ADMIN1)   {
        dbFileName = "admin1.sqlite";
        listFieldsToKeep
                << "OBJECTID"
                << "NAME_1"         // Admin1 region name
                << "VARNAME_1"      // Admin1 alt name (not very reliable)
                << "NL_NAME_1"      // Admin1 region name in national language (not reliable)
                << "Postal";        // 2 Letter Postal Code (not reliable)
    }

    // get number of fields in db
    size_t numRecords = 0;
    numRecords = DBFGetRecordCount(hDBF);

    if(numRecords > 0)
    {   qDebug() << "Info: DBF file has" << numRecords << "records";   }
    else
    {   qDebug() << "Error: DBF file has no records!";   return -1;   }

    // create sqlite database
    qDebug() << "Info: Creating SQLite Database...";
    Kompex::SQLiteDatabase * pDatabase =
            new Kompex::SQLiteDatabase(dbFileName.toStdString(),
                SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE,0);

    Kompex::SQLiteStatement * pStmt =
            new Kompex::SQLiteStatement(pDatabase);

    // create database schema (flat)
    if(ADMIN1)   {
        qDebug() << "Info: Creating database schema for ADMIN1 profile";

        pStmt->SqlStatement("CREATE TABLE IF NOT EXISTS data("
                            "regionid INTEGER PRIMARY KEY NOT NULL UNIQUE,"
                            "name TEXT NOT NULL,"
                            "code TEXT);");

        // regionid <-> internel shape/record id [integer]
        size_t idx_objectid = DBFGetFieldIndex(hDBF,"OBJECTID");

        // name <-> NAME_1 [text]
        size_t idx_name = DBFGetFieldIndex(hDBF,"NAME_1");

        // code <-> Postal [text]
        size_t idx_code = DBFGetFieldIndex(hDBF,"Postal");

        qDebug() << "Info: Writing records to database...";
        pStmt->BeginTransaction();

        for(size_t i=0; i < numRecords; i++)
        {
            QString record_id = QString::number(i+1);
            QString record_name(DBFReadStringAttribute(hDBF,i,idx_name));
            QString record_code(DBFReadStringAttribute(hDBF,i,idx_code));

            QString inStmt = "INSERT INTO data(regionid,name,code) VALUES(\"" +
                    record_id + "\",\"" +record_name + "\",\"" + record_code + "\");";

            pStmt->SqlStatement(inStmt.toUtf8().data());

//            qDebug() << record_name;

            if(i % 1000 == 0)   {
                qDebug() << "Info: Wrote" << i+1 << "/" << numRecords << "records";
            }
        }

        pStmt->CommitTransaction();
        qDebug() << "Info: Done!";
    }

    // close dbf file
    DBFClose(hDBF);

    // clean up database
    delete pStmt;
    delete pDatabase;

    return 0;
}
