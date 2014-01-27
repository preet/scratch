#include <sys/time.h>
#include <string>

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

// ============================================================== //

class Timer
{
public:
    Timer() {}

    void Start()
    {   gettimeofday(&m_t1,NULL);   }

    void Stop()
    {   gettimeofday(&m_t2,NULL);   }

    double ElapsedMs()
    {
        double time_taken = 0;
        time_taken += (m_t2.tv_sec - m_t1.tv_sec) * 1000.0 * 1000.0;
        time_taken += (m_t2.tv_usec - m_t1.tv_usec);
        return time_taken/1000.0;
    }

private:
    timeval m_t1;
    timeval m_t2;
};

// ============================================================== //

struct GeoBoundingBox
{
    double minLat;
    double maxLat;
    double minLon;
    double maxLon;
};

struct Tile
{
    int32_t id;
    GeoBoundingBox bbox;  
};

struct OffsetGroup
{
    std::vector<osmscout::FileOffset> node_offsets;
    std::vector<osmscout::FileOffset> way_offsets;
    std::vector<osmscout::FileOffset> area_offsets;
};

struct PointLLA
{
    PointLLA() :
        lon(0),lat(0),alt(0) {}

    PointLLA(double myLat, double myLon) :
        lon(myLon),lat(myLat),alt(0) {}

    PointLLA(double myLat, double myLon, double myAlt) :
        lon(myLon),lat(myLat),alt(myAlt) {}

    double lon;
    double lat;
    double alt;
};

#define K_PI 3.141592653589
#define K_DEG2RAD K_PI/180.0
#define K_RAD2DEG 180.0/K_PI

double CalcGeoDist2RadsApprox(PointLLA const &pointA,
                              PointLLA const &pointB)
{
    // This method was taken from [1] and is originally
    // Copyright 2002-2012 Chris Veness. See this file's
    // header for more information.

    // [1] - Latitude/longitude spherical geodesy formulae & scripts
    //       http://www.movable-type.co.uk/scripts/latlong.html

    // WARN: This method doesn't work if the points cross
    //       the antimeridian, so we need to split them
    // TODO: need to verify this works
    if(fabs(pointA.lon - pointB.lon) > 180)   {
        PointLLA pointB_N;
        pointB_N.lat = pointB.lat;
        if(pointB.lon > 0)   {
            pointB_N.lon = pointB.lon-360;
        }
        else   {
            pointB_N.lon = pointB.lon+360;
        }
        return CalcGeoDist2RadsApprox(pointA,pointB_N);
    }

    double diffLonRads = (pointB.lon-pointA.lon)*K_DEG2RAD;
    double diffLatRads = (pointB.lat-pointA.lat)*K_DEG2RAD;
    double cosMeanLatRads = cos((pointA.lat+pointB.lat)*0.5*K_DEG2RAD);

    double dist2 = (diffLatRads*diffLatRads) +
            (diffLonRads*cosMeanLatRads*diffLonRads*cosMeanLatRads);

    // NOTE: to convert to meters:
    // dist_m = sqrt(dist2) * 6371000;

    return dist2;
}

// ============================================================== //

// each id is 64 bits (8 bytes), and contains both a name_id and tile_id
// the lower 'g_sz_tile_id' bytes of those 8 bytes will contain
// the tile_id and the rest of the bytes will be the name_id
int const g_sz_bits_tile_id = 24;  // 24 because 32 (4 bytes) would overflow
int const g_zoom_level = 10;
int const g_side_tiles_n = pow(2,g_zoom_level);


// search parameters

// g_search_max_tiles
// * search this many of the closest tiles
int const g_search_max_tiles=10;

// g_search_max_unq_results
// * max

// ============================================================== //

bool buildTileFromId(int32_t const tile_id,
                     Tile &tile)
{
    // the number of columns/rows the
    // world map is divided into
    double const div = g_side_tiles_n;
    double const div_lon = 360.0/div;
    double const div_lat = 180.0/div;

    int32_t tile_x = tile_id%g_side_tiles_n;
    int32_t tile_y = tile_id/g_side_tiles_n;

    tile.id = tile_id;
    tile.bbox.minLon = (tile_x * div_lon)-180.0;
    tile.bbox.maxLon = tile.bbox.minLon + div_lon;
    tile.bbox.maxLat = 90.0 - (tile_y * div_lat);
    tile.bbox.minLat = tile.bbox.maxLat - div_lat;
}

std::string intToString(int64_t num)
{
    return QString::number(num,10).toStdString();
}

void stressDistCalcTest()
{
    PointLLA refLLA(43,-79,0);

    // use a fake name_id of 0 -- we dont
    // actually read in the results
    int64_t name_id=0;
    int64_t min_id = name_id << g_sz_bits_tile_id;
    int64_t max_id = min_id | (1024*1024);

    Timer timer;
    timer.Start();

    //std::multimap<double,std::pair<int,qint64> > table_dist_tile;
    std::vector<double> list_dist2_rads;
    list_dist2_rads.reserve(max_id-min_id);
    for(int64_t tile_id=min_id; tile_id <= max_id; tile_id++)   {
        Tile tile;
        buildTileFromId(tile_id,tile);
        double midLon = (tile.bbox.minLon + tile.bbox.maxLon)*0.5;
        double midLat = (tile.bbox.minLat + tile.bbox.maxLat)*0.5;

        PointLLA cenLLA(midLat,midLon,0);
        double dist2_rads = CalcGeoDist2RadsApprox(refLLA,cenLLA);

        // save
        std::pair<int,qint64> data_idx_rowid(0,tile_id);
        std::pair<double,std::pair<int,qint64> > data_dist_tile;
        data_dist_tile.first  = dist2_rads;
        data_dist_tile.second = data_idx_rowid;

        //table_dist_tile.insert(data_dist_tile);
        list_dist2_rads.push_back(dist2_rads);
    }

    std::sort(list_dist2_rads.begin(),
              list_dist2_rads.end());

    timer.Stop();
    std::cout << "That took:" << timer.ElapsedMs() << "ms, "
              << "or an average of " << timer.ElapsedMs()/(1024*1024)
              << "ms per tile" << std::endl;
}

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
    stressDistCalcTest();
    return 0;
}

//int main(int argc, char *argv[])
//{
//    QCoreApplication app(argc,argv);

////    stressDistCalcTest();
////    return 0;

//    // check input args
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

//    // open search database
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

//    // define a search origin
//    double origin_lon = -79.33;
//    double origin_lat = 43.33;

//    qDebug() << "INFO: Start typing in a search term";
//    bool request_quit=false;

//    while(!request_quit)   {
//        // get input string
//        std::string input_string;
//        std::cin >> input_string;

//        if(input_string.length() < 4)   {
//            qDebug() << "INFO: 4 or more characters req'd to actually search";
//            continue;
//        }

//        if(input_string == "quit")   {
//            request_quit=true;
//            break;
//        }

//        // get name_id for the given search string
//        std::string name_lookup = input_string.substr(0,4);
//        std::string query =
//                "SELECT name_id FROM name_lookup WHERE "
//                "name_lookup.name_lookup = \""+name_lookup+"\";";

//        try   {
//            stmt->Sql(query);
//        }
//        catch(Kompex::SQLiteException &e)   {
//            qDebug() << "ERROR: SQLite exception for query:"
//                     << QString::fromStdString(query);
//            qDebug() << "ERROR:" << QString::fromStdString(e.GetString());
//            return -1;
//        }

//        if(!stmt->FetchRow())   {
//            qDebug() << "INFO: No results";
//            stmt->FreeQuery();
//            continue;
//        }

//        // determine the id range we need to query for
//        int64_t name_id = stmt->GetColumnInt(0);
//        stmt->FreeQuery();

//        int32_t mask = pow(2,g_sz_bits_tile_id)-1;
//        int64_t min_id = name_id << g_sz_bits_tile_id;
//        int64_t max_id = min_id | mask;

//        // use name_id to get search results from
//        // admin_region, streets and pois
//        query = "SELECT id,node_offsets,way_offsets,area_offsets FROM admin_regions WHERE "
//                "admin_regions.id BETWEEN "+intToString(min_id)+" AND "+intToString(max_id)+";";
//        qDebug() << QString::fromStdString(query);
//        try   {
//            stmt->Sql(query);
//        }
//        catch(Kompex::SQLiteException &e)   {
//            qDebug() << "ERROR: SQLite exception for query:"
//                     << QString::fromStdString(query);
//            qDebug() << "ERROR:" << QString::fromStdString(e.GetString());
//            return -1;
//        }

//        // sort by distance using tiles first
//        std::multimap<double,Tile> table_dist2_tiles;
//        std::map<int64_t,OffsetGroup> table_tile_streetoffsets;

//        while(stmt->FetchRow())   {
//            int64_t id = stmt->GetColumnInt64(0);
//            int32_t tile_id = (id & mask);

//            // save to OffsetGroup
//            OffsetGroup g;

//            // copy node offsets
//            void const * blob_node_offsets = stmt->GetColumnBlob(1);
//            size_t sz_node_offsets = stmt->GetColumnBytes(1);
//            g.node_offsets.resize(sz_node_offsets/sizeof(osmscout::FileOffset));
//            memcpy(&(g.node_offsets[0]),blob_node_offsets,sz_node_offsets);

//            // copy way offsets
//            void const * blob_way_offsets = stmt->GetColumnBlob(2);
//            size_t sz_way_offsets = stmt->GetColumnBytes(2);
//            g.way_offsets.resize(sz_way_offsets/sizeof(osmscout::FileOffset));
//            memcpy(&(g.way_offsets[0]),blob_way_offsets,sz_way_offsets);

//            // copy area offsets
//            void const * blob_area_offsets = stmt->GetColumnBlob(3);
//            size_t sz_area_offsets = stmt->GetColumnBytes(3);
//            g.area_offsets.resize(sz_area_offsets/sizeof(osmscout::FileOffset));
//            memcpy(&(g.area_offsets[0]),blob_area_offsets,sz_area_offsets);

//            {   // insert offset data
//                std::pair<int64_t,OffsetGroup> data;
//                data.first = tile_id;
//                data.second = g;    // could avoid the copy here
//                table_tile_streetoffsets.insert(data);
//            }


//            Tile tile;
//            buildTileFromId(tile_id,tile);

//            // straight line squared distance in degrees
//            // from the search origin to the tile center
//            double diff_lon = (tile.bbox.minLon+tile.bbox.maxLon)*0.5 - origin_lon;
//            double diff_lat = (tile.bbox.minLat+tile.bbox.maxLat)*0.5 - origin_lat;
//            double dist2 = (diff_lon*diff_lon) + (diff_lat*diff_lat);

//            {   // insert tile data
//                std::pair<double,Tile> data;
//                data.first  = dist2;
//                data.second = tile;
//                table_dist2_tiles.insert(data);
//            }
//        }
//        stmt->FreeQuery();

//        int64_t num_results=0;
//        std::map<int64_t,OffsetGroup>::iterator it;
//        for(it  = table_tile_streetoffsets.begin();
//            it != table_tile_streetoffsets.end(); ++it)
//        {
//            OffsetGroup &g = it->second;
//            num_results += g.node_offsets.size();
//            num_results += g.way_offsets.size();
//            num_results += g.area_offsets.size();

////            if(it->first == 270621)   {
//            if(1)   {
//                std::vector<osmscout::WayRef> listWayRefs;
//                map.GetWaysByOffset(g.way_offsets,listWayRefs);
//                for(size_t n=0; n < listWayRefs.size(); n++)   {
//                    osmscout::WayRef const &way = listWayRefs[n];
//                    qDebug() << QString::fromStdString(way->GetName());
//                }

//                std::vector<osmscout::NodeRef> listNodeRefs;
//                map.GetNodesByOffset(g.node_offsets,listNodeRefs);
//                for(size_t n=0; n < listNodeRefs.size(); n++)   {
//                    osmscout::NodeRef const &nd = listNodeRefs[n];
//                    qDebug() << QString::fromStdString(nd->GetName());
//                }
//            }
//        }

//        qDebug() << "INFO: Results Found: " << num_results;
//    }

//    return 0;
//}

//// ============================================================== //
//// ============================================================== //
