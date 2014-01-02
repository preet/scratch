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
    QStringList inputArgs = app.arguments();
    if(inputArgs.size() != 3)   {
        badInput();
        return -1;
    }

    // open osmscout map
    osmscout::DatabaseParameter map_param;
    osmscout::Database map(map_param);
    if(!map.Open(inputArgs[1].toStdString()))   {
        qDebug() << "ERROR: Failed to open osmscout map";
        return -1;
    }

    // open search database
    Kompex::SQLiteDatabase * database;
    Kompex::SQLiteStatement * stmt;

    try   {
        database = new Kompex::SQLiteDatabase(inputArgs[2].toStdString(),
                                              SQLITE_OPEN_READONLY,0);

        stmt = new Kompex::SQLiteStatement(database);
    }
    catch(Kompex::SQLiteException &exception)   {
        qDebug() << "ERROR: SQLite exception opening database:"
                 << QString::fromStdString(exception.GetString());
        return -1;
    }

    // define a search origin
    double origin_lon = -79.33;
    double origin_lat = 43.33;

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

        // get name_id for the given search string
        std::string name_lookup = input_string.substr(0,4);
        std::string query =
                "SELECT name_id FROM name_lookup WHERE "
                "name_lookup.name_lookup = \""+name_lookup+"\";";

        try   {
            stmt->Sql(query);
        }
        catch(Kompex::SQLiteException &e)   {
            qDebug() << "ERROR: SQLite exception for query:"
                     << QString::fromStdString(query);
            qDebug() << "ERROR:" << QString::fromStdString(e.GetString());
            return -1;
        }

        if(!stmt->FetchRow())   {
            qDebug() << "INFO: No results";
            stmt->FreeQuery();
            continue;
        }

        // determine the id range we need to query for
        int64_t name_id = stmt->GetColumnInt(0);
        stmt->FreeQuery();

        int32_t mask = pow(2,g_sz_bits_tile_id)-1;
        int64_t min_id = name_id << g_sz_bits_tile_id;
        int64_t max_id = min_id | mask;

        // use name_id to get search results from
        // admin_region, streets and pois
        query = "SELECT id,node_offsets,way_offsets,area_offsets FROM admin_regions WHERE "
                "admin_regions.id BETWEEN "+intToString(min_id)+" AND "+intToString(max_id)+";";
        qDebug() << QString::fromStdString(query);
        try   {
            stmt->Sql(query);
        }
        catch(Kompex::SQLiteException &e)   {
            qDebug() << "ERROR: SQLite exception for query:"
                     << QString::fromStdString(query);
            qDebug() << "ERROR:" << QString::fromStdString(e.GetString());
            return -1;
        }

        // sort by distance using tiles first
        std::multimap<double,Tile> table_dist2_tiles;
        std::map<int64_t,OffsetGroup> table_tile_streetoffsets;

        while(stmt->FetchRow())   {
            int64_t id = stmt->GetColumnInt64(0);
            int32_t tile_id = (id & mask);

            // save to OffsetGroup
            OffsetGroup g;

            // copy node offsets
            void const * blob_node_offsets = stmt->GetColumnBlob(1);
            size_t sz_node_offsets = stmt->GetColumnBytes(1);
            g.node_offsets.resize(sz_node_offsets/sizeof(osmscout::FileOffset));
            memcpy(&(g.node_offsets[0]),blob_node_offsets,sz_node_offsets);

            // copy way offsets
            void const * blob_way_offsets = stmt->GetColumnBlob(2);
            size_t sz_way_offsets = stmt->GetColumnBytes(2);
            g.way_offsets.resize(sz_way_offsets/sizeof(osmscout::FileOffset));
            memcpy(&(g.way_offsets[0]),blob_way_offsets,sz_way_offsets);

            // copy area offsets
            void const * blob_area_offsets = stmt->GetColumnBlob(3);
            size_t sz_area_offsets = stmt->GetColumnBytes(3);
            g.area_offsets.resize(sz_area_offsets/sizeof(osmscout::FileOffset));
            memcpy(&(g.area_offsets[0]),blob_area_offsets,sz_area_offsets);

            {   // insert offset data
                std::pair<int64_t,OffsetGroup> data;
                data.first = tile_id;
                data.second = g;    // could avoid the copy here
                table_tile_streetoffsets.insert(data);
            }


            Tile tile;
            buildTileFromId(tile_id,tile);

            // straight line squared distance in degrees
            // from the search origin to the tile center
            double diff_lon = (tile.bbox.minLon+tile.bbox.maxLon)*0.5 - origin_lon;
            double diff_lat = (tile.bbox.minLat+tile.bbox.maxLat)*0.5 - origin_lat;
            double dist2 = (diff_lon*diff_lon) + (diff_lat*diff_lat);

            {   // insert tile data
                std::pair<double,Tile> data;
                data.first  = dist2;
                data.second = tile;
                table_dist2_tiles.insert(data);
            }
        }
        stmt->FreeQuery();

        int64_t num_results=0;
        std::map<int64_t,OffsetGroup>::iterator it;
        for(it  = table_tile_streetoffsets.begin();
            it != table_tile_streetoffsets.end(); ++it)
        {
            OffsetGroup &g = it->second;
            num_results += g.node_offsets.size();
            num_results += g.way_offsets.size();
            num_results += g.area_offsets.size();

//            if(it->first == 270621)   {
            if(1)   {
                std::vector<osmscout::WayRef> listWayRefs;
                map.GetWaysByOffset(g.way_offsets,listWayRefs);
                for(size_t n=0; n < listWayRefs.size(); n++)   {
                    osmscout::WayRef const &way = listWayRefs[n];
                    qDebug() << QString::fromStdString(way->GetName());
                }

                std::vector<osmscout::NodeRef> listNodeRefs;
                map.GetNodesByOffset(g.node_offsets,listNodeRefs);
                for(size_t n=0; n < listNodeRefs.size(); n++)   {
                    osmscout::NodeRef const &nd = listNodeRefs[n];
                    qDebug() << QString::fromStdString(nd->GetName());
                }
            }
        }

        qDebug() << "INFO: Results Found: " << num_results;
    }

    return 0;
}

// ============================================================== //
// ============================================================== //
