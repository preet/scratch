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

// openscenegraph
#ifdef DEBUG_WITH_OSG
#include <osg/PolygonMode>
#include <osg/Drawable>
#include <osg/Geometry>
#include <osgViewer/Viewer>

const char * g_shader_v =
        "// VERTEX SHADER\n"
        "\n"
        "// notes:\n"
        "// to maintain compatibility, the version\n"
        "// preprocessor call needs to be added to the\n"
        "// beginning of this file by the (cpu) compiler:\n"
        "//\n"
        "// \"#version 100\" for OpenGL ES 2 and\n"
        "// \"#version 120\" (or higher) for desktop OpenGL\n"
        "\n"
        "#ifdef GL_ES\n"
        "    // vertex shader defaults for types are:\n"
        "    // precision highp float;\n"
        "    // precision highp int;\n"
        "    // precision lowp sampler2D;\n"
        "    // precision lowp samplerCube;\n"
        "#else\n"
        "    // with default (non ES) OpenGL shaders, precision\n"
        "    // qualifiers aren't used -- we explicitly set them\n"
        "    // to be defined as 'nothing' so they are ignored\n"
        "    #define lowp\n"
        "    #define mediump\n"
        "    #define highp\n"
        "#endif\n"
        "\n"
        "// varyings\n"
        "varying mediump vec4 vColor;\n"
        "\n"
        "void main()\n"
        "{\n"
        "   vColor = gl_Color;\n"
        "   gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;\n"
        "}\n"
        "";


const char * g_shader_f =
        "// FRAGMENT SHADER\n"
        "\n"
        "// notes:\n"
        "// to maintain compatibility, the version\n"
        "// preprocessor call needs to be added to the\n"
        "// beginning of this file by the (cpu) compiler:\n"
        "//\n"
        "// \"#version 100\" for OpenGL ES 2 and\n"
        "// \"#version 120\" (or higher) for desktop OpenGL\n"
        "\n"
        "#ifdef GL_ES\n"
        "    // the fragment shader in ES 2 doesn't have a\n"
        "    // default precision qualifier for floats so\n"
        "    // it needs to be explicitly specified\n"
        "    precision mediump float;\n"
        "\n"
        "    // note: highp may not be available for float types in\n"
        "    // the fragment shader -- use the following to set it:\n"
        "    // #ifdef GL_FRAGMENT_PRECISION_HIGH\n"
        "    // precision highp float;\n"
        "    // #else\n"
        "    // precision mediump float;\n"
        "    // #endif\n"
        "\n"
        "    // fragment shader defaults for other types are:\n"
        "    // precision mediump int;\n"
        "    // precision lowp sampler2D;\n"
        "    // precision lowp samplerCube;\n"
        "#else\n"
        "    // with default (non ES) OpenGL shaders, precision\n"
        "    // qualifiers aren't used -- we explicitly set them\n"
        "    // to be defined as 'nothing' so they are ignored\n"
        "    #define lowp\n"
        "    #define mediump\n"
        "    #define highp\n"
        "#endif\n"
        "\n"
        "// varyings\n"
        "varying vec4 vColor;\n"
        "\n"
        "void main()\n"
        "{\n"
        "   gl_FragColor = vColor;\n"
        "}\n"
        "";

int displayTiles(quadtiles::GeoBoundingBox &bbox_map,
                 std::vector<quadtiles::QuadTile*> &list_tiles);

#endif

// ============================================================== //
// ============================================================== //

struct MapObject
{
    std::string             name;
    osmscout::FileOffset    offset;
    osmscout::RefType       type;

};

struct OffsetGroup
{
    std::vector<quint64> node_offsets;
    std::vector<quint64> way_offsets;
    std::vector<quint64> area_offsets;
};

// ============================================================== //

std::string convNameToLookup(std::string const &name)
{
    QString temp = QString::fromStdString(name).left(4).toLower();
    return temp.toStdString();
}

// ============================================================== //

bool buildTable(Kompex::SQLiteStatement * stmt,
                qint64 &name_id,
                boost::unordered_map<std::string,qint64> &table_names,
                std::string const &sql_table_name,
                std::vector<quadtiles::QuadTile*> list_tiles,
                osmscout::Database &map,
                osmscout::TypeSet const &typeSet,
                bool allow_duplicate_nodes,
                bool allow_duplicate_ways,
                bool allow_duplicate_areas)
{
    // create the sql statement
    std::string stmt_insert = "INSERT INTO "+sql_table_name;
    stmt_insert +=
            "(quad_key,name_id,node_offsets,way_offsets,area_offsets) VALUES("
            "@quad_key,@name_id,@node_offsets,@way_offsets,@area_offsets);";

    try   {
        stmt->BeginTransaction();
        stmt->Sql(stmt_insert);
    }
    catch(Kompex::SQLiteException &exception)   {
        qDebug() << "ERROR: SQLite exception with insert statement:"
                 << QString::fromStdString(exception.GetString());
        return false;
    }

    // osmscout may return nearby results again so we make
    // sure offsets are only included once if requested
    std::set<quint64> set_node_offsets;
    std::set<quint64> set_way_offsets;
    std::set<quint64> set_area_offsets;

    // container for blob memory we delete after
    // committing the sql transaction
    std::vector<char*> list_blobs;

    // keep track of the number of transactions and
    // commit after a certain limit
    size_t transaction_limit=5000;
    size_t transaction_count=0;

    for(size_t i=0; i < list_tiles.size(); i++)   {
        // for each tile

        // get objects from osmscout
        quadtiles::GeoBoundingBox const &bbox = list_tiles[i]->bbox;
        std::vector<osmscout::NodeRef> listNodes;
        std::vector<osmscout::WayRef>  listWays;
        std::vector<osmscout::AreaRef> listAreas;
        map.GetObjects(bbox.minLon,bbox.minLat,
                       bbox.maxLon,bbox.maxLat,
                       typeSet,
                       listNodes,
                       listWays,
                       listAreas);

        // merge all of the object refs into one list
        // of MapObjects so its easier to manage
        std::vector<MapObject> list_map_objects;
        list_map_objects.reserve(listNodes.size()+listWays.size()+listAreas.size());

        for(size_t j=0; j < listNodes.size(); j++)   {
            osmscout::NodeRef &nodeRef = listNodes[j];

            if(nodeRef->GetName().empty())   {
                continue;
            }
            if(!allow_duplicate_nodes)   {
                if(set_node_offsets.count(nodeRef->GetFileOffset()) != 0)   {
                    continue;
                }
                set_node_offsets.insert(nodeRef->GetFileOffset());
            }
            MapObject map_object;
            map_object.name     = nodeRef->GetName();
            map_object.offset   = nodeRef->GetFileOffset();
            map_object.type     = osmscout::refNode;
            list_map_objects.push_back(map_object);
        }
        for(size_t j=0; j < listWays.size(); j++)   {
            osmscout::WayRef &wayRef = listWays[j];

            if(wayRef->GetName().empty())   {
                continue;
            }
            if(!allow_duplicate_ways)   {
                if(set_way_offsets.count(wayRef->GetFileOffset()) != 0)   {
                    continue;
                }
                set_way_offsets.insert(wayRef->GetFileOffset()).second;
            }
            MapObject map_object;
            map_object.name     = wayRef->GetName();
            map_object.offset   = wayRef->GetFileOffset();
            map_object.type     = osmscout::refWay;
            list_map_objects.push_back(map_object);
        }
        for(size_t j=0; j < listAreas.size(); j++)   {
            osmscout::AreaRef &areaRef = listAreas[j];

            if(areaRef->rings.front().GetName().empty())   {
                continue;
            }
            if(!allow_duplicate_areas)   {
                if(set_area_offsets.count(areaRef->GetFileOffset()) != 0)   {
                    continue;
                }
                set_area_offsets.insert(areaRef->GetFileOffset());
            }
            MapObject map_object;
            map_object.name     = areaRef->rings.front().GetName();
            map_object.offset   = areaRef->GetFileOffset();
            map_object.type     = osmscout::refArea;
            list_map_objects.push_back(map_object);
        }

        // create structs to sort file offsets by name lookup
        // [name_lookup_id] [list_offsets]
        boost::unordered_map<qint64,OffsetGroup> entry_admin_regions;

        for(size_t j=0; j < list_map_objects.size(); j++)   {

            MapObject &map_object = list_map_objects[j];

            // add name_lookup up to table_names
            std::string name_lookup = convNameToLookup(map_object.name);
            qint64 name_lookup_id;

            // check if this lookup string already exists
            if(table_names.count(name_lookup) == 0)   {
                // insert lookup string
                std::pair<std::string,qint64> data;
                data.first  = name_lookup;
                data.second = name_id;
                table_names.insert(data);

                name_lookup_id = name_id;
                name_id++;
            }
            else   {
                name_lookup_id = table_names.find(name_lookup)->second;
            }

            // check if this lookup string already exists
            if(entry_admin_regions.count(name_lookup_id) == 0)   {
                // insert new list of offsets
                std::pair<qint64,OffsetGroup> data;
                data.first = name_lookup_id;
                entry_admin_regions.insert(data);
            }

            // add offset to list
            boost::unordered_map<qint64,OffsetGroup>::iterator it;
            it = entry_admin_regions.find(name_lookup_id);
            if(map_object.type == osmscout::refNode)   {
                it->second.node_offsets.push_back(map_object.offset);
            }
            else if(map_object.type == osmscout::refWay)   {
                it->second.way_offsets.push_back(map_object.offset);
            }
            else if(map_object.type == osmscout::refArea)   {
                it->second.area_offsets.push_back(map_object.offset);
            }
        }

        // write information for this tile to the database
        boost::unordered_map<qint64,OffsetGroup>::iterator it;
        for(it  = entry_admin_regions.begin();
            it != entry_admin_regions.end(); ++it)   {
            // convert offsets to byte arrays
            char * data_node_offsets = NULL; size_t sz_node_offsets=0;
            char * data_way_offsets  = NULL; size_t sz_way_offsets=0;
            char * data_area_offsets = NULL; size_t sz_area_offsets=0;

            OffsetGroup &g = it->second;

            if(!(g.node_offsets.empty()))   {
                size_t num_bytes = sizeof(quint64)*g.node_offsets.size();
                data_node_offsets = new char[num_bytes];
                memcpy(data_node_offsets,&(g.node_offsets[0]),num_bytes);
                list_blobs.push_back(data_node_offsets);
            }
            if(!(g.way_offsets.empty()))   {
                size_t num_bytes = sizeof(quint64)*g.way_offsets.size();
                data_way_offsets = new char[num_bytes];
                memcpy(data_way_offsets,&(g.way_offsets[0]),num_bytes);
                list_blobs.push_back(data_way_offsets);
            }
            if(!(g.area_offsets.empty()))   {
                size_t num_bytes = sizeof(quint64)*g.area_offsets.size();
                data_area_offsets = new char[num_bytes];
                memcpy(data_area_offsets,&(g.area_offsets[0]),num_bytes);
                list_blobs.push_back(data_area_offsets);
            }

            if(data_node_offsets == NULL &&
               data_way_offsets  == NULL &&
               data_area_offsets == NULL)   {
                continue;
            }

            // prepare sql
            try   {
                stmt->BindInt64(1,list_tiles[i]->key);
                stmt->BindInt64(2,it->first);

                if(data_node_offsets)   {
                    stmt->BindBlob(3,data_node_offsets,sz_node_offsets);
                }
                else   {
                    stmt->BindNull(3);
                }
                if(data_way_offsets)   {
                    stmt->BindBlob(4,data_way_offsets,sz_way_offsets);
                }
                else   {
                    stmt->BindNull(4);
                }
                if(data_area_offsets)   {
                    stmt->BindBlob(5,data_area_offsets,sz_area_offsets);
                }
                else   {
                    stmt->BindNull(5);
                }
                stmt->Execute();
                stmt->Reset();

                if(transaction_count > transaction_limit)   {
                    stmt->FreeQuery();
                    stmt->CommitTransaction();

                    stmt->BeginTransaction();
                    stmt->Sql(stmt_insert);
                    transaction_count=0;
                }
                transaction_count++;
            }
            catch(Kompex::SQLiteException &exception)   {
                qDebug() << "ERROR: SQLite exception writing tile data:"
                         << QString::fromStdString(exception.GetString());
                qDebug() << "ERROR: key:" << list_tiles[i]->key;
                qDebug() << "ERROR: name_lookup_id:" << it->first;
                qDebug() << "ERROR:" << sz_node_offsets << sz_way_offsets << sz_area_offsets;
                return false;
            }
        }
        // debug
//        qDebug() << i << "/" << list_tiles.size();
    }

    stmt->FreeQuery();
    stmt->CommitTransaction();

    // free up blob memory
    for(size_t i=0; i < list_blobs.size(); i++)   {
        delete[] list_blobs[i];
    }

    return true;
}

// ============================================================== //

bool buildNameLookupTable(Kompex::SQLiteStatement * stmt,
                          boost::unordered_map<std::string,qint64> &table_names)
{
    std::string stmt_insert = "INSERT INTO name_lookup";
    stmt_insert += "(name_id,name_lookup) VALUES(@name_id,@name_lookup);";

    try   {
        stmt->BeginTransaction();
        stmt->Sql(stmt_insert);
    }
    catch(Kompex::SQLiteException &exception)   {
        qDebug() << "ERROR: SQLite exception with insert statement:"
                 << QString::fromStdString(exception.GetString());
        return false;
    }

    // keep track of the number of transactions and
    // commit after a certain limit
    size_t transaction_limit=5000;
    size_t transaction_count=0;

    // write name lookup info the database
    boost::unordered_map<std::string,qint64>::iterator it;
    for(it  = table_names.begin(); it != table_names.end(); ++it)   {
        // prepare sql
        try   {
            //[name_id, name_lookup]
            stmt->BindInt64(1,it->second);
            stmt->BindString(2,it->first);
            stmt->Execute();
            stmt->Reset();

            if(transaction_count > transaction_limit)   {
                stmt->FreeQuery();
                stmt->CommitTransaction();

                stmt->BeginTransaction();
                stmt->Sql(stmt_insert);
                transaction_count=0;
            }
            transaction_count++;
        }
        catch(Kompex::SQLiteException &exception)   {
            qDebug() << "ERROR: SQLite exception writing tile data:"
                     << QString::fromStdString(exception.GetString());
            qDebug() << "ERROR: name_id:" << it->second;
            qDebug() << "ERROR: name_lookup:" << QString::fromStdString(it->first);
            return false;
        }
    }

    stmt->FreeQuery();
    stmt->CommitTransaction();

    return true;
}

// ============================================================== //

void setTypesForAdminRegions(osmscout::TypeConfig const * typeConfig,
                             osmscout::TypeSet &typeSet)
{
    typeSet.Clear();

    typeSet.SetType(typeConfig->GetAreaTypeId("place_city"));
    typeSet.SetType(typeConfig->GetAreaTypeId("place_town"));
    typeSet.SetType(typeConfig->GetAreaTypeId("place_village"));
    typeSet.SetType(typeConfig->GetAreaTypeId("place_hamlet"));
    typeSet.SetType(typeConfig->GetAreaTypeId("place_suburb"));

    typeSet.SetType(typeConfig->GetNodeTypeId("place_city"));
    typeSet.SetType(typeConfig->GetNodeTypeId("place_town"));
    typeSet.SetType(typeConfig->GetNodeTypeId("place_village"));
    typeSet.SetType(typeConfig->GetNodeTypeId("place_hamlet"));
    typeSet.SetType(typeConfig->GetNodeTypeId("place_suburb"));
}

// ============================================================== //

void setTypesForStreets(osmscout::TypeConfig const * typeConfig,
                        osmscout::TypeSet &typeSet)
{
    typeSet.Clear();

    typeSet.SetType(typeConfig->GetWayTypeId("highway_motorway"));
    typeSet.SetType(typeConfig->GetWayTypeId("highway_motorway_link"));
    typeSet.SetType(typeConfig->GetWayTypeId("highway_trunk"));
    typeSet.SetType(typeConfig->GetWayTypeId("highway_trunk_link"));
    typeSet.SetType(typeConfig->GetWayTypeId("highway_primary"));
    typeSet.SetType(typeConfig->GetWayTypeId("highway_primary_link"));
    typeSet.SetType(typeConfig->GetWayTypeId("highway_secondary"));
    typeSet.SetType(typeConfig->GetWayTypeId("highway_secondary_link"));
    typeSet.SetType(typeConfig->GetWayTypeId("highway_tertiary"));
    typeSet.SetType(typeConfig->GetWayTypeId("highway_tertiary_link"));
    typeSet.SetType(typeConfig->GetWayTypeId("highway_residential"));
    typeSet.SetType(typeConfig->GetWayTypeId("highway_living_street"));
    typeSet.SetType(typeConfig->GetWayTypeId("highway_service"));
    typeSet.SetType(typeConfig->GetWayTypeId("highway_pedestrian"));
    typeSet.SetType(typeConfig->GetWayTypeId("highway_unclassified"));

    typeSet.SetType(typeConfig->GetWayTypeId("highway_road"));
    typeSet.SetType(typeConfig->GetWayTypeId("highway_track_paved"));
    typeSet.SetType(typeConfig->GetWayTypeId("highway_track_unpaved"));

    typeSet.SetType(typeConfig->GetWayTypeId("highway_cycleway"));
    typeSet.SetType(typeConfig->GetWayTypeId("highway_footway"));
    typeSet.SetType(typeConfig->GetWayTypeId("highway_path"));
}

// ============================================================== //

void setTypesForPOIs(osmscout::TypeConfig const * typeConfig,
                     osmscout::TypeSet &typeSet)
{
    typeSet.Clear();

    // nodes
    typeSet.SetType(typeConfig->GetNodeTypeId("aeroway_aerodrome"));

    typeSet.SetType(typeConfig->GetNodeTypeId("leisure_park"));
    typeSet.SetType(typeConfig->GetNodeTypeId("leisure_nature_reserve"));
    typeSet.SetType(typeConfig->GetNodeTypeId("leisure_beach"));
    typeSet.SetType(typeConfig->GetNodeTypeId("leisure_stadium"));

    typeSet.SetType(typeConfig->GetNodeTypeId("amenity_cafe"));
    typeSet.SetType(typeConfig->GetNodeTypeId("amenity_fast_food"));
    typeSet.SetType(typeConfig->GetNodeTypeId("amenity_restaurant"));

    typeSet.SetType(typeConfig->GetNodeTypeId("amenity_fuel"));
    typeSet.SetType(typeConfig->GetNodeTypeId("amenity_parking"));

    typeSet.SetType(typeConfig->GetNodeTypeId("amenity_atm"));

    typeSet.SetType(typeConfig->GetNodeTypeId("amenity_hospital"));
    typeSet.SetType(typeConfig->GetNodeTypeId("amenity_pharmacy"));

    typeSet.SetType(typeConfig->GetNodeTypeId("amenity_bar"));
    typeSet.SetType(typeConfig->GetNodeTypeId("amenity_nightclub"));

    typeSet.SetType(typeConfig->GetNodeTypeId("shop"));

    typeSet.SetType(typeConfig->GetNodeTypeId("tourism_guest_house"));
    typeSet.SetType(typeConfig->GetNodeTypeId("tourism_hostel"));
    typeSet.SetType(typeConfig->GetNodeTypeId("tourism_hotel"));
    typeSet.SetType(typeConfig->GetNodeTypeId("tourism_motel"));

    typeSet.SetType(typeConfig->GetNodeTypeId("highway_services"));

    // areas
    typeSet.SetType(typeConfig->GetAreaTypeId("aeroway_aerodrome"));

    typeSet.SetType(typeConfig->GetAreaTypeId("leisure_park"));
    typeSet.SetType(typeConfig->GetAreaTypeId("leisure_nature_reserve"));
    typeSet.SetType(typeConfig->GetAreaTypeId("leisure_beach"));
    typeSet.SetType(typeConfig->GetAreaTypeId("leisure_stadium"));

    typeSet.SetType(typeConfig->GetAreaTypeId("amenity_cafe"));
    typeSet.SetType(typeConfig->GetAreaTypeId("amenity_fast_food"));
    typeSet.SetType(typeConfig->GetAreaTypeId("amenity_restaurant"));

    typeSet.SetType(typeConfig->GetAreaTypeId("amenity_fuel"));
    typeSet.SetType(typeConfig->GetAreaTypeId("amenity_parking"));

    typeSet.SetType(typeConfig->GetAreaTypeId("amenity_atm"));

    typeSet.SetType(typeConfig->GetAreaTypeId("amenity_hospital"));
    typeSet.SetType(typeConfig->GetAreaTypeId("amenity_pharmacy"));

    typeSet.SetType(typeConfig->GetAreaTypeId("amenity_bar"));
    typeSet.SetType(typeConfig->GetAreaTypeId("amenity_nightclub"));

    typeSet.SetType(typeConfig->GetAreaTypeId("shop"));

    typeSet.SetType(typeConfig->GetAreaTypeId("tourism_guest_house"));
    typeSet.SetType(typeConfig->GetAreaTypeId("tourism_hostel"));
    typeSet.SetType(typeConfig->GetAreaTypeId("tourism_hotel"));
    typeSet.SetType(typeConfig->GetAreaTypeId("tourism_motel"));
}

// ============================================================== //

void badInput()
{
    qDebug() << "ERROR: Bad arguments";
    qDebug() << "ex:";
    qDebug() << "./gensearchdb <osmscout_map_dir>";
}

// ============================================================== //
// ============================================================== //

int main(int argc, char *argv[])
{
    QCoreApplication app(argc,argv);

    // check input args
    QStringList inputArgs = app.arguments();
    if(inputArgs.size() != 2)   {
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

    // create search database
    Kompex::SQLiteDatabase * database;
    Kompex::SQLiteStatement * stmt;

    try   {
        database = new Kompex::SQLiteDatabase("searchdb.sqlite",
                                              SQLITE_OPEN_READWRITE |
                                              SQLITE_OPEN_CREATE,0);

        stmt = new Kompex::SQLiteStatement(database);

        stmt->SqlStatement("CREATE TABLE name_lookup("
                           "name_id INTEGER PRIMARY KEY NOT NULL,"
                           "name_lookup TEXT NOT NULL UNIQUE);");

        stmt->SqlStatement("CREATE TABLE admin_regions("
                           "quad_key INTEGER NOT NULL,"
                           "name_id INTEGER NOT NULL,"
                           "node_offsets BLOB,"
                           "way_offsets BLOB,"
                           "area_offsets BLOB,"
                           "PRIMARY KEY(quad_key,name_id)"
                           ");");

        stmt->SqlStatement("CREATE TABLE streets("
                           "quad_key INTEGER NOT NULL,"
                           "name_id INTEGER NOT NULL,"
                           "node_offsets BLOB,"
                           "way_offsets BLOB,"
                           "area_offsets BLOB,"
                           "PRIMARY KEY(quad_key,name_id)"
                           ");");

        stmt->SqlStatement("CREATE TABLE pois("
                           "quad_key INTEGER NOT NULL,"
                           "name_id INTEGER NOT NULL,"
                           "node_offsets BLOB,"
                           "way_offsets BLOB,"
                           "area_offsets BLOB,"
                           "PRIMARY KEY(quad_key,name_id)"
                           ");");
    }
    catch(Kompex::SQLiteException &exception)   {
        qDebug() << "ERROR: SQLite exception creating database:"
                 << QString::fromStdString(exception.GetString());
        return -1;
    }


    // get all level 10 quad tiles for the dataset

    // world bbox
    quadtiles::GeoBoundingBox bbox_world;
    bbox_world.minLon = -180.0; bbox_world.maxLon = 180.0;
    bbox_world.minLat = -90.0;  bbox_world.maxLat = 90.0;

    // map data bbox
    quadtiles::GeoBoundingBox bbox_map;
    map.GetBoundingBox(bbox_map.minLat,bbox_map.minLon,
                       bbox_map.maxLat,bbox_map.maxLon);

    // generate tile list
    std::vector<quadtiles::QuadTile*> list_tiles;

    {
        std::vector<quadtiles::QuadTile*> list_tiles_temp;
        quadtiles::QuadTile * root = new quadtiles::QuadTile(0,0,bbox_world);
        quadtiles::buildQuadTilesForBounds(root,9,bbox_map,list_tiles_temp);

        // save tiles to local list
        for(size_t i=0; i < list_tiles_temp.size(); i++)   {
            quadtiles::QuadTile * tile = new quadtiles::QuadTile(
                        list_tiles_temp[i]->level,
                        list_tiles_temp[i]->key,
                        list_tiles_temp[i]->bbox);
            list_tiles.push_back(tile);
        }

        // clean up
        quadtiles::deleteQuadTree(root);
    }

#ifdef DEBUG_WITH_OSG
    return displayTiles(bbox_map,list_tiles);
#endif


    // [name_id] [name_key]
    qint64 name_id=1;
    boost::unordered_map<std::string,qint64> table_names;

    // use the tile bounds to query osmscout

    osmscout::TypeConfig * typeConfig = map.GetTypeConfig();
    osmscout::TypeSet typeSet;

    // build database tables
    bool opOk=false;

    // admin_regions
    qDebug() << "INFO: Building admin_regions table...";
    setTypesForAdminRegions(typeConfig,typeSet);
    opOk = buildTable(stmt,name_id,table_names,"admin_regions",
                      list_tiles,map,typeSet,false,true,true);
    if(opOk)   {
        qDebug() << "INFO: Finished building admin_regions table";
    }
    else   {
        qDebug() << "ERROR: Failed to build admin_regions table";
        return -1;
    }

    // streets
    qDebug() << "INFO: Building streets table...";
    setTypesForStreets(typeConfig,typeSet);
    opOk = buildTable(stmt,name_id,table_names,"streets",
                      list_tiles,map,typeSet,false,false,true);
    if(opOk)   {
        qDebug() << "INFO: Finished building streets table";
    }
    else   {
        qDebug() << "ERROR: Failed to build streets table";
        return -1;
    }

    // pois
    qDebug() << "INFO: Building pois table...";
    setTypesForPOIs(typeConfig,typeSet);
    opOk = buildTable(stmt,name_id,table_names,"pois",
                      list_tiles,map,typeSet,false,false,false);
    if(opOk)   {
        qDebug() << "INFO: Finished building pois table";
    }
    else   {
        qDebug() << "ERROR: Failed to build pois table";
        return -1;
    }

    // build name_lookup table
    qDebug() << "INFO: Building name_lookup table...";
    opOk = buildNameLookupTable(stmt,table_names);
    if(opOk)   {
        qDebug() << "INFO: Finished building name_lookup table";
    }
    else   {
        qDebug() << "ERROR: Failed to build name_lookup table";
        return -1;
    }

    // clean up
    for(size_t i=0; i < list_tiles.size(); i++)   {
        delete list_tiles[i];
    }
    list_tiles.clear();
    delete stmt;
    delete database;

//    // debug
//    boost::unordered_map<std::string,qint64>::iterator it;
//    for(it  = table_names.begin(); it != table_names.end(); ++it)
//    {
//        qDebug() << it->second << ": " << QString::fromStdString(it->first);
//    }
}

// ============================================================== //
// ============================================================== //


#ifdef DEBUG_WITH_OSG
int displayTiles(quadtiles::GeoBoundingBox &bbox_map,
                 std::vector<quadtiles::QuadTile*> &list_tiles)
{
    // setup shaders
    std::string ver_str;
    std::string v_shader_str(g_shader_v);
    std::string f_shader_str(g_shader_f);
    ver_str = "#version 120\n"; // desktop opengl 2

    osg::ref_ptr<osg::Program> shader = new osg::Program;
    shader->setName("shader");
    shader->addShader(new osg::Shader(osg::Shader::VERTEX,ver_str+v_shader_str));
    shader->addShader(new osg::Shader(osg::Shader::FRAGMENT,ver_str+f_shader_str));

    // root node
    osg::ref_ptr<osg::Group> grRoot = new osg::Group;
    osg::StateSet * ss = grRoot->getOrCreateStateSet();
    ss->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

    // tiles geode
    osg::ref_ptr<osg::Geode> gdTiles = new osg::Geode;
    ss = gdTiles->getOrCreateStateSet();
    ss->setAttributeAndModes(shader);

    // tiles geometry
    for(size_t i=0; i < list_tiles.size(); i++)   {
        quadtiles::GeoBoundingBox const &bbox = list_tiles[i]->bbox;
        osg::ref_ptr<osg::Vec3dArray> listVx = new osg::Vec3dArray;
        listVx->push_back(osg::Vec3d(bbox.minLon,bbox.minLat,0.1));  // BL
        listVx->push_back(osg::Vec3d(bbox.maxLon,bbox.minLat,0.1));  // BR
        listVx->push_back(osg::Vec3d(bbox.maxLon,bbox.maxLat,0.1));  // TR
        listVx->push_back(osg::Vec3d(bbox.minLon,bbox.maxLat,0.1));  // TL

        osg::ref_ptr<osg::Vec4Array> listCx = new osg::Vec4Array;
        listCx->push_back(osg::Vec4(0,1,0,1));
        listCx->push_back(osg::Vec4(0,1,0,1));
        listCx->push_back(osg::Vec4(0,1,0,1));
        listCx->push_back(osg::Vec4(0,1,0,1));

        osg::ref_ptr<osg::Geometry> gmTile = new osg::Geometry;
        gmTile->setVertexArray(listVx);
        gmTile->setColorArray(listCx);
        gmTile->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
        gmTile->addPrimitiveSet(new osg::DrawArrays(GL_LINE_LOOP,0,4));

        gdTiles->addDrawable(gmTile);
    }

    // bbox geometry
    {
        osg::ref_ptr<osg::Vec3dArray> listVx = new osg::Vec3dArray;
        listVx->push_back(osg::Vec3d(bbox_map.minLon,bbox_map.minLat,0));  // BL
        listVx->push_back(osg::Vec3d(bbox_map.maxLon,bbox_map.minLat,0));  // BR
        listVx->push_back(osg::Vec3d(bbox_map.maxLon,bbox_map.maxLat,0));  // TR
        listVx->push_back(osg::Vec3d(bbox_map.minLon,bbox_map.maxLat,0));  // TL

        osg::ref_ptr<osg::Vec4Array> listCx = new osg::Vec4Array;
        listCx->push_back(osg::Vec4(1,0,0,1));
        listCx->push_back(osg::Vec4(1,0,0,1));
        listCx->push_back(osg::Vec4(1,0,0,1));
        listCx->push_back(osg::Vec4(1,0,0,1));

        osg::ref_ptr<osg::Geometry> gmMapBounds = new osg::Geometry;
        gmMapBounds->setVertexArray(listVx);
        gmMapBounds->setColorArray(listCx);
        gmMapBounds->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
        gmMapBounds->addPrimitiveSet(new osg::DrawArrays(GL_LINE_LOOP,0,4));

        gdTiles->addDrawable(gmMapBounds);
    }

    grRoot->addChild(gdTiles);

    // viewer
    osgViewer::Viewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    viewer.setUpViewInWindow(100,100,800,480);
    viewer.setSceneData(grRoot.get());
    viewer.getCamera()->setClearColor(osg::Vec4(0.15,0.15,0.15,1.0));

    osgViewer::Viewer::Windows windows;
    viewer.getWindows(windows);
    for(osgViewer::Viewer::Windows::iterator itr = windows.begin();
        itr != windows.end();
        ++itr)
    {
        (*itr)->getState()->setUseModelViewAndProjectionUniforms(true);
        (*itr)->getState()->setUseVertexAttributeAliasing(true);
    }

    return viewer.run();
}
#endif
