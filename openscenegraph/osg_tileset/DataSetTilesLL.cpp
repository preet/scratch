#include <DataSetTilesLL.h>
#include <TileImageSourceLL.h>
#include <TileVisibilityLLPixelsPerMeter.h>

namespace scratch
{
    DataSetTilesLL::DataSetTilesLL(osg::Group * gp_tiles) :
        m_gp_tiles(gp_tiles)
    {
        // TileDataSource

        // Create the TileLL::Id -> Image file path
        // generator function
        std::function<std::string(TileLL::Id)> image_path_gen =
                [](TileLL::Id id) -> std::string {

            uint8_t level;
            uint32_t x,y;
            TileLL::GetLevelXYFromId(id,level,x,y);

            std::string path =
                    "/home/preet/Dev/misc/tile_test/" +
                    std::to_string(level)+
                    ".png";

            return path;
        };

        std::unique_ptr<TileImageSourceLL> tile_data_source(
                    new TileImageSourceLL(
                        GeoBounds(-180,180,-90,90),
                        0,
                        18,
                        1,
                        1,
                        image_path_gen));


        // TileVisibility
        std::unique_ptr<TileVisibilityLLPixelsPerMeter> tile_visibility(
                    new TileVisibilityLLPixelsPerMeter(
                        640,
                        480,
                        256));


        // TileSet
        TileSetLL::Options options;
        m_tileset.reset(new TileSetLL(std::move(tile_data_source),
                                      std::move(tile_visibility),
                                      options));
    }

    DataSetTilesLL::~DataSetTilesLL()
    {

    }

    void DataSetTilesLL::Update(osg::Camera const * cam)
    {
        std::vector<TileSetLL::TileItem> list_tiles_add;
        std::vector<TileSetLL::TileItem> list_tiles_upd;
        std::vector<TileSetLL::TileItem> list_tiles_rem;
        m_tileset->UpdateTileSet(cam,
                                 list_tiles_add,
                                 list_tiles_upd,
                                 list_tiles_rem);


    }
}
