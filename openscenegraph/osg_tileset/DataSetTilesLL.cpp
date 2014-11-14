#include <DataSetTilesLL.h>
#include <TileImageSourceLL.h>
#include <TileVisibilityLLPixelsPerMeter.h>

#include <OSGUtils.h>

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

        // poly mode for wireframe tiles
        m_poly_mode = new osg::PolygonMode;
        m_poly_mode->setMode(osg::PolygonMode::FRONT_AND_BACK,
                             osg::PolygonMode::LINE);
    }

    DataSetTilesLL::~DataSetTilesLL()
    {

    }

    void DataSetTilesLL::Update(osg::Camera const * cam)
    {
        std::vector<TileLL::Id> list_tiles_add;
        std::vector<TileLL::Id> list_tiles_upd;
        std::vector<TileLL::Id> list_tiles_rem;
        m_tileset->UpdateTileSet(cam,
                                 list_tiles_add,
                                 list_tiles_upd,
                                 list_tiles_rem);

        // remove
        for(auto const tile_id : list_tiles_rem) {
            // Remove this tile from the scene and lookup
            auto it = m_lkup_sg_tiles.find(tile_id);
            assert(it != m_lkup_sg_tiles.end());

            m_gp_tiles->removeChild(it->second);
            m_lkup_sg_tiles.erase(tile_id);
        }

        // add
        for(auto tile_id : list_tiles_add) {
            // Add this tile to the scene and lookup
            auto it = m_lkup_sg_tiles.find(tile_id);
            assert(it == m_lkup_sg_tiles.end());

            // Get the tile
            TileSetLL::TileItem const * tile_item =
                    m_tileset->GetTile(tile_id);

            assert(tile_item);

            osg::ref_ptr<osg::Group> gp_gm =
                    createTileGm(tile_item);

            assert(m_lkup_sg_tiles.insert(std::make_pair(tile_item->id,gp_gm)).second);

            m_gp_tiles->addChild(gp_gm);
        }

//        static uint8_t max_level=0;

//        // remove
//        for(auto const &tile_item : list_tiles_rem) {
//            // Remove this tile from the scene and lookup
//            TileLL::Id const tile_id = tile_item.tile->id;

//            auto it = m_lkup_sg_tiles.find(tile_id);
//            assert(it != m_lkup_sg_tiles.end());

//            m_gp_tiles->removeChild(it->second);
//            m_lkup_sg_tiles.erase(it);
//        }

//        // add
//        for(auto const &tile_item : list_tiles_add) {

//            max_level = std::max(tile_item.tile->level,max_level);

//            // Add this tile to the scene and lookup

//            TileLL::Id const tile_id = tile_item.tile->id;

//            auto it = m_lkup_sg_tiles.find(tile_id);
//            assert(it == m_lkup_sg_tiles.end());

//            osg::ref_ptr<osg::Group> gp_gm =
//                    createTileGm(tile_item);

//            std::pair<TileLL::Id,osg::ref_ptr<osg::Group>> ins_data;
//            ins_data.first = tile_id;
//            ins_data.second = gp_gm;

//            assert(m_lkup_sg_tiles.insert(ins_data).second);
//            m_gp_tiles->addChild(gp_gm);
//        }

//        std::cout << "max_level: " << int(max_level) << std::endl;

        // upd
        // (do nothing)
    }

    osg::ref_ptr<osg::Group>
    DataSetTilesLL::createTileGm(TileSetLL::TileItem const *tile_item)
    {
        TileLL const * tile = tile_item->tile;

    //    uint32_t surf_divs = 256/K_LIST_TWO_EXP[tile->level];
    //    surf_divs = std::min(surf_divs,static_cast<uint32_t>(32));

    //    uint32_t const lon_segments =
    //            std::max(static_cast<uint32_t>(surf_divs),
    //                     static_cast<uint32_t>(1));

    //    uint32_t const lat_segments =
    //            std::max(static_cast<uint32_t>(lon_segments/2),
    //                     static_cast<uint32_t>(1));

        double const min_angle_degs = 360.0/32.0;
        uint32_t lon_segments = std::max((tile->bounds.maxLon-tile->bounds.minLon)/min_angle_degs,1.0);
        uint32_t lat_segments = std::max((tile->bounds.maxLat-tile->bounds.minLat)/min_angle_degs,1.0);

        std::vector<osg::Vec3d> list_vx;
        std::vector<osg::Vec2d> list_tx;
        std::vector<uint16_t> list_ix;
        BuildEarthSurface(tile->bounds.minLon,
                          tile->bounds.maxLon,
                          tile->bounds.minLat,
                          tile->bounds.maxLat,
                          lon_segments,
                          lat_segments,
                          list_vx,
                          list_tx,
                          list_ix);

        osg::ref_ptr<osg::Vec3dArray> vx_array = new osg::Vec3dArray;
        vx_array->reserve(list_vx.size());
        for(auto const &vx : list_vx) {
            vx_array->push_back(vx);
        }

        osg::ref_ptr<osg::Vec2dArray> tx_array = new osg::Vec2dArray;
        tx_array->reserve(list_tx.size());
        for(auto const &tx : list_tx) {
            tx_array->push_back(tx);
        }

        osg::ref_ptr<osg::Vec4Array>  cx_array = new osg::Vec4Array;
        cx_array->push_back(K_COLOR_TABLE[tile->level]);

        osg::ref_ptr<osg::DrawElementsUShort> ix_array =
                new osg::DrawElementsUShort(GL_TRIANGLES);
        ix_array->reserve(list_ix.size());
        for(auto ix : list_ix) {
            ix_array->push_back(ix);
        }

        osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
        gm->setVertexArray(vx_array);
        gm->setTexCoordArray(0,tx_array,osg::Array::BIND_PER_VERTEX);
        gm->setColorArray(cx_array,osg::Array::BIND_OVERALL);
        gm->addPrimitiveSet(ix_array);

        osg::ref_ptr<osg::Geode> gd = new osg::Geode;
        gd->addDrawable(gm);
        gd->getOrCreateStateSet()->setRenderBinDetails(
                    -101+tile->level,"RenderBin");
        gd->getOrCreateStateSet()->setMode(
                    GL_CULL_FACE,
                    osg::StateAttribute::ON |
                    osg::StateAttribute::OVERRIDE);

        // texture
    //    gd->getOrCreateStateSet()->setTextureAttributeAndModes(
    //                0,m_list_tile_level_tex[tile->level]);

        // polygon mode
        gd->getOrCreateStateSet()->setAttributeAndModes(
                    m_poly_mode,
                    osg::StateAttribute::ON);

        osg::ref_ptr<osg::Group> gp = new osg::Group;
        gp->addChild(gd);

        return gp;
    }
}
