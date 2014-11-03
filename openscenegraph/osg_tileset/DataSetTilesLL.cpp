/*
   Copyright (C) 2014 Preet Desai (preet.desai@gmail.com)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#include <mutex>

#include <GeometryUtils.h>
#include <DataSetTilesLL.h>
#include <OSGUtils.h>
#include <osgText/Text>
#include <osgDB/ReadFile>
#include <TileSetLLByPixelRes.h>

//
std::mutex mutex_tile_textures;

DataSetTilesLL::DataSetTilesLL(Options const &opts,
                               std::unique_ptr<TileSetLL> tileset,
                               osg::Group * gp_tiles) :
    m_opts(opts),
    m_tileset(std::move(tileset)),
    m_gp_tiles(gp_tiles)
{
    //
    m_num_base_textures = 0;
    for(auto level : m_opts.list_base_levels) {
        m_num_base_textures += pow(4,level);
    }

    //
    m_max_view_textures = 0;
    if(m_opts.max_textures > m_num_base_textures) {
        m_max_view_textures = m_opts.max_textures-m_num_base_textures;
    }

    // Preload the base textures
    m_base_textures_loaded = false;

    // debug
    m_gp_debug = new osg::Group;
    m_gp_tiles->addChild(m_gp_debug);
}

DataSetTilesLL::DataSetTilesLL(osg::Group * gp_tiles,
                               std::unique_ptr<TileSetLL> tileset) :
    m_gp_tiles(gp_tiles),
    m_tileset(std::move(tileset))
{
    m_gp_debug = new osg::Group;
    m_gp_tiles->addChild(m_gp_debug);

    m_poly_mode = new osg::PolygonMode;
    m_poly_mode->setMode(osg::PolygonMode::FRONT_AND_BACK,
                         osg::PolygonMode::LINE);

    for(size_t i=0; i < 20; i++) {
        std::string const path =
                "/home/preet/Dev/misc/tile_test/" +
                std::to_string(i)+
                ".png";

        osg::ref_ptr<osg::Image> image = osgDB::readImageFile(path);

        osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;
        texture->setImage(image);
        texture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
        texture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
        texture->setWrap(osg::Texture2D::WRAP_S,osg::Texture2D::CLAMP_TO_EDGE);
        texture->setWrap(osg::Texture2D::WRAP_T,osg::Texture2D::CLAMP_TO_EDGE);

        m_list_tile_level_tex.push_back(texture);
    }
}

DataSetTilesLL::~DataSetTilesLL()
{

}

void DataSetTilesLL::Update(osg::Camera const * cam)
{
    std::vector<uint64_t> list_tiles_add;
    std::vector<uint64_t> list_tiles_upd;
    std::vector<uint64_t> list_tiles_rem;
    m_tileset->UpdateTileSet(cam,
                             list_tiles_add,
                             list_tiles_upd,
                             list_tiles_rem);

}

//void DataSetTilesLL::Update(osg::Camera const * cam)
//{
//    std::vector<uint64_t> list_tiles_add;
//    std::vector<uint64_t> list_tiles_upd;
//    std::vector<uint64_t> list_tiles_rem;
//    m_tileset->UpdateTileSet(cam,
//                             list_tiles_add,
//                             list_tiles_upd,
//                             list_tiles_rem);

//    for(auto tile_id : list_tiles_rem) {
//        osg::Group * gp_tile = m_list_sg_tiles.find(tile_id)->second;
//        m_gp_tiles->removeChild(gp_tile);

//        m_list_sg_tiles.erase(tile_id);
//    }

//    for(auto tile_id : list_tiles_add) {
//        TileSetLL::Tile const * tile = m_tileset->GetTile(tile_id);
//        osg::ref_ptr<osg::Group> gp = createTileGm(tile_id);
//        auto gp_text = createTileTextGm(tile,"?");
//        gp_text->setName("text");
//        gp->addChild(gp_text);

////        GeoBounds const tile_bounds(tile->min_lon,
////                                    tile->max_lon,
////                                    tile->min_lat,
////                                    tile->max_lat);
////        auto gp = BuildGeoBoundsNode("",
////                                     tile_bounds,
////                                     K_COLOR_TABLE[tile->level],
////                                     360.0/16.0,
////                                     tile->level);

//        m_list_sg_tiles.emplace(tile_id,gp.get());
//        m_gp_tiles->addChild(gp);
//    }

//    // update tile px values

//    for(auto it = m_list_sg_tiles.begin();
//        it != m_list_sg_tiles.end(); ++it)
//    {
//        osg::Group * gp = it->second;
//        for(size_t i=0; i < gp->getNumChildren(); i++) {
//            osg::Group * child = static_cast<osg::Group*>(gp->getChild(i));
//            if(child->getName() == "text") {
//                TileSetLL::Tile const * tile = m_tileset->GetTile(it->first);
//                std::string s = std::to_string(tile->tile_px_res);
//                osg::Geode * gd = static_cast<osg::Geode*>(child->getChild(0));
//                osgText::Text * txt = static_cast<osgText::Text*>(gd->getDrawable(0));
//                txt->setText(s);

//            }
//        }
//    }

////    // debug
////    for(size_t i=0; i < m_gp_debug->getNumChildren(); i++) {
////        std::string const name =
////                m_gp_debug->getChild(i)->getName();

////        if(name == "debug0") {
////            m_gp_debug->removeChild(i);
////            i--;
////        }
////    }

////    TileSetLLByPixelArea * tileset_ptr =
////            static_cast<TileSetLLByPixelArea*>(m_tileset.get());

////    GeoBounds const &debug0 = tileset_ptr->m_debug0;

////    if(debug0.minLon == 0.0 &&
////       debug0.maxLon == 0.0 &&
////       debug0.minLat == 0.0 &&
////       debug0.maxLat == 0.0)
////    {
////        return;
////    }

////    auto gp0 = BuildGeoBoundsSurfaceNode(
////                "debug0",tileset_ptr->m_debug0,osg::Vec4(1,1,1,1),20,true,0,0);
////    m_gp_debug->addChild(gp0);

////    auto gp1 = BuildGeoBoundsSurfaceNode(
////                "debug0",tileset_ptr->m_debug1,osg::Vec4(1,0,0,1),21,true,0,0);
////    m_gp_debug->addChild(gp1);
//}

osg::ref_ptr<osg::Group> DataSetTilesLL::createTileGm(uint64_t tile_id)
{
    TileSetLL::Tile const * tile = m_tileset->GetTile(tile_id);

//    uint32_t surf_divs = 256/K_LIST_TWO_EXP[tile->level];
//    surf_divs = std::min(surf_divs,static_cast<uint32_t>(32));

//    uint32_t const lon_segments =
//            std::max(static_cast<uint32_t>(surf_divs),
//                     static_cast<uint32_t>(1));

//    uint32_t const lat_segments =
//            std::max(static_cast<uint32_t>(lon_segments/2),
//                     static_cast<uint32_t>(1));

    double const min_angle_degs = 360.0/32.0;
    uint32_t lon_segments = std::max((tile->max_lon-tile->min_lon)/min_angle_degs,1.0);
    uint32_t lat_segments = std::max((tile->max_lat-tile->min_lat)/min_angle_degs,1.0);

    std::vector<osg::Vec3d> list_vx;
    std::vector<osg::Vec2d> list_tx;
    std::vector<uint16_t> list_ix;
    BuildEarthSurface(tile->min_lon,
                      tile->max_lon,
                      tile->min_lat,
                      tile->max_lat,
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

osg::ref_ptr<osg::Group>
DataSetTilesLL::createTileTextGm(TileSetLL::Tile const * tile,
                                 std::string const &text)
{
    // center
    LLA lla_mid;
    lla_mid.lon = (tile->min_lon+tile->max_lon)*0.5;
    lla_mid.lat = (tile->min_lat+tile->max_lat)*0.5;
    lla_mid.alt = 0.0;
    osg::Vec3d ecef_mid = ConvLLAToECEF(lla_mid);

    // height
    LLA lla_corner(tile->min_lon,tile->min_lat,0.0);
    osg::Vec3d ecef_corner = ConvLLAToECEF(lla_corner);
    double const height = (ecef_mid-ecef_corner).length() / 5.0;

    return BuildTextNode("",text,K_COLOR_TABLE[tile->level],ecef_mid,height);
//    return BuildTextNode("",text,K_COLOR_TABLE[0],ecef_mid,height);
}
