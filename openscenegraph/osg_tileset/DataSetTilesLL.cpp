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

#include <GeometryUtils.h>
#include <DataSetTilesLL.h>
#include <OSGUtils.h>

DataSetTilesLL::DataSetTilesLL(osg::Group * gp_tiles,
                               std::unique_ptr<TileSetLL> tileset) :
    m_gp_tiles(gp_tiles),
    m_tileset(std::move(tileset))
{
    m_poly_mode = new osg::PolygonMode;
    m_poly_mode->setMode(osg::PolygonMode::FRONT_AND_BACK,
                         osg::PolygonMode::LINE);
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

    for(auto tile_id : list_tiles_rem) {
        osg::Group * gp_tile = m_list_sg_tiles.find(tile_id)->second;
        m_gp_tiles->removeChild(gp_tile);

        m_list_sg_tiles.erase(tile_id);
    }

    for(auto tile_id : list_tiles_add) {
        osg::ref_ptr<osg::Group> gp = createTileGm(tile_id);
        m_list_sg_tiles.emplace(tile_id,gp.get());
        m_gp_tiles->addChild(gp);
    }
}

osg::ref_ptr<osg::Group> DataSetTilesLL::createTileGm(uint64_t tile_id)
{
    TileSetLL::Tile const * tile = m_tileset->GetTile(tile_id);

    uint32_t surf_divs = 256/K_LIST_TWO_EXP[tile->level];
    surf_divs = std::min(surf_divs,static_cast<uint32_t>(32));

//    uint32_t const lon_segments =
//            std::max(static_cast<uint32_t>(surf_divs),
//                     static_cast<uint32_t>(1));

//    uint32_t const lat_segments =
//            std::max(static_cast<uint32_t>(lon_segments/2),
//                     static_cast<uint32_t>(1));

    double const min_angle_degs = 360.0/16.0;

    uint32_t lon_segments = std::max((tile->max_lon-tile->min_lon)/min_angle_degs,1.0);
    uint32_t lat_segments = std::max((tile->max_lat-tile->min_lat)/min_angle_degs,1.0);

    std::vector<osg::Vec3d> list_vx;
    std::vector<uint16_t> list_ix;
    BuildEarthSurface(tile->min_lon,
                      tile->max_lon,
                      tile->min_lat,
                      tile->max_lat,
                      lon_segments,
                      lat_segments,
                      list_vx,
                      list_ix);

    osg::ref_ptr<osg::Vec3dArray> vx_array = new osg::Vec3dArray;
    vx_array->reserve(list_vx.size());
    for(auto const &vx : list_vx) {
        vx_array->push_back(vx);
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
    gd->getOrCreateStateSet()->setAttributeAndModes(
                m_poly_mode,
                osg::StateAttribute::ON);

    osg::ref_ptr<osg::Group> gp = new osg::Group;
    gp->addChild(gd);

    return gp;
}
