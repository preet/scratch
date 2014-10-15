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

#include <iostream>

#include <TileSetLLByPixelArea.h>
#include <GeometryUtils.h>

// ============================================================= //

TileSetLLByPixelArea::Eval::Eval(Tile const * tile,
                                 double min_angle_degs)
{
    uint32_t lon_segments = std::max((tile->max_lon-tile->min_lon)/min_angle_degs,1.0);
    uint32_t lat_segments = std::max((tile->max_lat-tile->min_lat)/min_angle_degs,1.0);

    BuildEarthSurface(tile->min_lon,
                      tile->max_lon,
                      tile->min_lat,
                      tile->max_lat,
                      lon_segments,
                      lat_segments,
                      list_vx,
                      list_ix);

    // calculate the normals for each tri
    list_tri_nx.clear();
    list_tri_nx.reserve(list_ix.size()/3);
    for(size_t i=0; i < list_ix.size(); i+=3) {
        osg::Vec3d const &v0 = list_vx[list_ix[i+0]];
        osg::Vec3d const &v1 = list_vx[list_ix[i+1]];
        osg::Vec3d const &v2 = list_vx[list_ix[i+2]];
        list_tri_nx.push_back((v1-v0)^(v2-v0));
    }

    // calculate the normals for each quad
    list_quad_nx.clear();
    list_quad_nx.reserve(list_ix.size()/6);
    for(size_t i=0; i < list_ix.size(); i+=6) {
        // 6 vx per quad (2 tris)
        // the normal of any tri == the quad normal
        // since this is a 'flat' quad
        osg::Vec3d const &v0 = list_vx[list_ix[i+0]];
        osg::Vec3d const &v1 = list_vx[list_ix[i+1]];
        osg::Vec3d const &v2 = list_vx[list_ix[i+2]];
        list_quad_nx.push_back((v1-v0)^(v2-v0));
    }
}

// ============================================================= //

TileSetLLByPixelArea::TileSetLLByPixelArea(double view_width,
                                           double view_height,
                                           Options const &opts,
                                           std::vector<RootTileDesc> const &list_root_tiles) :
    m_opts(opts),
    m_tile_px_area(m_opts.tile_sz_px*m_opts.tile_sz_px),
    m_view_width(view_width),
    m_view_height(view_height)
{
    // Generate tiles from list_root_tiles
    for(auto const &root_tile : list_root_tiles) {
        std::unique_ptr<Tile> tile(new Tile(root_tile));
        m_list_root_tiles.push_back(std::move(tile));
    }
    m_tile_count = m_list_root_tiles.size();
}

TileSetLLByPixelArea::~TileSetLLByPixelArea()
{

}

void TileSetLLByPixelArea::UpdateTileSet(osg::Camera const * cam,
                                         std::vector<uint64_t> &list_tiles_add,
                                         std::vector<uint64_t> &list_tiles_upd,
                                         std::vector<uint64_t> &list_tiles_rem)
{
    // update view data
    m_mvp = cam->getViewMatrix()*
            cam->getProjectionMatrix();

    osg::Vec3d eye,vpt,up;
    cam->getViewMatrixAsLookAt(eye,vpt,up);
    m_view_dirn = vpt-eye;

    double ar,fovy,z_near,z_far;
    cam->getProjectionMatrixAsPerspective(fovy,ar,z_near,z_far);

    // rebuild the tileset with the new view data
    m_tile_count = m_list_root_tiles.size();
    std::map<uint64_t,Tile const *> list_tileset_new;
    for(auto & root_tile : m_list_root_tiles) {
        buildTileSet(root_tile,m_tile_count);
        buildTileSetList(root_tile,list_tileset_new);
    }

    // get tileset lists as sorted lists of tile ids
    std::vector<uint64_t> list_tiles_new;
    list_tiles_new.reserve(list_tileset_new.size());
    for(auto it = list_tileset_new.begin();
        it != list_tileset_new.end(); ++it)
    {
        list_tiles_new.push_back(it->first);
    }

    std::vector<uint64_t> list_tiles_old;
    list_tiles_old.reserve(m_list_tileset.size());
    for(auto it = m_list_tileset.begin();
        it != m_list_tileset.end(); ++it)
    {
        list_tiles_old.push_back(it->first);
    }

    // split the new and old tile sets into
    // tiles added, removed and common
    SplitSets(list_tiles_new,
              list_tiles_old,
              list_tiles_add,  // tiles added
              list_tiles_rem,  // tiles removed
              list_tiles_upd); // tiles common

    // TEMP TODO
    list_tiles_upd.clear();

    m_list_tileset = list_tileset_new;
}

TileSetLL::Tile const * TileSetLLByPixelArea::GetTile(uint64_t tile_id) const
{
    auto it = m_list_tileset.find(tile_id);
    if(it == m_list_tileset.end()) {
        return nullptr;
    }
    else {
        return it->second;
    }
}

void TileSetLLByPixelArea::buildTileSet(std::unique_ptr<Tile> &tile,
                                        size_t &tile_count)
{
    if( (tile->level < m_opts.max_level) &&
        (tile_count < (m_opts.max_tiles - 2)) &&
        tilePxlAreaExceedsRes(tile.get()) ) {
        //
        if(tile->clip == Tile::k_clip_NONE) {
            uint32_t const x = tile->x*2;
            uint32_t const y = tile->y*2;
            tile->tile_LT.reset(new Tile(tile.get(),x,y+1));
            tile->tile_LB.reset(new Tile(tile.get(),x,y));
            tile->tile_RB.reset(new Tile(tile.get(),x+1,y));
            tile->tile_RT.reset(new Tile(tile.get(),x+1,y+1));
            tile->clip = Tile::k_clip_ALL;
        }

        tile_count += 4; // add children
        tile_count -= 1; // rem parent

        buildTileSet(tile->tile_LT,tile_count);
        buildTileSet(tile->tile_LB,tile_count);
        buildTileSet(tile->tile_RB,tile_count);
        buildTileSet(tile->tile_RT,tile_count);
    }
    else {
        if(tile->clip == Tile::k_clip_ALL) {
            tile->tile_LT = nullptr;
            tile->tile_LB = nullptr;
            tile->tile_RB = nullptr;
            tile->tile_RT = nullptr;
            tile->clip = Tile::k_clip_NONE;
        }
    }
}

void TileSetLLByPixelArea::buildTileSetList(std::unique_ptr<Tile> const &tile,
                                            std::map<uint64_t,Tile const *> &list_tiles)
{
    if(tile->clip == Tile::k_clip_NONE) { // visible w no children
        list_tiles.insert(std::pair<uint64_t,Tile const *>(tile->id,tile.get()));
    }
    else {
        buildTileSetList(tile->tile_LT,list_tiles);
        buildTileSetList(tile->tile_LB,list_tiles);
        buildTileSetList(tile->tile_RB,list_tiles);
        buildTileSetList(tile->tile_RT,list_tiles);
    }
}

bool TileSetLLByPixelArea::tilePxlAreaExceedsRes(Tile const * tile)
{
    // TODO
    // fixme/proper metric for clearing this list
    if(m_list_tile_eval.size() > m_opts.max_tiles*2) {
        m_list_tile_eval.clear();
    }

    auto it = m_list_tile_eval.find(tile->id);
    if(it == m_list_tile_eval.end()) {
        // Create eval data if required
        it = m_list_tile_eval.emplace(
                    std::pair<uint64_t,Eval>(
                        tile->id,
                        Eval(tile,m_opts.min_eval_angle_degs))).first;
    }

    // Get area in NDC units and convert to pixels
//    double ndc_area = calcTileNDCArea(it->second);
    double ndc_area = calcTileNDCAreaQuad(it->second);
    double px_area = ndc_area * (m_view_width*m_view_height*0.25);

//    if(tile->level > 2) {
//        for(size_t i=0; i < tile->level; i++) {
//            std::cout << "_";
//        }
//        std::cout << "#: px_area: " << px_area << std::endl;
//    }

    return (px_area > m_tile_px_area);
}

double TileSetLLByPixelArea::calcTileNDCArea(Eval const &eval) const
{
    // Convert tile ecef positions to NDC
    std::vector<std::pair<bool,osg::Vec2d>> list_ndc;
    list_ndc.reserve(eval.list_vx.size());

    for(auto const &vx : eval.list_vx) {
        list_ndc.push_back(ConvWorldToNDC(m_mvp,vx));
    }

    // NDC bounding rectangle
    static const std::vector<osg::Vec2d> ndc_rect = {
        osg::Vec2d(-1,-1),    // bl
        osg::Vec2d( 1,-1),    // br
        osg::Vec2d( 1, 1),    // tr
        osg::Vec2d(-1, 1)     // tl
    };

    // For each triangle
    double area=0.0;
    for(size_t i=0; i < eval.list_ix.size(); i+=3) {
        // If this is a valid, front facing triangle
        if(((eval.list_tri_nx[i/3]*m_view_dirn) < 0) &&
           list_ndc[eval.list_ix[i+0]].first &&
           list_ndc[eval.list_ix[i+1]].first &&
           list_ndc[eval.list_ix[i+2]].first)
        {
            // Check if its within the view frustum
            std::vector<osg::Vec2d> tri = {
                list_ndc[eval.list_ix[i+0]].second,
                list_ndc[eval.list_ix[i+1]].second,
                list_ndc[eval.list_ix[i+2]].second
            };

            if(CalcTriangleAARectIntersection(tri,ndc_rect)) {
                area += CalcTriangleArea(tri[0],tri[1],tri[2]);
            }
        }
    }

    return area;
}

double TileSetLLByPixelArea::calcTileNDCAreaQuad(Eval const &eval) const
{
    // Convert tile ecef positions to NDC
    std::vector<std::pair<bool,osg::Vec2d>> list_ndc;
    list_ndc.reserve(eval.list_vx.size());

    for(auto const &vx : eval.list_vx) {
        list_ndc.push_back(ConvWorldToNDC(m_mvp,vx));
    }

    // NDC bounding rectangle
    static const std::vector<osg::Vec2d> ndc_rect = {
        osg::Vec2d(-1,-1),    // bl
        osg::Vec2d( 1,-1),    // br
        osg::Vec2d( 1, 1),    // tr
        osg::Vec2d(-1, 1)     // tl
    };

    // For each quad
    double area=0.0;
    for(size_t i=0; i < eval.list_ix.size(); i+=6) {
        // If this is a valid, front facing triangle
        if(((eval.list_quad_nx[i/6]*m_view_dirn) < 0) &&
           list_ndc[eval.list_ix[i+0]].first &&
           list_ndc[eval.list_ix[i+1]].first &&
           list_ndc[eval.list_ix[i+2]].first &&
           list_ndc[eval.list_ix[i+5]].first)   // 6th vx is the 4th quad vx
        {                                       // (see BuildEarthSurface)
            // Check if its within the view frustum
            std::vector<osg::Vec2d> quad = {
                list_ndc[eval.list_ix[i+0]].second,
                list_ndc[eval.list_ix[i+1]].second,
                list_ndc[eval.list_ix[i+2]].second,
                list_ndc[eval.list_ix[i+5]].second
            };

            if(CalcQuadAARectIntersection(quad,ndc_rect)) {
                area += CalcTriangleArea(quad[0],quad[1],quad[2]);
                area += CalcTriangleArea(quad[0],quad[2],quad[3]);
            }
        }
    }

    return area;
}

