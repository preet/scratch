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

#include <cassert>
#include <TileSetLLByPixelArea.h>
#include <GeometryUtils.h>
#include <OSGUtils.h>

// ============================================================= //

TileSetLLByPixelArea::Eval::Eval(GeoBounds const &bounds,
                                 osg::Matrixd const &mvp,
                                 double min_angle_degs)
{
    uint32_t lon_segments = std::max((bounds.maxLon-bounds.minLon)/min_angle_degs,1.0);
    uint32_t lat_segments = std::max((bounds.maxLat-bounds.minLat)/min_angle_degs,1.0);

    BuildEarthSurface(bounds.minLon,
                      bounds.maxLon,
                      bounds.minLat,
                      bounds.maxLat,
                      lon_segments,
                      lat_segments,
                      list_vx,
                      list_ix);

    // get the ndc for each vertex
    bool ok;
    list_ndc.reserve(list_vx.size());
    for(auto const &vx : list_vx) {
        list_ndc.push_back(ConvWorldToNDC(mvp,vx,ok));
    }

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
    m_cam = cam;
    m_mvp = cam->getViewMatrix()*
            cam->getProjectionMatrix();

    osg::Vec3d eye,vpt,up;
    cam->getViewMatrixAsLookAt(eye,vpt,up);
    m_view_dirn = vpt-eye;

    m_near_plane.n = m_view_dirn;
    m_near_plane.n.normalize();

    m_near_plane.p = eye + m_near_plane.n*100.0;
    m_near_plane.d = m_near_plane.n*m_near_plane.p;

    // temp
    m_near_plane_alt.n = m_view_dirn;
    m_near_plane_alt.n.normalize();
    m_near_plane_alt.p = eye + m_near_plane_alt.n*1000.0;
    m_near_plane_alt.d = m_near_plane_alt.n * m_near_plane_alt.p;

    double ar,fovy,z_near,z_far;
    cam->getProjectionMatrixAsPerspective(fovy,ar,z_near,z_far);

    Frustum frustum;
    {
        auto osg_frustum = BuildFrustumNode(
                    "frustum",cam,frustum,z_near,z_far);
    }

    Plane horizon_plane = CalcHorizonPlane(eye);
    std::vector<osg::Vec3d> list_ecef;
    CalcProjFrustumPoly(frustum,horizon_plane,list_ecef);
    CalcMinGeoBoundsFromLLAPoly(ConvECEFToLLA(eye),
                                ConvListECEFToLLA(list_ecef),
                                m_list_frustum_bounds);


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

GeoBounds const & TileSetLLByPixelArea::GetDebug0() const
{
    return m_debug0;
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
    static const double k_ndc_to_px =
            (m_view_width*m_view_height*0.25);

    // Find the intersection of the tile bounds
    // and the frustum proj bounds
    GeoBounds tile_bounds(tile->min_lon,
                          tile->max_lon,
                          tile->min_lat,
                          tile->max_lat);

    uint8_t xsec_count=0;
    GeoBounds view_b;
    GeoBounds xsec_bounds;
    for(auto const &view_bounds : m_list_frustum_bounds) {
        if(CalcGeoBoundsIntersection(tile_bounds,
                                     view_bounds,
                                     xsec_bounds)) {
            view_b = view_bounds;
            xsec_count++;
        }
    }

    assert(xsec_count < 2);

    if(xsec_count == 0) {
        // Tile is not visible in the current view extents
//        if(tile->level == 1) {
//            std::cout << (tile->id) << ": (not visible)" << std::endl;
//        }
        return false;
    }

    // Tile may be visible in the current view extents.
    // Create eval geometry to calculate pixel area
    Eval const eval(xsec_bounds,m_mvp,m_opts.min_eval_angle_degs);
    double ndc_area = calcTileNDCAreaQuad(eval,m_near_plane);
    double ndc_area_alt = calcTileNDCAreaQuad(eval,m_near_plane_alt);

    if(tile->id == 281474976710656) {
        m_debug0 = xsec_bounds;
    }

    if(tile->level == 1) {
//        double area_ratio =
//                CalcGeoBoundsArea(xsec_bounds)/
//                CalcGeoBoundsArea(tile_bounds);

        double px_area = ndc_area * k_ndc_to_px;
//        double px_area_alt = ndc_area_alt * k_ndc_to_px;

//        std::cout << "#: " << (tile->id)
//                  << ": px_area: " << px_area
//                  << ": px_area_alt: " << px_area_alt
//                  << ", area_ratio: " << area_ratio
//                  << std::endl;

        if(px_area == 0) {
//            osg::Vec3d eye,vpt,up;
//            m_cam->getViewMatrixAsLookAt(eye,vpt,up);

//            double fovy,ar,zn,zf;
//            m_cam->getProjectionMatrixAsPerspective(fovy,ar,zn,zf);

//            std::cout << "#:\n";
//            std::cout << "  tile.lon [" << tile_bounds.minLon << "," << tile_bounds.maxLon << "]"
//                      << ", tile.lat [" << tile_bounds.minLat << "," << tile_bounds.maxLat << "]\n"
//                      << "  xsec.lon [" << xsec_bounds.minLon << "," << xsec_bounds.maxLon << "]"
//                      << ", xsec.lat [" << xsec_bounds.minLat << "," << xsec_bounds.maxLat << "]\n"
//                      << "  view.lon [" << view_b.minLon << "," << view_b.maxLon << "]"
//                      << ", view.lat [" << view_b.minLat << "," << view_b.maxLat << "]\n";

//            std::cout << "# cam: " << std::endl;
//            std::cout << "# eye: " << eye << std::endl;
//            std::cout << "# vpt: " << vpt << std::endl;
//            std::cout << "# up: " << up << std::endl;
//            std::cout << "# fovy: " << fovy << std::endl;
//            std::cout << "# ar: " << ar << std::endl;
//            std::cout << "# zn: " << zn << std::endl;
//            std::cout << "# zf: " << zf << std::endl;

//            if(tile->id == 281474976710656) {
//                m_debug0 = xsec_bounds;

//                m_debug0.minLon = tile->min_lon;
//                m_debug0.maxLon = tile->max_lon;
//                m_debug0.minLat = tile->min_lat;
//                m_debug0.maxLat = tile->max_lat;
//            }
        }
    }

    if(ndc_area > 0) {
        //
        double px_area = ndc_area * k_ndc_to_px;

        // determine (tile_bounds_area : xsec_bounds_area)
        double area_ratio =
                CalcGeoBoundsArea(xsec_bounds)/
                CalcGeoBoundsArea(tile_bounds);

        if(px_area > m_tile_px_area*area_ratio) {
//        if(px_area > m_tile_px_area) {
            return true;
        }
    }

    return false;
}

double TileSetLLByPixelArea::calcTileNDCAreaQuad(Eval const &eval,
                                                 Plane const &clip_plane) const
{
    // For each quad
    double area=0.0;
    for(size_t i=0; i < eval.list_ix.size(); i+=6) {
        // If this is a front facing quad
        if((eval.list_quad_nx[i/6]*m_view_dirn) < 0) {
            //

            //
            std::vector<osg::Vec3d> const tri0 = {
                eval.list_vx[eval.list_ix[i+0]],
                eval.list_vx[eval.list_ix[i+1]],
                eval.list_vx[eval.list_ix[i+2]]
            };
            std::vector<osg::Vec2d> const tri_ndc0 = {
                eval.list_ndc[eval.list_ix[i+0]],
                eval.list_ndc[eval.list_ix[i+1]],
                eval.list_ndc[eval.list_ix[i+2]]
            };

            std::vector<osg::Vec3d> const tri1 = {
                eval.list_vx[eval.list_ix[i+3]],
                eval.list_vx[eval.list_ix[i+4]],
                eval.list_vx[eval.list_ix[i+5]]
            };
            std::vector<osg::Vec2d> const tri_ndc1 = {
                eval.list_ndc[eval.list_ix[i+3]],
                eval.list_ndc[eval.list_ix[i+4]],
                eval.list_ndc[eval.list_ix[i+5]]
            };
            //


            area += calcTriangleNDCArea(clip_plane,m_mvp,tri0,tri_ndc0);
            area += calcTriangleNDCArea(clip_plane,m_mvp,tri1,tri_ndc1);
        }
    }

    return area;
}

double TileSetLLByPixelArea::calcTriangleNDCArea(Plane const &clip_plane,
                                                 osg::Matrixd const &mvp,
                                                 std::vector<osg::Vec3d> const &tri,
                                                 std::vector<osg::Vec2d> const &tri_ndc) const
{
    bool ok;
    double area=0.0;

    // NDC bounding rectangle
    static const std::vector<osg::Vec2d> ndc_rect = {
        osg::Vec2d(-1,-1),    // bl
        osg::Vec2d( 1,-1),    // br
        osg::Vec2d( 1, 1),    // tr
        osg::Vec2d(-1, 1)     // tl
    };

    std::vector<osg::Vec3d> inside;
    std::vector<osg::Vec3d> outside;

    GeometryResult const result =
            CalcTrianglePlaneClip(tri,
                                  clip_plane,
                                  inside,
                                  outside);

    if(result == GeometryResult::CLIP_OUTSIDE) {
//        std::cout << "clip_out" << std::endl;
        // Completely outside the near plane
        if(CalcTriangleAARectIntersection(tri_ndc,ndc_rect)) {
            area += CalcTriangleArea(tri_ndc[0],tri_ndc[1],tri_ndc[2]);
        }
    }
    else if(result == GeometryResult::CLIP_XSEC) {
//        std::cout << "clip_xsec" << std::endl;

        // Triangle was clipped against the clip plane.
        // Use the outside tri or quad to calculate area
        std::vector<osg::Vec2d> list_ndc;
        for(auto const & vx : outside) {
            list_ndc.push_back(ConvWorldToNDC(mvp,vx,ok));
        }
        if(list_ndc.size() > 3) { // outside is a quad
            if(CalcQuadAARectIntersection(list_ndc,ndc_rect)) {
                area += CalcTriangleArea(list_ndc[0],list_ndc[1],list_ndc[2]);
                area += CalcTriangleArea(list_ndc[0],list_ndc[2],list_ndc[3]);
            }
        }
        else { // outside is a tri
            if(CalcTriangleAARectIntersection(list_ndc,ndc_rect)) {
                area += CalcTriangleArea(list_ndc[0],list_ndc[1],list_ndc[2]);
            }
        }
    }

    return area;
}

