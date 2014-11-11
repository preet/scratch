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

#ifndef SCRATCH_TILE_VISIBILITY_LL_PPM_H
#define SCRATCH_TILE_VISIBILITY_LL_PPM_H

#include <osg/Camera>

#include <MiscUtils.h>
#include <TileVisibilityLL.h>

namespace scratch
{
    class TileVisibilityLLPixelsPerMeter : public TileVisibilityLL
    {
    public:
        TileVisibilityLLPixelsPerMeter(double view_width_px,
                                       double view_height_px,
                                       size_t tile_tex_sz_px,
                                       size_t eval_cache_hint=128);
        ~TileVisibilityLLPixelsPerMeter();

        void Update(osg::Camera const * cam);

        virtual void GetVisibility(TileLL const * tile,
                                   bool & is_visible,
                                   bool & exceeds_err_threshold);

    private:
        struct Eval
        {
            Eval(TileLL::Id id,
                 GeoBounds const &bounds);

            TileLL::Id const id;

            // surface area (m^2)
            double surf_area_m2;

            // mid
            osg::Vec3d ecef_mid;

            // corner points
            // (min/max)lon_(min,max)lat
            osg::Vec3d c_min_min;
            osg::Vec3d c_max_min;
            osg::Vec3d c_max_max;
            osg::Vec3d c_min_max;

            // planes
            Plane plane_min_lon;
            Plane plane_max_lon;
            Plane plane_min_lat;
            Plane plane_max_lat;

            // circle arc edges
            Circle circle_min_lon;
            Circle circle_max_lon;
            Circle circle_min_lat;
            Circle circle_max_lat;
        };

        // * checks whether or not the projected frustum poly
        //   as specified by @list_frustum_vx,)_bounds,_tri_planes
        //   intersects the tile given by @tile_bounds
        bool calcFrustumTileIntersection(Eval const &eval,
                                         std::vector<osg::Vec3d> const &list_frustum_vx,
                                         std::vector<GeoBounds> const &list_frustum_bounds,
                                         std::vector<Plane> const &list_frustum_tri_planes,
                                         GeoBounds const &tile_bounds) const;

        bool calcPointWithinTilePlanes(osg::Vec3d const &point,
                                       Plane const &plane_min_lon,
                                       Plane const &plane_max_lon,
                                       Plane const &plane_min_lat,
                                       Plane const &plane_max_lat) const;

        // * calculates the edge planes of the triangles that
        //   make up a triangulated frustum proj poly
        // * plane normals face outward
        // * triangulation is pre-determined; expects:
        //   - list_frustum_vx must have 8 vertices in CCW order
        std::vector<Plane>
        calcFrustumPolyTriPlanes(std::vector<osg::Vec3d> const &list_frustum_vx,
                                 bool normalize) const;

        //
        osg::Vec3d calcTileClosestPoint(LLA const &lla_distal,
                                        osg::Vec3d const &ecef_distal,
                                        GeoBounds const &bounds,
                                        Eval const &eval) const;

        //
        double calcPixelsPerMeterForDist(double dist_m,
                                         double screen_height_px,
                                         osg::Camera const * cam) const;

        // view data
        double const m_view_width;
        double const m_view_height;
        osg::Camera const * m_cam;

        osg::Vec3d m_eye;
        LLA m_lla_eye;

        std::vector<osg::Vec3d> m_list_frustum_ecef;
        std::vector<LLA>        m_list_frustum_lla;
        std::vector<GeoBounds>  m_list_frustum_bounds;
        std::vector<Plane>      m_list_frustum_tri_planes;

        //
        double const m_texture_px_size;
        double const m_texture_px_area;

        //
        size_t const m_eval_cache_size;

        //
        LRUCacheMap<
                TileLL::Id,
                std::unique_ptr<Eval>,
                std::map
                > m_lru_eval;


    };

} // scratch

#endif // SCRATCH_TILE_VISIBILITY_LL_PPM_H
