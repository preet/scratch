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

#ifndef SCRATCH_OSG_UTILS_H
#define SCRATCH_OSG_UTILS_H

#include <osg/io_utils>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/PrimitiveSet>
#include <osg/PolygonMode>
#include <osgViewer/Viewer>
#include <osg/Material>
#include <osg/MatrixTransform>
#include <osg/AutoTransform>
#include <osgViewer/CompositeViewer>
#include <osgGA/TrackballManipulator>
#include <osg/ShapeDrawable>

#include <GeometryUtils.h>


osg::ref_ptr<osg::Group> BuildEarthSurfaceNode(std::string const &name,
                                               osg::Vec4 const &color,
                                               bool auto_colorize=false);

osg::ref_ptr<osg::Group> BuildGeoBoundsSurfaceNode(std::string const &name,
                                                   GeoBounds const &b,
                                                   osg::Vec4 const &color,
                                                   int level_offset,
                                                   bool poly_mode_line=false,
                                                   uint32_t lon_segments=0,
                                                   uint32_t lat_segments=0);

osg::ref_ptr<osg::Group> BuildFrustumNode(std::string const &name,
                                          osg::Camera const * camera,
                                          Frustum & frustum,
                                          double near_dist=0.0,
                                          double far_dist=0.0);

osg::ref_ptr<osg::Group> BuildSurfacePolyNode(std::string const &name,
                                              std::vector<osg::Vec3d> const &list_ecef,
                                              osg::Vec4 const &cx,
                                              double vx_size=1.0,
                                              int level_offset=0);

osg::ref_ptr<osg::Group> BuildGeoBoundsNode(std::string const &name,
                                            GeoBounds const &b,
                                            osg::Vec4 const &color,
                                            double min_angle=360.0/16.0,
                                            int level_offset=0);

osg::ref_ptr<osg::Group> BuildAxesGeometry(std::string const &name,
                                           double length=3.0);

osg::ref_ptr<osg::AutoTransform> BuildFacingCircleNode(std::string const &name,
                                                       osg::Vec3d const &position,
                                                       double const scale,
                                                       size_t const num_vx,
                                                       osg::Vec4 const &color);

osg::ref_ptr<osg::Group> BuildRingNode(std::string const &name,
                                       osg::Vec3d const &center,
                                       osg::Vec4 const &color,
                                       double const radius,
                                       bool auto_xf=true);

osg::ref_ptr<osg::Group> BuildPlaneNode(std::string const &name,
                                        Plane const &plane,
                                        double plane_radius,
                                        osg::Vec4 const &color);

osg::ref_ptr<osg::Group> BuildTextNode(std::string const &name,
                                       std::string const &text_str,
                                       osg::Vec4 const &color,
                                       osg::Vec3d const &center,
                                       double const height_m);


#endif // SCRATCH_OSG_UTILS_H
