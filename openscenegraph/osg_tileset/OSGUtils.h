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

osg::ref_ptr<osg::Group> BuildFrustumNode(std::string const &name,
                                          osg::Camera * camera,
                                          Frustum & frustum,
                                          double near_dist=0.0,
                                          double far_dist=0.0);
#endif // SCRATCH_OSG_UTILS_H
