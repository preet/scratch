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

#include <OSGUtils.h>
#include <osgText/Text>

// ============================================================= //

osg::ref_ptr<osg::Group> BuildLines(std::string const &name,
                                    osg::Vec4 const &color,
                                    std::vector<osg::Vec3d> list_vx)
{
    osg::ref_ptr<osg::Vec3dArray> vx_array =
            new osg::Vec3dArray;

    for(auto const &vx : list_vx) {
        vx_array->push_back(vx);
    }

    osg::ref_ptr<osg::Vec4Array> cx_array = new osg::Vec4Array;
    cx_array->push_back(color);

    osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
    gm->setVertexArray(vx_array);
    gm->setColorArray(cx_array,osg::Array::BIND_OVERALL);
    gm->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0,vx_array->size()));

    osg::ref_ptr<osg::Geode> gd = new osg::Geode;
    gd->addDrawable(gm);

    osg::ref_ptr<osg::Group> gp = new osg::Group;
    gp->addChild(gd);
    gp->setName(name);
    return gp;
}

// ============================================================= //

osg::ref_ptr<osg::Group> BuildEarthSurfaceNode(std::string const &name,
                                               osg::Vec4 const &color,
                                               bool auto_colorize)
{
    std::vector<osg::Vec3d> list_vx;
    std::vector<osg::Vec2d> list_tx;
    std::vector<uint16_t> list_ix;

    BuildEarthSurface(-180,180,
                      -90,90,
                      32,16,
                      list_vx,
                      list_tx,
                      list_ix);

    osg::ref_ptr<osg::Vec3dArray> vx_array =
            new osg::Vec3dArray;
    for(auto const &vx : list_vx) {
        vx_array->push_back(vx);
    }

    osg::ref_ptr<osg::Vec4Array> cx_array =
            new osg::Vec4Array;
    if(auto_colorize) {
        for(auto const &vx : list_vx) {
            osg::Vec4 cx;
            double r = vx.x()/RAD_AV;
            double g = vx.y()/RAD_AV;
            double b = vx.z()/RAD_AV;
            cx.r() = r*r*r; if(cx.r() < 0) { cx.r() = 0; }
            cx.g() = g*g*g; if(cx.g() < 0) { cx.g() = 0; }
            cx.b() = b*b*b; if(cx.b() < 0) { cx.b() = 0; }
            cx.a() = 1.0;
            cx_array->push_back(cx);
        }
    }
    else {
        cx_array->push_back(color);
    }

    osg::ref_ptr<osg::DrawElementsUShort> ix_array =
            new osg::DrawElementsUShort(GL_TRIANGLES);
    for(auto const ix : list_ix) {
        ix_array->push_back(ix);
    }

    // geometry
    osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
    gm->setVertexArray(vx_array.get());
    if(auto_colorize) {
        gm->setColorArray(cx_array,osg::Array::BIND_PER_VERTEX);
    }
    else {
        gm->setColorArray(cx_array,osg::Array::Binding::BIND_OVERALL);
    }
    gm->addPrimitiveSet(ix_array.get());

    // geode
    osg::ref_ptr<osg::Geode> gd = new osg::Geode;
    gd->addDrawable(gm.get());
    gd->getOrCreateStateSet()->setRenderBinDetails(-102,"RenderBin");
    gd->getOrCreateStateSet()->setMode(
                GL_CULL_FACE,
                osg::StateAttribute::ON |
                osg::StateAttribute::OVERRIDE);

    // group
    osg::ref_ptr<osg::Group> gp = new osg::Group;
    gp->addChild(gd.get());
    gp->setName(name);

    return gp;
}

// ============================================================= //

osg::ref_ptr<osg::Group> BuildGeoBoundsSurfaceNode(std::string const &name,
                                                   GeoBounds const &b,
                                                   osg::Vec4 const &color,
                                                   int level_offset,
                                                   bool poly_mode_line,
                                                   uint32_t lon_segments,
                                                   uint32_t lat_segments)
{
    //    uint32_t surf_divs = 256/K_LIST_TWO_EXP[tile->level];
    //    surf_divs = std::min(surf_divs,static_cast<uint32_t>(32));

    //    uint32_t const lon_segments =
    //            std::max(static_cast<uint32_t>(surf_divs),
    //                     static_cast<uint32_t>(1));

    //    uint32_t const lat_segments =
    //            std::max(static_cast<uint32_t>(lon_segments/2),
    //                     static_cast<uint32_t>(1));

    double const min_angle_degs = 360.0/64.0;
    if(lon_segments == 0) {
        lon_segments = std::max((b.maxLon-b.minLon)/min_angle_degs,1.0);
    }
    if(lat_segments == 0) {
        lat_segments = std::max((b.maxLat-b.minLat)/min_angle_degs,1.0);
    }

    std::vector<osg::Vec3d> list_vx;
    std::vector<osg::Vec2d> list_tx;
    std::vector<uint16_t> list_ix;
    BuildEarthSurface(b.minLon,
                      b.maxLon,
                      b.minLat,
                      b.maxLat,
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
    cx_array->push_back(color);

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
                -101+level_offset,"RenderBin");
//    gd->getOrCreateStateSet()->setMode(
//                GL_CULL_FACE,
//                osg::StateAttribute::ON |
//                osg::StateAttribute::OVERRIDE);

    // texture
    //    gd->getOrCreateStateSet()->setTextureAttributeAndModes(
    //                0,m_list_tile_level_tex[tile->level]);

    // polygon mode
    if(poly_mode_line) {
        osg::PolygonMode * poly_mode = new osg::PolygonMode;
        poly_mode->setMode(osg::PolygonMode::FRONT_AND_BACK,
                           osg::PolygonMode::LINE);
        gd->getOrCreateStateSet()->setAttributeAndModes(
                    poly_mode,
                    osg::StateAttribute::ON);
    }

    osg::ref_ptr<osg::Group> gp = new osg::Group;
    gp->addChild(gd);
    gp->setName(name);

    return gp;
}

// ============================================================= //

osg::ref_ptr<osg::Group> BuildFrustumNode(std::string const &name,
                                          osg::Camera const * camera,
                                          Frustum & frustum,
                                          double near_dist,
                                          double far_dist)
{
    // Projection and ModelView matrices

    osg::Matrixd proj;
    osg::Matrixd mv;
    osg::Matrixd vm;
    osg::Vec3d eye(0,0,0);

    if (camera)
    {
        proj = camera->getProjectionMatrix();
        mv = camera->getViewMatrix();
        vm = camera->getViewMatrix();

        osg::Vec3d vpt,up;
        camera->getViewMatrixAsLookAt(eye,vpt,up);
    }
    else
    {
        // return empty group if camera is invalid
        osg::ref_ptr<osg::Group> gp = new osg::Group;
        return gp;
    }


    osg::Matrixd const mv_inv = osg::Matrixd::inverse( mv );

    // Get near and far from the Projection matrix.
    double near = proj(3,2) / (proj(2,2)-1.0);
    double far = proj(3,2) / (1.0+proj(2,2));

    if(near_dist > 0.0) {
        near = near_dist;
    }
    if(far_dist > 0.0) {
        far = far_dist;
    }

    // Get the sides of the near plane.
    const double nLeft = near * (proj(2,0)-1.0) / proj(0,0);
    const double nRight = near * (1.0+proj(2,0)) / proj(0,0);
    const double nTop = near * (1.0+proj(2,1)) / proj(1,1);
    const double nBottom = near * (proj(2,1)-1.0) / proj(1,1);

    // Get the sides of the far plane.
    const double fLeft = far * (proj(2,0)-1.0) / proj(0,0);
    const double fRight = far * (1.0+proj(2,0)) / proj(0,0);
    const double fTop = far * (1.0+proj(2,1)) / proj(1,1);
    const double fBottom = far * (proj(2,1)-1.0) / proj(1,1);

    // Our vertex array needs only 9 vertices: The origin, and the
    // eight corners of the near and far planes.
    osg::ref_ptr<osg::Vec3dArray> v = new osg::Vec3dArray;
    v->resize( 21 );

    // near and far are negated because the opengl
    // camera is at (0,0,0) with the view dirn pointing
    // down the -Z axis

    osg::Vec3d NBL(nLeft,nBottom,-near);
    NBL = NBL * mv_inv;

    osg::Vec3d NBR(nRight,nBottom,-near);
    NBR = NBR *mv_inv;

    osg::Vec3d NTR(nRight,nTop,-near);
    NTR = NTR * mv_inv;

    osg::Vec3d NTL(nLeft,nTop,-near);
    NTL = NTL * mv_inv;

    osg::Vec3d FBL(fLeft, fBottom, -far);
    FBL = FBL * mv_inv;

    osg::Vec3d FBR(fRight, fBottom, -far);
    FBR = FBR * mv_inv;

    osg::Vec3d FTR(fRight, fTop, -far);
    FTR = FTR * mv_inv;

    osg::Vec3d FTL(fLeft, fTop, -far);
    FTL = FTL* mv_inv;

    // get the normals for the frustum planes
    osg::Vec3d p_left = (NBL+FTL)*0.5;
    osg::Vec3d d_left = (FTL-NTL)^(NBL-NTL); d_left.normalize();

    osg::Vec3d p_right = (NBR+FTR)*0.5;
    osg::Vec3d d_right = (NTR-FTR)^(FBR-FTR); d_right.normalize();

    osg::Vec3d p_top = (NTL+FTR)*0.5;
    osg::Vec3d d_top = (FTR-NTR)^(NTL-NTR); d_top.normalize();

    osg::Vec3d p_btm = (NBL+FBR)*0.5;
    osg::Vec3d d_btm = (FBL-NBL)^(NBR-NBL); d_btm.normalize();

    osg::Vec3d p_near = (NBL+NTR)*0.5;
    osg::Vec3d d_near = (NTL-NTR)^(NBR-NTR); d_near.normalize();

    osg::Vec3d p_far = (FBL+FTR)*0.5;
    osg::Vec3d d_far = (FTR-FBL)^(FBL-FTL); d_far.normalize();

    // save
    {
        frustum.list_planes[0].n = d_left;
        frustum.list_planes[0].p = p_left;
        frustum.list_planes[0].d = d_left*p_left;

        frustum.list_planes[1].n = d_btm;
        frustum.list_planes[1].p = p_btm;
        frustum.list_planes[1].d = d_btm*p_btm;

        frustum.list_planes[2].n = d_right;
        frustum.list_planes[2].p = p_right;
        frustum.list_planes[2].d = d_right*p_right;

        frustum.list_planes[3].n = d_top;
        frustum.list_planes[3].p = p_top;
        frustum.list_planes[3].d = d_top*p_top;

        frustum.list_planes[4].n = d_near;
        frustum.list_planes[4].p = p_near;
        frustum.list_planes[4].d = d_near*p_near;

        frustum.list_planes[5].n = d_far;
        frustum.list_planes[5].p = p_far;
        frustum.list_planes[5].d = d_far*p_far;

        // TODO/Note: The magnitude of these edges
        // should be similar to the magnitude of the
        // edges of any geometry used in the SAT!

        // near edges
        frustum.list_edges[0].dirn_ab = NTL-NBL; // left
        frustum.list_edges[0].a = NBL;

        frustum.list_edges[1].dirn_ab = NBL-NBR; // btm
        frustum.list_edges[1].a = NBR;

        frustum.list_edges[2].dirn_ab = NBR-NTR; // right
        frustum.list_edges[2].a = NTR;

        frustum.list_edges[3].dirn_ab = NTR-NTL; // top
        frustum.list_edges[3].a = NTL;

        // side edges
        frustum.list_edges[4].dirn_ab = FTL-NTL; // tl
        frustum.list_edges[4].a = NTL; // tl

        frustum.list_edges[5].dirn_ab = FBL-NBL;
        frustum.list_edges[5].a = NBL;

        frustum.list_edges[6].dirn_ab = FBR-NBR;
        frustum.list_edges[6].a = NBR;

        frustum.list_edges[7].dirn_ab = FTR-NTR;
        frustum.list_edges[7].a = NTR;

        // far edges
        frustum.list_edges[8].dirn_ab = FTL-FBL; // left
        frustum.list_edges[8].a = FBL;

        frustum.list_edges[9].dirn_ab = FBL-FBR; // btm
        frustum.list_edges[9].a = FBR;

        frustum.list_edges[10].dirn_ab = FBR-FTR; // right
        frustum.list_edges[10].a = FTR;

        frustum.list_edges[11].dirn_ab = FTR-FTL; // top
        frustum.list_edges[11].a = FTL;

        // frustum vx
        frustum.list_vx[0] = NBL;
        frustum.list_vx[1] = NBR;
        frustum.list_vx[2] = NTR;
        frustum.list_vx[3] = NTL;
        frustum.list_vx[4] = FBL;
        frustum.list_vx[5] = FBR;
        frustum.list_vx[6] = FTR;
        frustum.list_vx[7] = FTL;

        // pyramid vx
        frustum.list_pyr_vx[0] = &(frustum.eye);
        frustum.list_pyr_vx[1] = &(frustum.list_vx[4]);
        frustum.list_pyr_vx[2] = &(frustum.list_vx[5]);
        frustum.list_pyr_vx[3] = &(frustum.list_vx[6]);
        frustum.list_pyr_vx[4] = &(frustum.list_vx[7]);

        // eye
        frustum.eye = eye;
    }

    // get a length to show the normals
    double const normal_length = (FTR-FBR).length()*0.5;
    d_left *= normal_length;
    d_right *= normal_length;
    d_top *= normal_length;
    d_btm *= normal_length;
    d_near *= normal_length;
    d_far *= normal_length;

    v->at(0).set(0.,0.,0.);
    v->at(0) = v->at(0) * mv_inv;

    v->at(1) = NBL;
    v->at(2) = NBR;
    v->at(3) = NTR;
    v->at(4) = NTL;

    v->at(5) = FBL;
    v->at(6) = FBR;
    v->at(7) = FTR;
    v->at(8) = FTL;

    v->at(9) = p_left;
    v->at(10) = p_left+d_left;

    v->at(11) = p_right;
    v->at(12) = p_right+d_right;

    v->at(13) = p_top;
    v->at(14) = p_top+d_top;

    v->at(15) = p_btm;
    v->at(16) = p_btm+d_btm;

    v->at(17) = p_near;
    v->at(18) = p_near+d_near;

    v->at(19) = p_far;
    v->at(20) = p_far+d_far;


    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    geom->setUseDisplayList( false );
    geom->setVertexArray( v );

    osg::ref_ptr<osg::Vec4Array> c = new osg::Vec4Array;
//    c->push_back(osg::Vec4(0.5,0.5,0.5,0.5));
    c->push_back(osg::Vec4(1,1,1,1));
    geom->setColorArray( c, osg::Array::BIND_OVERALL );

    GLushort idxLines[8] = {
        0, 5, 0, 6, 0, 7, 0, 8 };
    GLushort idxLoops0[4] = {
        1, 2, 3, 4 };
    GLushort idxLoops1[4] = {
        5, 6, 7, 8 };
//    GLushort idxNormals[12] = {
//        9,10,11,12,13,14,15,16,17,18,19,20 };
    geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINES, 8, idxLines ) );
    geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINE_LOOP, 4, idxLoops0 ) );
    geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINE_LOOP, 4, idxLoops1 ) );
//    geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINES, 12, idxNormals ) );

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable( geom );

    // Create parent MatrixTransform to transform the view volume by
    // the inverse ModelView matrix.
    osg::ref_ptr<osg::Group> gp = new osg::Group;
    gp->setName(name);
    gp->addChild(geode);

    return gp;
}


// ============================================================= //

osg::ref_ptr<osg::Group> BuildSurfacePolyNode(std::string const &name,
                                              std::vector<osg::Vec3d> const &list_ecef,
                                              osg::Vec4 const &cx,
                                              double vx_size,
                                              int level_offset)
{
    osg::ref_ptr<osg::Group> gp = new osg::Group;
    if(list_ecef.empty()) {
        return gp;
    }

    // Create a ring node
    if(vx_size > 0) {
        osg::ref_ptr<osg::Geometry> gm_ring = new osg::Geometry;
        {
            // Create a ring of vertices
            osg::ref_ptr<osg::Vec3dArray> list_vx = new osg::Vec3dArray(8);
            double const rotate_by_rads = (2.0*K_PI/list_vx->size());

            for(size_t j=0; j < list_vx->size(); j++){
                list_vx->at(j) = osg::Vec3d(
                            cos(rotate_by_rads*j),
                            sin(rotate_by_rads*j),
                            0);
            }

            osg::ref_ptr<osg::Vec4Array> list_cx = new osg::Vec4Array;
            list_cx->push_back(cx);

            gm_ring->setVertexArray(list_vx);
            gm_ring->setColorArray(list_cx,osg::Array::BIND_OVERALL);
            gm_ring->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP,0,list_vx->size()));
        }

        for(auto const & ecef : list_ecef) {
            osg::ref_ptr<osg::AutoTransform> xf_ring = new osg::AutoTransform;
            osg::ref_ptr<osg::Geode> gd_ring = new osg::Geode;
            gd_ring->getOrCreateStateSet()->setRenderBinDetails(
                        -101+level_offset,"RenderBin");

            gd_ring->addDrawable(gm_ring);
            xf_ring->addChild(gd_ring);
            xf_ring->setScale(vx_size);
            xf_ring->setPosition(ecef);
            xf_ring->setAutoRotateMode(osg::AutoTransform::ROTATE_TO_SCREEN);
            gp->addChild(xf_ring);
        }
    }

    // Create poly edges
    osg::ref_ptr<osg::Geometry> gm_edges = new osg::Geometry;
    {
        osg::ref_ptr<osg::Vec3dArray> list_vx = new osg::Vec3dArray;
        list_vx->reserve(list_ecef.size());

        for(auto const & ecef : list_ecef) {
            list_vx->push_back(ecef);
        }

        osg::ref_ptr<osg::Vec4Array> list_cx = new osg::Vec4Array;
        list_cx->push_back(cx);

        gm_edges->setVertexArray(list_vx);
        gm_edges->setColorArray(list_cx,osg::Array::BIND_OVERALL);
        gm_edges->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP,0,list_vx->size()));

        osg::ref_ptr<osg::Geode> gd = new osg::Geode;
        gd->getOrCreateStateSet()->setRenderBinDetails(
                    -101+level_offset,"RenderBin");
        gd->addDrawable(gm_edges);
        gp->addChild(gd);
    }
    gp->setName(name);
    return gp;
}

// ============================================================= //

osg::ref_ptr<osg::Group> BuildGeoBoundsNode(std::string const &name,
                                            GeoBounds const &b,
                                            osg::Vec4 const &color,
                                            double min_angle,
                                            int level_offset)
{
    static const double k_eps = 1E-8;

    double const lon_delta = b.maxLon-b.minLon;
    double const lat_delta = b.maxLat-b.minLat;
    if((lon_delta < k_eps) ||
       (lat_delta < k_eps)) {
        osg::ref_ptr<osg::Group> gp = new osg::Group;
        gp->setName(name);
        return gp;
    }

    double const lon_div = (b.maxLon - b.minLon)/min_angle;
    double const lat_div = (b.maxLat - b.minLat)/min_angle;

    std::vector<osg::Vec3d> list_ecef;

    // left edge
    for(double lat=b.maxLat; lat >= b.minLat; lat-=lat_div) {
        LLA lla; lla.lon = b.minLon; lla.lat = lat; lla.alt = 0;
        list_ecef.push_back(ConvLLAToECEF(lla));
    }

    // bottom edge
    for(double lon=b.minLon; lon <= b.maxLon; lon+=lon_div) {
        LLA lla; lla.lon = lon; lla.lat = b.minLat; lla.alt = 0;
        list_ecef.push_back(ConvLLAToECEF(lla));
    }

    // right edge
    for(double lat=b.minLat; lat <= b.maxLat; lat+=lat_div) {
        LLA lla; lla.lon = b.maxLon; lla.lat = lat; lla.alt = 0;
        list_ecef.push_back(ConvLLAToECEF(lla));
    }

    // top edge
    for(double lon=b.maxLon; lon >= b.minLon; lon-=lon_div) {
        LLA lla; lla.lon = lon; lla.lat = b.maxLat; lla.alt = 0;
        list_ecef.push_back(ConvLLAToECEF(lla));
    }

    osg::ref_ptr<osg::Group> gp = BuildSurfacePolyNode(name,list_ecef,color,0.0,level_offset);

    return gp;
}

// ============================================================= //

osg::ref_ptr<osg::Group> BuildAxesGeometry(std::string const &name,
                                           double length)
{
    osg::ref_ptr<osg::Vec3dArray> list_vx = new osg::Vec3dArray;
    list_vx->push_back(osg::Vec3d(0,0,0));
    list_vx->push_back(osg::Vec3d(length,0,0));

    list_vx->push_back(osg::Vec3d(0,0,0));
    list_vx->push_back(osg::Vec3d(0,length,0));

    list_vx->push_back(osg::Vec3d(0,0,0));
    list_vx->push_back(osg::Vec3d(0,0,length));

    osg::ref_ptr<osg::Vec4Array> list_cx = new osg::Vec4Array;
    list_cx->push_back(osg::Vec4(1,0,0,1));
    list_cx->push_back(osg::Vec4(1,0,0,1));

    list_cx->push_back(osg::Vec4(0,1,0,1));
    list_cx->push_back(osg::Vec4(0,1,0,1));

    list_cx->push_back(osg::Vec4(0,0,1,1));
    list_cx->push_back(osg::Vec4(0,0,1,1));

    osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
    gm->setVertexArray(list_vx);
    gm->setColorArray(list_cx,osg::Array::BIND_PER_VERTEX);
    gm->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0,list_vx->size()));

    osg::ref_ptr<osg::Geode> gd = new osg::Geode;
    gd->addDrawable(gm);

    osg::ref_ptr<osg::Group> gp = new osg::Group;
    gp->addChild(gd);
    gp->setName(name);
    return gp;
}

// ============================================================= //

osg::ref_ptr<osg::AutoTransform> BuildFacingCircleNode(std::string const &name,
                                                       osg::Vec3d const &position,
                                                       double const scale,
                                                       size_t const num_vx,
                                                       osg::Vec4 const &color)
{
    // Create a ring node
    osg::ref_ptr<osg::Geometry> gm_ring = new osg::Geometry;
    {
        // Create a ring of vertices

        osg::ref_ptr<osg::Vec3dArray> list_vx = new osg::Vec3dArray;
        double const rotate_by_rads = (2.0*K_PI/num_vx);

        for(size_t j=0; j < num_vx; j++){
            list_vx->push_back(osg::Vec3d(0,0,0));
            list_vx->push_back(osg::Vec3d(cos(rotate_by_rads*j),
                                          sin(rotate_by_rads*j),
                                          0));
            list_vx->push_back(osg::Vec3d(cos(rotate_by_rads*(j+1)),
                                          sin(rotate_by_rads*(j+1)),
                                          0));
        }

        osg::ref_ptr<osg::Vec4Array> list_cx = new osg::Vec4Array;
        list_cx->push_back(color);

        gm_ring->setVertexArray(list_vx);
        gm_ring->setColorArray(list_cx,osg::Array::BIND_OVERALL);
        gm_ring->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES,0,list_vx->size()));
    }

    osg::ref_ptr<osg::AutoTransform> xf_ring = new osg::AutoTransform;
    osg::ref_ptr<osg::Geode> gd_ring = new osg::Geode;
    gd_ring->addDrawable(gm_ring);
    xf_ring->addChild(gd_ring);
    xf_ring->setScale(scale);
    xf_ring->setPosition(position);
    xf_ring->setAutoRotateMode(osg::AutoTransform::ROTATE_TO_SCREEN);
    xf_ring->setName(name);

    return xf_ring;
}


osg::ref_ptr<osg::Group> BuildRingNode(std::string const &name,
                                       osg::Vec3d const &center,
                                       osg::Vec4 const &color,
                                       double const radius,
                                       bool auto_xf)
{
    osg::ref_ptr<osg::Group> gp = new osg::Group;
    gp->setName(name);

    // Create a ring node
    osg::ref_ptr<osg::Geometry> gm_ring = new osg::Geometry;
    {
        // Create a ring of vertices
        osg::ref_ptr<osg::Vec3dArray> list_vx = new osg::Vec3dArray(16);
        double const rotate_by_rads = (2.0*K_PI/list_vx->size());

        for(size_t j=0; j < list_vx->size(); j++){
            list_vx->at(j) = osg::Vec3d(
                        cos(rotate_by_rads*j),
                        sin(rotate_by_rads*j),
                        0);
        }

        osg::ref_ptr<osg::Vec4Array> list_cx = new osg::Vec4Array;
        list_cx->push_back(color);

        gm_ring->setVertexArray(list_vx);
        gm_ring->setColorArray(list_cx,osg::Array::BIND_OVERALL);
        gm_ring->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP,0,list_vx->size()));
    }

    osg::ref_ptr<osg::Geode> gd_ring = new osg::Geode;
    gd_ring->addDrawable(gm_ring);

    if(auto_xf) {
        osg::ref_ptr<osg::AutoTransform> xf_ring = new osg::AutoTransform;
        xf_ring->addChild(gd_ring);
        xf_ring->setScale(radius);
        xf_ring->setPosition(center);
        xf_ring->setAutoRotateMode(osg::AutoTransform::ROTATE_TO_SCREEN);
        gp->addChild(xf_ring);
    }
    else {
        gp->addChild(gd_ring);
    }

    return gp;
}

osg::ref_ptr<osg::Group> BuildPlaneNode(std::string const &name,
                                        Plane const &plane,
                                        double plane_radius,
                                        osg::Vec4 const &color)
{
    auto gp_ring = BuildRingNode("ring",
                                 plane.p,
                                 color,
                                 plane_radius,
                                 false);

    osg::Vec3d plane_n = plane.n;
    plane_n.normalize();

    osg::ref_ptr<osg::MatrixTransform> xf =
            new osg::MatrixTransform;
    xf->setMatrix(osg::Matrixd::rotate(osg::Vec3d(0,0,1),plane_n) *
                  osg::Matrixd::translate(plane.p));
    xf->addChild(gp_ring->getChild(0));

    osg::ref_ptr<osg::Group> gp_plane = new osg::Group;
    gp_plane->setName(name);
    gp_plane->addChild(xf);

    return gp_plane;
}

osg::ref_ptr<osg::Group> BuildTextNode(std::string const &name,
                                       std::string const &text_str,
                                       osg::Vec4 const &color,
                                       osg::Vec3d const &center,
                                       double const height_m)
{
    osg::ref_ptr<osg::Group> gp = new osg::Group;
    gp->setName(name);

    if(text_str.empty()) {
        return gp;
    }

    osg::ref_ptr<osgText::Text> text = new osgText::Text;
    text->setText(text_str);
    text->setCharacterSize(height_m);
    text->setAutoRotateToScreen(true);
    text->setPosition(center);
    text->setColor(color);

    osg::ref_ptr<osg::Geode> gd = new osg::Geode;
    gd->addDrawable(text);

    gp->addChild(gd);
    return gp;
}
