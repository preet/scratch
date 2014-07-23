#ifndef OSG_NODES_H
#define OSG_NODES_H

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

#include <gmutil.hpp>
#include <tileutil.hpp>

osg::ref_ptr<osg::Group> BuildCelestialSurfaceNode()
{
    std::vector<osg::Vec3d> list_vx;
    std::vector<osg::Vec2d> list_tx;
    std::vector<size_t> list_ix;

    BuildEarthSurfaceGeometry(-180,-90,
                              180,90,
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
    cx_array->push_back(osg::Vec4(0.2,0.2,0.2,0.1));

    osg::ref_ptr<osg::DrawElementsUShort> ix_array =
            new osg::DrawElementsUShort(GL_TRIANGLES);
    for(auto const ix : list_ix) {
        ix_array->push_back(ix);
    }

    // geometry
    osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
    gm->setVertexArray(vx_array.get());
    gm->setColorArray(cx_array,osg::Array::Binding::BIND_OVERALL);
    gm->addPrimitiveSet(ix_array.get());

    // geode
    osg::ref_ptr<osg::Geode> gd = new osg::Geode;
    gd->addDrawable(gm.get());

    // group
    osg::ref_ptr<osg::Group> gp = new osg::Group;
    gp->addChild(gd.get());
    gp->setName("celestialbody");

    return gp;
}

osg::ref_ptr<osg::Group> BuildFrustumNode(osg::Camera * camera,
                                          Frustum & frustum,
                                          double far_dist=0.0)
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
    const double near = proj(3,2) / (proj(2,2)-1.0);
    double far = proj(3,2) / (1.0+proj(2,2));
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
    osg::Vec3d p_left = NBL+ (FTL-NBL)*0.5;
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
    c->push_back(osg::Vec4(0.5,0.5,0.5,0.5));
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
    gp->setName("frustum");
    gp->addChild(geode);

    return gp;
}

osg::ref_ptr<osg::Group> BuildMinCamDistLineNode(osg::Vec3d const &eye)
{
    osg::ref_ptr<osg::Group> gp = new osg::Group;
    gp->setName("mincamdistline");

    osg::Vec3d xsecNear,xsecFar;
    if(CalcRayEarthIntersection(eye,eye*-1.0,xsecNear,xsecFar))
    {
        osg::ref_ptr<osg::Geode> gd = new osg::Geode;

        osg::ref_ptr<osg::Vec3dArray> list_vx = new osg::Vec3dArray(2);
        list_vx->at(0) = eye;
        list_vx->at(1) = xsecNear;

        osg::ref_ptr<osg::Vec4Array> list_cx = new osg::Vec4Array(1);
        list_cx->at(0) = osg::Vec4(1,0,0,1);

        osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
        gm->setVertexArray(list_vx);
        gm->setColorArray(list_cx,osg::Array::BIND_OVERALL);
        gm->addPrimitiveSet(new osg::DrawArrays(
            osg::PrimitiveSet::LINES,0,list_vx->size()));

        gd->addDrawable(gm);
        gp->addChild(gd);
    }

    return gp;
}

osg::ref_ptr<osg::AutoTransform> BuildLodRingsNode(osg::Vec3d const &eye)
{
    osg::ref_ptr<osg::Geode> gd_rings = new osg::Geode;

    // For each ring
    for(size_t i=0; i < K_MAX_LOD; i++) {
        // Get the distance for this LOD
        double dist = K_LIST_LOD_DIST[i];
        osg::Vec4 color = K_COLOR_TABLE[i];

        // Create a ring of vertices
        osg::ref_ptr<osg::Vec3dArray> list_vx = new osg::Vec3dArray((K_MAX_LOD-(i+1))*2 + 12);
        double const rotate_by_rads = (2.0*K_PI/list_vx->size());

        for(size_t j=0; j < list_vx->size(); j++){
            list_vx->at(j) = osg::Vec3d(
                        dist*cos(rotate_by_rads*j),
                        dist*sin(rotate_by_rads*j),
                        0);
        }

        osg::ref_ptr<osg::Vec4Array> list_cx = new osg::Vec4Array;
        list_cx->push_back(color);

        osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
        gm->setVertexArray(list_vx);
        gm->setColorArray(list_cx,osg::Array::BIND_OVERALL);
        gm->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP,0,list_vx->size()));

        gd_rings->addDrawable(gm.get());
    }

    osg::ref_ptr<osg::AutoTransform> xf_rings = new osg::AutoTransform;
    xf_rings->addChild(gd_rings);
    xf_rings->setAutoRotateMode(osg::AutoTransform::ROTATE_TO_SCREEN);
    xf_rings->setPosition(eye);
    xf_rings->setName("lodrings");

    return xf_rings;
}

osg::ref_ptr<osg::Group> BuildBaseViewExtentsNode(std::vector<VxTile*> const &list_base_vx_tiles)
{
    osg::ref_ptr<osg::Group> gp = new osg::Group;

    osg::ref_ptr<osg::Geode> gd = new osg::Geode;

    for(auto & tile : list_base_vx_tiles)
    {
        osg::ref_ptr<osg::Vec3dArray> list_vx = new osg::Vec3dArray(4);
        list_vx->at(0) = *(tile->p_ecef_LT);
        list_vx->at(1) = *(tile->p_ecef_LB);
        list_vx->at(2) = *(tile->p_ecef_RB);
        list_vx->at(3) = *(tile->p_ecef_RT);

        osg::ref_ptr<osg::Vec4Array> list_cx =
                new osg::Vec4Array(1);
        list_cx->at(0) = K_COLOR_TABLE[tile->level];

        osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
        gm->setVertexArray(list_vx);
        gm->setColorArray(list_cx,osg::Array::BIND_OVERALL);
        gm->addPrimitiveSet(new osg::DrawArrays(
            osg::PrimitiveSet::LINE_LOOP,0,list_vx->size()));

        gd->addDrawable(gm);
    }

    gp->addChild(gd);

    return gp;
}

void BuildViewExtentsGeometry(VxTile const * t, osg::Group * gp)
{
    osg::ref_ptr<osg::Geode> gd = new osg::Geode;

    osg::ref_ptr<osg::Vec3dArray> list_vx = new osg::Vec3dArray(4);
    list_vx->at(0) = *(t->p_ecef_LT);
    list_vx->at(1) = *(t->p_ecef_LB);
    list_vx->at(2) = *(t->p_ecef_RB);
    list_vx->at(3) = *(t->p_ecef_RT);

    osg::ref_ptr<osg::Vec3Array> list_nx = new osg::Vec3Array(4);
    list_nx->at(0) = list_vx->at(0)/(list_vx->at(0).length());
    list_nx->at(1) = list_vx->at(1)/(list_vx->at(1).length());
    list_nx->at(2) = list_vx->at(2)/(list_vx->at(2).length());
    list_nx->at(3) = list_vx->at(3)/(list_vx->at(3).length());

    osg::ref_ptr<osg::Vec4Array> list_cx = new osg::Vec4Array(1);
//    list_cx->at(0) = K_COLOR_TABLE[t->level];


//    if(t->_fvis && t->_hvis) {
//        list_cx->at(0) = osg::Vec4(0,1,1,1);
//    }
//    else if(t->_fvis) {
//        list_cx->at(0) = osg::Vec4(0,1,0,1);
//    }
//    else if(t->_hvis) {
//        list_cx->at(0) = osg::Vec4(0,0,1,1);
//    }

    if(t->_fvis && t->_hvis) {
        list_cx->at(0) = osg::Vec4(0,1,0,1);
    }
    else {
        list_cx->at(0) = osg::Vec4(1,0,0,1);
    }

    osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
    gm->setVertexArray(list_vx);
    gm->setNormalArray(list_nx,osg::Array::BIND_PER_VERTEX);
    gm->setColorArray(list_cx,osg::Array::BIND_OVERALL);
    gm->addPrimitiveSet(new osg::DrawArrays(
        osg::PrimitiveSet::QUADS,0,list_vx->size()));

    gd->addDrawable(gm);
    gp->addChild(gd);

    if(t->tile_LT) {
        BuildViewExtentsGeometry(t->tile_LT.get(),gp);
    }
    if(t->tile_LB) {
        BuildViewExtentsGeometry(t->tile_LB.get(),gp);
    }
    if(t->tile_RB) {
        BuildViewExtentsGeometry(t->tile_RB.get(),gp);
    }
    if(t->tile_RT) {
        BuildViewExtentsGeometry(t->tile_RT.get(),gp);
    }
}

osg::ref_ptr<osg::Group> BuildViewExtentsNode(std::vector<VxTile*> const &list_base_vx_tiles)
{
    osg::ref_ptr<osg::Group> gp = new osg::Group;

    for(auto vx_tile : list_base_vx_tiles)
    {
        BuildViewExtentsGeometry(vx_tile,gp.get());
    }

    gp->getOrCreateStateSet()->setMode( GL_DEPTH_TEST,
                                        osg::StateAttribute::ON |
                                        osg::StateAttribute::OVERRIDE);

    gp->getOrCreateStateSet()->setMode( GL_LIGHTING,
                                        osg::StateAttribute::ON |
                                        osg::StateAttribute::OVERRIDE);

    gp->getOrCreateStateSet()->setRenderBinDetails(-1,"RenderBin");

    gp->setName("vxtiles");
    return gp;
}

void BuildViewExtentsOBBGeometry(VxTile * tile, osg::Group * gp)
{
    osg::ref_ptr<osg::Geode> gd = new osg::Geode;

    osg::ref_ptr<osg::Vec3dArray> list_vx = new osg::Vec3dArray(8);

    list_vx->at(0) =
            tile->obb.center +
            osg::Vec3d(tile->obb.ori[0]*-tile->obb.ext.x() +
                       tile->obb.ori[1]*tile->obb.ext.y() +
                       tile->obb.ori[2]*tile->obb.ext.z());

    list_vx->at(1) =
            tile->obb.center +
            osg::Vec3d(tile->obb.ori[0]*-tile->obb.ext.x()+
                       tile->obb.ori[1]*-tile->obb.ext.y()+
                       tile->obb.ori[2]*tile->obb.ext.z());

    list_vx->at(2) =
            tile->obb.center +
            osg::Vec3d(tile->obb.ori[0]*tile->obb.ext.x()+
                       tile->obb.ori[1]*-tile->obb.ext.y()+
                       tile->obb.ori[2]*tile->obb.ext.z());

    list_vx->at(3) =
            tile->obb.center +
            osg::Vec3d(tile->obb.ori[0]*tile->obb.ext.x()+
                       tile->obb.ori[1]*tile->obb.ext.y()+
                       tile->obb.ori[2]*tile->obb.ext.z());

    list_vx->at(4) =
            tile->obb.center +
            osg::Vec3d(tile->obb.ori[0]*-tile->obb.ext.x() +
                       tile->obb.ori[1]*tile->obb.ext.y() +
                       tile->obb.ori[2]*-tile->obb.ext.z());

    list_vx->at(5) =
            tile->obb.center +
            osg::Vec3d(tile->obb.ori[0]*-tile->obb.ext.x()+
                       tile->obb.ori[1]*-tile->obb.ext.y()+
                       tile->obb.ori[2]*-tile->obb.ext.z());

    list_vx->at(6) =
            tile->obb.center +
            osg::Vec3d(tile->obb.ori[0]*tile->obb.ext.x()+
                       tile->obb.ori[1]*-tile->obb.ext.y()+
                       tile->obb.ori[2]*-tile->obb.ext.z());

    list_vx->at(7) =
            tile->obb.center +
            osg::Vec3d(tile->obb.ori[0]*tile->obb.ext.x()+
                       tile->obb.ori[1]*tile->obb.ext.y()+
                       tile->obb.ori[2]*-tile->obb.ext.z());


    list_vx->push_back(tile->obb.center);

    list_vx->push_back(tile->obb.faces[0].p);


    osg::ref_ptr<osg::Vec4Array> list_cx = new osg::Vec4Array;
    for(size_t i=0; i < 8; i++) {
        list_cx->push_back(osg::Vec4(1,0.5,0,0.5));
    }

    list_cx->push_back(osg::Vec4(1,0,1,1));
    list_cx->push_back(osg::Vec4(1,1,1,1));


    uint16_t list_ix_top_face[4] = { 0, 1, 2, 3 };
    uint16_t list_ix_btm_face[4] = { 4, 5, 6, 7 };
    uint16_t list_ix_edges[10] = { 0, 4, 1, 5, 2, 6, 3, 7 };
    uint16_t list_ix_else[2] = { 8, 9 };

    osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
    gm->setVertexArray(list_vx);
    gm->setColorArray(list_cx,osg::Array::BIND_PER_VERTEX);
    gm->addPrimitiveSet(new osg::DrawElementsUShort(osg::PrimitiveSet::LINE_LOOP,4,list_ix_top_face));
    gm->addPrimitiveSet(new osg::DrawElementsUShort(osg::PrimitiveSet::LINE_LOOP,4,list_ix_btm_face));
    gm->addPrimitiveSet(new osg::DrawElementsUShort(osg::PrimitiveSet::LINES,8,list_ix_edges));
    gm->addPrimitiveSet(new osg::DrawElementsUShort(osg::PrimitiveSet::LINES,2,list_ix_else));

    gd->addDrawable(gm);
    gp->addChild(gd);
}

osg::ref_ptr<osg::Group> BuildViewExtentsOBBNode(std::vector<VxTile*> const &list_base_vx_tiles)
{
    osg::ref_ptr<osg::Group> gp = new osg::Group;

    for(auto vx_tile : list_base_vx_tiles)
    {
        BuildViewExtentsOBBGeometry(vx_tile,gp.get());
    }

    gp->setName("vxtilesobb");
    return gp;
}

osg::ref_ptr<osg::Group> BuildHorizonPlaneNode(osg::Camera * camera,
                                               Plane & horizon_plane)
{
    osg::ref_ptr<osg::Group> gp_horizon_plane = new osg::Group;

    gp_horizon_plane->setName("horizonplane");

    osg::Vec3d eye,vpt,up;
    if(camera) {
        camera->getViewMatrixAsLookAt(eye,vpt,up);

        if(CalcHorizonPlane(eye,horizon_plane)) {
            // Draw the plane as a circle centered on horizon_pt
            // with radius RAD_AV*0.5
            osg::ref_ptr<osg::Vec3dArray> list_vx = new osg::Vec3dArray(16);
            double const rotate_by_rads = (2.0*K_PI/list_vx->size());
            double const dist = RAD_AV*1.1;

            // Rotate the circle so that its normal is
            // aligned to the horizon_norm
            osg::Matrixd rotate_to_horizon =
                    osg::Matrixd::rotate(osg::Vec3d(0,0,1),
                                         horizon_plane.n);

            for(size_t i=0; i < 16; i++) {
                list_vx->at(i) = osg::Vec3d(
                            dist*cos(rotate_by_rads*i),
                            dist*sin(rotate_by_rads*i),
                            0);

                list_vx->at(i) = list_vx->at(i) * rotate_to_horizon;
                list_vx->at(i) = list_vx->at(i) + horizon_plane.p;
            }

            osg::ref_ptr<osg::Vec4Array> list_cx = new osg::Vec4Array;
            list_cx->push_back(osg::Vec4(0.4,0.5,0.9,1.0)); // smurple

            osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
            gm->setVertexArray(list_vx);
            gm->setColorArray(list_cx,osg::Array::BIND_OVERALL);
            gm->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP,0,list_vx->size()));

            osg::ref_ptr<osg::Geode> gd = new osg::Geode;
            gd->addDrawable(gm);

            gp_horizon_plane->addChild(gd);
        }
    }

    return gp_horizon_plane;
}

osg::ref_ptr<osg::Group> BuildFrustumOBBProjNode(Frustum const &frustum,
                                                 OBB const &obb)
{
    osg::ref_ptr<osg::Group> gp = new osg::Group;

    // Create a ring node
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
        list_cx->push_back(osg::Vec4(1,1,0,1));

        gm_ring->setVertexArray(list_vx);
        gm_ring->setColorArray(list_cx,osg::Array::BIND_OVERALL);
        gm_ring->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP,0,list_vx->size()));
    }

    // Create the line
    double const obb_ext_length = obb.ext.length();

    // Project Frustum vertices onto line
    for(size_t i=0; i < frustum.list_vx.size(); i++)
    {
        double t = (frustum.list_vx[i]-obb.faces[0].p) * obb.faces[0].n;

        osg::ref_ptr<osg::AutoTransform> xf_ring = new osg::AutoTransform;
        osg::ref_ptr<osg::Geode> gd_ring = new osg::Geode;
        gd_ring->addDrawable(gm_ring);
        xf_ring->addChild(gd_ring);
        xf_ring->setPosition(obb.faces[0].p + (obb.faces[0].n * t));
        xf_ring->setAutoRotateMode(osg::AutoTransform::ROTATE_TO_SCREEN);

        if(t > 0) {
            xf_ring->setScale(obb_ext_length*0.2);
        }
        else {
            xf_ring->setScale(obb_ext_length*0.1);
        }

        gp->addChild(xf_ring);
    }

    gp->setName("frustumobbproj");
    return gp;
}

osg::ref_ptr<osg::Group> BuildSurfacePoly(std::vector<osg::Vec3d> const &list_ecef)
{
    osg::ref_ptr<osg::Group> gp = new osg::Group;

    // Create a ring node
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
        list_cx->push_back(osg::Vec4(1,0,0,1));

        gm_ring->setVertexArray(list_vx);
        gm_ring->setColorArray(list_cx,osg::Array::BIND_OVERALL);
        gm_ring->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP,0,list_vx->size()));
    }

    for(auto const & ecef : list_ecef) {
        osg::ref_ptr<osg::AutoTransform> xf_ring = new osg::AutoTransform;
        osg::ref_ptr<osg::Geode> gd_ring = new osg::Geode;
        gd_ring->addDrawable(gm_ring);
        xf_ring->addChild(gd_ring);
        xf_ring->setScale(RAD_AV*0.05);
        xf_ring->setPosition(ecef);
        xf_ring->setAutoRotateMode(osg::AutoTransform::ROTATE_TO_SCREEN);
        gp->addChild(xf_ring);
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
        list_cx->push_back(osg::Vec4(1,0,0,1));

        gm_edges->setVertexArray(list_vx);
        gm_edges->setColorArray(list_cx,osg::Array::BIND_OVERALL);
        gm_edges->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP,0,list_vx->size()));

        osg::ref_ptr<osg::Geode> gd = new osg::Geode;
        gd->addDrawable(gm_edges);
        gp->addChild(gd);
    }
    gp->setName("surfacepoly");
    return gp;
}


#endif // OSG_NODES_H
