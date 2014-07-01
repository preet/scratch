// sys includes
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <cassert>
#include <memory>
#include <numeric>
#include <chrono>

// osg includes
#include <osg/Vec3>
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

#define K_PI 3.141592653589
#define K_DEG2RAD K_PI/180.0
#define K_RAD2DEG 180.0/K_PI

double const K_MAX_POS_DBL = std::numeric_limits<double>::max();
double const K_MIN_NEG_DBL = -K_MAX_POS_DBL;

std::vector<osg::Vec4> const K_COLOR_TABLE {
    {0., 0., 0., 1.},
    {41/255., 41/255., 41/255., 1.},
    {102/255., 102/255., 102/255., 1.},
    {140/255., 140/255., 140/255., 1.},
    {200/255., 200/255., 200/255., 1.},
    {66/255., 206/255., 252/255., 1.},
    {124/255., 160/255., 252/255., 1.},
    {173/255., 146/255., 252/255., 1.},
    {255/255., 120/255., 252/255., 1.},
    {255/255., 117/255., 172/255., 1.},
    {255/255., 142/255., 107/255., 1.},
    {252/255., 174/255., 91/255., 1.},
    {252/255., 194/255., 0/255., 1.},
    {202/255., 245/255., 29/255., 1.},
    {0/255., 191/255., 0/255., 1.},
    {100/255., 245/255., 174/255., 1.},
    {0/255., 235/255., 231/255., 1.},
    {255/255., 255/255., 255/255., 1.}
};

// frustum geometry
std::vector<osg::Vec3d> g_list_frustum_plane_norms(6);
std::vector<osg::Vec3d> g_list_frustum_plane_pts(6);
std::vector<osg::Vec3d> g_list_frustum_edges(8);
std::vector<osg::Vec3d> g_list_frustum_vx(8);

void GetProjectionInterval(osg::Vec3d const &axis,
                           std::vector<osg::Vec3d> const &list_vx,
                           double &min,
                           double &max)
{
    // Project each vertex in @list_vx along @axis
    // and return the min and max of the projection
    // interval

    min = K_MAX_POS_DBL;
    max = K_MIN_NEG_DBL;

    for(auto const &vx : list_vx) {
        double proj = vx*axis;
        min = std::min(min,proj);
        max = std::max(max,proj);
    }
}


bool CalcPolysIntersectSAT(std::vector<osg::Vec3d> const &polyA_list_vx,
                           std::vector<osg::Vec3d> const &polyA_list_face_norms,
                           std::vector<osg::Vec3d> const &polyA_list_edges,
                           std::vector<osg::Vec3d> const &polyB_list_vx,
                           std::vector<osg::Vec3d> const &polyB_list_face_norms,
                           std::vector<osg::Vec3d> const &polyB_list_edges)
{
    std::vector<std::pair<double,double>> list_polyA_intervals;
    std::vector<std::pair<double,double>> list_polyB_intervals;

    // Test the faces of polyA
    for(auto const &polyA_face_norm : polyA_list_face_norms)
    {
        double polyA_min,polyA_max;
        GetProjectionInterval(polyA_face_norm,
                              polyA_list_vx,
                              polyA_min,
                              polyA_max);

        double polyB_min,polyB_max;
        GetProjectionInterval(polyA_face_norm,
                              polyB_list_vx,
                              polyB_min,
                              polyB_max);

        list_polyA_intervals.push_back(std::pair<double,double>(polyA_min,polyA_max));
        list_polyB_intervals.push_back(std::pair<double,double>(polyB_min,polyB_max));

        if((polyA_min > polyB_max) || (polyB_min > polyA_max)) {
            // there's an axis with no overlap so the polys dont intersect
            return false;
        }
    }

    // Test the faces of polyA
    for(auto const &polyB_face_norm : polyB_list_face_norms)
    {
        double polyA_min,polyA_max;
        GetProjectionInterval(polyB_face_norm,
                              polyA_list_vx,
                              polyA_min,
                              polyA_max);

        double polyB_min,polyB_max;
        GetProjectionInterval(polyB_face_norm,
                              polyB_list_vx,
                              polyB_min,
                              polyB_max);

        if((polyA_min > polyB_max) || (polyB_min > polyA_max)) {
            return false;
        }
    }

    // Test the cross product of the edges of polyA and polyB
    for(auto const &polyA_edge : polyA_list_edges) {
        for(auto const &polyB_edge : polyB_list_edges)
        {
            osg::Vec3d const axis = polyA_edge^polyB_edge;

            // Edge may be invalid -- need to handle
            // this case better
            if(axis.length2() < 1E-5) {
                continue;
            }

            double polyA_min,polyA_max;
            GetProjectionInterval(axis,
                                  polyA_list_vx,
                                  polyA_min,
                                  polyA_max);

            double polyB_min,polyB_max;
            GetProjectionInterval(axis,
                                  polyB_list_vx,
                                  polyB_min,
                                  polyB_max);

            if((polyA_min > polyB_max) || (polyB_min > polyA_max)) {
                return false;
            }
        }
    }

//    for(auto const &interval : list_polyA_intervals) {
//        std::cout << "###: min: " << interval.first
//                  << "max: " << interval.second << std::endl;
//    }

    // Intersection
    return true;
}



osg::ref_ptr<osg::Group> BuildTriangleNode(osg::Vec3d const &a,
                                           osg::Vec3d const &b,
                                           osg::Vec3d const &c,
                                           std::vector<osg::Vec3d> &list_face_norms,
                                           std::vector<osg::Vec3d> &list_edges)
{
    // build the geometry
    osg::ref_ptr<osg::Vec3dArray> list_vx = new osg::Vec3dArray;
    list_vx->push_back(a);
    list_vx->push_back(b);
    list_vx->push_back(c);

    osg::ref_ptr<osg::Vec4Array> list_cx = new osg::Vec4Array;
    list_cx->push_back(osg::Vec4(1,0,0,1));

    osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
    gm->setVertexArray(list_vx);
    gm->setColorArray(list_cx,osg::Array::BIND_OVERALL);
    gm->addPrimitiveSet(new osg::DrawArrays(GL_LINE_LOOP,0,list_vx->size()));

    osg::ref_ptr<osg::Geode> gd = new osg::Geode;
    gd->addDrawable(gm);

    osg::ref_ptr<osg::Group> gp = new osg::Group;
    gp->addChild(gd);

    // get edge and face vectors
    list_face_norms.push_back(b-a); // edge ab
    list_face_norms.push_back(c-b); // edge bc
    list_face_norms.push_back(a-c); // edge ca
    list_edges.push_back(a^b); // face

    return gp;
}

void UpdateTriangleNode(osg::ref_ptr<osg::Group> const &gp,
                        osg::Vec4 const &color)
{
    // Change the color of the triangle if
    // its currently intersecting the view frustum

    osg::Geode * gd = static_cast<osg::Geode*>(gp->getChild(0));
    osg::Geometry * gm = static_cast<osg::Geometry*>(gd->getDrawable(0));

    osg::ref_ptr<osg::Vec4Array> list_cx = new osg::Vec4Array;
    list_cx->push_back(color);

    gm->setColorArray(list_cx,osg::Array::BIND_OVERALL);
}

osg::ref_ptr<osg::Group> BuildFrustumNode(osg::Camera * camera)
{
    // Projection and ModelView matrices

    osg::Matrixd proj;
    osg::Matrixd mv;
    osg::Matrixd vm;

    if (camera)
    {
        proj = camera->getProjectionMatrix();
        mv = camera->getViewMatrix();
        vm = camera->getViewMatrix();
    }
    else
    {
        // Create some kind of reasonable default Projection matrix.
        proj.makePerspective( 30., 1., 1., 10. );
        // leave mv as identity
    }

    osg::Matrixd const mv_inv = osg::Matrixd::inverse( mv );

    // Get near and far from the Projection matrix.
    const double near = proj(3,2) / (proj(2,2)-1.0);
    const double far = proj(3,2) / (1.0+proj(2,2));

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
        g_list_frustum_plane_pts[0] = p_left;
        g_list_frustum_plane_pts[1] = p_right;
        g_list_frustum_plane_pts[2] = p_top;
        g_list_frustum_plane_pts[3] = p_btm;
        g_list_frustum_plane_pts[4] = p_near;
        g_list_frustum_plane_pts[5] = p_far;

        g_list_frustum_plane_norms[0] = d_left;
        g_list_frustum_plane_norms[1] = d_right;
        g_list_frustum_plane_norms[2] = d_top;
        g_list_frustum_plane_norms[3] = d_btm;
        g_list_frustum_plane_norms[4] = d_near;
        g_list_frustum_plane_norms[5] = d_far;

        // TODO/Note: The magnitude of these edges
        // should be similar to the magnitude of the
        // edges of any geometry used in the SAT!

        // near edges
        g_list_frustum_edges[0] = NTL-NBL; // left
        g_list_frustum_edges[1] = NBL-NBR;
        g_list_frustum_edges[2] = NBR-NTR;
        g_list_frustum_edges[3] = NTR-NTL;

        // side edges
        g_list_frustum_edges[4] = FTL-NTL; // tl
        g_list_frustum_edges[5] = FBL-NBL;
        g_list_frustum_edges[6] = FBR-NBR;
        g_list_frustum_edges[7] = FTR-NTR;

        // far edges
        // (assume symmetric frustum so these
        //  arent needed)
//        g_list_frustum_edges[8] = FTL-FBL; // left
//        g_list_frustum_edges[9] = FBL-FBR;
//        g_list_frustum_edges[10] = FBR-FTR;
//        g_list_frustum_edges[11] = FTR-FTL;

        // frustum vx
        g_list_frustum_vx[0] = NBL;
        g_list_frustum_vx[1] = NBR;
        g_list_frustum_vx[2] = NTR;
        g_list_frustum_vx[3] = NTL;
        g_list_frustum_vx[4] = FBL;
        g_list_frustum_vx[5] = FBR;
        g_list_frustum_vx[6] = FTR;
        g_list_frustum_vx[7] = FTL;
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





int main(int argc, const char *argv[])
{
    (void)argc;
    (void)argv;

    // triangle model
    std::vector<osg::Vec3d> list_tri_vx(3);
    list_tri_vx[0] = osg::Vec3d(1,0,0);
    list_tri_vx[1] = osg::Vec3d(0,1,0);
    list_tri_vx[2] = osg::Vec3d(0,0,1);

    std::vector<osg::Vec3d> list_tri_edges;
    std::vector<osg::Vec3d> list_tri_face_norms;
    auto gp_triangle = BuildTriangleNode(list_tri_vx[0],
                                         list_tri_vx[1],
                                         list_tri_vx[2],
                                         list_tri_face_norms,
                                         list_tri_edges);

    // View0 root
    osg::ref_ptr<osg::Group> gp_root0 = new osg::Group;
    gp_root0->addChild(gp_triangle);

    // View1 root
    osg::ref_ptr<osg::Group> gp_root1 = new osg::Group;
    gp_root1->addChild(gp_triangle);

    // disable lighting and enable blending
    gp_root0->getOrCreateStateSet()->setMode( GL_LIGHTING,osg::StateAttribute::OFF);
    gp_root0->getOrCreateStateSet()->setMode( GL_BLEND,osg::StateAttribute::ON);
    gp_root0->getOrCreateStateSet()->setMode( GL_DEPTH_TEST,osg::StateAttribute::OFF);

    gp_root1->getOrCreateStateSet()->setMode( GL_LIGHTING,osg::StateAttribute::OFF);
    gp_root1->getOrCreateStateSet()->setMode( GL_BLEND,osg::StateAttribute::ON);
    gp_root1->getOrCreateStateSet()->setMode( GL_DEPTH_TEST,osg::StateAttribute::OFF);


    osgViewer::CompositeViewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);

    // Create View 0
    {
        osgViewer::View* view = new osgViewer::View;
        viewer.addView( view );

        view->setUpViewInWindow( 10, 10, 640, 480 );
        view->setSceneData( gp_root0.get() );
        view->getCamera()->setClearColor(osg::Vec4(0.1,0.1,0.1,1.0));
        view->setCameraManipulator( new osgGA::TrackballManipulator );
    }

    // Create view 1 (this view shows View0's frustum)
    {
        osgViewer::View* view = new osgViewer::View;
        viewer.addView( view );

//        view->addEventHandler(new KeyboardEventHandler());
        view->setUpViewInWindow( 10, 510, 640, 480 );
        view->setSceneData( gp_root1.get() );
        view->getCamera()->setClearColor(osg::Vec4(0.1,0.1,0.1,1.0));
        view->setCameraManipulator( new osgGA::TrackballManipulator );
    }

    // Create the fixed base VxTiles we use to
    // calculate view extents

    while (!viewer.done())
    {
        osg::Camera * camera = viewer.getView(0)->getCamera();

        // Create a new camera frustum node
        auto new_frustum = BuildFrustumNode(camera);

        // Update geometry color based on SAT result
        if(CalcPolysIntersectSAT(list_tri_vx,
                                 list_tri_face_norms,
                                 list_tri_edges,
                                 g_list_frustum_vx,
                                 g_list_frustum_plane_norms,
                                 g_list_frustum_edges))
        {
            UpdateTriangleNode(gp_triangle,osg::Vec4(0,1,0,1));
        }
        else {
            UpdateTriangleNode(gp_triangle,osg::Vec4(1,0,0,1));
        }


        // Update gp_root0
        {
//            for(size_t i=0; i < gp_root0->getNumChildren(); i++) {
//                std::string const name = gp_root0->getChild(i)->getName();
//            }
        }

        // Update gp_root1
        {
            for(size_t i=0; i < gp_root1->getNumChildren(); i++) {
                std::string const name = gp_root1->getChild(i)->getName();
                // Remove prev camera node
                if(name == "frustum") {
                    gp_root1->removeChild(i);
                    i--;
                }
            }
            // Add new nodes
            gp_root1->addChild(new_frustum);
        }
        viewer.frame();
    }
    return 0;
}
