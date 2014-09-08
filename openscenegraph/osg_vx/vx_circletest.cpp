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

//
#include <osgnodes.hpp>

osg::Vec4 K_COLOR_GRAY = osg::Vec4(0.5,0.5,0.5,1.0);
osg::Vec4 K_COLOR_RED = osg::Vec4(1.0,0.1,0.1,1.0);

struct Circle
{
    osg::Vec3d center;
    osg::Vec3d normal;
    osg::Vec3d u;
    osg::Vec3d v;
    double radius;
};

// From WildMagic, (c) Geometric Tools LLC
// See Eberly, Distance between Point and Circle
osg::Vec3d ClosestPointCirclePoint(Circle const &c,
                                   osg::Vec3d const &p)
{
    // Signed distance from point to plane of circle.
    osg::Vec3d diff0 = p-c.center;
    double dist = diff0*c.normal;

    // Projection of P-C onto plane is Q-C = P-C - (fDist)*N.
    osg::Vec3d diff1 = diff0 - c.normal*dist;
    double sqrLen = diff1.length2();

    double sqrDistance;
    osg::Vec3d closestPoint1;

    if(sqrLen >= K_EPS) {
        closestPoint1 = c.center + diff1*(c.radius/sqrt(sqrLen));
        osg::Vec3d diff2 = p - closestPoint1;
        sqrDistance = diff2.length2();
    }
    else {
        closestPoint1 = p;
        sqrDistance = c.radius*c.radius + dist*dist;
    }

    return closestPoint1;
}

// From WildMagic, (c) Geometric Tools LLC
// See Eberly, Distance between Line and Circle
void CriticalPointsLineCircle(Circle const &c,
                              osg::Vec3d const &line_p, // line point
                              osg::Vec3d const &line_d, // line dirn
                              osg::Vec3d &min,
                              osg::Vec3d &max)
{
    osg::Vec3d diff = line_p - c.center;
    double diffSqrLen = diff.length2();
    double MdM = line_d.length2();
    double DdM = diff*line_d;
    double NdM = c.normal*line_d;
    double DdN = diff*c.normal;

    double a0 = DdM;
    double a1 = MdM;
    double b0 = DdM - NdM*DdN;
    double b1 = MdM - NdM*NdM;
    double c0 = diffSqrLen - DdN*DdN;
    double c1 = b0;
    double c2 = b1;
    double rsqr = c.radius*c.radius;

    double a0sqr = a0*a0;
    double a1sqr = a1*a1;
    double twoA0A1 = 2.0*a0*a1;
    double b0sqr = b0*b0;
    double b1Sqr = b1*b1;
    double twoB0B1 = 2.0*b0*b1;
    double twoC1 = 2.0*c1;

    // The minimum point B+t*M occurs when t is a root of the quartic
    // equation whose coefficients are defined below.

    // I think index specifies polynomial degree
    // ie. poly[3] = coefficient for x^3

    Polynomial1<Real> poly(4);
    poly[0] = a0sqr*c0 - b0sqr*rsqr;
    poly[1] = twoA0A1*c0 + a0sqr*twoC1 - twoB0B1*rsqr;
    poly[2] = a1sqr*c0 + twoA0A1*twoC1 + a0sqr*c2 - b1Sqr*rsqr;
    poly[3] = a1sqr*twoC1 + twoA0A1*c2;
    poly[4] = a1sqr*c2;

    PolynomialRoots<Real> polyroots(Math<Real>::ZERO_TOLERANCE);
    polyroots.FindB(poly, 6);
    int count = polyroots.GetCount();
    const Real* roots = polyroots.GetRoots();

    Real minSqrDist = Math<Real>::MAX_REAL;
    for (int i = 0; i < count; ++i)
    {
        // Compute distance from P(t) to circle.
        Vector3<Real> P = mLine->Origin + roots[i]*mLine->Direction;
        DistPoint3Circle3<Real> query(P, *mCircle);
        Real sqrDist = query.GetSquared();
        if (sqrDist < minSqrDist)
        {
            minSqrDist = sqrDist;
            mClosestPoint0 = query.GetClosestPoint0();
            mClosestPoint1 = query.GetClosestPoint1();
        }
    }

    return minSqrDist;
}


osg::Vec3d CalcVectorOnPlane(osg::Vec3d const &n)
{
    // First find a vector R that isn't N (and
    // with a big difference in direction)
    osg::Vec3d r_x = n^osg::Vec3d(1,0,0);
    osg::Vec3d r_y = n^osg::Vec3d(0,1,0);
    osg::Vec3d r_z = n^osg::Vec3d(0,0,1);
    osg::Vec3d r = r_x;
    if(r_y.length2() > r.length2()) {
        r_y = r;
    }
    if(r_z.length2() > r.length2()) {
        r_z = r;
    }

    osg::Vec3d const v = n^r;
    return v;
}

osg::ref_ptr<osg::Group> BuildCircle(Circle const &c,
                                     osg::Vec4 const &color)
{
    osg::Vec3d u = CalcVectorOnPlane(c.normal);
    u.normalize();

    osg::Vec3d v = c.normal^u;
    v.normalize();

    size_t const num_vx = 24;
    osg::ref_ptr<osg::Vec3dArray> vx_array = new osg::Vec3dArray;

    double const rotate_by_rads = (2.0*K_PI/num_vx);
    for(size_t i=0; i < num_vx; i++) {
        vx_array->push_back(
                c.center +
                (u * c.radius*cos(rotate_by_rads*i)) +
                (v * c.radius*sin(rotate_by_rads*i)));
    }

    osg::ref_ptr<osg::Vec4Array> cx_array = new osg::Vec4Array;
    cx_array->push_back(color);

    osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
    gm->setVertexArray(vx_array);
    gm->setColorArray(cx_array);
    gm->addPrimitiveSet(new osg::DrawArrays(GL_LINE_LOOP,0,vx_array->size()));

    osg::ref_ptr<osg::Geode> gd = new osg::Geode;
    gd->addDrawable(gm);

    osg::ref_ptr<osg::Group> gp = new osg::Group;
    gp->addChild(gd);

    return gp;
}

osg::ref_ptr<osg::Group> BuildLine(osg::Vec3d const &a,
                                   osg::Vec3d const &b,
                                   osg::Vec4 const &color)
{
    osg::ref_ptr<osg::Vec3dArray> vx_array = new osg::Vec3dArray;
    vx_array->push_back(a);
    vx_array->push_back(b);

    osg::ref_ptr<osg::Vec4Array>  cx_array = new osg::Vec4Array;
    cx_array->push_back(color);

    osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
    gm->setVertexArray(vx_array);
    gm->setColorArray(cx_array,osg::Array::BIND_OVERALL);
    gm->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0,vx_array->size()));

    osg::ref_ptr<osg::Geode> gd = new osg::Geode;
    gd->addDrawable(gm);

    osg::ref_ptr<osg::Group> gp = new osg::Group;
    gp->addChild(gd);

    return gp;
}


// =========================================================== //
// =========================================================== //

//
int main()
{
    Frustum frustum;

    osgViewer::CompositeViewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);

    // Circle
    Circle c;
    c.normal = osg::Vec3d(1,1,1);
    c.normal.normalize();

    c.center = osg::Vec3d(1,3,4);
    c.radius = 25;
    auto gp_circle = BuildCircle(c,osg::Vec4(0.5,0.5,0.5,1.0));

    // View0 root
    osg::ref_ptr<osg::Group> gp_root0 = new osg::Group;
    gp_root0->addChild(gp_circle);

    // View1 root
    osg::ref_ptr<osg::Group> gp_root1 = new osg::Group;
    gp_root1->addChild(gp_circle);

    // disable lighting and enable blending
    gp_root0->getOrCreateStateSet()->setMode( GL_LIGHTING,osg::StateAttribute::OFF);
    gp_root0->getOrCreateStateSet()->setMode( GL_BLEND,osg::StateAttribute::ON);
    gp_root0->getOrCreateStateSet()->setMode( GL_DEPTH_TEST,osg::StateAttribute::OFF);

    gp_root1->getOrCreateStateSet()->setMode( GL_LIGHTING,osg::StateAttribute::OFF);
    gp_root1->getOrCreateStateSet()->setMode( GL_BLEND,osg::StateAttribute::ON);
    gp_root1->getOrCreateStateSet()->setMode( GL_DEPTH_TEST,osg::StateAttribute::OFF);

    // Create View 0
    {
        osgViewer::View* view = new osgViewer::View;
        viewer.addView( view );

        view->setUpViewInWindow( 10, 10, 640, 480 );
        view->setSceneData( gp_root0.get() );
        view->getCamera()->setClearColor(osg::Vec4(0.1,0.1,0.1,1.0));

        osg::ref_ptr<osgGA::TrackballManipulator> view_manip =
                new osgGA::TrackballManipulator;
        view_manip->setMinimumDistance(100);

        view->setCameraManipulator(view_manip);

    }

    // Create view 1 (this view shows View0's frustum)
    {
        osgViewer::View* view = new osgViewer::View;
        viewer.addView( view );

        view->setUpViewInWindow( 10, 510, 640, 480 );
        view->setSceneData( gp_root1.get() );
        view->getCamera()->setClearColor(osg::Vec4(0.1,0.1,0.1,1.0));

        osg::ref_ptr<osgGA::TrackballManipulator> view_manip =
                new osgGA::TrackballManipulator;
        view_manip->setMinimumDistance(100);

        view->setCameraManipulator(view_manip);
    }

    while(!viewer.done())
    {
        osg::Camera * camera = viewer.getView(0)->getCamera();
        osg::Vec3d eye,vpt,up;
        double fovy,ar,near,far;
        camera->getViewMatrixAsLookAt(eye,vpt,up);
        camera->getProjectionMatrixAsPerspective(fovy,ar,near,far);
        osg::Vec3d viewdirn = vpt-eye;
        viewdirn.normalize();

        // new frustum
        auto new_frustum = BuildFrustumNode(camera,frustum);

        // new viewdirn
        auto new_viewdirn = BuildLine(eye,eye+(viewdirn*far),K_COLOR_GRAY);
        new_viewdirn->setName("viewdirn");

        // new closest point circle/point
        osg::Vec3d const cp_circle_pt = ClosestPointCirclePoint(c,eye);
        auto new_cp_circle_pt = BuildFacingCircle(cp_circle_pt,
                                                  eye.length()/150.0,
                                                  8,K_COLOR_RED);
        new_cp_circle_pt->setName("cp_circle_pt");

        // Update gp_root0
        {
            for(size_t i=0; i < gp_root0->getNumChildren(); i++) {
                std::string const name = gp_root0->getChild(i)->getName();
                // Remove prev camera node
                if(name == "cp_circle_pt") {
                    gp_root0->removeChild(i);
                    i--;
                }
            }
            // Add new nodes
            gp_root0->addChild(new_cp_circle_pt);
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
                else if(name == "viewdirn") {
                    gp_root1->removeChild(i);
                    i--;
                }
                else if(name == "cp_circle_pt") {
                    gp_root1->removeChild(i);
                    i--;
                }
            }
            // Add new nodes
            gp_root1->addChild(new_frustum);
            gp_root1->addChild(new_viewdirn);
            gp_root1->addChild(new_cp_circle_pt);
        }

        viewer.frame();

    }
    return 0;
}
