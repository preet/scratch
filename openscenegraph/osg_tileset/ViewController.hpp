#ifndef SCRATCH_VIEW_CONTROLLER_H
#define SCRATCH_VIEW_CONTROLLER_H

#include <iostream>
#include <cassert>

#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>

#include <GeometryUtils.h>

class ViewController : public osgGA::GUIEventHandler
{
public:
    ViewController(double view_width,
                   double view_height) :
        m_view_width(view_width),
        m_view_height(view_height),
        m_cam_min_surf_dist(500.0),
        m_cam_max_surf_dist(5*RAD_AV),
        m_cam_min_tilt_rads(10*K_DEG2RAD),
        m_cam_max_tilt_rads(90*K_DEG2RAD),
        m_cam_max_near_surf_dist(2000.0),
        m_cam_square_fov_rads(55.0*K_DEG2RAD)
    {

    }

    bool handle(osgGA::GUIEventAdapter const &ea,
                osgGA::GUIActionAdapter &aa)
    {
        osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
        if(!view) {
            std::cout << "ping" << std::endl;
            return false;
        }

        // update view data
        m_camera = view->getCamera();
        m_camera->getViewMatrixAsLookAt(m_eye,m_vpt,m_up);
        m_mvp = m_camera->getViewMatrix()*
                m_camera->getProjectionMatrix();

        switch(ea.getEventType())
        {
            case osgGA::GUIEventAdapter::PUSH: {
                std::cout << "#: PUSH: " << ea.getX() << "," << ea.getY() << std::endl;
                break;
            }
            case osgGA::GUIEventAdapter::DRAG: {
                std::cout << "#: DRAG: " << ea.getX() << "," << ea.getY() << std::endl;
                break;
            }
            case osgGA::GUIEventAdapter::RELEASE: {
                std::cout << "#: RELEASE: " << ea.getX() << "," << ea.getY() << std::endl;
                break;
            }
            case osgGA::GUIEventAdapter::KEYDOWN: {
                std::cout << "#: KEYDOWN: " << ea.getKey() << std::endl;
            }
            default: {
                break;
            }
        }

        return true; // true == handled
    }

    void accept(osgGA::GUIEventHandlerVisitor& v)
    {
        v.visit(*this);
    }

private:
    void convScreenToNDC(double x,
                         double y,
                         double &x_norm,
                         double &y_norm) const
    {
        x_norm = 2.0*x/m_view_width - 1.0;
        y_norm = 1.0 - 2.0*y*m_view_height;
    }

    osg::Vec3d convNDCToWorld(double x_norm,
                              double y_norm,
                              double depth=0.0) const
    {
        osg::Matrixd xfInvMVP = osg::Matrixd::inverse(m_mvp);
        return (osg::Vec3d(x_norm,y_norm,depth) * xfInvMVP);
    }

    osg::Vec3d convScreenToWorld(double x,
                                 double y,
                                 double depth=0.0) const
    {
        double x_norm,y_norm;
        convScreenToNDC(x,y,x_norm,y_norm);
        return convNDCToWorld(x_norm,y_norm,depth);
    }

    double calcAngleRads(osg::Vec3d const &a,
                         osg::Vec3d const &b) const
    {
        double cos_theta  = (a*b)/(a.length()*b.length());
        return acos(cos_theta);
    }

    void calcAndSetCamNearFarDist()
    {
        double dist_near,dist_far;
        bool ok = CalcCameraNearFarDist(m_eye,
                                        m_vpt-m_eye,
                                        m_cam_max_near_surf_dist,
                                        dist_near,
                                        dist_far);

        assert(ok);

        double fovy,ar,z_near,z_far;
        m_camera->getProjectionMatrixAsPerspective(
                    fovy,ar,z_near,z_far);

        m_camera->setProjectionMatrixAsPerspective(
                    fovy,ar,dist_near,dist_far);
    }

    void onMapViewPan(double x_prev,
                      double y_prev,
                      double x_next,
                      double y_next)
    {
        // prev
        osg::Vec3d const world_prev = convScreenToWorld(x_prev,y_prev);
        osg::Vec3d xsec_near_prev,xsec_far_prev;
        if(!CalcRayEarthIntersection(m_eye,
                                     world_prev-m_eye,
                                     xsec_near_prev,
                                     xsec_far_prev))
        {
            std::cout << "ERROR: Pan: prev input out of range" << std::endl;
            return;
        }

        // next
        osg::Vec3d const world_next = convScreenToWorld(x_next,y_next);
        osg::Vec3d xsec_near_next,xsec_far_next;
        if(!CalcRayEarthIntersection(m_eye,
                                     world_next-m_eye,
                                     xsec_near_next,
                                     xsec_far_next))
        {
            std::cout << "ERROR: Pan: next input out of range" << std::endl;
            return;
        }

        osg::Vec3d vec_prev = xsec_near_prev;
        vec_prev.normalize();

        osg::Vec3d vec_next = xsec_near_next;
        vec_next.normalize();

        osg::Vec3d pan_axis = vec_prev^vec_next;
        double angle_rads = calcAngleRads(vec_prev,vec_next);

        if(angle_rads == 0) {
            std::cout << "WARN: Pan: prev/next vectors are identical" << std::endl;
            return;
        }

        // rotate view matrix
        osg::Matrixd xf_rotate = osg::Matrixd::rotate(angle_rads,pan_axis);
        m_eye = m_eye * xf_rotate;
        m_vpt = m_vpt * xf_rotate;
        m_up  = m_up * xf_rotate;

        m_camera->setViewMatrixAsLookAt(m_eye,m_vpt,m_up);
        calcAndSetCamNearFarDist();
    }

    osg::Camera * m_camera;
    osg::Matrixd m_mvp;
    osg::Vec3d m_eye;
    osg::Vec3d m_vpt;
    osg::Vec3d m_up;

    double m_view_width;
    double m_view_height;
    double m_cam_min_surf_dist;
    double m_cam_max_surf_dist;
    double m_cam_min_tilt_rads;
    double m_cam_max_tilt_rads;
    double m_cam_max_near_surf_dist;
    double m_cam_square_fov_rads;
    double m_cam_tilt_rads;

    osg::Vec3d m_cam_pan_start;
};

#endif // SCRATCH_VIEW_CONTROLLER_H
