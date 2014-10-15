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

#ifndef SCRATCH_VIEW_CONTROLLER_H
#define SCRATCH_VIEW_CONTROLLER_H

#include <iostream>
#include <cassert>

#include <osg/io_utils>
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
        m_cam_square_fov_rads(55.0*K_DEG2RAD),
        m_cam_tilt_rads(90*K_DEG2RAD),
        m_view_mode(0) // 0==pan, 1==zoom, 2==tilt, 3==rotate
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
                m_prev_x = ea.getX();
                m_prev_y = m_view_height-ea.getY();
                break;
            }
            case osgGA::GUIEventAdapter::DRAG: {
                if(m_view_mode == 0) {
                    onMapViewPan(m_prev_x,m_prev_y,ea.getX(),m_view_height-ea.getY());
                }
                else if(m_view_mode == 1) {
                    onMapViewZoom(m_prev_x,m_prev_y,ea.getX(),m_view_height-ea.getY());
                }
                else if(m_view_mode == 2) {
                    onMapViewTilt(m_prev_x,m_prev_y,ea.getX(),m_view_height-ea.getY());
                }
                else if(m_view_mode == 3) {
                    onMapViewRotate(m_prev_x,m_prev_y,ea.getX(),m_view_height-ea.getY());
                }

                m_prev_x = ea.getX();
                m_prev_y = m_view_height-ea.getY();
                break;
            }
            case osgGA::GUIEventAdapter::RELEASE: {
                break;
            }
            case osgGA::GUIEventAdapter::KEYDOWN: {
                if(ea.getKey() == 'a') {
                    m_view_mode = 0;
                    std::cout << "#: MODE: PAN" << std::endl;
                }
                else if(ea.getKey() == 'z') {
                    m_view_mode = 1;
                    std::cout << "#: MODE: ZOOM" << std::endl;
                }
                else if(ea.getKey() == 't') {
                    m_view_mode = 2;
                    std::cout << "#: MODE: TILT" << std::endl;
                }
                else if(ea.getKey() == 'r') {
                    m_view_mode = 3;
                    std::cout << "#: MODE: ROTATE" << std::endl;
                }
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
        y_norm = 1.0 - 2.0*y/m_view_height;
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
        osg::Matrixd xf_rotate = osg::Matrixd::rotate(angle_rads*-1.0,pan_axis);
        m_eye = m_eye * xf_rotate;
        m_vpt = m_vpt * xf_rotate;
        m_up  = m_up * xf_rotate;

        m_camera->setViewMatrixAsLookAt(m_eye,m_vpt,m_up);
        calcAndSetCamNearFarDist();
    }

    void onMapViewRotate(double x_prev,
                         double y_prev,
                         double x_next,
                         double y_next)
    {
        double x_prev_norm,y_prev_norm;
        convScreenToNDC(x_prev,y_prev,x_prev_norm,y_prev_norm);

        double x_next_norm,y_next_norm;
        convScreenToNDC(x_next,y_next,x_next_norm,y_next_norm);

        double x_delta = x_next_norm-x_prev_norm;
        double angle_rads = x_delta/2.0 * K_PI*2.0 * 1.0;

        if(angle_rads == 0) {
            return;
        }

        osg::Vec3d xsec_near,xsec_far;
        if(!CalcRayEarthIntersection(m_eye,
                                     m_vpt-m_eye,
                                     xsec_near,
                                     xsec_far))
        {
            std::cout << "ERROR: Rotate: Camera out of allowed range" << std::endl;
            return;
        }

        osg::Matrixd xf_rotate = osg::Matrixd::rotate(angle_rads,xsec_near);
        m_eye = m_eye * xf_rotate;
        m_vpt = m_vpt * xf_rotate;
        m_up  = m_up * xf_rotate;

        m_camera->setViewMatrixAsLookAt(m_eye,m_vpt,m_up);
        calcAndSetCamNearFarDist();
    }

    void onMapViewTilt(double x_prev,
                       double y_prev,
                       double x_next,
                       double y_next)
    {
        double x_prev_norm,y_prev_norm;
        convScreenToNDC(x_prev,y_prev,x_prev_norm,y_prev_norm);

        double x_next_norm,y_next_norm;
        convScreenToNDC(x_next,y_next,x_next_norm,y_next_norm);

        double y_delta = y_next_norm-y_prev_norm;
        double angle_rads = y_delta/6.0 * K_PI*2.0 * -1.0;

        if(angle_rads == 0) {
            return;
        }

        double final_angle_rads = m_cam_tilt_rads + angle_rads;

        if(final_angle_rads > m_cam_max_tilt_rads) {
            final_angle_rads = m_cam_max_tilt_rads;
            angle_rads = m_cam_max_tilt_rads-m_cam_tilt_rads;
        }
        else if(final_angle_rads < m_cam_min_tilt_rads) {
            final_angle_rads = m_cam_min_tilt_rads;
            angle_rads = m_cam_min_tilt_rads-m_cam_tilt_rads;
        }

        osg::Vec3d xsec_near,xsec_far;
        if(!CalcRayEarthIntersection(m_eye,
                                     m_vpt-m_eye,
                                     xsec_near,
                                     xsec_far))
        {
            std::cout << "ERROR: Tilt: Camera out of allowed range" << std::endl;
            return;
        }

        osg::Vec3d const rot_axis = xsec_near^m_up;
        osg::Matrixd xf_rotate = osg::Matrixd::rotate(angle_rads,rot_axis);

        m_eye = m_eye - xsec_near;
        m_eye = m_eye * xf_rotate;
        m_eye = m_eye + xsec_near;

        m_up = m_up * xf_rotate;
        m_vpt = xsec_near;

        m_camera->setViewMatrixAsLookAt(m_eye,m_vpt,m_up);
        calcAndSetCamNearFarDist();

        m_cam_tilt_rads = final_angle_rads;
    }

    void onMapViewZoom(double x_prev,
                       double y_prev,
                       double x_next,
                       double y_next)
    {
        double x_prev_norm,y_prev_norm;
        convScreenToNDC(x_prev,y_prev,x_prev_norm,y_prev_norm);

        double x_next_norm,y_next_norm;
        convScreenToNDC(x_next,y_next,x_next_norm,y_next_norm);

        double zoom = (y_next_norm-y_prev_norm)*0.25;

        osg::Vec3d view_dirn = m_vpt-m_eye;
        view_dirn.normalize();

        double fovy,ar,z_near,z_far;
        m_camera->getProjectionMatrixAsPerspective(fovy,ar,z_near,z_far);
        double const zoom_length = zoom * z_far;

        osg::Vec3d const zoom_disp = (view_dirn*zoom_length);
        m_eye = m_eye + zoom_disp;
        m_vpt = m_vpt + zoom_disp;

        // Ensure the new camera position is within
        // min/max zoom limits
        double const new_eye_surf_dist =
                (m_eye.length())-RAD_AV;

        if(zoom > 0) {
            if(new_eye_surf_dist < m_cam_min_surf_dist) {
                return;
            }
        }
        else if(zoom < 0) {
            if(new_eye_surf_dist > m_cam_max_surf_dist) {
                return;
            }
        }

        // apply changes
        m_camera->setViewMatrixAsLookAt(m_eye,m_vpt,m_up);
        calcAndSetCamNearFarDist();
    }

    osg::Camera * m_camera;
    osg::Matrixd m_mvp;
    osg::Vec3d m_eye;
    osg::Vec3d m_vpt;
    osg::Vec3d m_up;

    double m_prev_x;
    double m_prev_y;

    double m_view_width;
    double m_view_height;
    double m_cam_min_surf_dist;
    double m_cam_max_surf_dist;
    double m_cam_min_tilt_rads;
    double m_cam_max_tilt_rads;
    double m_cam_max_near_surf_dist;
    double m_cam_square_fov_rads;
    double m_cam_tilt_rads;

    int m_view_mode;
};

#endif // SCRATCH_VIEW_CONTROLLER_H
