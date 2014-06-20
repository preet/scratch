#ifndef OBB_H
#define OBB_H

#include <osg/Vec3d>

class OBB
{
public:
    osg::Vec3d pos;
    osg::Vec3d r;
    osg::Vec3d axis[3];

    OBB(osg::Vec3d const &pos,
        osg::Vec3d const &r,
        osg::Vec3d const &axis0,
        osg::Vec3d const &axis1,
        osg::Vec3d const &axis2) :
        pos(pos),
        r(r)
    {
        axis[0] = axis0;
        axis[1] = axis1;
        axis[2] = axis2;

        axis[0].normalize();
        axis[1].normalize();
        axis[2].normalize();
    }

    osg::Vec3d ClosestPoint(osg::Vec3d targetPoint)
    {
        osg::Vec3d d = targetPoint - pos;
        osg::Vec3d closestPoint = pos; // Start at the center point of the OBB.

        // Project the target onto the OBB axes and walk towards that point.
        for(int i = 0; i < 3; ++i) {

            double dist = d * axis[i]; // dot product

            if(dist > r[i]) { dist = r[i]; }
            if(dist < (r[i]*-1.0)) { dist = r[i]*-1.0; }

            closestPoint += (axis[i] * dist);
        }

        return closestPoint;
    }

    bool IntersectsSphere(osg::Vec3d const &center, double radius)
    {
        // Find the point on this AABB closest to the sphere center.
        osg::Vec3d pt = ClosestPoint(center);

        // If that point is inside sphere, the AABB and sphere intersect.
        if((pt-center).length2() <= (radius*radius)) {
            return true;
        }
        return false;
    }

};

#endif // OBB_H
