#ifndef TRI_H
#define TRI_H

#include <osg/Vec3d>

osg::Vec3d TriangleClosestPoint(osg::Vec3d const &a,
                                osg::Vec3d const &b,
                                osg::Vec3d const &c,
                                osg::Vec3d const &p)
{
    /** The code for Triangle-float3 test is from Christer Ericson's Real-Time Collision Detection, pp. 141-142. */

    // Check if P is in vertex region outside A.
    osg::Vec3d ab = b - a;
    osg::Vec3d ac = c - a;
    osg::Vec3d ap = p - a;
    float d1 = (ab*ap);
    float d2 = (ac*ap);
    if (d1 <= 0.f && d2 <= 0.f)
    return a; // Barycentric coordinates are (1,0,0).

    // Check if P is in vertex region outside B.
    osg::Vec3d bp = p - b;
    float d3 = (ab*bp);
    float d4 = (ac*bp);
    if (d3 >= 0.f && d4 <= d3)
    return b; // Barycentric coordinates are (0,1,0).

    // Check if P is in edge region of AB, and if so, return the projection of P onto AB.
    float vc = d1*d4 - d3*d2;
    if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f)
    {
    float v = d1 / (d1 - d3);
    return a + (ab*v); // The barycentric coordinates are (1-v, v, 0).
    }

    // Check if P is in vertex region outside C.
    osg::Vec3d cp = p - c;
    float d5 = (ab*cp);
    float d6 = (ac*cp);
    if (d6 >= 0.f && d5 <= d6)
    return c; // The barycentric coordinates are (0,0,1).

    // Check if P is in edge region of AC, and if so, return the projection of P onto AC.
    float vb = d5*d2 - d1*d6;
    if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f)
    {
    float w = d2 / (d2 - d6);
    return a + (ac*w); // The barycentric coordinates are (1-w, 0, w).
    }

    // Check if P is in edge region of BC, and if so, return the projection of P onto BC.
    float va = d3*d6 - d5*d4;
    if (va <= 0.f && d4 - d3 >= 0.f && d5 - d6 >= 0.f)
    {
    float w = (d4 - d3) / (d4 - d3 + d5 - d6);
    return b + ((c - b)*w); // The barycentric coordinates are (0, 1-w, w).
    }

    // P must be inside the face region. Compute the closest point through its barycentric coordinates (u,v,w).
    float denom = 1.f / (va + vb + vc);
    float v = vb * denom;
    float w = vc * denom;
    return a + (ab * v) + (ac * w);
}

bool TriangleIntersectsSphere(osg::Vec3d const &a,
                        osg::Vec3d const &b,
                        osg::Vec3d const &c,
                        osg::Vec3d const &sphere_center,
                        double sphere_radius)
{
    osg::Vec3d pt = TriangleClosestPoint(a,b,c,sphere_center);

    if((pt-sphere_center).length2() <= (sphere_radius*sphere_radius)) {
//        std::cout << "###: ping" << std::endl;
        return true;
    }
    return false;
}

#endif // TRI_H
