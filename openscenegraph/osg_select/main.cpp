// sys includes
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sys/time.h>

#define K_EPS 1E-11

    // cartesian coordinate system 3d vector class

    class Vec3
    {
    public:
        Vec3() :
            x(0),y(0),z(0) {}

        Vec3(double myX, double myY, double myZ) :
            x(myX),y(myY),z(myZ) {}

        inline double Dot(Vec3 const & otherVec) const
        {
            return (x*otherVec.x)+
                   (y*otherVec.y)+
                   (z*otherVec.z);
        }

        inline Vec3 Cross(Vec3 const & otherVec) const
        {
            return Vec3((y*otherVec.z - z*otherVec.y),
                        (z*otherVec.x - x*otherVec.z),
                        (x*otherVec.y - y*otherVec.x));
        }

        inline double Magnitude() const
        {
            return sqrt(x*x + y*y + z*z);
        }

        inline double DistanceTo(Vec3 const &otherVec) const
        {
            return sqrt((x-otherVec.x)*(x-otherVec.x) +
                        (y-otherVec.y)*(y-otherVec.y) +
                        (z-otherVec.z)*(z-otherVec.z));
        }

        inline double Distance2To(Vec3 const &otherVec) const
        {
            return ((x-otherVec.x)*(x-otherVec.x) +
                    (y-otherVec.y)*(y-otherVec.y) +
                    (z-otherVec.z)*(z-otherVec.z));
        }

        inline Vec3 Normalized() const
        {
            double vecMagnitude = sqrt(x*x + y*y + z*z);

            return Vec3(x/vecMagnitude,
                        y/vecMagnitude,
                        z/vecMagnitude);
        }

        inline Vec3 ScaledBy(double scaleFactor) const
        {
            return Vec3(x*scaleFactor,
                        y*scaleFactor,
                        z*scaleFactor);
        }

        inline Vec3 RotatedBy(Vec3 const &axisVec, double angleDegCCW)
        {
            if(!angleDegCCW)
            {   return Vec3(this->x,this->y,this->z);   }

            Vec3 rotatedVec;
            double angleRad = angleDegCCW*3.141592653589/180.0;
            rotatedVec = this->ScaledBy(cos(angleRad)) +
                         (axisVec.Cross(*this)).ScaledBy(sin(angleRad)) +
                         axisVec.ScaledBy(axisVec.Dot(*this)).ScaledBy(1-cos(angleRad));

            return rotatedVec;
        }

        inline Vec3 operator+ (const Vec3 &otherVec) const
        {
            return Vec3(x+otherVec.x,
                        y+otherVec.y,
                        z+otherVec.z);
        }

        inline Vec3 operator- (const Vec3 &otherVec) const
        {
            return Vec3(x-otherVec.x,
                        y-otherVec.y,
                        z-otherVec.z);
        }

        double x;
        double y;
        double z;
    };


    bool CalcRayTriIntersection(Vec3 const &rayPoint,
                                Vec3 const &rayDirn,
                                Vec3 const &triA,
                                Vec3 const &triB,
                                Vec3 const &triC)
    {
        // This method was taken from [1] and is originally
        // Copyright 2001 softSurfer, 2012 Dan Sunday.

        // [1] - http://geomalgorithms.com/a06-_intersect-2.html

        Vec3    u, v, n;              // triangle vectors
        Vec3    dir, w0, w;           // ray vectors
        double     r, a, b;           // params to calc ray-plane intersect

        // get triangle edge vectors and plane normal
        u = triB - triA;
        v = triC - triA;
        n = u.Cross(v);               // cross product
        if (n.Magnitude() == 0)             // triangle is degenerate
            return false;                // do not deal with this case

        dir = rayDirn;              // ray direction vector
        w0 = rayPoint - triA;
        a = (n.Dot(w0))*-1.0;
        b = n.Dot(dir);
        if (fabs(b) < K_EPS) {
            // ray is  parallel to triangle plane
            if (a == 0)   {
                // ray lies in triangle plane
                // we consider this to be an intersection
                return true;
            }
            else   {
                // ray disjoint from plane
                return false;
            }
        }

        // get intersect point of ray with triangle plane
        r = a / b;
        if(r < 0.0)   {
            // ray goes away from triangle,
            // so no intersection
            return false;
        }

        // intersect point of ray and plane
        Vec3 I = rayPoint + (dir.ScaledBy(r));

        // is I inside T?
        double uu, uv, vv, wu, wv, D;
        uu = u.Dot(u);
        uv = u.Dot(v);
        vv = v.Dot(v);
        w = I - triA;
        wu = w.Dot(u);
        wv = w.Dot(v);
        D = uv * uv - uu * vv;

        // get and test parametric coords
        double s, t;

        s = (uv * wv - vv * wu) / D;
        if (s < 0.0 || s > 1.0)         // I is outside T
        {    return false;   }

        t = (uv * wu - uu * wv) / D;
        if (t < 0.0 || (s + t) > 1.0)   // I is outside T
        {    return false;   }

        return true;                    // I is in T
    }


int main()
{
    Vec3 rayPoint(0,0,20);
    Vec3 rayDirn(0,0,-1);

    Vec3 triA(-1,-1,0);
    Vec3 triB(1,-1,0);
    Vec3 triC(2,1,0);

    if(CalcRayTriIntersection(rayPoint,rayDirn,
                           triA,triB,triC))
    {
        std::cout << "Ray intersects triangle!\n";
    }
    else
    {
        std::cout << "Ray misses triangle!\n";
    }


    return 0;
}
