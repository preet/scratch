#include <math.h>

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

        bool operator== (const Vec3 &otherVec) const
        {
            if(this->x == otherVec.x &&
               this->y == otherVec.y &&
               this->z == otherVec.z)
            {
                return true;
            }

            return false;
        }

        double x;
        double y;
        double z;
    };
