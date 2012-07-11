#include <math.h>

    class Vec2
    {
    public:
        Vec2() :
            x(0),y(0) {}

        Vec2(double myX, double myY) :
            x(myX),y(myY) {}

        inline double Dot(Vec2 const & otherVec) const
        {
            return (x*otherVec.x)+
                   (y*otherVec.y);
        }

        inline double DistanceTo(Vec2 const &otherVec) const
        {
            return sqrt((x-otherVec.x)*(x-otherVec.x) +
                        (y-otherVec.y)*(y-otherVec.y));
        }

        inline double Distance2To(Vec2 const &otherVec) const
        {
            return ((x-otherVec.x)*(x-otherVec.x) +
                    (y-otherVec.y)*(y-otherVec.y));
        }

        inline Vec2 Normalized() const
        {      
            double vecMagnitude = sqrt(x*x + y*y);

            return Vec2(x/vecMagnitude,
                        y/vecMagnitude);
        }

        inline Vec2 ScaledBy(double scaleFactor) const
        {
            return Vec2(x*scaleFactor,
                        y*scaleFactor);
        }

        inline Vec2 operator+ (const Vec2 &otherVec) const
        {
            return Vec2(x+otherVec.x,
                        y+otherVec.y);
        }

        inline Vec2 operator- (const Vec2 &otherVec) const
        {
            return Vec2(x-otherVec.x,
                        y-otherVec.y);
        }

        double x;
        double y;
    };
