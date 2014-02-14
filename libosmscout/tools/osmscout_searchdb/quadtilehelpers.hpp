#include <stdint.h>
#include <string>
#include <sstream>
#include <vector>

#include <QDebug>

namespace quadtiles
{
    typedef uint8_t uint8_t;
    typedef int64_t int64_t;

    struct GeoBoundingBox
    {
        GeoBoundingBox() :
            minLon(0),minLat(0),maxLon(0),maxLat(0) {}

        double minLon;
        double minLat;
        double maxLon;
        double maxLat;

        void resizeToQuadTL()
        {
            maxLon = (maxLon+minLon)/2;
            minLat = (maxLat+minLat)/2;
        }

        void resizeToQuadTR()
        {
            minLon = (maxLon+minLon)/2;
            minLat = (maxLat+minLat)/2;
        }

        void resizeToQuadBL()
        {
            maxLon = (maxLon+minLon)/2;
            maxLat = (maxLat+minLat)/2;
        }

        void resizeToQuadBR()
        {
            minLon = (maxLon+minLon)/2;
            maxLat = (maxLat+minLat)/2;
        }
    };

    struct QuadTile
    {
        QuadTile(uint8_t p_level,
                 int64_t p_key,
                 GeoBoundingBox p_bbox) :
            level(p_level),
            key(p_key),
            TL(NULL),
            TR(NULL),
            BL(NULL),
            BR(NULL),
            bbox(p_bbox)
        {}

        uint8_t level;
        int64_t key;

        QuadTile * TL;
        QuadTile * TR;
        QuadTile * BL;
        QuadTile * BR;

        GeoBoundingBox bbox;
    };


    double stringToDouble(std::string const &text)
    {
        std::stringstream ss(text);
        double result;
        return ss >> result ? result : 0;
    }

    bool calcRectOverlap(double r1_bl_x, double r1_bl_y,
                         double r1_tr_x, double r1_tr_y,
                         double r2_bl_x, double r2_bl_y,
                         double r2_tr_x, double r2_tr_y)
    {
        if((r1_tr_x < r2_bl_x) ||
           (r1_bl_x > r2_tr_x) ||
           (r1_tr_y < r2_bl_y) ||
           (r1_bl_y > r2_tr_y))
        {   return false;   }

        return true;
    }

    bool calcBoundsOverlap(GeoBoundingBox const &bbox1,
                           GeoBoundingBox const &bbox2)
    {
        return calcRectOverlap(bbox1.minLon,bbox1.minLat,
                               bbox1.maxLon,bbox1.maxLat,
                               bbox2.minLon,bbox2.minLat,
                               bbox2.maxLon,bbox2.maxLat);
    }

    // buildQuadTilesForBounds
    // * builds a quadtile tree with max_level that
    //   overlaps the given bounding box
    // * saves quadtiles that have level==max_level
    void buildQuadTilesForBounds(QuadTile * tile,
                                 uint8_t const max_level,
                                 GeoBoundingBox const &bbox,
                                 std::vector<QuadTile*> &listTiles)
    {
        if(tile->level == max_level)   {
            listTiles.push_back(tile);
            return;
        }

        // check if this tile overlaps the bbox
        if(calcBoundsOverlap(tile->bbox,bbox))   {
            // check which quadrants of this tile
            // overlap with the bbox and recurse
            // on overlapping tiles
            GeoBoundingBox bboxTL = tile->bbox;
            bboxTL.resizeToQuadTL();
            if(calcBoundsOverlap(bboxTL,bbox))   {
                tile->TL = new QuadTile(tile->level+1,((tile->key) << 2) | 0x0,bboxTL);
                buildQuadTilesForBounds(tile->TL,max_level,bbox,listTiles);
            }

            GeoBoundingBox bboxTR = tile->bbox;
            bboxTR.resizeToQuadTR();
            if(calcBoundsOverlap(bboxTR,bbox))   {
                tile->TR = new QuadTile(tile->level+1,((tile->key) << 2) | 0x1,bboxTR);
                buildQuadTilesForBounds(tile->TR,max_level,bbox,listTiles);
            }

            GeoBoundingBox bboxBL = tile->bbox;
            bboxBL.resizeToQuadBL();
            if(calcBoundsOverlap(bboxBL,bbox))   {
                tile->BL = new QuadTile(tile->level+1,((tile->key) << 2) | 0x2,bboxBL);
                buildQuadTilesForBounds(tile->BL,max_level,bbox,listTiles);
            }

            GeoBoundingBox bboxBR = tile->bbox;
            bboxBR.resizeToQuadBR();
            if(calcBoundsOverlap(bboxBR,bbox))   {
                tile->BR = new QuadTile(tile->level+1,((tile->key) << 2) | 0x3,bboxBR);
                buildQuadTilesForBounds(tile->BR,max_level,bbox,listTiles);
            }
        }
    }

    void deleteQuadTree(QuadTile * root)
    {
        if(root)   {
            if(root->TL)   {
                deleteQuadTree(root->TL);
                delete root->TL;
                root->TL = NULL;
            }
            if(root->TR)   {
                deleteQuadTree(root->TR);
                delete root->TR;
                root->TR = NULL;
            }
            if(root->BL)   {
                deleteQuadTree(root->BL);
                delete root->BL;
                root->BL = NULL;
            }
            if(root->BR)   {
                deleteQuadTree(root->BR);
                delete root->BR;
                root->BR = NULL;
            }
        }
    }

    // calcQuadKeyValue
    // * given a (lon,lat) coordinate and a level,
    //   look up the corresponding quad tile the
    //   coordinate lies in and return its key
    int64_t calcQuadKeyValue(double lon, double lat, uint8_t level)
    {
        double minLon = -180.0;
        double maxLon = 0.0;
        double minLat = -90.0;
        double maxLat = 90.0;

        int64_t quadkey = 0;

        for(uint8_t i=0; i < level; i++)   {
            double diffLon = (maxLon-minLon)/2.0;
            double diffLat = (maxLat-minLat)/2.0;

            double midLon = (maxLon+minLon)/2.0;
            double midLat = (maxLat+minLat)/2.0;

            if(lat > midLat)   {            // upper
                minLat += diffLat;
                if(lon < midLon)   {        // left
                    quadkey = quadkey << 2;
                    maxLon -= diffLon;
                }
                else   {                    // right
                    quadkey = quadkey << 2;
                    quadkey |= 1;
                    minLon += diffLon;
                }
            }
            else   {                        // lower
                maxLat -= diffLat;
                if(lon < midLon)   {        // left
                    quadkey = quadkey << 2;
                    quadkey |= 2;
                    maxLon -= diffLon;
                }
                else   {                    // right
                    quadkey = quadkey << 2;
                    quadkey |= 3;
                    minLon += diffLon;
                }
            }
        }

        return quadkey;
    }
}
