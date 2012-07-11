#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

#include <osmscout/Database.h>
#include <osmscout/TypeConfig.h>
#include <osmscout/StyleConfigLoader.h>


namespace osmscout
{
    class Vec2
    {
    public:
        Vec2() :
            x(0),y(0) {}

        Vec2(double myX, double myY) :
            x(myX),y(myY) {}

        double x;
        double y;
    };
    typedef std::pair<Vec2,Vec2> LineVec2;

    bool calcLinesIntersect(double a_x1, double a_y1,
                            double a_x2, double a_y2,
                            double b_x1, double b_y1,
                            double b_x2, double b_y2)
    {
        double ua_numr = (b_x2-b_x1)*(a_y1-b_y1)-(b_y2-b_y1)*(a_x1-b_x1);
        double ub_numr = (a_x2-a_x1)*(a_y1-b_y1)-(a_y2-a_y1)*(a_x1-b_x1);
        double denr = (b_y2-b_y1)*(a_x2-a_x1)-(b_x2-b_x1)*(a_y2-a_y1);

        if(denr == 0.0)
        {
            // lines are coincident
            if(ua_numr == 0.0 && ub_numr == 0.0)
            {   return true;   }

            // lines are parallel
            else
            {   return false;   }
        }

        double ua = ua_numr/denr;
        double ub = ub_numr/denr;

        if(ua >= 0.0 && ua <= 1.0 && ub >= 0.0 && ub <= 1.0)
        {   return true;   }

        return false;
    }

    bool calcPolyIsSimple(const std::vector<LineVec2> &listEdges,
                          const std::vector<bool> &edgeStartsNewPoly)
    {
        unsigned int edgesIntersect = 0;
        for(int i=0; i < listEdges.size(); i++)  {
            edgesIntersect = 0;
            for(int j=i+1; j < listEdges.size(); j++)  {
                if(calcLinesIntersect(listEdges[i].first.x,
                                      listEdges[i].first.y,
                                      listEdges[i].second.x,
                                      listEdges[i].second.y,
                                      listEdges[j].first.x,
                                      listEdges[j].first.y,
                                      listEdges[j].second.x,
                                      listEdges[j].second.y))
                {
                    edgesIntersect++;

                    // we check the first edge of a sole poly against every
                    // other edge and expect to see 2 intersections for
                    // adjacent edges; poly is complex if there are more
                    // intersections
                    if(edgeStartsNewPoly[i] == true) {
                        if(edgesIntersect > 2)
                        {
                            std::cout << "   WARN: " << edgesIntersect
                                      << " intersections" << std::endl;
                            return false;
                        }
                    }

                    // otherwise we check an edge that isn't the first
                    // edge against every other edge excluding those that
                    // have already been tested (this means one adjacent
                    // edge); poly is complex if there is more than one
                    // intersection
                    else  {
                        if(edgesIntersect > 1)
                        {
                            std::cout << "   WARN: " << edgesIntersect
                                      << " intersections" << std::endl;
                            return false;
                        }
                    }
                }
            }
        }
        return true;
    }

    bool calcPolyIsCCW(const std::vector<Vec2> &listPoints)
    {
        // based on  hxxp://en.wikipedia.org/wiki/Curve_orientation
        // and       hxxp://local.wasp.uwa.edu.au/~pbourke/geometry/clockwise/

        // note: poly must be simple
        // note: this assumes 2d cartesian coordinate space is used;
        //       for geographic, expect Vec2.x=lon and Vec2.y=lat!

        int ptIdx = 0;
        for(int i=1; i < listPoints.size(); i++)
        {
            // find the point with the smallest y value,
            if(listPoints[i].y < listPoints[ptIdx].y)
            {   ptIdx = i;   }

            // if y values are equal save the point with greatest x
            else if(listPoints[i].y == listPoints[ptIdx].y)
            {
                if(listPoints[i].x < listPoints[ptIdx].x)
                {   ptIdx = i;   }
            }
        }

        int prevIdx = (ptIdx == 0) ? listPoints.size()-1 : ptIdx-1;
        int nextIdx = (ptIdx == listPoints.size()-1) ? 0 : ptIdx+1;

        double signedArea = (listPoints[ptIdx].x-listPoints[prevIdx].x) *
                            (listPoints[nextIdx].y-listPoints[ptIdx].y) -
                            (listPoints[ptIdx].y-listPoints[prevIdx].y) *
                            (listPoints[nextIdx].x-listPoints[ptIdx].x);

        return (signedArea > 0.0);
    }

    bool calcAreaIsValid(std::vector<Vec2> &listOuterPts,
                         std::vector<std::vector<Vec2> > &listListInnerPts)
    {
        if(listOuterPts.size() < 3)   {
            return false;
        }

        unsigned int numEdges = listOuterPts.size();
        for(int i=0; i < listListInnerPts.size(); i++)
        {   numEdges += listListInnerPts[i].size();   }

        std::vector<bool> edgeStartsNewPoly(numEdges,false);
        std::vector<LineVec2> listEdges(numEdges);
        unsigned int cEdge = 0;

        // temporarily wrap around vertices
        // (first == last) to generate edge lists
        listOuterPts.push_back(listOuterPts[0]);
        for(int i=0; i < listListInnerPts.size(); i++)
        {   listListInnerPts[i].push_back(listListInnerPts[i][0]);   }

        // outer poly
        edgeStartsNewPoly[0] = true;
        for(int i=1;i < listOuterPts.size(); i++)
        {
            LineVec2 outerEdge;
            outerEdge.first = listOuterPts[i-1];
            outerEdge.second = listOuterPts[i];
            listEdges[cEdge] = outerEdge; cEdge++;
        }

        // inner polys
        for(int i=0; i < listListInnerPts.size(); i++)
        {
            edgeStartsNewPoly[cEdge] = true;
            for(int j=1; j < listListInnerPts[i].size(); j++)
            {
                LineVec2 innerEdge;
                innerEdge.first = listListInnerPts[i][j-1];
                innerEdge.second = listListInnerPts[i][j];
                listEdges[cEdge] = innerEdge; cEdge++;
            }
        }

        // revert vertex list modifications (not
        // really the 'nicest' way of doing this)
        listOuterPts.pop_back();
        for(int i=0; i < listListInnerPts.size(); i++)
        {   listListInnerPts[i].pop_back();   }

        if(calcPolyIsSimple(listEdges,edgeStartsNewPoly))
        {
            // expect listOuterPts to be CCW and innerPts
            // to be CW, if not then reverse point order

            if(!calcPolyIsCCW(listOuterPts))   {
                std::reverse(listOuterPts.begin(),
                             listOuterPts.end());
            }

            for(int i=0; i < listListInnerPts.size(); i++)   {
                if(calcPolyIsCCW(listListInnerPts[i]))   {
                    std::reverse(listListInnerPts[i].begin(),
                                 listListInnerPts[i].end());
                }
            }
        }
        else   {
            return false;
        }

        return true;
    }

    bool calcAreaIsValid(std::vector<Vec2> &listOuterPts)
    {
        std::vector<std::vector<Vec2> > listListInnerPts; //empty
        return(calcAreaIsValid(listOuterPts,listListInnerPts));
    }
}

// example of how to use with libosmscout
int main()
{
    // get area data from libosmscout
    std::string dataPath("/home/preet/Documents/Maps/default/toronto");
    osmscout::DatabaseParameter databaseParam;
    osmscout::Database database(databaseParam);
    if(database.Open(dataPath))
    {   std::cerr << "INFO: Opened Database Successfully" << std::endl;   }
    else
    {   std::cerr << "ERROR: Could not open database" << std::endl;   }

    osmscout::TypeSet typeSet;
    osmscout::TypeConfig * typeConfig = database.GetTypeConfig();
    std::vector<osmscout::TypeInfo> listTypeInfo = typeConfig->GetTypes();

    for(int i=0; i < listTypeInfo.size(); i++)
    {   typeSet.SetType(listTypeInfo[i].GetId());   }

    std::vector<osmscout::NodeRef>        listNodeRefs;
    std::vector<osmscout::WayRef>         listWayRefs;
    std::vector<osmscout::WayRef>         listAreaRefs;
    std::vector<osmscout::RelationRef>    listRelWayRefs;
    std::vector<osmscout::RelationRef>    listRelAreaRefs;

    double minLat = 43.6463;
    double minLon = -79.4076;
    double maxLat = 43.67980;
    double maxLon = -79.3931;

    if(database.GetObjects(minLon,minLat,
                           maxLon,maxLat,
                           typeSet,
                           listNodeRefs,
                           listWayRefs,
                           listAreaRefs,
                           listRelWayRefs,
                           listRelAreaRefs))
    {
        std::cerr << "INFO: Queried Database Successfully" << std::endl;
        std::cerr << "INFO: Found " << listNodeRefs.size() << " nodes" << std::endl;
        std::cerr << "INFO: Found " << listWayRefs.size() << " ways" << std::endl;
        std::cerr << "INFO: Found " << listAreaRefs.size() << " areas" << std::endl;
        std::cerr << "INFO: Found " << listRelWayRefs.size() << " relation ways" << std::endl;
        std::cerr << "INFO: Found " << listRelAreaRefs.size() << " relation areas" << std::endl;

        // SINGLE AREAS
        for(int i=0; i < listAreaRefs.size(); i++)
        {
            std::vector<osmscout::Vec2> listOuterPts;
            for(int n=0; n < listAreaRefs[i]->nodes.size(); n++)
            {
                listOuterPts.push_back(osmscout::Vec2(listAreaRefs[i]->nodes[n].GetLon(),
                                                      listAreaRefs[i]->nodes[n].GetLat()));
            }
            if(osmscout::calcAreaIsValid(listOuterPts))   {
                // area isn't complex! yay!
            }
            else   {
                std::cerr << "ERROR: Area " <<  listAreaRefs[i]->GetId()
                          << " is complex!" << std::endl;
            }
        }

        // RELATION AREAS
        std::vector<osmscout::RelationRef>::iterator relIt;
        for(relIt = listRelAreaRefs.begin();
            relIt != listRelAreaRefs.end(); ++relIt)
        {
            std::vector<osmscout::Vec2>                 listOuterPts;
            std::vector<std::vector<osmscout::Vec2> >   listListInnerPts;

            osmscout::RelationRef &areaRel = (*relIt);
            for(int i=0; i < areaRel->roles.size(); i++)
            {
                // look for outerRing
                if(areaRel->roles[i].ring%2 == 0 &&
                   areaRel->roles[i].GetType() != osmscout::typeIgnore)
                {
                    // save outerRing nodes
                    for(int r=0; r < areaRel->roles[i].nodes.size(); r++)
                    {
                        osmscout::Vec2 myPt(areaRel->roles[i].nodes[r].GetLon(),
                                            areaRel->roles[i].nodes[r].GetLat());

                        listOuterPts.push_back(myPt);
                    }

                    i++;

                    // see if there are any innerRings
                    // that belong to the outerRing
                    while(i < areaRel->roles.size())
                    {
                        // keep iterating until we finish saving
                        // all innerRing data
                        if(areaRel->roles[i].ring%2 == 1)
                        {
                            // save innerRing nodes
                            std::vector<osmscout::Vec2> listInnerPts;
                            for(int r=0; r < areaRel->roles[i].nodes.size(); r++)
                            {
                                osmscout::Vec2 myPt(areaRel->roles[i].nodes[r].GetLon(),
                                                    areaRel->roles[i].nodes[r].GetLat());

                                listInnerPts.push_back(myPt);
                            }
                            listListInnerPts.push_back(listInnerPts);

                            i++;
                        }
                        else    // means ringId is outerRing
                        {
                            i--;
                            break;
                        }
                    }

                    if(osmscout::calcAreaIsValid(listOuterPts,listListInnerPts))
                    {
                        // area isn't complex! yay!
                    }
                    else   {
                        std::cerr << "ERROR: Relation Area " <<  areaRel->GetId()
                                  << " is complex!" << std::endl;
                    }
                }
            }
        }
    }

    return 0;
}
