#include <iostream>
#include <unordered_map>

#define OSMIUM_WITH_PBF_INPUT

#include <osmium.hpp>
#include <osmium/handler/debug.hpp>

struct Node
{
    double lon;
    double lat;
};

namespace Osmium
{
    namespace Handler
    {
        class DumpCoastlines : public Base
        {
        public:
            void init(OSM::Meta &meta) const
            {}

            void before_nodes() const {}

            void node(const shared_ptr<const OSM::Node> &node)
            {
                Node myNode;
                myNode.lon = node->position().lon();
                myNode.lat = node->position().lat();

                std::pair<size_t,Node> insData;
                insData.first = node->id();
                insData.second = myNode;

                m_listNodes.insert(insData);
            }

            void after_nodes() const
            {   std::cerr << "Saved " << m_listNodes.size() << " nodes! \n";   }

            void before_ways() const {}

            void way(const shared_ptr<const OSM::Way> &way)
            {
                Osmium::OSM::TagList::const_iterator tagIt;
                for(tagIt = way->tags().begin();
                    tagIt != way->tags().end(); ++tagIt)
                {
                    std::string sKey(tagIt->key());
                    std::string sVal(tagIt->value());

                    if(sKey.compare("natural") == 0)   {
                        if(sVal.compare("coastline") == 0)   {

                            Osmium::OSM::WayNodeList::const_iterator nodeIt;
                            for(nodeIt = way->nodes().begin();
                                nodeIt != way->nodes().end(); ++nodeIt)
                            {
                                std::unordered_map<size_t,Node>::iterator findIt;
                                size_t findId = nodeIt->ref();
                                findIt = m_listNodes.find(findId);
                                if(findIt == m_listNodes.end())
                                {   return;   }     // way pointing to node not in data set

                                std::cout << std::fixed << std::setprecision(7)
                                          << findIt->second.lon << ","
                                          << std::fixed << std::setprecision(7)
                                          << findIt->second.lat << "," << 0 << "\n";
                            }
                        }
                    }
                }
            }

            void after_ways() const {}
            void before_relations() const {}
            void after_relations() const {}
            void final() const {}

        private:
            std::unordered_map<size_t,Node> m_listNodes;
        };
    }
}

int main()
{
    std::ios_base::sync_with_stdio(false);

    Osmium::OSMFile infile("/home/preet/Documents/maps/openstreetmap/ontario.osm.pbf");
    Osmium::Handler::DumpCoastlines handler;
    Osmium::Input::read(infile,handler);

    return 0;
}
