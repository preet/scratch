#include <DataSetTilesLL.h>

namespace scratch
{
    DataSetTilesLL::DataSetTilesLL(std::unique_ptr<TileSetLL> tileset,
                                   osg::Group * gp_tiles) :
        m_tileset(std::move(tileset)),
        m_gp_tiles(gp_tiles)
    {

    }

    DataSetTilesLL::~DataSetTilesLL()
    {

    }

    void DataSetTilesLL::Update(osg::Camera const * cam)
    {

    }
}
