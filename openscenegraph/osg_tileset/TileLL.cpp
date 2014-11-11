#include <TileLL.h>

namespace scratch
{
    TileLL::Id TileLL::GetIdFromLevelXY(uint8_t level,
                                        uint32_t x,
                                        uint32_t y)
    {
        uint64_t level64 = level;
        uint64_t x64 = x;
        uint64_t y64 = y;
        uint64_t tile_id = 0;
        tile_id |= (level64 << 48);
        tile_id |= (x64 << 24);
        tile_id |= y64;

        return tile_id;
    }

    TileLL::Id TileLL::GetIdFromParentXY(TileLL const * parent,
                                         uint32_t x,
                                         uint32_t y)
    {
        uint8_t level = (parent==nullptr) ? 0 : parent->level + 1;
        return GetIdFromLevelXY(level,x,y);
    }

    void TileLL::GetLevelXYFromId(Id const id,
                                  uint8_t &level,
                                  uint32_t &x,
                                  uint32_t &y)
    {
        level = static_cast<uint8_t>(id >> 48);
        x = static_cast<uint32_t>((id >> 24) & 0xFFFFFF);
        y = static_cast<uint32_t>(id & 0xFFFFFF);
    }

    GeoBounds TileLL::GetBounds(TileLL const * p,
                                uint32_t x,
                                uint32_t y)
    {
        GeoBounds b;
        double const lon_width = (p->bounds.maxLon - p->bounds.minLon)*0.5;
        double const lat_width = (p->bounds.maxLat - p->bounds.minLat)*0.5;

        b.minLon = p->bounds.minLon + (lon_width * (x - p->x*2));
        b.maxLon = b.minLon + lon_width;

        b.minLat = p->bounds.minLat + (lat_width * (y - p->y*2));
        b.maxLat = b.minLat + lat_width;

        return b;
    }
}
