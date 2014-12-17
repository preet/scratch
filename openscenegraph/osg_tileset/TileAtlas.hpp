#include <osg/Vec4>
#include <osgDB/ReadFile>
#include <osg/Texture2D>
#include <list>


namespace scratch
{
    typedef unsigned int uint;
    typedef uint8_t u8;
    typedef uint16_t u16;
    typedef uint32_t u32;
    typedef uint64_t u64;

    typedef int sint;
    typedef int8_t s8;
    typedef int16_t s16;
    typedef int32_t s32;
    typedef int64_t s64;


    class TileAtlas
    {
        struct Atlas
        {
            struct Space
            {
                Space(uint col,
                      uint row,
                      osg::Vec4 region) :
                    col(col),
                    row(row),
                    region(region),
                    id(0),
                    refs(0)
                {}

                uint const col;
                uint const row;
                osg::Vec4 const region;

                u64 id;
                uint refs;
            };

            Atlas(uint cols, uint rows, uint width, uint height)
            {
                // create spaces
                list_avail.reserve(rows*cols);
                list_spaces.reserve(rows*cols);

                uint index=0;
                for(uint r=0; r < rows; r++) {
                    for(uint c=0; c < cols; c++) {
                        list_spaces.emplace_back(
                                    c,r,osg::Vec4(float(c)/cols,
                                                  float(r)/rows,
                                                  1.0/cols,
                                                  1.0/rows));

                        list_avail.push_back(index);
                        index++;
                    }
                }

                // create image
                image = new osg::Image;
                image->allocateImage(width,height,1,GL_RGBA,GL_UNSIGNED_BYTE);

                // create texture
                texture = new osg::Texture2D;
                texture->setImage(image.get());
                texture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
                texture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
                texture->setWrap(osg::Texture2D::WRAP_S,osg::Texture2D::CLAMP_TO_EDGE);
                texture->setWrap(osg::Texture2D::WRAP_T,osg::Texture2D::CLAMP_TO_EDGE);
            }

            std::vector<uint> list_avail;
            std::vector<Space> list_spaces;
            osg::ref_ptr<osg::Image> image;
            osg::ref_ptr<osg::Texture2D> texture;
        };

    public:

        TileAtlas(uint atlas_width_px,
                  uint atlas_height_px,
                  uint tile_width_px,
                  uint tile_height_px) :
            m_atlas_width_px(atlas_width_px),
            m_atlas_height_px(atlas_height_px),
            m_tile_width_px(tile_width_px),
            m_tile_height_px(tile_height_px),
            m_atlas_cols(atlas_width_px/tile_width_px),
            m_atlas_rows(atlas_height_px/tile_height_px),
            m_atlas_tiles(m_atlas_cols*m_atlas_rows)
        {
            // read in def noise tile image
            m_noise_tile = osgDB::readImageFile("/home/preet/Dev/misc/tile_test/noise.png");

            // more debug
            m_tile_count=0;
        }

        void print()
        {
            std::cout << "atlas_count: " << m_list_atlases.size()
                      << ", tile_count; " << m_tile_count
                      << std::endl;
            for(auto const &id_space : m_lkup_id_space) {
                u64 id = id_space.first;
                atlas_iterator atlas_it = id_space.second.first;
                uint space_idx = id_space.second.second;
                Atlas::Space & space = atlas_it->list_spaces[id_space.second.second];

                std::cout << "id: " << id
                          << ", atlas_ptr: " << &(*atlas_it)
                          << ", space_idx: " << space_idx
                          << ", space: {" << space.col << "," << space.row << "," << space.id << "," << space.refs <<  "}"
                          << ", avail sz: " << atlas_it->list_avail.size()
                          << std::endl;
            }
        }

        // add or get
        void add(u64 id,
                 osg::Image const * tile_image,
                 osg::Texture2D * &atlas_texture,
                 osg::Vec4 &atlas_region)
        {
            // lookup id
            auto lkup_it = m_lkup_id_space.find(id);
            if(lkup_it == m_lkup_id_space.end()) {
                // create another atlas if required
                atlas_iterator atlas_it;
                bool space_avail=false;
                for(atlas_it  = m_list_atlases.begin();
                    atlas_it != m_list_atlases.end(); ++atlas_it)
                {
                    if(!(atlas_it->list_avail.empty())) {
                        space_avail = true;
                        break;
                    }
                }

                if(!space_avail) {
                    m_list_atlases.emplace_back(
                                m_atlas_cols,
                                m_atlas_rows,
                                m_atlas_width_px,
                                m_atlas_height_px);
                    atlas_it = m_list_atlases.end();
                    std::advance(atlas_it,-1); // last
                    std::cout << "atlas_count: " << m_list_atlases.size() << std::endl;
                }

                // get space
                uint space_idx = atlas_it->list_avail.back();
                Atlas::Space &space = atlas_it->list_spaces[space_idx];
                atlas_it->list_avail.pop_back();

                // save data
                space.id = id;

                // update texture
                if(tile_image != nullptr) {
                    uint s = space.col*m_tile_width_px;
                    uint t = space.row*m_tile_height_px;
                    uint r = 0;

                    atlas_it->image->copySubImage(s,t,r,tile_image);
                    atlas_it->image->dirty();
                }

                // save lookup
                std::pair<u64,std::pair<atlas_iterator,uint>> lkup;
                lkup.first = id;
                lkup.second.first = atlas_it;
                lkup.second.second = space_idx;
                lkup_it = m_lkup_id_space.insert(lkup).first;
                m_tile_count++;
            }

            // get data
            atlas_iterator atlas_it = lkup_it->second.first;
            uint space_idx = lkup_it->second.second;
            Atlas::Space &space = atlas_it->list_spaces[space_idx];

            atlas_texture = atlas_it->texture.get();
            atlas_region = space.region;

            // increase ref count
            space.refs++;
        }

        bool remove(u64 id)
        {
            // lookup id
            auto lkup_it = m_lkup_id_space.find(id);
            if(lkup_it == m_lkup_id_space.end()) {
                return false;
            }

            // get space
            atlas_iterator atlas_it = lkup_it->second.first;
            uint space_idx = lkup_it->second.second;
            Atlas::Space &space = atlas_it->list_spaces[space_idx];

            // decrease ref count
            space.refs--;

            if(space.refs == 0) {
                // make this space available again

                // DEBUG update texture
                atlas_it->image->copySubImage(
                            space.col*m_tile_width_px,
                            space.row*m_tile_height_px,0,
                            m_noise_tile.get());
                atlas_it->image->dirty();

                space.id = 0;
                atlas_it->list_avail.push_back(lkup_it->second.second);

                // remove lookup
                m_lkup_id_space.erase(lkup_it);

                // remove atlas if its empty
                if(atlas_it->list_avail.size() == atlas_it->list_spaces.size()) {
                    m_list_atlases.erase(atlas_it);
                    std::cout << "atlas_count: " << m_list_atlases.size() << std::endl;
                }

                m_tile_count--;
            }
            return true;
        }

        uint m_tile_count;


    private:
        uint const m_atlas_width_px;
        uint const m_atlas_height_px;
        uint const m_tile_width_px;
        uint const m_tile_height_px;
        uint const m_atlas_cols;
        uint const m_atlas_rows;
        uint const m_atlas_tiles;

        std::list<Atlas> m_list_atlases;
        typedef std::list<Atlas>::iterator atlas_iterator;

        // <id: Atlas*, space_index>
        std::map<u64,std::pair<atlas_iterator,uint>> m_lkup_id_space;

        // noise debug
        osg::ref_ptr<osg::Image> m_noise_tile;
    };
}

