/*
   Copyright (C) 2014 Preet Desai (preet.desai@gmail.com)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#include <mutex>
#include <algorithm>

#include <GeometryUtils.h>
#include <DataSetTilesLL.h>
#include <OSGUtils.h>
#include <osgText/Text>
#include <osgDB/ReadFile>
#include <TileSetLLByPixelRes.h>

DataSetTilesLL::DataSetTilesLL(Options const &opts,
                               std::unique_ptr<TileSetLL> tileset,
                               osg::Group * gp_tiles) :
    m_opts(opts),
    m_tileset(std::move(tileset)),
    m_gp_tiles(gp_tiles)
{
    // create a list of which levels have base textures
    m_list_level_is_base.resize(m_tileset->GetMaxLevel(),0);
    for(size_t i=0; i <  m_list_level_is_base.size(); i++) {
        for(auto base_level : m_opts.list_base_levels) {
            if(i == base_level) {
                m_list_level_is_base[i] = 1;
            }
        }
    }

    // init the tile image source
    std::function<std::string(TileLL::Id)> image_path_gen =
            [](TileLL::Id id) -> std::string {

        uint8_t level;
        uint32_t x,y;
        TileLL::GetLevelXYFromId(id,level,x,y);

        std::string path =
                "/home/preet/Dev/misc/tile_test/" +
                std::to_string(level)+
                ".png";

        return path;
    };

    m_tile_image_source.reset(
                new TileImageSourceLLLocal(image_path_gen));

    //
    m_num_base_textures = 0;
    for(auto level : m_opts.list_base_levels) {
        m_num_base_textures += pow(4,level);
    }

    // TODO what if max_textures < base_textures ?
    m_max_view_textures = 0;
    if(m_opts.max_textures > m_num_base_textures) {
        m_max_view_textures = m_opts.max_textures-m_num_base_textures;
    }

    // Preload the base textures
    m_base_textures_loaded = false;
    for(auto level : m_opts.list_base_levels) {
        // request an image for all tiles in this level
        size_t const tiles_in_x =
                pow(4,level)*m_tileset->GetNumRootTilesX();

        size_t const tiles_in_y =
                pow(4,level)*m_tileset->GetNumRootTilesY();

        for(size_t y=0; y < tiles_in_y; y++) {
            for(size_t x=0; x < tiles_in_x; x++) {
                // save request
                auto const tile_id = TileLL::GetIdFromLevelXY(level,x,y);
                m_lkup_base_textures.emplace(
                            tile_id,
                            TextureRequest(
                                m_tile_image_source->RequestImage(tile_id)));
            }
        }
    }

    // poly mode for wireframe tiles
    m_poly_mode = new osg::PolygonMode;
    m_poly_mode->setMode(osg::PolygonMode::FRONT_AND_BACK,
                         osg::PolygonMode::LINE);

    // debug
    m_gp_debug = new osg::Group;
    m_gp_tiles->addChild(m_gp_debug);
}

DataSetTilesLL::~DataSetTilesLL()
{

}

void DataSetTilesLL::applyDefaultTextureSettings(osg::Texture2D *texture) const
{
    texture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
    texture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
    texture->setWrap(osg::Texture2D::WRAP_S,osg::Texture2D::CLAMP_TO_EDGE);
    texture->setWrap(osg::Texture2D::WRAP_T,osg::Texture2D::CLAMP_TO_EDGE);
}

osg::ref_ptr<osg::Texture2D>
DataSetTilesLL::createTextureForImage(osg::Image * image) const
{
    osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;
    texture->setImage(image);

    texture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
    texture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
    texture->setWrap(osg::Texture2D::WRAP_S,osg::Texture2D::CLAMP_TO_EDGE);
    texture->setWrap(osg::Texture2D::WRAP_T,osg::Texture2D::CLAMP_TO_EDGE);

    return texture;
}

void DataSetTilesLL::getTileTexture(TileLL const * tile,
                                    osg::ref_ptr<osg::Texture2D> &texture,
                                    double &s_start,
                                    double &s_delta,
                                    double &t_start,
                                    double &t_delta)
{
    // get the texture for this tile
    s_start = 0.0;
    s_delta = 1.0;
    t_start = 0.0;
    t_delta = 1.0;

    if(m_list_level_is_base[tile->level]) {
        // lookup tile texture in base textures
        auto it = m_lkup_base_textures.find(tile->id);
        assert(it != m_lkup_base_textures.end());

        texture = it->second;
    }
    else {
        // The tile texture hasn't been preloaded.
        // The texture is either
        // 1. already available
        // 2a. unavailable but has been requested
        // 2b. unavailable and has not been requested

        if(m_lru_view_textures.exists(tile->id)) {
            // 1. already available

        }
        else {
            // 2. unavailable

            if(m_lkup_req_images.find(tile->id) ==
               m_lkup_req_images.end())
            {   // request the texture since no
                // prior request exists
                m_lkup_req_images.emplace(
                            tile_id,
                            m_tile_image_source->RequestImage(tile->id));
            }

            // Since the tile texture is unavailable,
            // use the closest parent in base textures
            // as a placeholder

            auto it = m_opts.list_base_levels.begin();
            for(it = m_opts.list_base_levels.begin();
                it != m_opts.list_base_levels.end(); ++it)
            {
                if((*it) > tile->level) {
                    break;
                }
            }
            std::advance(it,-1);

            TileLL * parent = tile;
            while(parent->level != (*it)) {
                parent = parent->parent;
            }

            // save texture
            texture = m_lkup_base_textures.find(tile->id)->second;

            // save tex coords
            double const parent_width =
                    parent->bounds.maxLon-parent->bounds.minLon;

            double const parent_height =
                    parent->bounds.maxLat-parent->bounds.minLat;

            s_start = tile->bounds.minLon/parent_width;
            s_delta = s_start-(tile->bounds.maxLon/parent_width);

            t_start = tile->bounds.minLat/parent_height;
            t_delta = t_start-(tile->bounds.maxLat/parent_height);
        }
    }
}

void DataSetTilesLL::moveReadyImagesToBaseTextures()
{
    for(auto it = m_lkup_req_images.begin();
        it != m_lkup_req_images.end();)
    {
        auto const &request = it->second;
        if(request->IsFinished()) {
            // convert to texture
            osg::ref_ptr<osg::Image> image = request->GetImage();
            osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;
            texture->setImage(image);
            applyDefaultTextureSettings(texture);

            // save to base textures
            m_lkup_base_textures.emplace(request->GetId(),texture);

            // remove from requests
            it = m_lkup_req_images.erase(it);
        }
        else {
            ++it;
        }
    }
}

void DataSetTilesLL::moveReadyImagesToViewTextures()
{
    for(auto it = m_lkup_req_images.begin();
        it != m_lkup_req_images.end();)
    {
        auto const &request = it->second;
        if(request->IsFinished()) {
            // convert to texture
            osg::ref_ptr<osg::Image> image = request->GetImage();
            osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;
            texture->setImage(image);
            applyDefaultTextureSettings(texture);

            // save to view textures
            m_lru_view_textures.insert(request->GetId(),texture);

            // remove from requests
            it = m_lkup_req_images.erase(it);
        }
        else {
            ++it;
        }
    }
}

void DataSetTilesLL::Update(osg::Camera const * cam)
{
    // Check if the base textures have been loaded yet
    if(!m_base_textures_loaded)
    {
        for(auto const & tex_req : m_lkup_base_textures) {
            if(tex_req.texture == nullptr) {
                if(tex_req.request->IsFinished()) {
                    // get texture for image
                    tex_req.texture =
                            createTextureForImage(
                                tex_req.request->GetImage().get());

                    // clear request data
                    tex_req.request = nullptr;
                }
                else {
                    // We don't do anything until all of
                    // the base textures have been loaded
                    return;
                }
            }
        }

        m_base_textures_loaded = true;
    }

    // Get the tileset difference
    std::vector<TileLL const *> list_tiles_add;
    std::vector<TileLL const *> list_tiles_upd;
    std::vector<TileLL const *> list_tiles_rem;
    m_tileset->UpdateTileSet(cam,
                             list_tiles_add,
                             list_tiles_upd,
                             list_tiles_rem);

    // remove
    for(auto tile : list_tiles_rem) {
        // Cancel any queued image requests
        bool ok;
        auto & tex_req = m_lru_view_textures.get(tile->id,ok);
        if(ok) {
            tex_req.request->Cancel();
        }

        // Delete this tile's geometry
        osg::Group * gp_tile = m_list_sg_tiles.find(tile->id)->second;
        m_gp_tiles->removeChild(gp_tile);
        m_list_sg_tiles.erase(tile->id);
    }

    // add new geometries

    // update

    // sort by some criteria (shouldn't add
    // and update be combined for this?)

    // bool==true/false indicates the tile is add/upd
    std::vector<std::pair<TileLL const *,bool>> list_tiles_add_and_upd;
    list_tiles_add_and_upd.reserve(list_tiles_upd.size()+
                                   list_tiles_add.size());

    for(auto tile_add : list_tiles_add) {
        list_tiles_add_and_upd.push_back(
                    std::make_pair(tile_add,true));
    }

    for(auto tile_upd : list_tiles_upd) {
        list_tiles_add_and_upd.push_back(
                    std::make_pair(tile_upd,false));
    }

    // sort list by increasing importance (since
    // they will be inserted in the opposite order
    // in the LRU)
    // TODO would a member function or functor be faster?
    std::sort(list_tiles_add_and_upd.begin(),
              list_tiles_add_and_upd.end(),
              [](std::pair<TileLL const *,bool> const &a,
                 std::pair<TileLL const *,bool> const &b) {
                    return (a.first->importance < b.first->importance);
              });

    // 1. Make all changes to textures

    // move these tiles to the front of the LRU
    for(auto tile_add_upd : list_tiles_add_and_upd)
    {
        TileLL const * tile = tile_add_upd.first;

        // IF NOT A BASE TEXTURE (check level)
        {
            // If the view texture req DNE, add it
            // to the front of the LRU

            // Else move the req to the front

            TextureRequest & texreq =
                m_lru_view_textures.insert_use_get(
                        tile->id,TextureRequest);

            if(texreq.status == TextureRequest::Status::Start) {
                // Create a new request
                texreq.request = m_tile_image_source->RequestImage(tile->id);
                texreq.status = TextureRequest::Status::Loading;
            }
            else if(texreq.status == TextureRequest::Status::Loading) {
                // Check if finished
                if(texreq.request->IsFinished()) {
                    texreq.status = TextureRequest::Status::Finished;
                    texreq.texture = createTextureForImage(
                                texreq.request->GetImage().get());
                    texreq.request = nullptr;
                }
            }
        }
    }



    // 2. Pick best avail textures for geometry

    // ... Given a random list of tiles and a random
    // list of tile textures how do i pick the best
    // textures for the tiles?

    // * get lru keys
    // * get tiles for each key (what if the tile doesnt exist?)
    //   well then you cant traverse to that tile so it doesnt matter
    // * attach texture req data to each tile
    // * use that data to get texture when traversing
    //   up the tree

    // get lru keys
    std::vector<TileLL const *> list_lru_tiles;
    list_lru_tiles.reserve(m_lru_view_textures.size());

    auto list_lru_tile_ids = m_lru_view_textures.get_keys();
    for(auto tile_id : list_lru_tile_ids) {
        bool ok;
        TextureRequest &texreq =
                m_lru_view_textures.get(tile_id,ok);

        if(ok) {
            if(texreq.status == TextureRequest::Status::Finished) {
                // this is a valid/ready texture
                // save ref to it as temporary placeholder data
                TileLL const * tile = m_tileset->GetTile(tile_id);
                tile->data.reset(new TextureRequestRefData(&texreq));
                list_lru_tiles.push_back(tile);
            }
        }
    }

    // Set the best texture for each add/upd tile
    // For tiles that do not have their texture
    // available, set the next closest parent

    for(auto tile_add_upd : list_tiles_add_and_upd)
    {
        TileLL const * tile = tile_add_upd.first;

        // IF NOT A BASE TEXTURE (check level)
        {
            TextureRequestRefData * data =
                    static_cast<TextureRequestRefData>(tile->data);

            // Update tile's texture if required
            if(data->texreq->status == TextureRequest::Status::Finished) {
                if(data->texreq->texture == 0);
            }
        }
    }



//    // add
//    for(auto tile_id : list_tiles_add) {
//        //
//        TileLL const * tile = m_tileset->GetTile(tile_id);

//        // get tile texture
//        osg::ref_ptr<osg::Texture2D> texture;
//        double s_start;
//        double s_delta;
//        double t_start;
//        double t_delta;

//        getTileTexture(texture,
//                       s_start,
//                       s_delta,
//                       t_start,
//                       t_delta);



//        osg::ref_ptr<osg::Group> gp = createTileGm(tile_id);

//        std::string desc =
//                std::to_string(tile->level) +
//                ":" +
//                std::to_string(tile->x) +
//                "," +
//                std::to_string(tile->y);

//        auto gp_text = createTileTextGm(tile,desc);
//        gp_text->setName("text");
//        gp->addChild(gp_text);

////        GeoBounds const tile_bounds(tile->min_lon,
////                                    tile->max_lon,
////                                    tile->min_lat,
////                                    tile->max_lat);
////        auto gp = BuildGeoBoundsNode("",
////                                     tile_bounds,
////                                     K_COLOR_TABLE[tile->level],
////                                     360.0/16.0,
////                                     tile->level);

//        m_list_sg_tiles.emplace(tile_id,gp.get());
//        m_gp_tiles->addChild(gp);
//    }
}

/*
void DataSetTilesLL::Update(osg::Camera const * cam)
{
    std::vector<TileLL::Id> list_tiles_add;
    std::vector<TileLL::Id> list_tiles_upd;
    std::vector<TileLL::Id> list_tiles_rem;
    m_tileset->UpdateTileSet(cam,
                             list_tiles_add,
                             list_tiles_upd,
                             list_tiles_rem);

    for(auto tile_id : list_tiles_rem) {
        osg::Group * gp_tile = m_list_sg_tiles.find(tile_id)->second;
        m_gp_tiles->removeChild(gp_tile);

        m_list_sg_tiles.erase(tile_id);
    }

    for(auto tile_id : list_tiles_add) {
        TileLL const * tile = m_tileset->GetTile(tile_id);
        osg::ref_ptr<osg::Group> gp = createTileGm(tile_id);

        std::string desc =
                std::to_string(tile->level) +
                ":" +
                std::to_string(tile->x) +
                "," +
                std::to_string(tile->y);

        auto gp_text = createTileTextGm(tile,desc);
        gp_text->setName("text");
        gp->addChild(gp_text);

//        GeoBounds const tile_bounds(tile->min_lon,
//                                    tile->max_lon,
//                                    tile->min_lat,
//                                    tile->max_lat);
//        auto gp = BuildGeoBoundsNode("",
//                                     tile_bounds,
//                                     K_COLOR_TABLE[tile->level],
//                                     360.0/16.0,
//                                     tile->level);

        m_list_sg_tiles.emplace(tile_id,gp.get());
        m_gp_tiles->addChild(gp);
    }

    // update tile px values

//    for(auto it = m_list_sg_tiles.begin();
//        it != m_list_sg_tiles.end(); ++it)
//    {
//        osg::Group * gp = it->second;
//        for(size_t i=0; i < gp->getNumChildren(); i++) {
//            osg::Group * child = static_cast<osg::Group*>(gp->getChild(i));
//            if(child->getName() == "text") {
//                TileSetLL::Tile const * tile = m_tileset->GetTile(it->first);
//                std::string s = std::to_string(tile->tile_px_res);
//                osg::Geode * gd = static_cast<osg::Geode*>(child->getChild(0));
//                osgText::Text * txt = static_cast<osgText::Text*>(gd->getDrawable(0));
//                txt->setText(s);

//            }
//        }
//    }

//    // debug
//    for(size_t i=0; i < m_gp_debug->getNumChildren(); i++) {
//        std::string const name =
//                m_gp_debug->getChild(i)->getName();

//        if(name == "debug0") {
//            m_gp_debug->removeChild(i);
//            i--;
//        }
//    }

//    TileSetLLByPixelArea * tileset_ptr =
//            static_cast<TileSetLLByPixelArea*>(m_tileset.get());

//    GeoBounds const &debug0 = tileset_ptr->m_debug0;

//    if(debug0.minLon == 0.0 &&
//       debug0.maxLon == 0.0 &&
//       debug0.minLat == 0.0 &&
//       debug0.maxLat == 0.0)
//    {
//        return;
//    }

//    auto gp0 = BuildGeoBoundsSurfaceNode(
//                "debug0",tileset_ptr->m_debug0,osg::Vec4(1,1,1,1),20,true,0,0);
//    m_gp_debug->addChild(gp0);

//    auto gp1 = BuildGeoBoundsSurfaceNode(
//                "debug0",tileset_ptr->m_debug1,osg::Vec4(1,0,0,1),21,true,0,0);
//    m_gp_debug->addChild(gp1);
}
*/

osg::ref_ptr<osg::Group> DataSetTilesLL::createTileGm(TileLL::Id tile_id)
{
    TileLL const * tile = m_tileset->GetTile(tile_id);

//    uint32_t surf_divs = 256/K_LIST_TWO_EXP[tile->level];
//    surf_divs = std::min(surf_divs,static_cast<uint32_t>(32));

//    uint32_t const lon_segments =
//            std::max(static_cast<uint32_t>(surf_divs),
//                     static_cast<uint32_t>(1));

//    uint32_t const lat_segments =
//            std::max(static_cast<uint32_t>(lon_segments/2),
//                     static_cast<uint32_t>(1));

    double const min_angle_degs = 360.0/32.0;
    uint32_t lon_segments = std::max((tile->bounds.maxLon-tile->bounds.minLon)/min_angle_degs,1.0);
    uint32_t lat_segments = std::max((tile->bounds.maxLat-tile->bounds.minLat)/min_angle_degs,1.0);

    std::vector<osg::Vec3d> list_vx;
    std::vector<osg::Vec2d> list_tx;
    std::vector<uint16_t> list_ix;
    BuildEarthSurface(tile->bounds.minLon,
                      tile->bounds.maxLon,
                      tile->bounds.minLat,
                      tile->bounds.maxLat,
                      lon_segments,
                      lat_segments,
                      list_vx,
                      list_tx,
                      list_ix);

    osg::ref_ptr<osg::Vec3dArray> vx_array = new osg::Vec3dArray;
    vx_array->reserve(list_vx.size());
    for(auto const &vx : list_vx) {
        vx_array->push_back(vx);
    }

    osg::ref_ptr<osg::Vec2dArray> tx_array = new osg::Vec2dArray;
    tx_array->reserve(list_tx.size());
    for(auto const &tx : list_tx) {
        tx_array->push_back(tx);
    }

    osg::ref_ptr<osg::Vec4Array>  cx_array = new osg::Vec4Array;
    cx_array->push_back(K_COLOR_TABLE[tile->level]);

    osg::ref_ptr<osg::DrawElementsUShort> ix_array =
            new osg::DrawElementsUShort(GL_TRIANGLES);
    ix_array->reserve(list_ix.size());
    for(auto ix : list_ix) {
        ix_array->push_back(ix);
    }

    osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
    gm->setVertexArray(vx_array);
    gm->setTexCoordArray(0,tx_array,osg::Array::BIND_PER_VERTEX);
    gm->setColorArray(cx_array,osg::Array::BIND_OVERALL);
    gm->addPrimitiveSet(ix_array);

    osg::ref_ptr<osg::Geode> gd = new osg::Geode;
    gd->addDrawable(gm);
    gd->getOrCreateStateSet()->setRenderBinDetails(
                -101+tile->level,"RenderBin");
    gd->getOrCreateStateSet()->setMode(
                GL_CULL_FACE,
                osg::StateAttribute::ON |
                osg::StateAttribute::OVERRIDE);

    // texture
//    gd->getOrCreateStateSet()->setTextureAttributeAndModes(
//                0,m_list_tile_level_tex[tile->level]);

    // polygon mode
    gd->getOrCreateStateSet()->setAttributeAndModes(
                m_poly_mode,
                osg::StateAttribute::ON);

    osg::ref_ptr<osg::Group> gp = new osg::Group;
    gp->addChild(gd);

    return gp;
}

osg::ref_ptr<osg::Group>
DataSetTilesLL::createTileTextGm(TileLL const * tile,
                                 std::string const &text)
{
    // center
    LLA lla_mid;
    lla_mid.lon = (tile->bounds.minLon+tile->bounds.maxLon)*0.5;
    lla_mid.lat = (tile->bounds.minLat+tile->bounds.maxLat)*0.5;
    lla_mid.alt = 0.0;
    osg::Vec3d ecef_mid = ConvLLAToECEF(lla_mid);

    // height
    LLA lla_corner(tile->bounds.minLon,tile->bounds.minLat,0.0);
    osg::Vec3d ecef_corner = ConvLLAToECEF(lla_corner);
    double const height = (ecef_mid-ecef_corner).length() / 5.0;

    return BuildTextNode("",text,K_COLOR_TABLE[tile->level],ecef_mid,height);
//    return BuildTextNode("",text,K_COLOR_TABLE[0],ecef_mid,height);
}
