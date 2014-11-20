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

#include <TileSetLL.h>

namespace scratch
{
    TileSetLL::TileSetLL(std::unique_ptr<TileDataSourceLL> tile_data_source,
                         std::unique_ptr<TileVisibilityLL> tile_visibility,
                         Options options) :
        m_tile_data_source(std::move(tile_data_source)),
        m_tile_visibility(std::move(tile_visibility)),
        m_opts(initOptions(options)),
        m_num_preload_data(initNumPreloadData()),
        m_max_view_data(initMaxViewData())
    {
        // debug
        std::cout << "m_opts.max_tile_data: " << m_opts.max_tile_data << std::endl;
        std::cout << "m_num_preload_data: " << m_num_preload_data << std::endl;
        std::cout << "m_max_view_data: " << m_max_view_data << std::endl;

        // Generate root tiles from the number of
        // root tiles in x and y
        auto const bounds = this->GetBounds();
        auto const num_root_tiles_x = this->GetNumRootTilesX();
        auto const num_root_tiles_y = this->GetNumRootTilesY();

        double const lon_width =
                (bounds.maxLon-bounds.minLon)/num_root_tiles_x;

        double const lat_width =
                (bounds.maxLat-bounds.minLat)/num_root_tiles_y;

        for(size_t y=0; y < num_root_tiles_y; y++) {
            for(size_t x=0; x < num_root_tiles_x; x++) {
                // Create bounds for this tile
                GeoBounds b(bounds.minLon,
                            bounds.minLon+(lon_width*(x+1)),
                            bounds.minLat,
                            bounds.minLat+(lat_width*(y+1)));

                // save
                m_list_root_tiles.emplace_back(new TileLL(b,x,y));
            }
        }


        // Preload the base textures
        m_preloaded_data_ready = false;

        m_list_level_is_preloaded.resize(m_opts.max_level,0);

        for(auto level : m_opts.list_preload_levels) {
            // mark as a preload level
            m_list_level_is_preloaded[level] = 1;

            // request data for all tiles in this level
            auto const tiles_in_x = ipow(2,level)*num_root_tiles_x;
            auto const tiles_in_y = ipow(2,level)*num_root_tiles_y;

            for(int64_t y=0; y < tiles_in_y; y++) {
                for(int64_t x=0; x < tiles_in_x; x++) {
                    // save request
                    auto const tile_id =
                            TileLL::GetIdFromLevelXY(level,x,y);

                    m_lkup_preloaded_data.emplace(
                                tile_id,
                                m_tile_data_source->RequestData(
                                    tile_id));
                }
            }
        }       
    }

    TileSetLL::~TileSetLL()
    {
        // empty
    }

    GeoBounds const & TileSetLL::GetBounds() const
    {
        return m_tile_data_source->GetBounds();
    }

    uint8_t TileSetLL::GetMaxLevel() const
    {
        return m_opts.max_level;
    }

    uint8_t TileSetLL::GetNumRootTilesX() const
    {
        return m_tile_data_source->GetNumRootTilesX();
    }

    uint8_t TileSetLL::GetNumRootTilesY() const
    {
        return m_tile_data_source->GetNumRootTilesY();
    }

    void TileSetLL::UpdateTileSet(osg::Camera const * cam,
                                  std::vector<TileLL::Id> &list_tile_id_add,
                                  std::vector<TileLL::Id> &list_tile_id_upd,
                                  std::vector<TileLL::Id> &list_tile_id_rem)
    {
        // Save new camera eye LLA
        osg::Vec3d eye,vpt,up;
        cam->getViewMatrixAsLookAt(eye,vpt,up);
        m_lla_cam_eye = ConvECEFToLLA(eye);

        // Ensure the base data has been loaded
        if(!m_preloaded_data_ready) {
            for(auto const &id_req : m_lkup_preloaded_data) {
                if(!id_req.second->IsFinished()) {
                    // We don't do anything until all of
                    // the base data is finished loading
                    return;
                }
            }
            m_preloaded_data_ready = true;
            std::cout << "#: [loaded base data]" << std::endl;
        }

        // Update tile visibility
        m_tile_visibility->Update(cam);

        // Build tile set
        std::vector<TileItem> list_tiles_new =
                buildTileSetRanked();

        // We need to sort by tile_id before we can
        // split into tiles added/removed
        std::sort(list_tiles_new.begin(),
                  list_tiles_new.end(),
                  TileSetLL::CompareTileItemIdIncreasing);

        // Create new and old id lists
        std::vector<TileLL::Id> list_tile_id_new;
        list_tile_id_new.reserve(list_tiles_new.size());
        for(auto const &item : list_tiles_new) {
            list_tile_id_new.push_back(item.id);
        }

        std::vector<TileLL::Id> list_tile_id_old;
        list_tile_id_old.reserve(m_list_tiles.size());
        for(auto const &item : m_list_tiles) {
            list_tile_id_old.push_back(item.id);
        }

        SplitSets(list_tile_id_new,
                  list_tile_id_old,
                  list_tile_id_add,
                  list_tile_id_rem,
                  list_tile_id_upd);

        // save new tile set
        std::swap(m_list_tiles,list_tiles_new);
    }

    void TileSetLL::quickTest()
    {
        //
        std::swap(m_list_tiles_prev,m_list_tiles_next);
        m_list_tiles_next = buildTileSetBFS();

        //
        std::vector<TileItem const *> list_tiles_old;
        list_tiles_old.reserve(m_list_tiles_prev.size());
        for(auto const &item : m_list_tiles_prev) {
            list_tiles_old.push_back(&item);
        }

        //
        std::vector<TileItem const *> list_tiles_new;
        list_tiles_new.reserve(m_list_tiles_next.size());
        for(auto const &item : m_list_tiles_next) {
            list_tiles_new.push_back(&item);
        }

        //
        std::sort(list_tiles_new.begin(),
                  list_tiles_new.end(),
                  TileSetLL::CompareTileItemPtrIdIncreasing);

        //
        std::vector<TileItem const *> list_tiles_add;
        std::vector<TileItem const *> list_tiles_upd;
        std::vector<TileItem const *> list_tiles_rem;

        SplitSets(list_tiles_new,
                  list_tiles_old,
                  list_tiles_add,
                  list_tiles_rem,
                  list_tiles_upd,
                  TileSetLL::CompareTileItemPtrIdIncreasing);

        //
        std::vector<TileLL::Id> list_tile_ids_rem;
        list_tile_ids_rem.reserve(list_tiles_rem.size());
        for(auto item_ptr : list_tiles_rem) {
            list_tile_ids_rem.push_back(item_ptr->id);
        }
    }

    TileSetLL::TileItem const * TileSetLL::GetTile(TileLL::Id tile_id) const
    {
        // expect m_list_tiles to be sorted
        // in increasing order (default)

        // TODO check if function or functor faster than lambda
        auto it = std::lower_bound(
                    m_list_tiles.begin(),
                    m_list_tiles.end(),
                    tile_id,
                    [](TileItem const &a, TileLL::Id b) {
                        return (a.id < b);
                    });

        if(it == m_list_tiles.end()) {
            return nullptr;
        }

        return (&(*it));
    }

    std::vector<TileSetLL::TileItem> TileSetLL::buildTileSetBFS()
    {
        // Build the tileset by doing a breadth first search
        // on all of the root tiles
        std::vector<TileMetaData*> queue_bfs;
        std::vector<TileItem> list_tile_items;

        // Mark the start of this update/tile traversal in
        // the view data LRU cache.
        auto it_mark_upd_start = m_ll_view_data.insert(
                    m_ll_view_data.begin(),
                    std::make_pair(TileLL::GetIdFromLevelXY(255,0,0),
                                   nullptr));

        // Enqueue all root tiles first, this ensures
        // a contiguous tileset
        for(auto & tile : m_list_root_tiles)
        {
            TileMetaData * meta = createMetaData(tile.get());

            // start with an empty quadtree
            destroyChildren(tile.get());

            // get visibility
            m_tile_visibility->GetVisibility(
                        meta->tile,
                        getData(meta->tile),
                        meta->is_visible,
                        meta->norm_error,
                        meta->closest_point);

            queue_bfs.push_back(meta);
        }

        // Create a list of tiles traversed in BFS order
        // according to tile visibility
        for(size_t i=0; i < queue_bfs.size(); i++)
        {
            TileMetaData * meta = queue_bfs[i];
            TileLL * tile = meta->tile;

            if((meta->norm_error > 1.0) &&
               (tile->level < m_opts.max_level) &&
               (queue_bfs.size()+4 <= m_opts.max_tile_data))
            {
                // Enqueue children for traversal
                createChildren(tile);

//                std::vector<TileMetaData*> list_children {
//                    createMetaData(tile->tile_LT.get()),
//                    createMetaData(tile->tile_LB.get()),
//                    createMetaData(tile->tile_RB.get()),
//                    createMetaData(tile->tile_RT.get())
//                };

                // NOTE/TODO: Creating child meta data with
                // @createChildrenMetaData(...) arranges children
                // in approximately increasing distance away from
                // @lla_cam_eye, but this doesnt make a huge visual
                // difference from the method above; maybe check
                // how much more expensive arranging is?

                std::vector<TileMetaData*> list_children =
                        createChildrenMetaData(tile,m_lla_cam_eye);

                for(auto &child : list_children) {
                    m_tile_visibility->GetVisibility(
                                child->tile,
                                getData(meta->tile),
                                child->is_visible,
                                child->norm_error,
                                child->closest_point);

                    queue_bfs.push_back(child);
                }
            }
        }

        // Create data requests for each tile
        for(auto r_it = queue_bfs.rbegin();
            r_it != queue_bfs.rend(); ++r_it)
        {
            TileMetaData * meta = (*r_it);
            meta->request = getOrCreateDataRequest(meta->tile,true);
            meta->ready = meta->request->IsFinished();
        }

        // Save tiles that are ready and have no children
        // or have children that are not yet ready
        for(auto meta : queue_bfs) {
            if(meta->ready) {
                bool save_this_tile=true;
                if(meta->tile->clip == TileLL::k_clip_ALL) {
                    // tile is ready and has children
                    TileMetaData * meta_LT = getMetaData(meta->tile->tile_LT.get());
                    TileMetaData * meta_LB = getMetaData(meta->tile->tile_LB.get());
                    TileMetaData * meta_RB = getMetaData(meta->tile->tile_RB.get());
                    TileMetaData * meta_RT = getMetaData(meta->tile->tile_RT.get());

                    bool children_ready =
                            meta_LT->ready &&
                            meta_LB->ready &&
                            meta_RB->ready &&
                            meta_RT->ready;

                    if(children_ready) {
                        save_this_tile = false;
                    }
                    else {
                        // explicitly mark all children not ready
                        meta_LT->ready = false;
                        meta_LB->ready = false;
                        meta_RB->ready = false;
                        meta_RT->ready = false;
                    }
                }

                if(save_this_tile) {
                    TileItem item;
                    item.id = meta->tile->id;
                    item.tile = meta->tile;
                    item.data = meta->request->GetData().get();
                    list_tile_items.push_back(item);
                }
            }
        }

        // Trim the tile data cache according to the cache_size_hint.
        // Only data inserted before the data requests made before
        // @it_mark_upd_start was inserted will be trimmed (even if
        // the total size of the cache exceeds the size hint
        m_ll_view_data.trim(it_mark_upd_start,m_opts.cache_size_hint);

        // Remove the update start marker if its still in the list
        m_ll_view_data.erase(TileLL::GetIdFromLevelXY(255,0,0));

        // Trim against the strict tile data limit, this will remove
        // tail elements until size == @max_view_data
        m_ll_view_data.trim(m_max_view_data);


//        std::cout << "." << std::endl;
//        std::cout << "bfs (" << queue_bfs.size() << "): ";
//        for(auto tmd : queue_bfs) {
//            std::string s = (tmd->ready) ? "F" : "W";
//            std::string c = "R";
//            if(tmd->tile->parent) {
//                if(tmd->tile->parent->clip == TileLL::k_clip_NONE) {
//                    c = "N";
//                }
//                else {
//                    c = "A";
//                }
//            }
//            std::cout << int(tmd->tile->level) << c << ",";
//        }
//        std::cout << std::endl;


//        std::cout << "lks (" << m_ll_view_data.size() << "): ";
//        for(auto it = m_ll_view_data.begin();
//            it != m_ll_view_data.end(); ++it)
//        {
//            uint8_t level;
//            uint32_t x;
//            uint32_t y;
//            TileLL::GetLevelXYFromId(it->first,level,x,y);
//            std::cout << int(level) << "_,";
//        }
//        std::cout << std::endl;


//        std::cout << "tms (" << list_tile_items.size() << "): ";
//        for(auto & item : list_tile_items) {
//            std::cout << int(item.tile->level) << "_,";
//        }
//        std::cout << std::endl;


        return list_tile_items;
    }

    std::vector<TileSetLL::TileItem> TileSetLL::buildTileSetRanked()
    {
        // In this method, tiles don't necessarily have
        // a unique associated TileData.

        // The entire tileset is built, and TileData with
        // the highest ranking is requested first

        // For tiles where the corresponding TileData isn't
        // immediately available, TileData is substituted
        // in by sampling from TileData that is available

        std::vector<TileItem> list_tile_items;


        // Mark the start of this update/tile traversal in
        // the view data LRU cache.
        auto it_mark_upd_start = m_ll_view_data.insert(
                    m_ll_view_data.begin(),
                    std::make_pair(TileLL::GetIdFromLevelXY(255,0,0),
                                   nullptr));

        std::vector<TileMetaData*> queue_bfs;

        // Root tiles
        for(auto & tile : m_list_root_tiles) {
            TileMetaData * meta = createMetaData(tile.get());

            // All root tiles must always be available
            meta->request = getOrCreateDataRequest(tile.get(),true);
            if(!meta->request->IsFinished()) {
                return list_tile_items;
            }

            // start with an empty quadtree
            destroyChildren(tile.get());

            // get visibility
            m_tile_visibility->GetVisibility(
                        meta->tile,
                        getData(meta->tile),
                        meta->is_visible,
                        meta->norm_error,
                        meta->closest_point);

            queue_bfs.push_back(meta);
        }

        // Create a list of tiles according to tile visibility
        for(size_t i=0; i < queue_bfs.size(); i++)
        {
            TileMetaData * meta = queue_bfs[i];
            TileLL * tile = meta->tile;

            if((meta->norm_error > 1.0) &&
               (tile->level < m_opts.max_level) &&
               (queue_bfs.size()+4 <= m_opts.max_tile_data))
            {
                // Enqueue children for traversal
                createChildren(tile);

                std::vector<TileMetaData*> list_children {
                    createMetaData(tile->tile_LT.get()),
                    createMetaData(tile->tile_LB.get()),
                    createMetaData(tile->tile_RB.get()),
                    createMetaData(tile->tile_RT.get())
                };

                for(auto &child : list_children) {
                    m_tile_visibility->GetVisibility(
                                child->tile,
                                getData(meta->tile),
                                child->is_visible,
                                child->norm_error,
                                child->closest_point);

                    queue_bfs.push_back(child);
                }
            }
        }

        // Split into root and ranked tiles
        auto it_tmd = queue_bfs.begin();
        std::advance(it_tmd,m_list_root_tiles.size());

        std::vector<TileMetaData*> list_root_tiles;
        list_root_tiles.reserve(m_list_root_tiles.size());
        list_root_tiles.insert(list_root_tiles.end(),
                               queue_bfs.begin(),
                               it_tmd);

        std::vector<TileMetaData*> list_ranked_tiles;
        list_ranked_tiles.reserve(queue_bfs.size());
        list_ranked_tiles.insert(list_ranked_tiles.end(),
                                 it_tmd,
                                 queue_bfs.end());

        // Sort non root tiles with a ranking function
        // that factors in tile level and distance
        std::sort(list_ranked_tiles.begin(),
                  list_ranked_tiles.end(),
                  [](TileMetaData const * a, TileMetaData const * b) {
                        // TODO
                        // Should tiles with clip==k_clip_ALL have
                        // a rank of 0?

                        double rank_a =
                                double(a->tile->level)*
                                double(a->is_visible)* // should work, false==0.0,true==1.0
                                a->norm_error;

                        double rank_b =
                                double(b->tile->level)*
                                double(b->is_visible)*
                                b->norm_error;

                        return (rank_a > rank_b);
                    }
                );

        // Request as much data as there is space: @max_view_data
        // and use substitution for everything else
        size_t const num_requests =
                std::min(list_ranked_tiles.size(),
                         static_cast<size_t>(m_max_view_data));

        for(size_t i=0; i < num_requests; i++) {
            TileMetaData * meta = list_ranked_tiles[i];
            meta->request = getOrCreateDataRequest(meta->tile,true);
        }


        //
        for(auto &meta : list_root_tiles) {
            if(meta->tile->clip == TileLL::k_clip_NONE) {
                TileItem item;
                item.id = meta->tile->id;
                item.tile = meta->tile;
                item.data = meta->request->GetData().get();
                list_tile_items.push_back(item);
            }
        }

        for(auto &meta : list_ranked_tiles) {
            if(meta->tile->clip == TileLL::k_clip_ALL) {
                continue;
            }

            // determine sample if required
            TileLL * sample_tile = meta->tile;
            TileMetaData * sample_meta;

            bool search_data = true;
            while(search_data) {
                sample_meta = getMetaData(sample_tile);
                if(sample_meta->request &&
                   sample_meta->request->IsFinished()) {
                    search_data = false;
                }
                else {
                    sample_tile = sample_tile->parent;
                }
            }

            // save
            TileItem item;
            item.id = meta->tile->id;
            item.tile = meta->tile;

            if(sample_tile != meta->tile) {
                item.uses_sample = true;
                item.sample_id = sample_tile->id;
                item.sample_tile = sample_tile;
                item.data = sample_meta->request->GetData().get();
            }
            else {
                item.data = meta->request->GetData().get();
            }

            list_tile_items.push_back(item);
        }

        // trim cache
        m_ll_view_data.trim(it_mark_upd_start,m_opts.cache_size_hint);
        m_ll_view_data.erase(TileLL::GetIdFromLevelXY(255,0,0));
        m_ll_view_data.trim(m_max_view_data);

        //
        std::cout << "#: sz ll view data: " << m_ll_view_data.size() << std::endl;

        return list_tile_items;
    }

    TileDataSourceLL::Data const *
    TileSetLL::getData(TileLL const * tile)
    {
        TileDataSourceLL::Request const * req =
                getDataRequest(tile,false);

        if(req && req->IsFinished()) {
            return req->GetData().get();
        }

        return nullptr;
    }


    TileDataSourceLL::Request const *
    TileSetLL::getDataRequest(TileLL const * tile,
                              bool reuse)
    {
        // check preloaded tile data
        if(m_list_level_is_preloaded[tile->level]) {
            auto it = m_lkup_preloaded_data.find(tile->id);
            return it->second.get();
        }

        // check dynamic tile data
        auto it = m_ll_view_data.find(tile->id);
        if(it == m_ll_view_data.end()) {
            return nullptr;
        }

        if(reuse) {
            // move to the front of the lru
            m_ll_view_data.move(it,m_ll_view_data.begin());
        }

        return it->second.get();
    }

    TileDataSourceLL::Request const *
    TileSetLL::getOrCreateDataRequest(TileLL const * tile,
                                      bool reuse,
                                      bool * existed)
    {
        // check preloaded tile data
        if(m_list_level_is_preloaded[tile->level]) {
            auto it = m_lkup_preloaded_data.find(tile->id);
            if(existed) {
                *existed = true;
            }
            return it->second.get();
        }

        // check dynamic tile data
        auto it = m_ll_view_data.find(tile->id);
        if(it == m_ll_view_data.end()) {
            // create request if it doesn't exist
            it = m_ll_view_data.insert(
                        m_ll_view_data.begin(),
                        std::make_pair(
                            tile->id,
                            m_tile_data_source->RequestData(tile->id)));

            if(existed) {
                *existed = false;
            }
        }
        else {
            if(reuse) {
                // move to the front of the lru
                m_ll_view_data.move(it,m_ll_view_data.begin());
            }
            if(existed) {
                *existed = true;
            }
        }

        return it->second.get();
    }

//    std::vector<TileSetLL::TileMetaData>
//    TileSetLL::getOrCreateChildDataRequests(TileLL * tile,
//                                            bool & child_data_ready)
//    {
//        // create children first if required
//        createChildren(tile);

//        std::vector<TileMetaData> list_children {
//            TileMetaData(tile->tile_LT.get()),
//            TileMetaData(tile->tile_LB.get()),
//            TileMetaData(tile->tile_RB.get()),
//            TileMetaData(tile->tile_RT.get())
//        };

//        // Check if the data for each child tile is ready
//        child_data_ready = true;
//        for(auto & meta_child : list_children) {
//            // create the request if it doesnt already exist
//            meta_child.request =
//                    getOrCreateDataRequest(meta_child.tile);

//            if(!meta_child.request->IsFinished()) {
//                child_data_ready = child_data_ready && false;
//            }
//        }

//        // If the child data is ready, calculate its
//        // visibility as well
//        if(child_data_ready) {
//            for(auto & meta_child : list_children) {
//                m_tile_visibility->GetVisibility(
//                            meta_child.tile,
//                            meta_child.is_visible,
//                            meta_child.exceeds_err);
//            }
//        }

//        return list_children;
//    }

    void TileSetLL::createChildren(TileLL *tile) const
    {
        if(tile->clip == TileLL::k_clip_NONE) {
            uint32_t const x = tile->x*2;
            uint32_t const y = tile->y*2;
            tile->tile_LT.reset(new TileLL(tile,x,y+1));
            tile->tile_LB.reset(new TileLL(tile,x,y));
            tile->tile_RB.reset(new TileLL(tile,x+1,y));
            tile->tile_RT.reset(new TileLL(tile,x+1,y+1));
            tile->clip = TileLL::k_clip_ALL;
        }
    }

    void TileSetLL::destroyChildren(TileLL *tile) const
    {
        if(tile->clip == TileLL::k_clip_ALL) {
            tile->tile_LT = nullptr;
            tile->tile_LB = nullptr;
            tile->tile_RB = nullptr;
            tile->tile_RT = nullptr;
            tile->clip = TileLL::k_clip_NONE;
        }
    }

    std::vector<TileSetLL::TileMetaData*>
    TileSetLL::createChildrenMetaData(TileLL const * tile,
                                      LLA const &lla) const
    {
        double const mid_lon =
                (tile->bounds.minLon+tile->bounds.maxLon)*0.5;

        double const mid_lat =
                (tile->bounds.minLat+tile->bounds.maxLat)*0.5;

        // Arrange the tiles by comparing @lla to mid_lla.

        // Example:
        // If @lla is to the NW of mid_lla, the LT tile is
        // closest, the LB and RT follow in any order and
        // the RB tile is furthest away.

        std::vector<TileMetaData*> list_children;
        list_children.reserve(4);

        if(lla.lon < mid_lon) { // west
            if(lla.lat < mid_lat) { // south
                // SW,NW,SE,NE
                list_children.push_back(createMetaData(tile->tile_LB.get()));
                list_children.push_back(createMetaData(tile->tile_LT.get()));
                list_children.push_back(createMetaData(tile->tile_RB.get()));
                list_children.push_back(createMetaData(tile->tile_RT.get()));
            }
            else { // north
                // NW,SW,NE,SE
                list_children.push_back(createMetaData(tile->tile_LT.get()));
                list_children.push_back(createMetaData(tile->tile_LB.get()));
                list_children.push_back(createMetaData(tile->tile_RT.get()));
                list_children.push_back(createMetaData(tile->tile_RB.get()));
            }
        }
        else { // east
            if(lla.lat < mid_lat) { // south
                // SE,NE,SW,NW
                list_children.push_back(createMetaData(tile->tile_RB.get()));
                list_children.push_back(createMetaData(tile->tile_RT.get()));
                list_children.push_back(createMetaData(tile->tile_LB.get()));
                list_children.push_back(createMetaData(tile->tile_LT.get()));
            }
            else { // north
                // NE,SE,NW,SW
                list_children.push_back(createMetaData(tile->tile_RT.get()));
                list_children.push_back(createMetaData(tile->tile_RB.get()));
                list_children.push_back(createMetaData(tile->tile_LT.get()));
                list_children.push_back(createMetaData(tile->tile_LB.get()));
            }
        }

        return list_children;
    }

    TileSetLL::Options
    TileSetLL::initOptions(Options opts) const
    {
        // max_tile_data must be less than the max possible
        // value of its data type to allow for safe
        // comparisons (ie, if(x > max_tile_data))
        assert(opts.max_tile_data < std::numeric_limits<uint64_t>::max());

        // Ensure min max level is less than or equal
        // the data source's max level
        if(opts.max_level > m_tile_data_source->GetMaxLevel()) {
            opts.max_level = m_tile_data_source->GetMaxLevel();
        }

        // The preload level list must be sorted
        // in increasing order
        std::sort(opts.list_preload_levels.begin(),
                  opts.list_preload_levels.end());

        // max_tile_data must be equal to or larger than
        // the total number of preload tiles if a valid
        // limit is set
        uint64_t num_base_data = 0;
        for(auto level : opts.list_preload_levels) {
            num_base_data += ipow(4,level);
        }

        if(num_base_data > opts.max_tile_data) {
            opts.max_tile_data = num_base_data;
        }

        // check upsampling hint
        if(opts.upsample_hint &&
           (!m_tile_data_source->CanBeSampled())) {
            opts.upsample_hint = false;
        }

        return opts;
    }

    uint64_t TileSetLL::initNumPreloadData() const
    {
        uint64_t num_preload_data=0;
        for(auto level : m_opts.list_preload_levels) {
            num_preload_data += ipow(4,level);
        }

        return num_preload_data;
    }

    uint64_t TileSetLL::initMaxViewData() const
    {
        uint64_t num_view_data =
                m_opts.max_tile_data-
                m_num_preload_data;

        return num_view_data;
    }

} // scratch
