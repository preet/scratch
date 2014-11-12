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

#ifndef SCRATCH_MISC_UTILS_H
#define SCRATCH_MISC_UTILS_H

#include <memory>
#include <list>
#include <vector>
#include <algorithm>
#include <limits>
#include <cassert>
#include <map>
#include <functional>



namespace scratch
{
    template <typename T>
    T clamp(const T& n, const T& lower, const T& upper) {
      return std::max(lower, std::min(n, upper));
    }

    int64_t ipow(int64_t base,uint64_t exp);


    template <typename T>
    void SplitSets(std::vector<T> const &sorted_list_a,
                   std::vector<T> const &sorted_list_b,
                   std::vector<T> &list_diff_a,
                   std::vector<T> &list_diff_b,
                   std::vector<T> &list_xsec)
    {
        list_diff_a.resize(sorted_list_a.size());
        list_diff_b.resize(sorted_list_b.size());
        list_xsec.resize(std::min(sorted_list_a.size(),
                                  sorted_list_b.size()));

        typename std::vector<T>::iterator it;

        it = std::set_difference(sorted_list_a.begin(),
                                 sorted_list_a.end(),
                                 sorted_list_b.begin(),
                                 sorted_list_b.end(),
                                 list_diff_a.begin());
        list_diff_a.resize(it-list_diff_a.begin());

        it = std::set_difference(sorted_list_b.begin(),
                                 sorted_list_b.end(),
                                 sorted_list_a.begin(),
                                 sorted_list_a.end(),
                                 list_diff_b.begin());
        list_diff_b.resize(it-list_diff_b.begin());

        it = std::set_intersection(sorted_list_a.begin(),
                                   sorted_list_a.end(),
                                   sorted_list_b.begin(),
                                   sorted_list_b.end(),
                                   list_xsec.begin());
        list_xsec.resize(it-list_xsec.begin());
    }

    template <typename T, typename Comparator>
    void SplitSets(std::vector<T> const &sorted_list_a,
                   std::vector<T> const &sorted_list_b,
                   std::vector<T> &list_diff_a,
                   std::vector<T> &list_diff_b,
                   std::vector<T> &list_xsec,
                   Comparator compare)
    {
        list_diff_a.resize(sorted_list_a.size());
        list_diff_b.resize(sorted_list_b.size());
        list_xsec.resize(std::min(sorted_list_a.size(),
                                  sorted_list_b.size()));

        typename std::vector<T>::iterator it;

        it = std::set_difference(sorted_list_a.begin(),
                                 sorted_list_a.end(),
                                 sorted_list_b.begin(),
                                 sorted_list_b.end(),
                                 list_diff_a.begin(),
                                 compare);
        list_diff_a.resize(it-list_diff_a.begin());

        it = std::set_difference(sorted_list_b.begin(),
                                 sorted_list_b.end(),
                                 sorted_list_a.begin(),
                                 sorted_list_a.end(),
                                 list_diff_b.begin(),
                                 compare);
        list_diff_b.resize(it-list_diff_b.begin());

        it = std::set_intersection(sorted_list_a.begin(),
                                   sorted_list_a.end(),
                                   sorted_list_b.begin(),
                                   sorted_list_b.end(),
                                   list_xsec.begin(),
                                   compare);
        list_xsec.resize(it-list_xsec.begin());
    }


    template<typename K,
             typename V,
             template<typename...> class map_type>
    class LRUCacheMap
    {
    private:
        size_t const m_capacity;
        std::list<K> m_lru;
        std::map<K,std::pair<V,typename std::list<K>::iterator>>  m_lkup;
        V m_null_value;

        // TODO allow specifying load factor

    public:
        LRUCacheMap(size_t capacity) :
            m_capacity(capacity)
        {
            // empty
        }

        // copy insert
        // @check: check if the key already exists, can be
        //         set to false for a possible speed up if
        //         you know for sure the key is new
        void insert(K const &key, V const &val, bool check=true)
        {
            if(check && (m_lkup.count(key) > 0)) {
                return;
            }

            // make room by erasing the least recently
            // used element if required
            if(size() == capacity()) {
                auto it = m_lkup.find(m_lru.back());
                m_lkup.erase(it);
                m_lru.pop_back();
            }

            // add to lru
            m_lru.push_front(key);

            // add to lookup
            m_lkup.insert(std::make_pair(
                              key,std::make_pair(
                                  val,
                                  m_lru.begin())));
        }

        // move insert
        // @check: check if the key already exists, can be
        //         set to false for a possible speed up if
        //         you know for sure the key is new
        void insert(K const &key, V && val, bool check=true)
        {
            if(check && (m_lkup.count(key) > 0)) {
                return;
            }

            // make room by erasing the least recently
            // used element if required
            if(size() == capacity()) {
                auto it = m_lkup.find(m_lru.back());
                m_lkup.erase(it);
                m_lru.pop_back();
            }

            // add to lru
            m_lru.push_front(key);

            // add to lookup
            m_lkup.insert(std::make_pair(
                              key,std::make_pair(
                                  std::move(val),
                                  m_lru.begin())));
        }

        void reuse(K const &key)
        {
            // move key to the front of the lru
            auto lkup_it = m_lkup.find(key);
            if(lkup_it == m_lkup.end()) {
                return;
            }

            auto lru_it = lkup_it->second.second;
            m_lru.splice(m_lru.begin(),m_lru,lru_it);
        }

        void erase(K const &key)
        {
            auto lkup_it = m_lkup.find(key);
            if(lkup_it == m_lkup.end()) {
                return;
            }

            // remove from lru and lkup
            auto lru_it = lkup_it->second.second;
            m_lru.erase(lru_it);
            m_lkup.erase(lkup_it);
        }

        void clear()
        {
            m_lkup.clear();
            m_lru.clear();
        }

        bool exists(K const &key) const
        {
            return (m_lkup.find(key) != m_lkup.end());
        }

        V & get(K const &key,bool reuse,bool &ok)
        {
            auto lkup_it = m_lkup.find(key);
            if(lkup_it == m_lkup.end()) {
                ok = false;
                return m_null_value;
            }

            if(reuse) {
                // move key to the front of the lru
                auto lru_it = lkup_it->second.second;
                m_lru.splice(m_lru.begin(),m_lru,lru_it);
            }

            ok = true;
            return lkup_it->second.first;
        }

        std::vector<K> get_keys() const
        {
            std::vector<K> list_keys;
            list_keys.reserve(m_lkup.size());

            for(auto it : m_lkup) {
                list_keys.push_back(it->first);
            }

            return list_keys;
        }

        size_t size() const
        {
            return m_lkup.size();
        }

        size_t capacity() const
        {
            return m_capacity;
        }
    };

//    template<typename K,
//             typename V,
//             template<typename,typename> class map_type>
//    class LRUCacheMap
//    {
//    public:

//        LRUCacheMap(size_t capacity) :
//            m_capacity(capacity)
//        {
//            // empty
//        }

//        void insert(K const &key, V const &val)
//        {
//            // make room by erasing the lru element if required
//            if(size() == capacity()) {
//                auto it = m_lkup.find(m_lru.back());
//                m_lkup.erase(it);
//                m_lru.pop_back();
//            }

//            // add to lru
//            m_lru.push_front(key);

//            // add to lookup

//            // check if insert successful before adding to lru

//            m_lkup.insert(std::make_pair(
//                              key,std::make_pair(
//                                  val,m_lru.begin())));
//        }


//        // * (insert) insert key,val if the key doesn't exist
//        // * (use) push key to the front of the lru if it already exists
//        // * (get) return val
//        V & insert_use_get(K const &key, V const &val)
//        {
//            // TODO
//            // THIS IS WRONG, WHAT IF THE KEY ALREADY EXISTS?
//            // WE HAVE TO INSERT FIRST, THEN REMOVE IF NECESSARY

//            // make room by erasing the lru element if required
//            if(size() == capacity()) {
//                auto it = m_lkup.find(m_lru.back());
//                m_lkup.erase(it);
//                m_lru.pop_back();
//            }

//            // Attempt an insert (the lru iterator is just
//            // a placeholder for now)
//            auto ins_it = m_lkup.insert(std::make_pair(
//                                            key,std::make_pair(
//                                                val,m_lru.end())));

//            if(ins_it.second) {
//                // insert was successful, push key to the
//                // front of the lru and update its iterator
//                m_lru.push_front(key);
//                ins_it.second->second.second = m_lru.begin();
//            }
//            else {
//                // key already existed, move it to the front
//                auto lru_it = ins_it.second->second.second;
//                m_lru.splice(m_lru.begin(),m_lru,lru_it);
//            }

//            return (ins_it.second->second.first);
//        }

//        void reuse(K const &key)
//        {
//            // move key to the front of the lru
//            auto lkup_it = m_lkup.find(key);
//            if(lkup_it == m_lkup.end()) {
//                return;
//            }

//            auto lru_it = lkup_it->second.second;
//            m_lru.splice(m_lru.begin(),m_lru,lru_it);
//        }

//        void erase(K const &key)
//        {
//            auto lkup_it = m_lkup.find(key);
//            if(lkup_it == m_lkup.end()) {
//                return;
//            }

//            // remove from lru and lkup
//            auto lru_it = lkup_it->second.second;
//            m_lru.erase(lru_it);
//            m_lkup.erase(lkup_it);
//        }

//        void clear()
//        {
//            m_lkup.clear();
//            m_lru.clear();
//        }

//        bool exists(K const &key) const
//        {
//            return (m_lkup.find(key) != m_lkup.end());
//        }

//        V & get(K const &key, bool &ok) const
//        {
//            auto lkup_it = m_lkup.find(key);
//            if(lkup_it == m_lkup.end()) {
//                ok = false;
//                return m_null_value;
//            }
//            ok = true;
//            return lkup_it->second.first;
//        }

//        std::vector<K> get_keys() const
//        {
//            std::vector<K> list_keys;
//            list_keys.reserve(m_lkup.size());

//            for(auto it : m_lkup) {
//                list_keys.push_back(it->first);
//            }

//            return list_keys;
//        }

//        size_t size() const
//        {
//            return m_lkup.size();
//        }

//        size_t capacity() const
//        {
//            return m_capacity;
//        }

//    private:
//        size_t m_capacity;
//        std::list<K> m_lru;
//        map_type<K,std::pair<V,std::list<K>::iterator>>  m_lkup;
//        V m_null_value;
//    };
}



#endif // SCRATCH_MISC_UTILS_H
