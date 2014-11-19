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
#include <iostream>

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
                   std::vector<T> &list_diff_a, // in a, not in b
                   std::vector<T> &list_diff_b, // in b, not in a
                   std::vector<T> &list_xsec,   // in both
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

    // ============================================================= //


    template<typename K,
             typename V,
             template<typename...> class map_type>
    class LRUCacheMap
    {
    private:
        typedef typename std::list<K>::iterator key_list_it;

        std::list<K> m_lru;
        map_type<K,std::pair<V,key_list_it>>  m_lkup;

        K m_null_key;
        V m_null_value;

        std::function<void(K&,V&)> m_deleter;

        key_list_it m_it_mark; // TODO initialize to tail

        // TODO allow specifying load factor for map

        void reuse_element(key_list_it lru_it)
        {
            if(lru_it == m_it_mark) {
                ++m_it_mark;
            }
            m_lru.splice(m_lru.begin(),m_lru,lru_it);
        }

    public:
        LRUCacheMap() :
            m_it_mark(m_lru.end())
          // Basically init with unlimited capacity,
          // its expected the user will manually trim
        {
            // Define a deleter that doesn't do anything
            m_deleter = [](K &key, V &val) {
                (void)key;
                (void)val;
            };
        }

        // copy insert
        // @check: check if the key already exists, can be
        //         set to false for a possible speed up if
        //         you know for sure the key is new
        V & insert(K const &key, V const &val,
                   bool check=true, bool reuse=false)
        {
            if(check) {
                auto lkup_it = m_lkup.find(key);
                if(lkup_it != m_lkup.end()) {
                    if(reuse) {
                        reuse_element(lkup_it->second.second);
                    }
                    return lkup_it->second.first;
                }
            }

            // make room if required
//            trim_against_capacity();

            // add to lru
            m_lru.push_front(key);

            // add to lookup
            auto lkup_it = m_lkup.insert(
                        std::make_pair(
                            key,std::make_pair(
                                val,m_lru.begin()))).first;

            return lkup_it->second.first;
        }

        // move insert
        // @check: check if the key already exists, can be
        //         set to false for a possible speed up if
        //         you know for sure the key is new
        V & insert(K const &key, V && val,
                   bool check=true, bool reuse=false)
        {
            if(check) {
                auto lkup_it = m_lkup.find(key);
                if(lkup_it != m_lkup.end()) {
                    if(reuse) {
                        reuse_element(lkup_it->second.second);
                    }
                    return lkup_it->second.first;
                }
            }

            // make room if required
//            trim_against_capacity();

            // add to lru
            m_lru.push_front(key);

            // add to lookup
            auto lkup_it = m_lkup.insert(
                        std::make_pair(
                         key,std::make_pair(
                             std::move(val),
                             m_lru.begin()))).first;

            return lkup_it->second.first;
        }

        V & get(K const &key,bool reuse,bool &ok)
        {
            auto lkup_it = m_lkup.find(key);
            if(lkup_it == m_lkup.end()) {
                ok = false;
                return m_null_value;
            }

            if(reuse) {
                reuse_element(lkup_it->second.second);
            }

            ok = true;
            return lkup_it->second.first;
        }

        bool mark_exists() const
        {
            return (m_it_mark != m_lru.end());
        }

        void mark_head()
        {
            m_it_mark = m_lru.begin();
        }

        bool mark_key(K const &key)
        {
            auto lkup_it = m_lkup.find(key);
            if(lkup_it == m_lkup.end()) {
                return false;
            }

            m_it_mark = lkup_it->second.second;
            return true;
        }

        void erase(K const &key)
        {
            auto lkup_it = m_lkup.find(key);
            if(lkup_it == m_lkup.end()) {
                return;
            }

            // remove from lru and lkup
            auto lru_it = lkup_it->second.second;
            if(lru_it = m_it_mark) {
                ++m_it_mark;
            }
            m_lru.erase(lru_it);
            m_lkup.erase(lkup_it);
        }

        void trim_against_mark(size_t max_size=0)
        {
            while((m_lru.size() > max_size) &&
                  (m_it_mark != m_lru.end()))
            {
                auto it_lru = m_lru.end();
                std::advance(it_lru,-1);
                if(m_it_mark == it_lru) {
                    m_it_mark = m_lru.end();
                }

                m_lkup.erase(m_lkup.find(m_lru.back()));
                m_lru.pop_back();
            }
        }

        void trim_against_capacity(size_t capacity)
        {
            while(size() > capacity) {
                if(m_it_mark != m_lru.end()) {
                    auto it_lru = m_lru.end();
                    std::advance(it_lru,-1);
                    if(m_it_mark == it_lru) {
                        m_it_mark = m_lru.end();
                    }
                }

                auto it_lkup = m_lkup.find(m_lru.back());
                m_lkup.erase(it_lkup);
                m_lru.pop_back();
            }
        }

        void clear()
        {
            m_lkup.clear();
            m_lru.clear();
            m_it_mark=m_lru.end();
        }

        bool exists(K const &key) const
        {
            return (m_lkup.find(key) != m_lkup.end());
        }

        std::vector<K> get_keys() const
        {
            std::vector<K> list_keys;
            list_keys.reserve(m_lru.size());

            for(auto const &it : m_lru) {
                list_keys.push_back(it);
            }

//            std::cout << "//" << m_lru.size() << std::endl;

            return list_keys;
        }

        size_t size() const
        {
            return m_lkup.size();
        }
    };


    // ============================================================= //
}



#endif // SCRATCH_MISC_UTILS_H
