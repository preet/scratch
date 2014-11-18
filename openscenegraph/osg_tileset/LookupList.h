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

#ifndef SCRATCH_LOOKUP_LIST_H
#define SCRATCH_LOOKUP_LIST_H

#include <list>
#include <map>
#include <unordered_map>

namespace scratch
{
    template<typename K,
             typename V,
             template<typename...> class map_type>
    class LookupList
    {
    private:
        typedef typename std::list<std::pair<K,V>>::iterator list_it;

        std::list<std::pair<K,V>> m_list;
        std::map<K,list_it> m_lkup;

    public:
        LookupList()
        {
            // empty
        }

        // Iterators
        list_it begin()
        {
            return m_list.begin();
        }

        list_it end()
        {
            return m_list.end();
        }

        list_it last()
        {
            auto it = m_list.end();
            if(m_list.size() > 0) {
                std::advance(it,-1);
            }
            return it;
        }

        // Capacity
        bool empty() const
        {
            return m_list.empty();
        }

        size_t size() const
        {
            return m_list.size();
        }

        // Element access
        std::pair<K,V> & front()
        {
            return m_list.front();
        }

        std::pair<K,V> & back()
        {
            return m_list.back();
        }

        // Modifiers
        list_it insert(list_it position,std::pair<K,V> const &val)
        {
            auto lkup_it = m_lkup.find(val.first);
            if(lkup_it != m_lkup.end()) {
                // already exists
                return lkup_it->second;
            }

            list_it inserted = m_list.insert(position,val);
            m_lkup.insert(std::make_pair(val.first,inserted));

            return inserted;
        }

        list_it insert(list_it position,std::pair<K,V> &&val)
        {
            auto lkup_it = m_lkup.find(val.first);
            if(lkup_it != m_lkup.end()) {
                // already exists
                return lkup_it->second;
            }

            auto key = val.first;
            list_it inserted = m_list.insert(position,std::move(val));
            m_lkup.insert(std::make_pair(key,inserted));

            return inserted;
        }

        list_it erase(list_it position)
        {
            m_lkup.erase(position->first);
            return m_list.erase(position);
        }

        list_it erase(K const &key)
        {
            auto lkup_it = m_lkup.find(key);
            if(lkup_it == m_lkup.end()) {
                return m_list.end();
            }

            return m_list.erase(lkup_it->second);
        }

        void move(list_it from, list_it to)
        {
            m_list.splice(to,m_list,from);
        }

        void trim(size_t size)
        {
            while(m_list.size() > size) {
                m_lkup.erase(m_list.back().first);
                m_list.pop_back();
            }
        }

        void trim(list_it position, size_t max_size=0)
        {
            auto it = last();

            while(m_list.size() > max_size)
            {
                if(it != position) {
                    std::advance(it,-1);
                    m_lkup.erase(m_list.back().first);
                    m_list.pop_back();
                }
                else {
                    m_lkup.erase(m_list.back().first);
                    m_list.pop_back();
                    break;
                }
            }
        }

        void clear()
        {
            m_lkup.clear();
            m_list.clear();
        }

        // Operations
        list_it find(K const &key)
        {
            auto lkup_it = m_lkup.find(key);
            if(lkup_it == m_lkup.end()) {
                return m_list.end();
            }

            return (lkup_it->second);
        }
    };
} // scratch

#endif // SCRATCH_LOOKUP_LIST_H
