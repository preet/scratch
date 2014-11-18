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

#ifndef SCRATCH_LOOKUP_QUEUE_H
#define SCRATCH_LOOKUP_QUEUE_H

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

        K m_null_key;
        V m_null_value;

        size_t const m_capacity;
        std::list<std::pair<K,V>> m_list;
        std::map<K,list_it> m_lkup;

    public:

    };


} // scratch

#endif // SCRATCH_LOOKUP_QUEUE_H
