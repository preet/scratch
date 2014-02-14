#ifndef OSMSCOUT_UTIL_HASHMAP_H
#define OSMSCOUT_UTIL_HASHMAP_H

#include <osmscout/CoreFeatures.h>

#if defined(USE_BOOST)
#undef OSMSCOUT_HAVE_UNORDERED_MAP
#endif

#if defined(OSMSCOUT_HAVE_UNORDERED_MAP)
#include <unordered_map>
#define OSMSCOUT_HASHMAP std::unordered_map
#define OSMSCOUT_HASHMAP_HAS_RESERVE 1
#elif defined(USE_BOOST)
#pragma message "Using boost::unordered_set for OSMSCOUT_HASHMAP"
#include <boost/unordered_map.hpp>
#define OSMSCOUT_HASHMAP boost::unordered::unordered_map
#define OSMSCOUT_HAVE_UNORDERED_MAP 1
#define OSMSCOUT_HASHMAP_HAS_RESERVE 1
#else
#include <map>
#define OSMSCOUT_HASHMAP std::map
#undef OSMSCOUT_HASHMAP_HAS_RESERVE
#endif


#endif
