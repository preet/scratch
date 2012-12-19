#ifndef OSMSCOUT_UTIL_HASHMAP_H
#define OSMSCOUT_UTIL_HASHMAP_H

#include <osmscout/CoreFeatures.h>

#if defined(USE_BOOST)
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
