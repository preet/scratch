#ifndef OSMSCOUT_UTIL_HASHSET_H
#define OSMSCOUT_UTIL_HASHSET_H

#include <osmscout/CoreFeatures.h>

#if defined(USE_BOOST)
#include <boost/unordered_set.hpp>
#define OSMSCOUT_HASHSET boost::unordered::unordered_set
#define OSMSCOUT_HAVE_UNORDERED_SET 1
#define OSMSCOUT_HASHSET_HAS_RESERVE 1
#else 															
#include <map>
#define OSMSCOUT_HASHSET std::set
#undef OSMSCOUT_HASHSET_HAS_RESERVE
#endif


#endif
