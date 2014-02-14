#ifndef OSMSCOUT_UTIL_HASHSET_H
#define OSMSCOUT_UTIL_HASHSET_H

#include <osmscout/CoreFeatures.h>

#if defined(USE_BOOST)
#undef OSMSCOUT_HAVE_UNORDERED_SET
#endif

#if defined(OSMSCOUT_HAVE_UNORDERED_SET)
#include <unordered_set>
#define OSMSCOUT_HASHSET std::unordered_set
#define OSMSCOUT_HASHSET_HAS_RESERVE 1

#elif defined(USE_BOOST)
#pragma message "Using boost::unordered_set for OSMSCOUT_HASHSET"
#include <boost/unordered_set.hpp>
#define OSMSCOUT_HASHSET boost::unordered::unordered_set
#define OSMSCOUT_HAVE_UNORDERED_SET 1
#define OSMSCOUT_HASHSET_HAS_RESERVE 1

#else
#include <set>
#define OSMSCOUT_HASHSET std::set
#undef OSMSCOUT_HASHSET_HAS_RESERVE
#endif


#endif
