TEMPLATE = lib
CONFIG -= qt
CONFIG += plugin # hack to avoid sonames
TARGET = osmscout

# path that contains autogen'd files and fixes to
# build libosmscout with qmake
PATH_LIBOSMSCOUT_EXTRA = /home/preet/Dev/projects/libosmscout_dev/libosmscout
PATH_LIBOSMSCOUT_CORE = /home/preet/Dev/projects/libosmscout-exp/libosmscout


# ======================================================= #
# autogen
# * environment specific files that automake generates
# ======================================================= #
INCLUDEPATH += $${PATH_LIBOSMSCOUT_EXTRA}/include
#INCLUDEPATH += $${PATH_LIBOSMSCOUT_EXTRA}/src

HEADERS += \
$${PATH_LIBOSMSCOUT_EXTRA}/include/osmscout/private/Config.h \
$${PATH_LIBOSMSCOUT_EXTRA}/include/osmscout/CoreFeatures.h


# ======================================================= #
# fixes
# * compatibility fix with qmake that requires renaming
#   some of the source and header files
# * (util/Parser->util/ParserUtil)
# ======================================================= #
HEADERS += \
$${PATH_LIBOSMSCOUT_EXTRA}/include/osmscout/TypeConfig.h \
$${PATH_LIBOSMSCOUT_EXTRA}/include/osmscout/util/ParserUtil.h

SOURCES += \
$${PATH_LIBOSMSCOUT_EXTRA}/src/osmscout/util/ParserUtil.cpp


# ======================================================= #
# boost/unordered_set,unordered_map
# * for access to unordered_set and unordered_map even
#   if c++11 isn't supported on the target
# * including HashSet.h and HashMap.h before the versions
#   included in libosmscout will supercede them
# ======================================================= #
PATH_BOOST = /home/preet/Dev/env/sys/boost-1.55
DEFINES += USE_BOOST
INCLUDEPATH += $${PATH_BOOST}

HEADERS += \
$${PATH_LIBOSMSCOUT_EXTRA}/include/osmscout/util/HashSet.h \
$${PATH_LIBOSMSCOUT_EXTRA}/include/osmscout/util/HashMap.h


# ======================================================= #
# symbol visibility
# * restricts symbol visibility to classes defined
#   with the OSMSCOUT_API macro -> smaller lib file size
# ======================================================= #
QMAKE_CXXFLAGS += -fvisibility=hidden -fvisibility-inlines-hidden
QMAKE_CXXFLAGS += -DOSMSCOUTDLL  # comment out to export all symbols

# ======================================================= #
# core lib
# find include -name \*.h -printf %p' \\'\\n
# find src -name \*.cpp -printf %p' \\'\\n
# ======================================================= #
INCLUDEPATH += $${PATH_LIBOSMSCOUT_CORE}/include
#INCLUDEPATH += $${PATH_LIBOSMSCOUT_CORE}/src

HEADERS += \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/RoutingProfile.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/Route.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/TurnRestriction.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/DataFile.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/AreaWayIndex.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/util/HashMap.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/util/Reference.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/util/Breaker.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/util/Projection.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/util/Magnification.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/util/NumberSet.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/util/StopClock.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/util/Color.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/util/Number.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/util/Progress.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/util/FileWriter.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/util/HashSet.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/util/String.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/util/FileScanner.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/util/Cache.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/util/NodeUseMap.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/util/File.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/util/Geometry.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/util/Transformation.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/Location.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/Database.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/TypeSet.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/OptimizeAreasLowZoom.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/GeoCoord.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/Intersection.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/AreaDataFile.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/RouteNode.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/NodeDataFile.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/Types.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/AreaAreaIndex.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/WayDataFile.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/Point.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/DebugDatabase.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/WaterIndex.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/Node.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/SRTM.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/system/Math.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/system/Assert.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/system/Types.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/system/SSEMathPublic.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/ost/Parser.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/ost/Scanner.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/Coord.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/Router.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/Pixel.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/RouteData.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/NumericIndex.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/private/CoreImportExport.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/Way.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/GroundTile.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/ObjectRef.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/TypeConfigLoader.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/CoordDataFile.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/AreaNodeIndex.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/Tag.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/OptimizeWaysLowZoom.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/Path.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/AttributeAccess.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/Area.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/LocationIndex.h \
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/RoutePostprocessor.h

SOURCES += \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/Database.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/Intersection.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/util/FileWriter.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/util/HashSet.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/util/Projection.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/util/Reference.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/util/Magnification.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/util/StopClock.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/util/NodeUseMap.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/util/FileScanner.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/util/Breaker.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/util/Cache.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/util/Geometry.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/util/NumberSet.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/util/String.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/util/HashMap.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/util/Transformation.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/util/Number.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/util/Color.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/util/Progress.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/util/File.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/Node.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/TypeConfigLoader.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/Router.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/Route.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/RoutingProfile.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/Coord.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/Pixel.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/Path.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/Point.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/GeoCoord.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/SRTM.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/CoordDataFile.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/TypeConfig.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/AttributeAccess.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/Area.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/ObjectRef.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/WaterIndex.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/AreaNodeIndex.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/LocationIndex.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/ost/Scanner.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/ost/Parser.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/OptimizeAreasLowZoom.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/RoutePostprocessor.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/AreaAreaIndex.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/DebugDatabase.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/TurnRestriction.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/Way.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/Types.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/TypeSet.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/GroundTile.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/Location.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/AreaWayIndex.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/RouteData.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/NumericIndex.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/Tag.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/OptimizeWaysLowZoom.cpp \
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/RouteNode.cpp


# ======================================================= #
# sse2 ext (safe to include even if you dont want SSE2 and
# --disable-see2 was passed to autogen)
# ======================================================= #
#HEADERS += $${PATH_LIBOSMSCOUT_CORE}/include/osmscout/system/SSEMath.h
#SOURCES += $${PATH_LIBOSMSCOUT_CORE}/src/osmscout/system/SSEMath.cpp


# ======================================================= #
# marisa (comment out if --disable-marisa was passed to
# autogen and you dont want to build with libmarisa!)
# ======================================================= #
CONFIG+=use_marisa
use_marisa {
    DEFINES += OSMSCOUT_HAVE_LIB_MARISA
    PATH_MARISA=/home/preet/Dev/env/sys/marisa
    INCLUDEPATH += $${PATH_MARISA}/include
    LIBS += -L$${PATH_MARISA}/lib -lmarisa

    HEADERS += $${PATH_LIBOSMSCOUT_CORE}/include/osmscout/TextSearchIndex.h
    SOURCES += $${PATH_LIBOSMSCOUT_CORE}/src/osmscout/TextSearchIndex.cpp
}


# ======================================================= #
# install
# ======================================================= #
PATH_INSTALL = /home/preet/Dev/env/sys/libosmscout

target.path = $${PATH_INSTALL}/lib

header_files.files = $${PATH_LIBOSMSCOUT_CORE}/include/osmscout/*
header_files.path = $${PATH_INSTALL}/include/osmscout

# note: even though include/private/Config.h gets copied over
#       its not used and is harmless
more_header_files.files = $${PATH_LIBOSMSCOUT_EXTRA}/include/osmscout/*
more_header_files.path = $${PATH_INSTALL}/include/osmscout

INSTALLS += target header_files more_header_files
