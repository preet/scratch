TEMPLATE = lib
CONFIG -= qt
CONFIG += plugin # hack to avoid sonames
TARGET = osmscout
INCLUDEPATH += include
INCLUDEPATH += src
INCLUDEPATH += /home/preet/Dev/env/sys/boost-1.53
DEFINES += USE_BOOST

# find include -name \*.h -printf %p' \\'\\n
HEADERS += \
include/osmscout/Way.h \
include/osmscout/ost/Parser.h \
include/osmscout/ost/Scanner.h \
include/osmscout/NumericIndex.h \
include/osmscout/AreaWayIndex.h \
include/osmscout/WaterIndex.h \
include/osmscout/RoutingProfile.h \
include/osmscout/WayDataFile.h \
include/osmscout/OptimizeWaysLowZoom.h \
include/osmscout/AreaDataFile.h \
include/osmscout/SRTM.h \
include/osmscout/CityStreetIndex.h \
include/osmscout/util/NodeUseMap.h \
include/osmscout/util/String.h \
include/osmscout/util/Breaker.h \
include/osmscout/util/Reference.h \
include/osmscout/util/FileScanner.h \
include/osmscout/util/Cache.h \
include/osmscout/util/Geometry.h \
include/osmscout/util/StopClock.h \
include/osmscout/util/NumberSet.h \
include/osmscout/util/HashMap.h \
include/osmscout/util/Progress.h \
include/osmscout/util/ParserUtil.h \
include/osmscout/util/Transformation.h \
include/osmscout/util/HashSet.h \
include/osmscout/util/Color.h \
include/osmscout/util/File.h \
include/osmscout/util/FileWriter.h \
include/osmscout/util/Projection.h \
include/osmscout/util/Magnification.h \
include/osmscout/util/Number.h \
include/osmscout/RoutePostprocessor.h \
include/osmscout/Tag.h \
include/osmscout/Location.h \
include/osmscout/Path.h \
include/osmscout/Pixel.h \
include/osmscout/Point.h \
include/osmscout/DebugDatabase.h \
include/osmscout/TypeConfig.h \
include/osmscout/private/Config.h \
include/osmscout/private/CoreImportExport.h \
include/osmscout/AreaAreaIndex.h \
include/osmscout/TurnRestriction.h \
include/osmscout/Route.h \
include/osmscout/RouteData.h \
include/osmscout/TypeConfigLoader.h \
include/osmscout/Coord.h \
include/osmscout/OptimizeAreasLowZoom.h \
include/osmscout/Router.h \
include/osmscout/ObjectRef.h \
include/osmscout/TypeSet.h \
include/osmscout/Database.h \
include/osmscout/CoreFeatures.h \
include/osmscout/AdminRegion.h \
include/osmscout/GeoCoord.h \
include/osmscout/system/Assert.h \
include/osmscout/system/Math.h \
include/osmscout/system/SSEMathPublic.h \
include/osmscout/system/SSEMath.h \
include/osmscout/system/Types.h \
include/osmscout/AreaNodeIndex.h \
include/osmscout/GroundTile.h \
include/osmscout/NodeDataFile.h \
include/osmscout/Node.h \
include/osmscout/Area.h \
include/osmscout/RouteNode.h \
include/osmscout/Types.h \
include/osmscout/DataFile.h

# in directory
# find src -name \*.cpp -printf %p' \\'\\n
SOURCES += \
src/osmscout/TypeConfigLoader.cpp \
src/osmscout/ObjectRef.cpp \
src/osmscout/AreaWayIndex.cpp \
src/osmscout/ost/Scanner.cpp \
src/osmscout/ost/Parser.cpp \
src/osmscout/WaterIndex.cpp \
src/osmscout/Pixel.cpp \
src/osmscout/RouteNode.cpp \
src/osmscout/SRTM.cpp \
src/osmscout/RouteData.cpp \
src/osmscout/GroundTile.cpp \
src/osmscout/GeoCoord.cpp \
src/osmscout/util/NumberSet.cpp \
src/osmscout/util/Cache.cpp \
src/osmscout/util/Projection.cpp \
src/osmscout/util/Reference.cpp \
src/osmscout/util/HashMap.cpp \
src/osmscout/util/HashSet.cpp \
src/osmscout/util/ParserUtil.cpp \
src/osmscout/util/Transformation.cpp \
src/osmscout/util/Geometry.cpp \
src/osmscout/util/Progress.cpp \
src/osmscout/util/Breaker.cpp \
src/osmscout/util/FileWriter.cpp \
src/osmscout/util/File.cpp \
src/osmscout/util/Number.cpp \
src/osmscout/util/FileScanner.cpp \
src/osmscout/util/StopClock.cpp \
src/osmscout/util/Color.cpp \
src/osmscout/util/Magnification.cpp \
src/osmscout/util/String.cpp \
src/osmscout/util/NodeUseMap.cpp \
src/osmscout/TypeSet.cpp \
src/osmscout/AreaNodeIndex.cpp \
src/osmscout/Way.cpp \
src/osmscout/Area.cpp \
src/osmscout/Database.cpp \
src/osmscout/Coord.cpp \
src/osmscout/TurnRestriction.cpp \
src/osmscout/RoutingProfile.cpp \
src/osmscout/AdminRegion.cpp \
src/osmscout/TypeConfig.cpp \
src/osmscout/AreaAreaIndex.cpp \
src/osmscout/Route.cpp \
src/osmscout/Path.cpp \
src/osmscout/OptimizeWaysLowZoom.cpp \
src/osmscout/Node.cpp \
src/osmscout/Point.cpp \
src/osmscout/system/SSEMath.cpp \
src/osmscout/Tag.cpp \
src/osmscout/CityStreetIndex.cpp \
src/osmscout/Router.cpp \
src/osmscout/RoutePostprocessor.cpp \
src/osmscout/OptimizeAreasLowZoom.cpp \
src/osmscout/Location.cpp \
src/osmscout/Types.cpp \
src/osmscout/DebugDatabase.cpp \
src/osmscout/NumericIndex.cpp

PATH_INSTALL = /home/preet/Dev/env/sys/libosmscout
target.path = $${PATH_INSTALL}/lib
header_files.files = include/osmscout/*
header_files.path = $${PATH_INSTALL}/include/osmscout

INSTALLS += header_files target


#depc
#find -name \*.h -printf %p' \\'\\n | sed "s|^\./||"
