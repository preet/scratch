TEMPLATE = app
CONFIG -= qt
#CONFIG += app
TARGET = import

# ======================================================= #
# import testing
# ======================================================= #

SOURCES += import/osmscout_json_import.cpp

LIBS += -pthread
LIBS += -lprotobuf -lz
LIBS += -ljansson

# ======================================================= #
# core lib
# ======================================================= #

# auto generated config and custom files
INCLUDEPATH += libosmscout/include
INCLUDEPATH += libosmscout/src

HEADERS += \
libosmscout/include/osmscout/private/Config.h \
libosmscout/include/osmscout/CoreFeatures.h \
libosmscout/include/osmscout/TypeConfig.h \
libosmscout/include/osmscout/util/ParserUtil.h

SOURCES += \
libosmscout/src/osmscout/util/ParserUtil.cpp


# default project files
PATH_LIBOSMSCOUT_CORE = /home/preet/Dev/projects/libosmscout-exp/libosmscout
INCLUDEPATH += $${PATH_LIBOSMSCOUT_CORE}
INCLUDEPATH += $${PATH_LIBOSMSCOUT_CORE}/include
INCLUDEPATH += $${PATH_LIBOSMSCOUT_CORE}/src

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
$${PATH_LIBOSMSCOUT_CORE}/include/osmscout/system/SSEMath.h \
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
$${PATH_LIBOSMSCOUT_CORE}/src/osmscout/system/SSEMath.cpp \
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
# import lib
# ======================================================= #

DEFINES += HAVE_LIB_PROTOBUF
DEFINES += HAVE_LIB_ZLIB

# auto generated config files
INCLUDEPATH += libosmscout-import/include
HEADERS += \
libosmscout-import/include/osmscout/private/Config.h \
libosmscout-import/include/osmscout/ImportFeatures.h

# protocol buffer
INCLUDEPATH += libosmscout-import/include/osmscout/import/pbf
HEADERS += \
libosmscout-import/include/osmscout/import/pbf/fileformat.pb.h \
libosmscout-import/include/osmscout/import/pbf/osmformat.pb.h

SOURCES += \
libosmscout-import/src/osmscout/import/pbf/fileformat.pb.cc \
libosmscout-import/src/osmscout/import/pbf/osmformat.pb.cc

# core lib
PATH_LIBOSMSCOUT_IMPORT = /home/preet/Dev/projects/libosmscout-exp/libosmscout-import
INCLUDEPATH += $${PATH_LIBOSMSCOUT_IMPORT}
INCLUDEPATH += $${PATH_LIBOSMSCOUT_IMPORT}/include
INCLUDEPATH += $${PATH_LIBOSMSCOUT_IMPORT}/src

HEADERS += \
$${PATH_LIBOSMSCOUT_IMPORT}/include/osmscout/private/ImportImportExport.h \
$${PATH_LIBOSMSCOUT_IMPORT}/include/osmscout/import/PreprocessOSM.h \
$${PATH_LIBOSMSCOUT_IMPORT}/include/osmscout/import/GenAreaAreaIndex.h \
$${PATH_LIBOSMSCOUT_IMPORT}/include/osmscout/import/RawCoastline.h \
$${PATH_LIBOSMSCOUT_IMPORT}/include/osmscout/import/Preprocess.h \
$${PATH_LIBOSMSCOUT_IMPORT}/include/osmscout/import/RawNode.h \
$${PATH_LIBOSMSCOUT_IMPORT}/include/osmscout/import/GenWayWayDat.h \
$${PATH_LIBOSMSCOUT_IMPORT}/include/osmscout/import/GenRouteDat.h \
$${PATH_LIBOSMSCOUT_IMPORT}/include/osmscout/import/GenLocationIndex.h \
$${PATH_LIBOSMSCOUT_IMPORT}/include/osmscout/import/GenWaterIndex.h \
$${PATH_LIBOSMSCOUT_IMPORT}/include/osmscout/import/GenWayAreaDat.h \
$${PATH_LIBOSMSCOUT_IMPORT}/include/osmscout/import/GenTypeDat.h \
$${PATH_LIBOSMSCOUT_IMPORT}/include/osmscout/import/Import.h \
$${PATH_LIBOSMSCOUT_IMPORT}/include/osmscout/import/RawWay.h \
$${PATH_LIBOSMSCOUT_IMPORT}/include/osmscout/import/GenOptimizeAreaWayIds.h \
$${PATH_LIBOSMSCOUT_IMPORT}/include/osmscout/import/GenTurnRestrictionDat.h \
$${PATH_LIBOSMSCOUT_IMPORT}/include/osmscout/import/GenNodeDat.h \
$${PATH_LIBOSMSCOUT_IMPORT}/include/osmscout/import/GenRelAreaDat.h \
$${PATH_LIBOSMSCOUT_IMPORT}/include/osmscout/import/GenAreaNodeIndex.h \
$${PATH_LIBOSMSCOUT_IMPORT}/include/osmscout/import/SortAreaDat.h \
$${PATH_LIBOSMSCOUT_IMPORT}/include/osmscout/import/SortWayDat.h \
$${PATH_LIBOSMSCOUT_IMPORT}/include/osmscout/import/SortDat.h \
$${PATH_LIBOSMSCOUT_IMPORT}/include/osmscout/import/PreprocessPBF.h \
$${PATH_LIBOSMSCOUT_IMPORT}/include/osmscout/import/GenOptimizeAreasLowZoom.h \
$${PATH_LIBOSMSCOUT_IMPORT}/include/osmscout/import/RawRelation.h \
$${PATH_LIBOSMSCOUT_IMPORT}/include/osmscout/import/GenNumericIndex.h \
$${PATH_LIBOSMSCOUT_IMPORT}/include/osmscout/import/SortNodeDat.h \
$${PATH_LIBOSMSCOUT_IMPORT}/include/osmscout/import/GenAreaWayIndex.h \
$${PATH_LIBOSMSCOUT_IMPORT}/include/osmscout/import/GenOptimizeWaysLowZoom.h

SOURCES += \
$${PATH_LIBOSMSCOUT_IMPORT}/src/osmscout/import/RawRelation.cpp \
$${PATH_LIBOSMSCOUT_IMPORT}/src/osmscout/import/SortWayDat.cpp \
$${PATH_LIBOSMSCOUT_IMPORT}/src/osmscout/import/SortDat.cpp \
$${PATH_LIBOSMSCOUT_IMPORT}/src/osmscout/import/Preprocess.cpp \
$${PATH_LIBOSMSCOUT_IMPORT}/src/osmscout/import/RawWay.cpp \
$${PATH_LIBOSMSCOUT_IMPORT}/src/osmscout/import/GenWayAreaDat.cpp \
$${PATH_LIBOSMSCOUT_IMPORT}/src/osmscout/import/SortAreaDat.cpp \
$${PATH_LIBOSMSCOUT_IMPORT}/src/osmscout/import/GenTypeDat.cpp \
$${PATH_LIBOSMSCOUT_IMPORT}/src/osmscout/import/GenTurnRestrictionDat.cpp \
$${PATH_LIBOSMSCOUT_IMPORT}/src/osmscout/import/GenOptimizeAreasLowZoom.cpp \
$${PATH_LIBOSMSCOUT_IMPORT}/src/osmscout/import/GenWaterIndex.cpp \
$${PATH_LIBOSMSCOUT_IMPORT}/src/osmscout/import/RawNode.cpp \
$${PATH_LIBOSMSCOUT_IMPORT}/src/osmscout/import/GenAreaAreaIndex.cpp \
$${PATH_LIBOSMSCOUT_IMPORT}/src/osmscout/import/Import.cpp \
$${PATH_LIBOSMSCOUT_IMPORT}/src/osmscout/import/GenNodeDat.cpp \
$${PATH_LIBOSMSCOUT_IMPORT}/src/osmscout/import/GenAreaWayIndex.cpp \
$${PATH_LIBOSMSCOUT_IMPORT}/src/osmscout/import/GenAreaNodeIndex.cpp \
$${PATH_LIBOSMSCOUT_IMPORT}/src/osmscout/import/GenLocationIndex.cpp \
$${PATH_LIBOSMSCOUT_IMPORT}/src/osmscout/import/GenWayWayDat.cpp \
$${PATH_LIBOSMSCOUT_IMPORT}/src/osmscout/import/GenOptimizeWaysLowZoom.cpp \
$${PATH_LIBOSMSCOUT_IMPORT}/src/osmscout/import/RawCoastline.cpp \
$${PATH_LIBOSMSCOUT_IMPORT}/src/osmscout/import/PreprocessPBF.cpp \
$${PATH_LIBOSMSCOUT_IMPORT}/src/osmscout/import/SortNodeDat.cpp \
$${PATH_LIBOSMSCOUT_IMPORT}/src/osmscout/import/GenRelAreaDat.cpp \
$${PATH_LIBOSMSCOUT_IMPORT}/src/osmscout/import/PreprocessOSM.cpp \
$${PATH_LIBOSMSCOUT_IMPORT}/src/osmscout/import/GenNumericIndex.cpp \
$${PATH_LIBOSMSCOUT_IMPORT}/src/osmscout/import/GenOptimizeAreaWayIds.cpp \
$${PATH_LIBOSMSCOUT_IMPORT}/src/osmscout/import/GenRouteDat.cpp

# new files
#HEADERS += \
#$${PATH_LIBOSMSCOUT_IMPORT}/include/osmscout/import/UpdateNodeDat.h

#SOURCES += \
#$${PATH_LIBOSMSCOUT_IMPORT}/src/osmscout/import/UpdateNodeDat.cpp

#libosmscout-import needs libxml2
INCLUDEPATH += /usr/include/libxml2
LIBS += -lxml2 -lz -lm
