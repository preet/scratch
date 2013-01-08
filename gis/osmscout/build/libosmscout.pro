#libosmscout
OSMSINCDIR = /home/preet/Dev/res/libosmscout/include/osmscout
OSMSLIBDIR = /home/preet/Dev/res/libosmscout/src/osmscout
INCLUDEPATH += /home/preet/Dev/res/libosmscout/include

HEADERS += \
    osmscout/private/Config.h \
    osmscout/util/HashSet.h \
    osmscout/util/HashMap.h \
    osmscout/CoreFeatures.h

HEADERS += \
    $${OSMSINCDIR}/ost/Scanner.h \
    $${OSMSINCDIR}/ost/Parser.h \
    $${OSMSINCDIR}/private/CoreImportExport.h \
    $${OSMSINCDIR}/system/Types.h \
    $${OSMSINCDIR}/util/Breaker.h \
    $${OSMSINCDIR}/util/Cache.h \
    $${OSMSINCDIR}/util/Color.h \
    $${OSMSINCDIR}/util/File.h \
    $${OSMSINCDIR}/util/FileScanner.h \
    $${OSMSINCDIR}/util/FileWriter.h \
    $${OSMSINCDIR}/util/Geometry.h \
    $${OSMSINCDIR}/util/Number.h \
    $${OSMSINCDIR}/util/NumberSet.h \
    $${OSMSINCDIR}/util/Progress.h \
    $${OSMSINCDIR}/util/Projection.h \
    $${OSMSINCDIR}/util/Reference.h \
    $${OSMSINCDIR}/util/StopClock.h \
    $${OSMSINCDIR}/util/String.h \
    $${OSMSINCDIR}/util/Transformation.h \
    $${OSMSINCDIR}/AdminRegion.h \
    $${OSMSINCDIR}/AreaAreaIndex.h \
    $${OSMSINCDIR}/AreaNodeIndex.h \
    $${OSMSINCDIR}/AreaWayIndex.h \
    $${OSMSINCDIR}/CityStreetIndex.h \
    $${OSMSINCDIR}/Database.h \
    $${OSMSINCDIR}/DataFile.h \
    $${OSMSINCDIR}/GroundTile.h \
    $${OSMSINCDIR}/Location.h \
    $${OSMSINCDIR}/NodeDataFile.h \
    $${OSMSINCDIR}/Node.h \
    $${OSMSINCDIR}/NumericIndex.h \
    $${OSMSINCDIR}/ObjectRef.h \
    $${OSMSINCDIR}/OptimizeLowZoom.h \
    $${OSMSINCDIR}/Path.h \
    $${OSMSINCDIR}/Point.h \
    $${OSMSINCDIR}/RelationDataFile.h \
    $${OSMSINCDIR}/Relation.h \
    $${OSMSINCDIR}/RouteData.h \
    $${OSMSINCDIR}/Route.h \
    $${OSMSINCDIR}/RouteNode.h \
    $${OSMSINCDIR}/RoutePostprocessor.h \
    $${OSMSINCDIR}/Router.h \
    $${OSMSINCDIR}/RoutingProfile.h \
    $${OSMSINCDIR}/SegmentAttributes.h \
    $${OSMSINCDIR}/Tag.h \
    $${OSMSINCDIR}/TurnRestriction.h \
    $${OSMSINCDIR}/TypeConfig.h \
    $${OSMSINCDIR}/TypeConfigLoader.h \
    $${OSMSINCDIR}/TypeSet.h \
    $${OSMSINCDIR}/Types.h \
    $${OSMSINCDIR}/WaterIndex.h \
    $${OSMSINCDIR}/WayDataFile.h \
    $${OSMSINCDIR}/Way.h

SOURCES += \
    $${OSMSLIBDIR}/ost/Parser.cpp \
    $${OSMSLIBDIR}/ost/Scanner.cpp \
    $${OSMSLIBDIR}/util/Breaker.cpp \
    $${OSMSLIBDIR}/util/Cache.cpp \
    $${OSMSLIBDIR}/util/Color.cpp \
    $${OSMSLIBDIR}/util/File.cpp \
    $${OSMSLIBDIR}/util/FileScanner.cpp \
    $${OSMSLIBDIR}/util/FileWriter.cpp \
    $${OSMSLIBDIR}/util/Geometry.cpp \
    $${OSMSLIBDIR}/util/HashMap.cpp \
    $${OSMSLIBDIR}/util/HashSet.cpp \
    $${OSMSLIBDIR}/util/Number.cpp \
    $${OSMSLIBDIR}/util/NumberSet.cpp \
    $${OSMSLIBDIR}/util/Progress.cpp \
    $${OSMSLIBDIR}/util/Projection.cpp \
    $${OSMSLIBDIR}/util/Reference.cpp \
    $${OSMSLIBDIR}/util/StopClock.cpp \
    $${OSMSLIBDIR}/util/String.cpp \
    $${OSMSLIBDIR}/util/Transformation.cpp \
    $${OSMSLIBDIR}/AdminRegion.cpp \
    $${OSMSLIBDIR}/AreaAreaIndex.cpp \
    $${OSMSLIBDIR}/AreaNodeIndex.cpp \
    $${OSMSLIBDIR}/AreaWayIndex.cpp \
    $${OSMSLIBDIR}/CityStreetIndex.cpp \
    $${OSMSLIBDIR}/Database.cpp \
    $${OSMSLIBDIR}/GroundTile.cpp \
    $${OSMSLIBDIR}/Location.cpp \
    $${OSMSLIBDIR}/Node.cpp \
    $${OSMSLIBDIR}/NumericIndex.cpp \
    $${OSMSLIBDIR}/ObjectRef.cpp \
    $${OSMSLIBDIR}/OptimizeLowZoom.cpp \
    $${OSMSLIBDIR}/Path.cpp \
    $${OSMSLIBDIR}/Point.cpp \
    $${OSMSLIBDIR}/Relation.cpp \
    $${OSMSLIBDIR}/Route.cpp \
    $${OSMSLIBDIR}/RouteData.cpp \
    $${OSMSLIBDIR}/RouteNode.cpp \
    $${OSMSLIBDIR}/RoutePostprocessor.cpp \
    $${OSMSLIBDIR}/Router.cpp \
    $${OSMSLIBDIR}/RoutingProfile.cpp \
    $${OSMSLIBDIR}/SegmentAttributes.cpp \
    $${OSMSLIBDIR}/Tag.cpp \
    $${OSMSLIBDIR}/TurnRestriction.cpp \
    $${OSMSLIBDIR}/TypeConfig.cpp \
    $${OSMSLIBDIR}/TypeConfigLoader.cpp \
    $${OSMSLIBDIR}/Types.cpp \
    $${OSMSLIBDIR}/TypeSet.cpp \
    $${OSMSLIBDIR}/WaterIndex.cpp \
    $${OSMSLIBDIR}/Way.cpp
