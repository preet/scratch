##########################
# START CONFIG FOR QT
# directory where qt and bb libs are
# (depends on device or simulator build config)
QT += opengl declarative
QTDIR = temp

device   {
    QTDIR = /home/preet/Documents/qnx/qt-4.8-arm
}
simulator {
    QTDIR = /home/preet/Documents/qnx/qt-4.8-x86
}

# define qt lib files and install path
dep_libs.path = $${OUT_PWD}/deploy/lib
dep_libs.files += $${QTDIR}/lib/*.so.4

# define bb lib file and install path
dep_plugins.path = $${OUT_PWD}/deploy/plugins/platforms
dep_plugins.files += $${QTDIR}/plugins/platforms/libblackberry.so

INSTALLS += dep_libs dep_plugins

# END CONFIG FOR QT
##########################

##########################
# START CONFIG FOR OSMSCOUT

USE_BOOST   {
   message(INFO: Using Boost)
   DEFINES += "USE_BOOST=1"
   INCLUDEPATH += /home/preet/Documents/boost-1.50
}

HEADERS += \
    osmscout/ost/Scanner.h \
    osmscout/ost/Parser.h \
    osmscout/private/Math.h \
    osmscout/private/CoreImportExport.h \
    osmscout/private/Config.h \
    osmscout/system/Types.h \
    osmscout/util/Transformation.h \
    osmscout/util/String.h \
    osmscout/util/StopClock.h \
    osmscout/util/Reference.h \
    osmscout/util/Projection.h \
    osmscout/util/Progress.h \
    osmscout/util/NumberSet.h \
    osmscout/util/Number.h \
    osmscout/util/HashSet.h \
    osmscout/util/HashMap.h \
    osmscout/util/Geometry.h \
    osmscout/util/FileWriter.h \
    osmscout/util/FileScanner.h \
    osmscout/util/File.h \
    osmscout/util/Color.h \
    osmscout/util/Cache.h \
    osmscout/util/Breaker.h \
    osmscout/WayDataFile.h \
    osmscout/Way.h \
    osmscout/WaterIndex.h \
    osmscout/Types.h \
    osmscout/TypeSet.h \
    osmscout/TypeConfigLoader.h \
    osmscout/TypeConfig.h \
    osmscout/TurnRestriction.h \
    osmscout/Tag.h \
    osmscout/SegmentAttributes.h \
    osmscout/RoutingProfile.h \
    osmscout/Router.h \
    osmscout/RoutePostprocessor.h \
    osmscout/RouteNode.h \
    osmscout/RouteData.h \
    osmscout/Route.h \
    osmscout/RelationDataFile.h \
    osmscout/Relation.h \
    osmscout/Point.h \
    osmscout/Path.h \
    osmscout/OptimizeLowZoom.h \
    osmscout/ObjectRef.h \
    osmscout/NumericIndex.h \
    osmscout/NodeDataFile.h \
    osmscout/Node.h \
    osmscout/Location.h \
    osmscout/GroundTile.h \
    osmscout/Database.h \
    osmscout/DataFile.h \
    osmscout/CoreFeatures.h \
    osmscout/CityStreetIndex.h \
    osmscout/AreaWayIndex.h \
    osmscout/AreaNodeIndex.h \
    osmscout/AreaAreaIndex.h \
    osmscout/AdminRegion.h

SOURCES += \
    osmscout/ost/Scanner.cpp \
    osmscout/ost/Parser.cpp \
    osmscout/util/Transformation.cpp \
    osmscout/util/String.cpp \
    osmscout/util/StopClock.cpp \
    osmscout/util/Reference.cpp \
    osmscout/util/Projection.cpp \
    osmscout/util/Progress.cpp \
    osmscout/util/NumberSet.cpp \
    osmscout/util/Number.cpp \
    osmscout/util/HashSet.cpp \
    osmscout/util/HashMap.cpp \
    osmscout/util/Geometry.cpp \
    osmscout/util/FileWriter.cpp \
    osmscout/util/FileScanner.cpp \
    osmscout/util/File.cpp \
    osmscout/util/Color.cpp \
    osmscout/util/Cache.cpp \
    osmscout/util/Breaker.cpp \
    osmscout/Way.cpp \
    osmscout/WaterIndex.cpp \
    osmscout/Types.cpp \
    osmscout/TypeSet.cpp \
    osmscout/TypeConfigLoader.cpp \
    osmscout/TypeConfig.cpp \
    osmscout/TurnRestriction.cpp \
    osmscout/Tag.cpp \
    osmscout/SegmentAttributes.cpp \
    osmscout/RoutingProfile.cpp \
    osmscout/Router.cpp \
    osmscout/RoutePostprocessor.cpp \
    osmscout/RouteNode.cpp \
    osmscout/RouteData.cpp \
    osmscout/Route.cpp \
    osmscout/Relation.cpp \
    osmscout/Point.cpp \
    osmscout/Path.cpp \
    osmscout/OptimizeLowZoom.cpp \
    osmscout/ObjectRef.cpp \
    osmscout/NumericIndex.cpp \
    osmscout/Node.cpp \
    osmscout/Location.cpp \
    osmscout/GroundTile.cpp \
    osmscout/Database.cpp \
    osmscout/CityStreetIndex.cpp \
    osmscout/AreaWayIndex.cpp \
    osmscout/AreaNodeIndex.cpp \
    osmscout/AreaAreaIndex.cpp \
    osmscout/AdminRegion.cpp

# END CONFIG FOR OSMSCOUT
##########################

##########################
# START PROJECT CONFIG

SOURCES += \
    main.cpp

OTHER_FILES += \
    ui/main.qml

# bar descriptor, icon, splashscreen
pkg_files.path = $${OUT_PWD}
pkg_files.files += \
    bar_descriptor.xml \
    icon.png \
    splashscreen.png

# define other resources and install path
res_local.path = $${OUT_PWD}/deploy
res_local.files += \
    ui

INSTALLS += pkg_files res_local

# END PROJECT CONFIG
##########################
