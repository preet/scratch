#!/usr/bin/python2

import logging as l
from optparse import OptionParser
import os
import sys
from osgeo import ogr
from subprocess import check_call

usage = "usage: %prog SRCFILE SRCLAYER OUTDIR ROWS COLS"
parser = OptionParser(usage=usage)

(options, args) = parser.parse_args()

if len(args) < 5:
    parser.error("You must specify a source file, source layer, output dir, rows, and cols.")

sourceFile = os.path.realpath(args[0])
sourceLayer = args[1]
outDir = os.path.realpath(args[2])
rows = int(args[3])
cols = int(args[4])

if not os.path.exists(outDir):
    l.error('Output directory %s does not exist. Create it and try again.' % (outDir))
    sys.exit(1)

dataSource = ogr.Open(sourceFile, 0)  # 0 means read-only
if dataSource is None:
    l.error('OGR failed to open ' + sourceFile + ', format may be unsuported')
    sys.exit(1)

dataLayer = dataSource.GetLayerByName(sourceLayer)
if dataLayer is None:
    l.error('OGR could not find layer ' + sourceLayer + '. Does it exist?')
    sys.exit(1)

(minx,maxx,miny,maxy) = dataLayer.GetExtent()

#AUS_NZ
#define MINLON 110
#define MAXLON 180
#define MINLAT -56
#define MAXLAT -8
#define LONSTEP 2
#define LATSTEP 2
#define DBLMT 1E10

minx = 110
maxx = 180
miny = -56
maxy = -8

print 'Extents: %s,%s,%s,%s' % (minx,miny,maxx,maxy)
deltax = maxx - minx
deltay = maxy - miny

cols = deltax/2
rows = deltay/2

tile_height = deltay / rows
tile_width = deltax / cols
print 'Tile Height: %s' % (tile_height)
print 'Tile Width: %s' % (tile_width)
x = 0
tileminx = minx
while tileminx < maxx:
    y = 0
    tileminy = miny
    while tileminy < maxy:
        tile_extent = (tileminx, tileminy, tileminx + tile_width, tileminy + tile_height)

        destFile = '%s/%04d_%04d_%s' % (outDir, x, y, os.path.basename(sourceFile))
        dataLayer.SetSpatialFilterRect(tile_extent[0], tile_extent[1], tile_extent[2], tile_extent[3])
        
        featureCount = dataLayer.GetFeatureCount()

        if featureCount > 0:
            # Call ogr2ogr with a spatial filter set
            check_call(['/usr/bin/ogr2ogr', '-spat', str(tile_extent[0]), str(tile_extent[1]), str(tile_extent[2]), str(tile_extent[3]), destFile, sourceFile])
            l.warn('Wrote %s features to %s' % (featureCount, destFile))
        else:
            l.warn('Skipping %s because it didn\'t have any features.' % (destFile))

        dataLayer.ResetReading()

        y += 1
        tileminy += tile_height
    x += 1
    tileminx += tile_width
