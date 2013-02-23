import mapnik

# https://github.com/mapnik/mapnik/blob/master/demo/python/rundemo.py

# int2hexcolor
def int2hexcolor(int):
   return str("#%0.6x" % int)

def int2hexcol(inp):
   # str("#%0.6x" % int)
   print inp

# create the map
vMap = mapnik.Map(3600,1800)

# set a background color
vMap.background = mapnik.Color('white')

# create text symbolizer
#vTextSym = mapnik.TextSymbolizer(mapnik.Expression('[COLOR]'), 'DejaVu Sans Book', 10, mapnik.Color('black'))
#vTextRule = mapnik.Rule()
#vTextRule.symbols.append(vTextSym)
#vTextStyle = mapnik.Style()
#vTextStyle.rules.append(vTextRule)

# create a polygon symbolizer
vPolySym = mapnik.PolygonSymbolizer(mapnik.Color('black'))

# create a rule and add symbolizer to it
vRule = mapnik.Rule()
vRule.symbols.append(vPolySym)

# create a style and add rule to it
vStyle = mapnik.Style()
vStyle.rules.append(vRule)

# add style to map as "MyStyle"
vMap.append_style('MyStyle',vStyle)
#vMap.append_style('TStyle',vTextStyle)

# get the shapefile
vFile = mapnik.Shapefile(file='/home/preet/Documents/maps/shapefiles/coastline/osm_full/polygons.shp')
#vFile = mapnik.Shapefile(file='/home/preet/Documents/maps/shapefiles/coastline/ned_coast/ne_50m_coastline.shp')

# create a layer and add shapefile and style
vLayer = mapnik.Layer('world')
vLayer.datasource = vFile
vLayer.styles.append('MyStyle')
vLayer.styles.append('TStyle')

# add layer to map
vMap.layers.append(vLayer)

# zoom to bounds
vMap.zoom_all()

# render
mapnik.render_to_file(vMap,'admin1.png','png')
print "Rendered map to file"
#print int2hexcolor(17895)
#print "#%0.6x" % 3456
