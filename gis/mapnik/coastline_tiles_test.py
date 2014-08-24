import os
import mapnik

### ======================================
### Setup Mapnik
# create the map
vMap = mapnik.Map(512,256)

# set a background color
vMap.background = mapnik.Color('transparent')

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

# get the shapefile
vFile = mapnik.Shapefile(file='/home/preet/Documents/maps/shapefiles/coastline/osm_mar2013_full/land_polygons.shp')

# create a layer and add shapefile and style
vLayer = mapnik.Layer('world')
vLayer.datasource = vFile
vLayer.styles.append('MyStyle')

# add layer to map
vMap.layers.append(vLayer)

### ======================================
### Save tiles based on zoom level
vMaxZoom = 1;

for i in range(vMaxZoom+1):
	print 'level: ' + str(i);
	
	latDivs = int(pow(2,i));
	latStep = 180.0/pow(2,i);
	
	lonDivs = latDivs;
	lonStep = latStep*2;
	
	# lon divs
	for lon in range(0,lonDivs):
		minLon = (lon*lonStep)-180.0;
		maxLon = minLon + lonStep;
		
		# lat divisions
		for lat in range(0,latDivs):
			minLat = -90.0 + (lat*latStep);
			maxLat = -90.0 + ((lat+1)*latStep);
			#print '    lat: [' + str(minLat) + ' to ' + str(maxLat) + ']';
					
			bbox = mapnik.Envelope(minLon,minLat,maxLon,maxLat);
			vMap.zoom_to_box(bbox);
			
			filePath = './tiles/' + str(i) + '/';
			fileName = filePath + str(lon) + '_' + str(latDivs-lat-1) + '.png';
			
			if not os.path.exists(filePath):
				os.makedirs(filePath)
				
			# png256:t=2:z=9 (256 colors, full transparency, best compression)
			mapnik.render_to_file(vMap,fileName,'png256:c=64:t=2:z=9');
