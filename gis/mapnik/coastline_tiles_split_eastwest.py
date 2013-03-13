import os
import mapnik

### ======================================
### Setup Mapnik
# create the map
vMap = mapnik.Map(256,256)

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
vFile = mapnik.Shapefile(file='/home/preet/Documents/maps/shapefiles/coastline/osm_full/polygons.shp')

# create a layer and add shapefile and style
vLayer = mapnik.Layer('world')
vLayer.datasource = vFile
vLayer.styles.append('MyStyle')

# add layer to map
vMap.layers.append(vLayer)

### ======================================
### Save tiles based on zoom level
vMaxZoom = 7;
vWorldMinLon = None;
vDirName  = None;

for x in range(0,2):
	# (x==0): western hemisphere
	# else  : eastern hemisphere
	if(x==0):
		vWorldMinLon = -180.0;
		vDirName = 'west/';
	else:
		vWorldMinLon = 0.0;
		vDirName = 'east/';

	for i in range(vMaxZoom+1):
		print 'level: ' + str(i);

		numDivs = int(pow(2,i));
		lonStep = 180.0/pow(2,i);
		latStep = 180.0/pow(2,i);
		
		# lon divisions
		for lon in range(0,numDivs):
			minLon = vWorldMinLon + (lon*lonStep);
			maxLon = vWorldMinLon + ((lon+1)*lonStep);
			#print '  lon: [' + str(minLon) + ' to ' + str(maxLon) + ']';
			
			# lat divisions
			for lat in range(0,numDivs):
				minLat = -90.0 + (lat*latStep);
				maxLat = -90.0 + ((lat+1)*latStep);
				#print '    lat: [' + str(minLat) + ' to ' + str(maxLat) + ']';
						
				bbox = mapnik.Envelope(minLon,minLat,maxLon,maxLat);
				vMap.zoom_to_box(bbox);
				
				filePath = './tiles/' + vDirName + str(i) + '/';
				fileName = filePath + str(lon) + '_' + str(numDivs-lat-1) + '.png';
				
				if not os.path.exists(filePath):
					os.makedirs(filePath)
					
				# png256:t=2:z=9 (256 colors, full transparency, best compression)
				mapnik.render_to_file(vMap,fileName,'png256:c=64:t=2:z=9');
