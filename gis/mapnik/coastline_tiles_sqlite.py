import os
import mapnik
import sqlite3
from hashlib import md5

### ======================================
NUM_IMG = 0;

### ======================================
### Setup Mapnik
# create the map
_map = mapnik.Map(256,256)

# set a background color
_map.background = mapnik.Color('transparent')

# create a polygon symbolizer
poly_color = mapnik.Color('black');
poly_sym = mapnik.PolygonSymbolizer(poly_color);

# create a rule and add symbolizer to it
rule = mapnik.Rule()
rule.symbols.append(poly_sym)

# create a style and add rule to it
style = mapnik.Style()
style.rules.append(rule)

# add style to map as "MyStyle"
_map.append_style('MyStyle',style)

# get the shapefile
input_file = mapnik.Shapefile(file='/home/preet/Documents/maps/shapefiles/coastline/osm_mar2013_full/land_polygons.shp')

# create a layer and add shapefile and style
layer = mapnik.Layer('world');
layer.datasource = input_file;
layer.styles.append('MyStyle');

# add layer to map
_map.layers.append(layer);

###
FILE_PATH = './tiles/';
if not os.path.exists(FILE_PATH):
	os.makedirs(FILE_PATH)

### ======================================
### Create base solid fill tiles (land only
### and water only)
map_empty = mapnik.Map(256,256);

# water only
file_name = FILE_PATH + 'temp_water.png';
map_empty.background = _map.background;
mapnik.render_to_file(map_empty,file_name,'png256:c=64:t=2:z=9');
tile_water_bin = open(file_name,"rb").read();
md5_water = md5(tile_water_bin);

# land only
file_name = FILE_PATH + 'temp_land.png';
map_empty.background = poly_color;
mapnik.render_to_file(map_empty,file_name,'png256:c=64:t=2:z=9');
#tile_land_file = open(file_name,"rb");
tile_land_bin = open(file_name,"rb").read();
md5_land = md5(tile_land_bin);

### ======================================
### Create the sqlite database
print '* creating database...';
conn = sqlite3.connect(FILE_PATH + 'tiles.db');
cursor = conn.cursor();

# Schema: We have two tables:
# TILES: ROWID,MAG,X,Y,IMGID
cursor.execute("""CREATE TABLE TILES(MAG INTEGER NOT NULL, X INTEGER NOT NULL, Y INTEGER NOT NULL, IMGID INTEGER NOT NULL)""");
conn.commit();
									 
# IMAGES: ROWID,BLOB
cursor.execute("""CREATE TABLE IMAGES(IMGID INTEGER PRIMARY KEY NOT NULL, IMAGE BLOB)""");
conn.commit();

# Insert the base land and water tiles
print '* creating land and water fill tiles...';
cursor.execute("INSERT INTO IMAGES(IMGID,IMAGE) VALUES(" + str(NUM_IMG) + ",?)",[sqlite3.Binary(tile_land_bin)]);
conn.commit();
NUM_IMG = NUM_IMG + 1;

cursor.execute("INSERT INTO IMAGES(IMGID,IMAGE) VALUES(" + str(NUM_IMG) + ",?)",[sqlite3.Binary(tile_water_bin)]);
conn.commit();
NUM_IMG = NUM_IMG + 1;

### ======================================
### Save tiles based on zoom level
print '* building feature tiles...';
MAX_ZOOM = 7;
for z in range(MAX_ZOOM+1):
	print '  [level: ' + str(z) + ']';
	
	lat_divs = int(pow(2,z));
	lon_divs = lat_divs*2;
	
	lon_step = 180.0/pow(2,z);
	lat_step = 180.0/pow(2,z);
	
	# lon divs
	for lon in range(0,lon_divs):
		min_lon = lon * lon_step - 180.0;
		max_lon = min_lon + lon_step;
		
		# lat divs
		for lat in range(0,lat_divs):
			min_lat = -90.0 + (lat*lat_step);
			max_lat = min_lat + lat_step;

			bbox = mapnik.Envelope(min_lon,min_lat,max_lon,max_lat);
			_map.zoom_to_box(bbox);
			
			file_name = FILE_PATH + 'temp.png';
					
			# png256:t=2:z=9 (256 colors, full transparency, best compression)
			mapnik.render_to_file(_map,file_name,'png256:c=64:t=2:z=9');
			
			tile_this_bin = open(file_name,"rb").read();
			md5_tile = md5(tile_this_bin);
			
			if(md5_tile.digest() == md5_land.digest()):
				cursor.execute("INSERT INTO TILES(MAG,X,Y,IMGID) VALUES(?,?,?,?)",(z,lon,lat_divs-lat-1,0));
			elif(md5_tile.digest() == md5_water.digest()):
				cursor.execute("INSERT INTO TILES(MAG,X,Y,IMGID) VALUES(?,?,?,?)",(z,lon,lat_divs-lat-1,1));
			else:
				cursor.execute("INSERT INTO IMAGES(IMGID,IMAGE) VALUES(" + str(NUM_IMG) + ",?)",[sqlite3.Binary(tile_this_bin)]);
				cursor.execute("INSERT INTO TILES(MAG,X,Y,IMGID) VALUES(?,?,?,?)",(z,lon,lat_divs-lat-1,NUM_IMG));
				NUM_IMG = NUM_IMG + 1;
				
			# save
			conn.commit();

# close the db
cursor.close();
conn.close();
print 'Done!';



# test to see if blobs are working right
# conn = sqlite3.connect(FILE_PATH + 'tiles.db');
# cursor = conn.cursor();

#cursor.execute("SELECT MAX(IMGID) FROM IMAGES");
#max_id = cursor.fetchone()[0];

#print 'max_id: ' + str(max_id);
#print 'NUM_IMG: ' + str(NUM_IMG);

#for i in range(max_id):
	#query = "SELECT IMAGE FROM IMAGES WHERE IMGID=" + str(i);
	#print query;
	#cursor.execute(query);
 	#image_blob = cursor.fetchone();
 	#output_file = open(FILE_PATH + str(i) + ".png","wb");
 	#output_file.write(image_blob[0]);
 	#print 'wrote: ' + str(i);
 
#cursor.close();
#conn.close();

