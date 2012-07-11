import mapnik

# Set up projections
# spherical mercator (most common target map projection of osm data imported with osm2pgsql)
merc = mapnik.Projection('+proj=merc +a=6378137 +b=6378137 +lat_ts=0.0 +lon_0=0.0 +x_0=0.0 +y_0=0 +k=1.0 +units=m +nadgrids=@null +no_defs +over')

# long/lat in degrees, aka ESPG:4326 and "WGS 84" 
longlat = mapnik.Projection('+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs')
# can also be constructed as:
#longlat = mapnik.Projection('+init=epsg:4326')

# remember that all osm data has lat extents of +/- 85 degrees

stylesheet = 'style.xml'
image = 'world_outline.png'
m = mapnik.Map(1024,1024)
mapnik.load_map(m, stylesheet)
bbox = mapnik.Envelope(-80,39.2,-77,42.2)
m.zoom_to_box(bbox)
mapnik.render_to_file(m, image)
print "rendered image to '%s'" % image
