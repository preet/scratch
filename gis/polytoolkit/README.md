polytk

polytk stands for "poly toolkit" and is a small collection
of *VERY* roughly put together tools for operating on wkt polygons

* ptk_gridify_wkt: used to divide a wkt csv into tiles
  NOTE: this is broken right now, DONT use it

* ptk_quadify_wkt: used to recursively divide a wkt csv
  into separate tiled wkt csvs

* ptk_repair_wkt: used to repair wkt polygons according
  using 'prepair' (https://github.com/tudelft-gist/prepair)

* ptk_simplify_wkt: uses either the Douglas-Peucker or
  Visvalingam-Whyatt algorithm to simplify wkt polygons

* ptk_xform_wkt: transforms wkt polygons from mercator to wgs84 (lat/lon)

* ptk_wkt_to_ply: converts a wkt file (expect wgs84 coordinates) to a ply
  by transforming wgs84 to ECEF and then triangulating the polygons

The files included with this project all use BSD or MIT licenses.
Before using it however, you should be aware that this project links
to the CGAL lib, specifically to algorithms that are GPL licensed.
The other libs (ie, excluding CGAL) not included with this project
are either MIT (glad/ogr) or equally liberal (boost,stl,etc).

The onus is on you to verify the above information, and understand
the applicable licensing terms before using any of the code.
