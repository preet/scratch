import os
import sqlite3

conn = sqlite3.connect("test.sqlite");
cursor = conn.cursor();

cursor.execute("SELECT MAX(ROWID) FROM TILES");
max_id = cursor.fetchone()[0];

for i in range(max_id):
	query = "SELECT MESH FROM TILES WHERE ROWID=" + str(i+1);
	print "query is: " + query;
	cursor.execute(query);
	mesh_blob = cursor.fetchone();
	output_file = open("mesh"+str(i)+".ctm","wb");
	output_file.write(mesh_blob[0]);
	
cursor.close();
conn.close();
