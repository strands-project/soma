Header header
string id 				# The id of the SOMA ROI.

string map_name				# The map that ROI belongs to

string map_unique_id    		# The unique Mongo ID of the map

string config           		# The configuration of the ROI

string type				# The type of the roi. Could be Office, Kitchen etc

uint32 logtimestamp 			# Unix timestamp information that holds the time when the object is logged. SOMA uses UTC time values

int8 logday 				# The day of week. Automatically filled by SOMA insert service when logtimestamp info is provided

int8 loghour 				# Automatically filled by SOMA insert service when logtimestamp info is provided

int8 logminute 				# Automatically filled by SOMA insert service when logtimestamp info is provided

int32 logtimeminutes 			# Automatically filled by SOMA insert service. Simplifies queries

geometry_msgs/PoseArray posearray 	# The poses of the vertices

string geotype 				# Point,Polygon,etc. For ROIs type is Polygon 

geometry_msgs/PoseArray geoposearray 	# Poses of the vertices in geospatial format lat,long
