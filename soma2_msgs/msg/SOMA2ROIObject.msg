Header header
string id 				# The id of the SOMA2 ROI.
string map_name				# The map that ROI belongs to
string map_unique_id    		# The unique Mongo ID of the map
string config           		# The configuration of the ROI
string type				# The type of the roi. Could be Office, Kitchen etc
geometry_msgs/PoseArray posearray 	# The poses of the vertices
string geotype 				# Point,Polygon,etc. This should be Polygon for regions
geometry_msgs/PoseArray geoposearray 	# the poses of the vertices in geospatial format lat,long
