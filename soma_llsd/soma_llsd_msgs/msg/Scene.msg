string id
string episode_id
string waypoint
string meta_data
int32 timestamp


tf2_msgs/TFMessage transform    # 2 seconds or so of TF, can be handled by using InsertSceneAuto service
sensor_msgs/PointCloud2 cloud
sensor_msgs/Image rgb_img
sensor_msgs/Image depth_img
sensor_msgs/CameraInfo camera_info
geometry_msgs/Pose robot_pose
