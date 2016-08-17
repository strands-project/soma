robot_state_viewer
====

A visual interface for querying Semantic Object Map 2.0(SOMA2) objects. SOMA2 objects can include objects, regions of interest (ROI), and trajectories. The viewer assumes that there are already stored objects and maps in the data center.

Using the visual interface, advanced queries with spatio-temporal constraints  can be specified. The returned SOMA2 objects are displayed in rviz as point clouds.

Prerequisites
-------------

- MongoDB (>=3.2)
- mongodb_store - use this [fork](https://github.com/hkaraoguz/mongodb_store)
- soma2 [link](https://github.com/hkaraoguz/soma2)
- Qt5


Getting started (general steps)
-------------------------------
1. Start the ros core:

    ```
   $ roscore
    ```
2. Launch the ROS datacentre:

    ```
    $ roslaunch mongodb_store mongodb_store.launch db_path:=<path_to_db>

    ```

SOMA2 map manager
-----------------
  1. Run the soma2 map manager for storing, reading and publishing 2D map. Running this node is essential for running the robot_state_viewer_node:
  ```
  $ rosrun soma2_map_manager soma2_map.py
  ```
  If there are any stored 2D occupancy maps in the datacentre then map manager will let you choose the map to be published. If not, it will wait for map_server. Run the map_server with a 2D map:
    ```
    $ rosrun map_server map_server <map.yaml>
    ```
  where `map.yaml` specifies the map you want to load. After running the `map_server`, you should save the published map using the `soma2 map manager`.

  2. If you want to check the published map, start RVIZ, add a Map display type and subscribe to the `soma2/map` topic:

    ```
    $ rosrun rviz rviz
    ```


SOMA2 ROI drawer
----------------

1. Run the SOMA2 ROI drawer:

    ```
    $ rosrun soma2_roi_manager soma2_roi_drawer.py
    ```
2. Add a MarkerArray display type in RVIZ and subscribe to the `visualization_marker_array` topic. The drawer will draw when a region is selected in the viewer.

robot_state_viewer
------------------

1. Run the robot state viewer:

```
$ rosrun robot_state_viewer robot_state_viewer_node <objects db name> <objects collection name> <roi db name>
```


2. If not running, start RVIZ to display the results of the queries. You can go back and forth between robot states using the slider. You can also execute advanced queries by setting dates, times, etc, enabling them using the checkboxes and by pressing the `Query` button. When you make changes in query constrains, make sure to press `Query` button for updating the query. You can also export the executed query in JSON format using the `Export JSON` button. You can reset the query by pressing `Reset` button. For any arbitrary query, if more than 30 objects are fetched, only first 30 of them are displayed as point clouds because of performance issues.

![marker](https://raw.githubusercontent.com/hkaraoguz/robot_state_viewer/master/doc/robot_state_viewer.png)
