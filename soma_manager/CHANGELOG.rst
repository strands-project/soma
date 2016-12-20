^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package soma_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.0 (2016-12-20)
------------------
* Fixed merge conflicts
* Modified package.xml files for 2.0.1 release
* Code refactoring and simplification. Added query by object config to soma visualizer. Query message is changed to include config array instead of single config
* Removed unnecesary cmakelists command. Changed db_name in launch file.
* integrates soma_llsd into soma data manager
* Removed query builder class dependency in soma_visualizer. Various bug fixes and code improvements. Added object detail view to soma_visualizer. Fixed issue 72
* Object detail view has been added
* Various bug fixes in visualizer. Added object list table
* Fixed a bug in query manager that causes crash with empty object type/id array. Visualizer now uses query service instead
* README has been updated. Query manager has been updated to fix issue69
* Query objects with custom rois
* Changes done to be compatible with mongodb update
* Updated soma_roi_manager for using soma roi query services. Moved geospatial indexing of rois to mongodb. Fixed various bugs in object managersoma_roi manager
* SOMA ROI query service has been updated to return the latest versions of the rois. SOMA ROI manager has been updated. Geospatial data is now handled in mongodb_store for ROIs. Various bug fixes
* Increased the wait timeout for the map service
* Additional features are added to the visualizer. You can go to the first and last step of the slider with buttons
* Fixes to remote launch file
* Launch files have been updated. SOMA ROI has been updated
* Object manager is updated to use the soma query services for querying objects. SOMa query manager has been updated to return unique mongo ids of queried objects. SOMAQueryObjs service message has been updated to perform config based queries for objects
* Various bug fixes. Started integrating soma_visualizer into whole soma package
* SOMA Query service has been updated. ROI and Object queries are seperated. SOMAObject has been updated with an additional metafield. SOMA roi drawer has been updated. SOMANewObjects message has been added for announcing newly added objects
* Miscalleneous updates and bugfixes
* Robot state viewer is added to the package as soma_visualizer. QueryManager has been updated
* Update documentation. Updated soma ROI manager for displaying latest version of ROIs.
* Clean up code. Removal of 2s
* 2s are removed from soma_trajectory.
* 2s have been removed from msgs and code
* Started removal of 2's from various places. Updated object_manager, query_manager and data_manager
* Msg documenation has been updated. ROI manager has been updated for storing the update history of the regions along with temporal information
* Contributors: Hakan, STRANDS, hkaraoguz

2.0.0 (2016-11-25)
------------------

1.0.4 (2016-06-20)
------------------
* added soma_map_manager as dependency (`#48 <https://github.com/strands-project/soma/issues/48>`_)
* Contributors: Marc Hanheide

1.0.3 (2016-06-20)
------------------
* Marc hanheide patch for release (`#44 <https://github.com/strands-project/soma/issues/44>`_)
  * updated changelogs
  * 1.0.1
  * updated changelogs
  * 1.0.2
  * added octomap_msgs dep
  * added libqt5-core
  * removed invalid install target
  * added interactive_markers dep
  * fix dev for QT5
* Contributors: Marc Hanheide

1.0.2 (2016-06-07)
------------------

1.0.1 (2016-06-06)
------------------

1.0.0 (2016-06-06)
------------------
* Updated package xml files.
* Big restucture for release
* Updated structure for release.
* Fixes for building on OSX and restructure for release.
* Fixed the time based query problem by changing the field type of lowerdate and upperdate to uint64
* fix for header dependency
* Try to solve query manager lock down while ros::shutdown. Moved to launch folder under soma_manager
* Fixed the hanging of query_manager during shutdown. Refined code for soma_roi
* Fixed issues with query manager crashing when empty fields or no data is present
* Fixed issue with empty typeids or objectsids. Updated launch file.
* Corrected msg dependencies
* Query results are returned as srv response. ROI drawing color has been set
* Listening map service is added
* Query service has been added
* Remove unnecesary folder
* Updated documentation
* Latest soma2 is merged into soma fork named as soma
* Contributors: Nick Hawes, Nils Bore, hkaraoguz

0.0.2 (2016-06-06)
------------------
* updated changelogs
* Contributors: Jenkins

0.0.1 (2016-02-03)
------------------
* added changelogs
* Fixed args so they work in a launch file.
* fix frame problem
* increase z offset for marker
* sort menu items
* convert xy-coords to longitude/latitude
* add dependency
* geospatial store for semantic object maps
* replaced ros_datacentre with mongodb_store
* renamed ros node
* fixed node name
* add new objects always on the ground plane
* added possibility of modelling object in 3D
* fixed indention
* read available object types from config file
* enable/disable interactive markers
* initial commit
* Contributors: Lars Kunze, Marc Hanheide, Nick Hawes
