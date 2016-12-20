^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package soma_roi_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.0 (2016-12-20)
------------------
* Fixed install targets
* Fixed merge conflicts
* Modified package.xml files for 2.0.1 release
* remove space on the description of config to avoid confusion
* attempt to make have roi with multiple configs
* The memory management problem of soma visualizer has been fixed. Draw most recent functionality has been added to the soma roi drawer. Various bug fixes in soma visualizer
* Fixed the issue of not updating timestamp when a vertex is deleted
* Updated soma_roi_manager for using soma roi query services. Moved geospatial indexing of rois to mongodb. Fixed various bugs in object managersoma_roi manager
* SOMA ROI query service has been updated to return the latest versions of the rois. SOMA ROI manager has been updated. Geospatial data is now handled in mongodb_store for ROIs. Various bug fixes
* Launch files have been updated. SOMA ROI has been updated
* Various bug fixes. Started integrating soma_visualizer into whole soma package
* SOMA Query service has been updated. ROI and Object queries are seperated. SOMAObject has been updated with an additional metafield. SOMA roi drawer has been updated. SOMANewObjects message has been added for announcing newly added objects
* Miscalleneous updates and bugfixes
* Update documentation. Updated soma ROI manager for displaying latest version of ROIs.
* Clean up code. Removal of 2s
* 2s have been removed from msgs and code
* Removed world_state_importer. Refactored soma_roi code
* Msg documenation has been updated. ROI manager has been updated for storing the update history of the regions along with temporal information
* SOMA2ROI object and roi_manager updates
* Contributors: Ferdian Jovan, Hakan, hkaraoguz

2.0.0 (2016-11-25)
------------------

1.0.4 (2016-06-20)
------------------

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
* Added missing import
* Big restucture for release
* Updated structure for release.
* Fixed the hanging of query_manager during shutdown. Refined code for soma_roi
* The corruption of regions after saving and loading has been fixed
* Query results are returned as srv response. ROI drawing color has been set
* SOMA2  to SOMA name changes
* Updated documentation
* Latest soma2 is merged into soma fork named as soma
* Contributors: Nick Hawes, hkaraoguz

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
* Add get_rois function to query class.
* add cs_1 config file for soma_roi
* g4s demo
* fixing the trajectory_importer.py
* sort menu items
* convert xy-coords to longitude/latitude
* use geopatial store for ROIs
* replaced ros_datacentre with mongodb_store
* added labels for ROIs
* removed debug printout
* fixed problem with polygon
* visualize roi using a line strip marker
* added manager for ROIs
* Contributors: Chris Burbridge, Ferdian Jovan, Lars Kunze, Marc Hanheide, Nick Hawes
