^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package soma_visualizer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.0 (2016-12-20)
------------------
* Fixed install targets
* Modified package.xml files for 2.0.1 release
* Code refactoring and simplification. Added query by object config to soma visualizer. Query message is changed to include config array instead of single config
* Removed unnecesary cmakelists command. Changed db_name in launch file.
* Updated readme for visualizer
* Updated readme for visualizer
* Removed unnecessary parameters from main and rosThread. Improved visualization
* The memory management problem of soma visualizer has been fixed. Draw most recent functionality has been added to the soma roi drawer. Various bug fixes in soma visualizer
* Removed query builder class dependency in soma_visualizer. Various bug fixes and code improvements. Added object detail view to soma_visualizer. Fixed issue 72
* Object detail view has been added
* Various bug fixes in visualizer. Added object list table
* Fixed a bug in query manager that causes crash with empty object type/id array. Visualizer now uses query service instead
* Changes done to be compatible with mongodb update
* SOMA ROI query service has been updated to return the latest versions of the rois. SOMA ROI manager has been updated. Geospatial data is now handled in mongodb_store for ROIs. Various bug fixes
* Additional features are added to the visualizer. You can go to the first and last step of the slider with buttons
* Soma visualizer has been updated to slide through time using the user-specified step-size
* Various bug fixes. Started integrating soma_visualizer into whole soma package
* SOMA Query service has been updated. ROI and Object queries are seperated. SOMAObject has been updated with an additional metafield. SOMA roi drawer has been updated. SOMANewObjects message has been added for announcing newly added objects
* Miscalleneous updates and bugfixes
* Robot state viewer is added to the package as soma_visualizer. QueryManager has been updated
* Contributors: Hakan, hkaraoguz

* Fixed install targets
* Modified package.xml files for 2.0.1 release
* Code refactoring and simplification. Added query by object config to soma visualizer. Query message is changed to include config array instead of single config
* Removed unnecesary cmakelists command. Changed db_name in launch file.
* Updated readme for visualizer
* Updated readme for visualizer
* Removed unnecessary parameters from main and rosThread. Improved visualization
* The memory management problem of soma visualizer has been fixed. Draw most recent functionality has been added to the soma roi drawer. Various bug fixes in soma visualizer
* Removed query builder class dependency in soma_visualizer. Various bug fixes and code improvements. Added object detail view to soma_visualizer. Fixed issue 72
* Object detail view has been added
* Various bug fixes in visualizer. Added object list table
* Fixed a bug in query manager that causes crash with empty object type/id array. Visualizer now uses query service instead
* Changes done to be compatible with mongodb update
* SOMA ROI query service has been updated to return the latest versions of the rois. SOMA ROI manager has been updated. Geospatial data is now handled in mongodb_store for ROIs. Various bug fixes
* Additional features are added to the visualizer. You can go to the first and last step of the slider with buttons
* Soma visualizer has been updated to slide through time using the user-specified step-size
* Various bug fixes. Started integrating soma_visualizer into whole soma package
* SOMA Query service has been updated. ROI and Object queries are seperated. SOMAObject has been updated with an additional metafield. SOMA roi drawer has been updated. SOMANewObjects message has been added for announcing newly added objects
* Miscalleneous updates and bugfixes
* Robot state viewer is added to the package as soma_visualizer. QueryManager has been updated
* Contributors: Hakan, hkaraoguz

2.0.0 (2016-11-25)
------------------

1.0.4 (2016-06-20 10:17)
------------------------

1.0.3 (2016-06-20 08:36)
------------------------

1.0.2 (2016-06-07)
------------------

1.0.1 (2016-06-06 22:45)
------------------------

1.0.0 (2016-06-06 14:44)
------------------------

0.0.2 (2016-06-06 12:04)
------------------------

0.0.1 (2016-02-03)
------------------
