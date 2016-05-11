^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_localization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.6 (2016-03-20)
------------------
* New support for range-only (RO) localization
* fix build against mrpt <1.3.0
* Contributors: Jose Luis Blanco, Jose Luis Blanco-Claraco, Raphael Zack

0.1.5 (2015-04-29)
------------------
* fix to strange pf-localization bug
* Cleaner build against mrpt 1.3.0
* Fix build against mrpt 1.3.0
* Contributors: Jose Luis Blanco

0.1.4 (2014-12-27)
------------------
* dont publish if numSubscribers()==0
* fixes for mrpt 1.3.0
* Removed 'mrpt' dep from catkin_package().
  I *think* this is giving problems to dependant pkgs and is not needed...
* pose_cov_ops removed from mrpt_navigation metapkg
* localization: New param to configure sensor sources in a flexible way
* Contributors: Jose Luis Blanco

0.1.3 (2014-12-18)
------------------
* Fix many missing install files
* Contributors: Jose Luis Blanco

0.1.2 (2014-12-18)
------------------

0.1.1 (2014-12-17)
------------------
* First public binary release.

0.1.0 (2014-12-17)
------------------
* consistent version numbers
* fix build error without WX
* Fixes broken dependencies
* config and demos tested
* localization working like amcl

