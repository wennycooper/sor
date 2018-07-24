# SOR
This is an example to use PCL StatisticsOutlierRemoval filter to remove laserscan noise points

# Demo
https://youtu.be/XfKqD-EItMc

https://youtu.be/ev6Rf_WXfQM

* Green/White are points before filtered
* Res are points after filtered

# Usage
    // launch lidar driver and laser messages are on /scan topic
    $ roslaunch rplidar_ros rplidar.launch
     
    // run sor rosnode
    $ rosrun sor sor

    // run rviz

    $ rviz
    // add display to show cloud_before_filtered topic
    // add display to show cloud_after_filtered topic

# References
http://pointclouds.org/documentation/tutorials/statistical_outlier.php#statistical-outlier-removal
http://wiki.ros.org/pcl/Tutorials

