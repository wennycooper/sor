# StatisticsOutlierRemoval to remove laser error points

# Result
Red are points before filtered
Green are points after filtered

# Usage
    // launch lidar driver and laser messages are on /scan topic
    $ roslaunch rplidar_ros rplidar.launch
     
    // run sor rosnode
    $ rosrun sor sor

    // run rviz

    $ rviz
    // add display to show cloud_before_filtered topic
    // add display to show cloud_after_filtered topic

