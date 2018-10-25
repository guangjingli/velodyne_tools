# velodyne_tools

multi_lidar_calibraion is forked from Autoware: https://github.com/CPFL/Autoware/tree/c2ba68b4e9f42136e6f055a9cd278dcebdcbab17/ros/src/sensing/fusion/packages/multi_lidar_calibrator

It will output the TF between two lidar coordinates.
And then, you can add TF "static_transform_publisher" in the lanuch of VLP16_2.launch (https://github.com/guangjingli/velodyne/blob/master/velodyne_pointcloud/launch/VLP16_2.launch)

<node pkg="tf" type="static_transform_publisher" name="velodyne_first_to_secod" args="-1 0 0 0 0 0 /velodyne_first /velodyne_second 10" />
