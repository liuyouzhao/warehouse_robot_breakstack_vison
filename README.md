## To Build
./make

## build model
./make ros
./make sys
./make core

## clean
./make clean

## To run
## On service side:
export ROS_IP=`hostname -I`
rosrun cam3d cam3d_above_arm_v1_depth

## On viewer side:
export ROS_IP=`hostname -I`
rosrun viewer viewer_cloud3d_depth

## run offline test
cd libs
./test_segoffline -w=752 -h=480 -p=../app/dat/list.conf

## run online test for orbbec camera
cd libs
./test_segonline


