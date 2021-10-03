# Udacity_RSEND_Project_4_Map_My_World
Udacity RSEND Project 4 Map My World


# Notes:

One key thing is to realize that rtabmap is utilizing a generic base_footprint for the frame_ids.  Changing the URDF or changing in the launch file is required to get the proper data from the topics.


Otherwise this error will occur where the data is not piped to the proper end topics during remapping.

```
[ WARN] [1633285689.744699814, 435.400000000]: /rtabmap/rtabmapviz: Did not receive data since
 5 seconds! Make sure the input topics are published ("$ rostopic hz my_topic") and the timest
amps in their header are set. If topics are coming from different computers, make sure the clo
cks of the computers are synchronized ("ntpdate"). If topics are not published at the same rat
e, you could increase "queue_size" parameter (current=10).
/rtabmap/rtabmapviz subscribed to (approx sync):
   /rtabmap/odom \
   /camera/rgb/image_raw \
   /camera/depth/image_raw \
   /camera/rgb/camera_info \
   /scan
```
