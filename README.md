# image_nav
---
Package that...

# csv_out_node
---
After path.csv is written by image_nav.launch and Robot's current position is directed by /ublox_gps, code below will publish /gps_goal topic to direct the next waypoint in path.csv
```
rosrun image_nav csv_out_node
```
