# Waypoint Generation Node
This node generates waypoints for the pure pursuit by taking the map, creating centerline, and then optimizing trajectory before publishing the waypoints.

Input/Output Topics
This node subscribes to:
/map: Subscribes to the occupancy grid of the map.
/car_state/pose: Reads the race car’s pose.

This node publishes to:
/centerline_waypoints: Publishes centerline way points as
/global_waypoints: Publishes waypoints as 

Implementation and Algorithm
Run Jupyter notebook and get the centerline from the map using map_converter.ipynb and save the center sanity_check.ipynb to generate the CSV and match the original map.
Input: PNG, YAML
Output: Save the centerline data as a CSV saved into the inputs/tracks folder.
Run global trajectory generation (main_globaltraj.py).
Input: CSV on centerline
Output: CSV for waypoints
Send to pure pursuit
