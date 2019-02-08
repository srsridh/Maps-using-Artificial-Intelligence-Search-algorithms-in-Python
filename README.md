# Maps-using-Artificial-Intelligence-Search-algorithms-in-Python

#Project Description

• Built maps using GPS co-ordinates and length of roadway between cities in USA by using A*,Uniform, BFS, DFS
and IDS search algorithms with distance, time and segment as cost functions
• Discovered the total distance , time taken and the paths between two cities
• Uniform search algorithm returned the most optimal path for any two cities within 4 seconds


#Files Description

city-gps.txt contains one line per city, with three fields per line, 
delimited by spaces. The first field is the city, followed by the latitude,
followed by the longitude.

road-segments.txt has one line per road segment connecting two cities.
The space delimited fields are:

- first city
- second city
- length (in miles)
- speed limit (in miles per hour)
- name of highway

route.py - Python implementation
