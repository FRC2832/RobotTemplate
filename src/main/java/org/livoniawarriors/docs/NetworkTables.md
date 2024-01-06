# Introduction
We use NetworkTables for our data collection on the robot.  NetworkTables was created by WPILib to be an efficient remote data collection mechanism on the robot.  

At the end of the day, NetworkTables is just a distributed dictionary, with keys (the name of the item) and values.  While values in dashboards appear to be nested folders, everything is actually in a flat structure, and "/" are replaced as folders.  (And AdvantageScope now also treats "_" as folders too)

# Keywords to Know
* Topic - The base entry in the NetworkTables.  Only contains the key and datatype in it.
* 