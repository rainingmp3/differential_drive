fri aug 15
topic duplicates: \

[x] configure bridge or make publisher work \
--- DONE \
\
fri aug 15 log 2 \

add basic perception: [ ] use lidar sensor
    learn and find topics like /goal_pose, /diff_drive/scan

sat aug 16

the issue with logging, for som reason subscriber logging doesnt work, everything seems to be fine though ,,,

sun aug 17 log

so, the issue was launch file wich held old node, which was still working through share dir(it was cached)
[ ] now we need to properly handle laser scan array, cause now i am misssing something .
[] also about on imports of xacro, yaml and so on
[x] clean the logs, or log differently
Edgar Dijkstra seems to be a cool man

Thu aug 21

Didnt log some stuff
[X] ] PID logic, 3 min deal reall (it wasnt 3 min deal((

Fri aug 22
Need to add to set timestep [ ]
How to set sub and pub updatet rates in meaningful way [X]
THe issue is that diff_drive cant publish max velocity and rotate at the same time -- [ ]
That was not the issue ...atan2 points in opposite direction [ ]

[ ] reorginize cpp/hpp structure
