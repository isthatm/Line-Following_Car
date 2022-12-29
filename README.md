# Final-Project
This program is designed for BBCar to follow the lines and respond to two special marks using QTIs sensors:
- 0b0111 (pattern 7): car turns left.
- 0b1111 (pattern 15): car stops.

Also, car will stop and switch to another path when it detects obstacle 11.5cm ahead on the the current path. 
On the new path, it will measure the gap length between obstacles to make sure the width is wide enough before continue
travelling.
