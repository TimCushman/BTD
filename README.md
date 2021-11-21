# BTD - Balloon's Tower Defense Robot
## COSC 81 Principles of Robot Design and Programming: Final Project 
### Anna Manning, Tim Cushman, Ben Lehrburger and Jack Gnibus
This program will cause the robot to avoid red objects and go towards green objects (in our test cases balloons)

[To view our demo video, download here!](https://github.com/TimCushman/BTD/blob/e420ff081cc216d16fb200c976b91b082324892b/BTDvideo.mp4)

## How to run `BTD.py` on the ROSbot 2.0 

Connect to the robot via Ethernet connection: 
1. Connect the robot to your compter with the ethernet cable. 
2. After the connection, your computer will get an IP address 
3. Use 'ssh' to connect to the robot: `ssh husarion@192.168.0.1` #Password: husarion
4. In another terminal, run `docker-compose exec ros bash` and run `husarnet daemon`
5. In another terminal, run `docker-compose exec ros bash` and use 'scp' to copy the code `BTD.py` to the robot: `scp -r BTD.py husarion@192.168.0.1:~/husarion_ws/src/`
6. then run `python BTD.py`

