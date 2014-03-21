Off board program for METR4810 Group 1.

Currently a lot of the code is just chucked in Main.cpp, and hasn't 
been organised properly yet. E.g. Simulator.cpp and CarController.cpp 
aren't used right now.

Main
Entry point

Car
Car class. Stores position, heading, left and right motor speeds.
Collision testing (not implemented yet) will assume rectangle, so
class also stores length and width. Has function 
step (double seconds) for moving the car based on current motor speeds.

Racetrack
Class for storing racetrack. Contains vector<Point> to store track midpoints, as well as racetrack width (currently assumed to be constant)

View
Handles visualisation of stuff.

