# Path Planning Project
Self-Driving Car Engineer Nanodegree Program

---

The goal of this project is to plot a real-time path for the vehicle in driving sumulator avoiding collisions and unpleasant speed and direction changes.

## Dependencies & Build Instructions

You can see detailed information on dependencies installation and build instructions in parent repository [here](https://github.com/udacity/CarND-Path-Planning-Project).
   
## Algorithm 

Plotting algorithm basically contains of 3 stages:
- Using sensor fusion data about the other vehicles on the road to determine safe speed and possible lane change
- User past, current and target future vehicle position as ancor points for the final path
- Use spline to calculate a smooth path based on the anchor points generated in the previous step

The last two stages are basically the same as in the Project Walkthrough video from the course materials with only slight tweaks and code re-factoring. The main challenge was to provide an algorithm for the first stage.

## Safe Lanes Logic

Logic for determining each lane safety is encapsulated in `Lane` class. Instance of this class is created for each lane. And then we update safety status of the lane by checking each other vehicle reported by sensor fusion.
Based on the current vehicles position and speed we calculate current and future distance between our vehicle and all the other vehicles on this lane. Then we analyse these distances which can result in 3 different decisions:

- If vehicle is far ahead or far behind and in the future this situation does not change we do not impose any safe speed limits on the lane
- If vehicle is in front of us and in the future distance becomes too small we calculate safe speed to keep the safe distance between the vehicles
- Otherwise the lane is considered unsafe

After we updated safety status of all the lanes we check the recommended safe speed for the current lane. If it is less than our target 50 mph speed we look for the safe speed in the adjacent lanes and pick the one with the higher safe speed.

## Results

Using the described algorithm vehicle was able to drive across the track without incidents. You can see a video below of one of the test attempts performed.

[![](https://img.youtube.com/vi/_RJ7UdSMe9M/0.jpg)](https://www.youtube.com/watch?v=_RJ7UdSMe9M)

