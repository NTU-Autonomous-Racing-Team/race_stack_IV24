# Gap Finder

This package contains "follow the gap" methods

## gap_finder_base

-   Disparity extender ++

### Description

This method borrows hevily from the disparity extender method but employs additional signal processing on the laser scans to produce a more stable planner.

### Components

1. Disparity Extender
2. Safety Bubble

-   bubble of a certain diameter e.g. width of car where all the points are set to the same value as the middle range.
-   Calculates the number of ranges within the safety bubble by assuming the arc=angle_increment\*range is a stright line colinear with the diameter.

3. lookahead distance

-   Ranges are clipped at this value.
-   Increases the priority of nearer end points
-   prevents jitter in steering by smoothing the max distances

4. field of view

-   limits the search space for the goal point
-   prevents the car from selecting goal points behind the car

5. Nearest point safety bubble

-   draws a safety bubble at the neearest point of the car
-   used for obstacle avoidance

6. Mean filter

-   Window rolling mean filter with the windows size the same as a safety bubble drawn at that point
-   It smooths the readings so that the edges of the safety bubble is slightly longer if the edges next to it are stretched

7. Priortise the center of scan

-   multiplies all the ranges with a linspace of 0.999 to 1.000 so that within a scan of equal ranges, the car moves in a stright line.

8. Selection of the max range

-   the max range after filtering is selected as the goal point

9. Speed controller

-   the speed is proportional to the ratio of the mean ranges to the lookahead distance, this is to slow the car in tight spaces (i.e. high uncertainty) this can be changed to the range or mean range in front of the car. 
- the speed is a function of the steering angle using the simple bicycle model. The equation is  
$\sqrt{\frac{g * u * b}{tan{|d|}}}$  
where:  
g is gravitational acceleration,  
u is the coefficient of friction,  
b is the wheelbase and  
d is the steering angle. 

### Resources

-   [Unifying F1TENTH Autonomous Racing: Survey, Methods and Benchmarks](https://arxiv.org/pdf/2402.18558)
-   [Disparity Extender](https://www.nathanotterness.com/2019/04/the-disparity-extender-algorithm-and.html)
-   [A novel obstacle avoidance algo- rithm:“follow the gap method”](https://www.sciencedirect.com/science/article/abs/pii/S0921889012000838)
