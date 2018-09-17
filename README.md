# Path Planning Project in Highway Scenario

[//]: # (Image References) 
[image1]: ./images/PathPlanning.png
[image2]: 
[image3]: 
   
### Goals

![alt text][image1]

In this project goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data is provided 
and there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, 
note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, 
unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, 
it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates
["y"] The car's y position in map coordinates
["s"] The car's s position in frenet coordinates
["d"] The car's d position in frenet coordinates
["yaw"] The car's yaw angle in the map
["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator
["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value
["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Implementation Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. 

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed.

## Algorithm

1. Fetch all cars' sensor fusion data to detect collision possibility. Collision is possible if the gap between the leading car and ego car is less than certain threshold, which chosen to be 50 meters.
   We check the Frenet's s-coordinate to determine ift he car is in front in the lane and if so if it is within 30 meters distance.

```cpp
		for (int i = 0; i < sensor_fusion.size(); i++)
		{
			// This Car's current lane
			float d = sensor_fusion[i][6];
			// 0-3 - lane 0, 4-7, lane 1, 8-11, lane 2
			int check_car_lane = (int) d / 4;

			// Check if car is in ego car's reference lane
			double vx = sensor_fusion[i][3];
			double vy = sensor_fusion[i][4];
			double check_speed = sqrt(vx * vx + vy * vy);
			double check_car_s = sensor_fusion[i][5];


			check_car_s += ((double)prev_size * 0.02 * check_speed);

			// Check if collision possible. Logic: if car infront is less than 30 m, handle it
			if ((check_car_s > car_s) && ((check_car_s - car_s) < 30) && (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)))
			{
				cout << "Detected car infront" << endl;
				too_close_in_lane = true;
			}

			if (abs(check_car_s - car_s) <= 30)
			{
				if (lane == 0)
				{
					safe_left = false;

					if (check_car_lane == 1 /*|| check_car_lane == 2*/)
						safe_right = false;

				}

				if (lane == 1)
				{
					if (check_car_lane == 0)
						safe_left = false;
					else if (check_car_lane == 2)
						safe_right = false;
				}

				if (lane == 2)
				{
					safe_right = false;

					if (/*check_car_lane == 0 || */check_car_lane == 1)
						safe_left = false;
				}

			}
		}
```


2. While detecting if the car is too close in front, we also detect if it is safe to switch lane to the left or right. It is safe to switch lane if there is no
   car in the left or right lane within 30 meters of distance (both ahead and behind car's current position).

3. If the car is too close and if it is safe enough to switch to left or right lane, we make the switch else After fetching the we slow down. This is done in below piece of code:

```cpp
 		// If too close - Change lane to left, right or slow down
		if (too_close_in_lane)
		{
			cout << "Detected car is too close in lane. Need to switch left or right" << endl;

			if (safe_left) // If it safe to switch left lane, then switch
			{
				cout << "Switching to left lane." << endl;
				lane--;
			}
			else if(safe_right) // If it is safe to switch right lane then switch
			{
				cout << "Switching to right lane." << endl;
				lane++;
			}
			else // Not safe to switch either left or right, so only option is to slow down
			{
				cout << "Slowing down." << endl;
				ref_vel -= 0.224;
			}
		} 
		else if (ref_vel < 49.5)
		{
			ref_vel += 0.224;
			cout << "ref_vel = " << ref_vel << endl;

		}
```

4. Way Point generation: Simulator provides us previous path traversed by the car. For the very first iteration, there is no previous path, in such case we just use the car's current location as 
   starting reference point.  For all the remaining cases, where we do have previous path, we use last two coordinates of the previous path and another 3 points which are 30, 60 and 90 meters from the 
   car's current position as Anchor points to generate the path.

```cpp
		// If previous size is nearly empty, will use car's current lcoation as starting reference
		if (prev_size < 2)
		{
			double prev_car_x = car_x - cos(car_yaw);
			double prev_car_y = car_y - sin(car_yaw);

			pts_x.push_back(prev_car_x);
			pts_x.push_back(car_x);

			pts_y.push_back(prev_car_y);
			pts_y.push_back(car_y);
		}
		else
		{
			ref_x = previous_path_x[prev_size-1];
			ref_y = previous_path_y[prev_size-1];

			double ref_x_prev = previous_path_x[prev_size-2];
			double ref_y_prev = previous_path_y[prev_size-2];

			ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

			pts_x.push_back(ref_x_prev);
			pts_x.push_back(ref_x);

			pts_y.push_back(ref_y_prev);
			pts_y.push_back(ref_y);

		}

		// In frenet add 30m spaced points ahead of the starting reference point
		vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

		pts_x.push_back(next_wp0[0]);
		pts_x.push_back(next_wp1[0]);
		pts_x.push_back(next_wp2[0]);

		pts_y.push_back(next_wp0[1]);
		pts_y.push_back(next_wp1[1]);
```

5. To generate a smooth evenly spaced trajectory, we use spline (http://kluge.in-chemnitz.de/opensource/spline/). Using 5 anchor points we generated in the previous step, we create
   spline that fits polynomial to these anchor points. Spline generation is done in car local coordinates by doing rotation and translation to car's reference x, y and yaw angle. 
   Points generated using spline, are later transformed back to map coordinates.

6. We generate 50 points that are evenly spaced. These 50 points are spaced based on the reference velocity. If the car in front is too close, we need to have the points tightly spaced
   and if we have space in front, points are going to be much more sparsely spaced thus providing acceleration to the vehicle. Below piece of the code achieves this:

```cpp
		// Start with all the previous path points from the last instance
		for (int i = 0; i < previous_path_x.size(); i++)
		{
			next_x_vals.push_back(previous_path_x[i]);
			next_y_vals.push_back(previous_path_y[i]);
		}


		// Breakup spline points such that we can traverse at reference velocity
		double target_x = 30.0;
		double target_y = s(target_x);
		double target_dist = sqrt(target_x * target_x + target_y * target_y);

		double x_add_on = 0;

		for (int i = 0; i <= 50 - previous_path_x.size(); i++)
		{
			double N = (target_dist / (0.02 * ref_vel / 2.24));

			double x_point = x_add_on + (target_x) / N;
			double y_point = s(x_point);

			x_add_on = x_point;

			double x_ref = x_point;
			double y_ref = y_point;

			// Rotate back from car coordinates to map coordinates
			x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
			y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

			x_point += ref_x;
			y_point += ref_y;

			next_x_vals.push_back(x_point);
			next_y_vals.push_back(y_point);
```


## Results

Below youtube shows the traversal of the car around the track. Car maintains <= 50 mph speed limit without any jerk or undesirable acceleartion, without creating any incident and 
travels more than 10 miles in about 14 minutes.

https://www.youtube.com/watch?v=HcM2HTf1VT4

There is further scope for improvement of this path planner. Car does not strategically slow down if there is a possibility to avoid the car in right lane and get to the right most lane
which may be completely free. This algorithm uses a simple finite state machine to handle the lane switches, however a custom-defined cost-function that can handle lane switches more efficiently 
or a reward function based Reinforcement Learning algorithm can be developed.

