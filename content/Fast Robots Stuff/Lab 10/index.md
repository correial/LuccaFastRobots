+++
title = "Lab 10: Grid Localization using Bayes Filter"
date = 2025-04-08
weight = 4
[taxonomies]
tags = ["Robotics", "C++", "Sensors", "Python", "Embedded Software", "Microcontroller" ]
+++

## Introduction 

Our robot lack absolute or "ground truth" knowledge of its position. As a result, it must infer its location using information from the environment (ToF sensors)—a process known as localization, which is most effective when approached probabilistically. To implement probabilistic localization, we use a Bayes filter, which maintains a "belief" about the robot's position. Based on its initial state as well as ToF sensor data and control inputs, the robot forms an estimate of where it might be. As new sensor readings and inputs are received, the robot continuously updates this belief using Bayesian inference. This lab aims to implement Bayes Filter via simulation before implementing on the robot.

The robot state is 3 dimensional and is given by (*x, y*, $ \theta $). The robot’s world is a continuous space that spans with dimensions:

- -5.5 ft < x < 6.5 ft in the x direction
- -4.5 ft < x < 4.5 ft in the y direction
- -180°, +180° along the theta axis

The robot's environment is represented by a grid with cells measuring 0.3048 m x 0.3048 m x 20° for a total number of cells along each axis being (12,9,18). Each cell stores the probability of the robot being in that position, with all probabilities summing to 1 to represent the robot's belief. The Bayes filter updates these probabilities over time, and the cell with the highest probability at each step indicates the robot's most likely pose, forming its estimated trajectory.

### Bayes Filter

The Bayes filter is a iterative process that begins by estimating the robot's next position using control inputs, and then refines this estimate based on sensor measurements. Here is the model skeleton:

<div id="line1"></div>
<div id="line2"></div>
<div id="line3"></div>
<div id="line4"></div>
<div id="line5"></div>
<div id="line6"></div>

<div style="width: fit-content; margin: 0 auto; text-align: left;">

**Algorithm** Bayes_Filter *($bel(x_{t-1}), \ u_t, \ z_t$)*  

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**for all** *$x_t$* **do**  

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;*$\overline{bel}(x_t) = \sum_{x_{t-1}} p(x_t \mid u_t, x_{t-1}) \ bel(x_{t-1})$*  

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;*$bel(x_t) = \eta \ p(z_t \mid x_t) \ \overline{bel}(x_t)$*  

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**end for**  

**return** *$bel(x_t)$*  

</div>

The prediction model can be broken down into two main parts:
1. Prediction Step: the robot estimates its current position based on its previous pose and control inputs 
2. Update Step: this is the correction step of the Bayes Filter, where the predicted belief and sensor liklihoods are incorperated to decreases the uncertainty in the robot’s position.

### Odometry Motion Model

For our purpose, control input u is expressed via an Odometry Motion Model which describes the difference between two positions or "poses". This difference is composed of three parameters: an initial rotation, a translation, and a final rotation.

## Algorithm Implementation

The following functions were used as the logic and implementation for the Bayes filter

### Compute Control

The `compute_control()` function takes two paramenters: current pose and a previous pose. From the parameters it determines the initial rotation, translation, and final rotation which are sufficient to reconstruct the relative motion between two robot states.

<img src="/Fast Robots Media/Lab 10/rotTrans.png" alt="Alt text" height = 300 style="display:block;">
<figcaption>Odometry Model Parameters</figcaption>

Below are the equations and code used to calculate and return the rotations and translation

<div align="center">

$\delta_{rot1} = \text{atan2}(\bar{y}' - \bar{y}, \bar{x}' - \bar{x}) - \bar{\theta}$

$\delta_{trans} = \sqrt{(\bar{y}' - \bar{y})^2 + (\bar{x}' - \bar{x})^2}$

$\delta_{rot2} = \bar{\theta}' - \bar{\theta} - \delta_{rot1}$

</div>

```python
def compute_control(cur_pose, prev_pose):
    # Extract x, y, and yaw (prev and cur)
    x_prev, y_prev, theta_prev = prev_pose
    x_cur, y_cur, theta_cur = cur_pose

    # Calculate translation in x and y and straight-line
    dx = x_cur - x_prev
    dy = y_cur - y_prev

    delta_trans = np.sqrt(dx**2 + dy**2)

    # Rotation 
    delta_rot_1 = np.degrees(np.arctan2(dy, dx)) - theta_prev # First rotation: from previous heading to direction of movement
    delta_rot_2 = theta_cur - np.degrees(np.arctan2(dy, dx)) # Final rotation: from direction of movement to current heading
    
    # Normalize rotations [-180, 180]
    delta_rot_1 = mapper.normalize_angle(np.degrees(np.arctan2(dy, dx)) - theta_prev)
    delta_rot_2 = mapper.normalize_angle(theta_cur - theta_prev - delta_rot_1)

    return delta_rot_1, delta_trans, delta_rot_2
```

### Odometry Motion Model

The `odom_motion_model()` function takes in a current pose and a previous pose and is responsible for  predicting the probability of where a robot might be after applying noisy motion commands. Extracting the current pose, previous pose, and actual vs. predicted control inputs via `compute_control()`, it  calculates the probabilities density of those parameters as Gaussian distributions passing in the standard deviations for rotation and translation noise.

```python
def odom_motion_model(cur_pose, prev_pose, u):
   
    delta_rot_1, delta_trans, delta_rot_2 = compute_control(cur_pose, prev_pose)

    prob_rot_1 = loc.gaussian(delta_rot_1, u[0], loc.odom_rot_sigma)
    prob_trans = loc.gaussian(delta_trans, u[1], loc.odom_trans_sigma)
    prob_rot_2 = loc.gaussian(delta_rot_2, u[2], loc.odom_rot_sigma)

    prob = prob_rot_1 * prob_trans * prob_rot_2

    return prob
```

### Prediction Step

The `prediction_step()` function estimates the robot's predicted belief by applying the odometry motion model to account for movement uncertainty. It takes the current and previous odometry readings, extracts control parameters, and iterates over all possible previous and current poses within the discretized 3D state space (x, y, θ). For each possible transition, it calculates the probability of moving from a previous pose to a current pose using `odom_motion_model()`. These probabilities are accumulated to form a predicted belief (bel_bar), which is then normalized to maintain a valid probability distribution.

To improve computational efficiency, `prediction_step()` skips previous states with extremely low probabilities (below 0.0001) since they contribute negligibly to the final belief. This trade-off sacrifices a small amount of accuracy and completeness in favor of significantly faster execution.

```python
def prediction_step(cur_odom, prev_odom):

    # Control input tuple
    u = compute_control(cur_odom, prev_odom)

    # Initialize belief grid
    loc.bel_bar = np.zeros((mapper.MAX_CELLS_X, mapper.MAX_CELLS_Y, mapper.MAX_CELLS_A))

    # Iterate over all prior belief states
    for prev_x in range(mapper.MAX_CELLS_X):
        for prev_y in range(mapper.MAX_CELLS_Y):
            for prev_theta in range(mapper.MAX_CELLS_A):
                bel_prev = loc.bel[prev_x, prev_y, prev_theta]

                # Skip smaller probabilities
                if bel_prev < 0.0001:
                    continue

                # Convert previous index to pose
                prev_pose = mapper.from_map(prev_x, prev_y, prev_theta)

                # Iterate over all possible current states
                for cur_x in range(mapper.MAX_CELLS_X):
                    for cur_y in range(mapper.MAX_CELLS_Y):
                        for cur_theta in range(mapper.MAX_CELLS_A):
                            cur_pose = mapper.from_map(cur_x, cur_y, cur_theta)

                            # Motion model: P(cur_pose | prev_pose, u)
                            prob = odom_motion_model(cur_pose, prev_pose, u)

                            loc.bel_bar[cur_x, cur_y, cur_theta] += prob * bel_prev

    # Normalize
    loc.bel_bar /= np.sum(loc.bel_bar)
```

### Sensor Model

The robot's sensors are assumed to have Gaussian noise in their readings. The `sensor_model()` function, like the odometry motion model, calculates the likelihood of data but based on sensor measurements instead of movement. It essentially takes a set of sensor observations at a given pose and returns the probability of receiving those observations.

```python 
def sensor_model(obs):

    prob_array = np.zeros(mapper.OBS_PER_CELL)
    
    for i in range(mapper.OBS_PER_CELL):
        # Calculate the likelihood of each sensor reading
        prob_array[i] = loc.gaussian(obs[i], loc.obs_range_data[i], loc.sensor_sigma)

    return prob_array
```

### Update Step

Finally, the `update_step()` function loops through the current state grid, calls the `sensor_model()` to obtain sensor likelihoods, and updates the belief accordingly. The updated belief is then normalized to ensure it sums to 1.

The below equation expresses how to calculate the total likelihood of a full set of sensor measurements *zₜ* given a robot’s pose *xₜ* and map *m*.

$$
p(z_t \mid x_t, m) = \prod_{k=1}^{18} p(z_t^k \mid x_t, m)
$$

```python
def update_step():

    # Loop through all poses (x, y, theta)
    for cur_x in range(mapper.MAX_CELLS_X):
        for cur_y in range(mapper.MAX_CELLS_Y):
            for cur_theta in range(mapper.MAX_CELLS_A):
                # Get expected sensor reading at this pose
                expected_obs = mapper.get_views(cur_x, cur_y, cur_theta)

                # Compare actual vs expected using the sensor model
                p = sensor_model(expected_obs)  # p is array of individual sensor probabilities

                # Update belief using product of individual sensor likelihoods
                likelihood = np.prod(p)

                # bel(x) = bel_bar(x) * p(z|x)
                loc.bel[cur_x, cur_y, cur_theta] = likelihood * loc.bel_bar[cur_x, cur_y, cur_theta]

    # Normalize final belief
    loc.bel /= np.sum(loc.bel)
```

## Simulation

The video below shows the shows a trajectory and localization simulation using the Bayes filter. The trajectory is pre-planned to move around the box. Here, the odometry model is plotted in red while the ground truth tracked by the simulator is plotted in green. From the video you can see the inaccuracy of the odometry model when operating in isolation. 

The probabilistic belief is plotted in blue which you can see is a very good approximation of the robot's true position. The probability distribution is also shown using the white boxes where stronger shades of white represent stronger probabilities. Note that we ignored all cells with probability <0.0001. 

<iframe width="450" height="315" src="https://www.youtube.com/embed/XKlgeoXnjIc"allowfullscreen></iframe>
<figcaption>Bayes Simulation</figcaption>

Notice that the initial estimation before a new sensor value is based on the odemetry model which makes sense. I also found that The Bayes Filter seems to perform better when the robot is near walls. This is likely due to the sensors being more accurate and consistent at sensing closer distances, and therfore more trustworthy. On the other hand, when it is in more open spaces such as the center of the map, the filter is slightly less accurate.


## Collaboration

I collaborated extensively on this project with [Jack Long](https://jack-d-long.github.io/) and [Trevor Dales](https://trevordales.github.io/). I referenced [Stephan Wagner's site](https://fast.synthghost.com/) for help with implementing the Bayes Filter. ChatGPT was used to help answer Bayes Filter implementation questions and conceptual misunderstandings.



