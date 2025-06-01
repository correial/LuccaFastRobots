+++
title = "Lab 11: Localization on the Real Robot"
date = 2025-05-03
weight = 3
[taxonomies]
tags = ["Robotics", "C++", "Sensors", "Python", "Embedded Software", "Microcontroller" ]
+++

## Simulation  

I started by verifying that the `lab11_sim.ipynb` file successfully produces the same localization results as from lab 10. From the image below, you can see that we obtain the expected outcome. Again, you can see the unreliability of the odometry model...

<img src="/Fast Robots Media/Lab 11/Sim.png" alt="Alt text" style="display:block;">
<figcaption>Localization Simulation</figcaption>

- Blue: belief pose
- Red: odometry model
- Green: ground truth

## Robot Implementation

### Python Code

Until next lab we do not have a way of determening ground truth for the robot; for this lab only the update step of the Bayes filter is implemented via the `perform_observation_loop()` function shown below. Its purpose is to perform the observation loop behavior on the real robot and return two column numpy arrays: (1) ToF range values (meters) and (2) sensor bearings in degrees.

In the function you can see that the `SPIN` case is called over BLE. It is responsible for conducting data collection and is further discussed in the Arudino Code section below. After the data is sent over BLE post collection, it is piped into a CSV file via the notification handler and passed into the localization algorithm by parsing through the CSV.

```python
async def perform_observation_loop(self, rot_vel=120):
    
    # Import libraries

    # Clear file
    with open("MappingData.csv", "w") as f:
        f.write("TOF,Gyro\n")

    # Run SPIN case over BLE
    ble.send_command(CMD.SPIN, "5.2|0|0")  # P|I|D

    # Ensure that the mapping file exists and is done populating
    while not os.path.exists("MappingData.csv") or sum(1 for _ in open("MappingData.csv")) < 37:
        await asyncio.sleep(3)

    # Read data from the CSV
    df = pd.read_csv("MappingData.csv")
    sensor_ranges = (df["TOF"].values / 1000)[np.newaxis].T  # shape (N, 1), in meters
    sensor_bearings = (df["Gyro"].values)[np.newaxis].T
    print(sensor_ranges)
    print(sensor_bearings)

    return sensor_ranges, sensor_bearings
    
    raise NotImplementedError("perform_observation_loop is not implemented")
```

### Arduino Code

A good deal of code and groundwork was reused from my lab 9 in order to have the robot spin on axis and collect DMP and ToF data. Angular PID was used via the DMP to reliably arrive at each target angle. The main difference is that for localization, I wanted to ensure that each measurment was as accurate as possible and that only a set number of ToF measurments were being taken. To achieve this, the ToF measruments were taken only when the robot arrived at each angle. After arriving at each angle, the robot waits for 500 ms to ensure a steady measurment, then the new target angle is set and the robot moves again. 

At first, I tried to take a total of 18 measurments (20° between each). However, this did not provde to be enough measurments for a reliable estimate of pose, thus I doubled it to 36 measurments (with 10° between each). Note that I had to update the `observations_count: 36` line in the `world.yaml` file.

```c++
case SPIN:{
    distanceSensor1.startRanging();
    success = robot_cmd.get_next_value(Kp_turn);
    if (!success) return;
    success = robot_cmd.get_next_value(Ki_turn);
    if (!success) return;
    success = robot_cmd.get_next_value(Kd_turn);
    if (!success) return;

    // Time varaibles
    spin_time = millis();
    start_time_pid_turn = millis();
    last_rotation_time = millis();
    
    while ((spin_counter < spin_len) && (rot_counter <= 36)) {
        
        getDMP();
        runPIDRot();
        
        spinTime[spin_counter] = millis() - spin_time;
        
        elapsed_time = millis() - last_rot_time;
        
        if (arrived) {
        arrived = 0;
        delay(500);
        collectTOF();
        last_rot_time = millis();
        rot_counter = rot_counter + 1;
        target_turn = target_turn + 10;
        }   
        spin_counter = spin_counter + 1;
    }
    stop();
    sendMapData();
    break;
}
```

*Refer to lab 9 for a detailed breakdown of each function and sub function being called.* 

### Results 

I localized about four different coordinates within the world. The true position of the robot is the green dot; the belief based on localization is the blue dot. From the four locations below, you can see that the localization is fairly good at getting close to the exact location of the robot, but is not completely accurate. From analyzing polar plots and ToF data, it appears that the ToF tends to underestimate which accounts for the deviation from the ground truth. For example, if you look at (-3,-2), you can see that the robot believes its closer to the corner than it really is due to this underestimation. 

For lab 12 I to continue troubleshooting the ToFs. One idea is to fuse measurments from the two ToF sensors on the robot to reduce the effect of noise. 

<img src="/Fast Robots Media/Lab 11/Loc_0,3.png" alt="Alt text" style="display:block;">
<figcaption>Localization @ (0,3)</figcaption>

<img src="/Fast Robots Media/Lab 11/Loc_5,-3.png" alt="Alt text" style="display:block;">
<figcaption>Localization @ (5,-3)</figcaption>

<img src="/Fast Robots Media/Lab 11/Loc_-3,-2.png" alt="Alt text" style="display:block;">
<figcaption>Localization @ (-3,-2)</figcaption>

<img src="/Fast Robots Media/Lab 11/Loc_5,3.png" alt="Alt text" style="display:block;">
<figcaption>Localization @ (5,3)</figcaption>

The following video shows the update step being completed at (5,-3)

<iframe width="450" height="315" src="https://www.youtube.com/embed/QVTVIbofqAE"allowfullscreen></iframe>
<figcaption>Localization @ (5,-3) Vid</figcaption>

### Discussion 
A good deal of debugging and trouble shooting was required to make the localization work. The change of 18 ToF to 36 ToF measurments was already discussed above. I also had to angle my ToF sensor up more to reduce ground interference. Furthermore, I played around with the sensor noise found in `world.yaml` and ended up reducing it from `sensor_sigma: 0.1` to `sensor_sigma: 0.05`.

## Collaboration

I collaborated extensively on this project with [Jack Long](https://jack-d-long.github.io/) and [Trevor Dales](https://trevordales.github.io/). ChatGPT was used to help answer Bayes Filter implementation questions.