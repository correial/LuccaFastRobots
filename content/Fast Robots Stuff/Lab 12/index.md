+++
title = "Lab 12: Path Planning and Execution"
date = 2025-05-12
weight = 2
[taxonomies]
tags = ["Robotics", "C++", "Sensors", "Python", "Embedded Software", "Microcontroller" ]
+++

## Overall  

I worked with Trevor and Jack to navigate our robot through a preset route of waypoints assigned to us in the lab (shown below). Overall, we were very successful in arriving and navigating through these waypoints completely autonomously with a combination of on-board PID control and off-board localization processing! The final result was incredibly rewarding as it required us to combine and implement various aspects of code and hardware debugging learned throughout the semester in addition to navigation and waypoint logic. 

We began with complementary arrays of waypoints and localization booleans (i.e. a point, and a boolean to signal the script to localize at that waypoint). This allowed us to manually assign which points we localized at based on the accuracy of localization at each waypoint for the most accurate navigation. To navigate between points, we used sequential orientation and position PID using the DMP onboard our IMU, and Kalman-filtered TOF sensor, respectively.

We used my robot for this lab, and collaborated jointly on both Arduino and Python code components! 

<img src="/Fast Robots Media/Lab 12/Plan.png" alt="Alt text" Height = 400 style="display:block;">
<figcaption>Navigation Plan</figcaption>

## Python Implementation

### Python Code

Below is the core logic and control flow for our entire lab. It is responsible for calling all functions and Arduino commands. The python code is explained in length below and all Arduino commands are covered in the next section.


```python
async def recieveData(localizing, current_waypoint):
# return current location, current orientation
    current_loc = [0, 0]
    print("localizing: ",localizing)
    if localizing:
        await loc.get_observation_data()
        
        # Run Update Step
        loc.update_step()
        belief = loc.plot_update_step_data(plot_data=True)
        
        current_loc[0] = 3.281*belief[0] # x
        current_loc[1] = 3.281*belief[1] # y

    else:
        #assume we are at the waypoint
        current_loc = [current_waypoint[0], current_waypoint[1]] #set current location to x,y of the current waypoint
        cmdr.plot_bel(current_waypoint[0]/3.281, current_waypoint[1]/3.281)
       
    return current_loc

def calculateTargetAngle(x, y, next_waypoint):
    # based on current location, calculate and return required angle
    x2 = next_waypoint[0]
    y2 = next_waypoint[1]
    
    angle_rad = math.atan2(y2 - y, x2 - x)
    target_angle_deg = math.degrees(angle_rad)
   
    return target_angle_deg

def calculateTargetDistance(x, y, current_tof, next_waypoint):
    #Implement logic

    x2 = next_waypoint[0]
    y2 = next_waypoint[1]
   
    distance_to_next_waypoint = math.sqrt((x2 - x)**2 + (y2 - y)**2)
    distance_to_next_waypoint = distance_to_next_waypoint*304.8 #convert from ft to mm
   
    target_distance_mm = current_tof - distance_to_next_waypoint
    return target_distance_mm

# Reset Plots
cmdr.reset_plotter()

# Init Uniform Belief
loc.init_grid_beliefs()

waypoint_list = [(-4, -3), (-2, -1), (1, -1), (2, -3), (5, -3), (5, -2), (5, 3), (0, 3),(0, 0)]
localize_flags = [False, True, False, False, False, True, True, True, True]  # encode whether to localize at a waypoint

current_loc = [-4,-3] #initialize starting loc as the first waypoint
current_tof = 0 # [mm]
cmdr.plot_bel(current_loc[0]/3.281, current_loc[1]/3.281)

while True:
   
    for i in range(len(waypoint_list)): # i represents current waypoint (starts at zero)

        target_angle = calculateTargetAngle(current_loc[0], current_loc[1], waypoint_list[i+1])
        print("Target angle:", target_angle)
   
        ble.send_command(CMD.PID_TURN_CONTROL, f"1.1|0|120|{target_angle}") # P|I|D

        await asyncio.sleep(5)

        ble.send_command(CMD.SEND_CURRENT_POSE, "")
        
        await asyncio.sleep(1.0)

        df = pd.read_csv("MappingData.csv")
        current_tof = df["TOF"].iloc[0]
        
        print("Current TOF:", current_tof)

        target_distance = calculateTargetDistance(current_loc[0], current_loc[1], current_tof, waypoint_list[i+1])

        print("Target Distance:", target_distance)
        
        ble.send_command(CMD.PID_CONTROL, f".14|0|40|{target_distance}") # P|I|D

        await asyncio.sleep(10.0)

        current_loc = await recieveData(localize_flags[i], waypoint_list[i+1])
        print("Current Location:", current_loc)

        with open("MappingData.csv", "w") as f:
            f.truncate()


        print("------------------------------------------------------------")
```

### Core Logic and Control Flow 

The navigation path is defined by a list of 2D waypoints `waypoint_list`, with each point representing a target location in feet. An accompanying list of boolean flags `localize_flags` determines whether the robot should re-localize its position at each waypoint using a grid-based localization system.

The script continuously loops through all waypoints in sequence, executing the following steps for each:

1. Turn Toward the Next Waypoint:
    - Calculates the angle between the robot’s current position and the next waypoint using the `atan2` function.
    - Sends a PID control command over Bluetooth to rotate the robot to the calculated heading (`CMD.PID_TURN_CONTROL` , described in artemis section).

2. Read Current TOF Sensor Data:
     Requests the current pose from the robot (`CMD.SEND_CURRENT_POSE`, described in artemis section).
    - Extracts the distance reading from the CSV.

3. Move Toward the Waypoint:
    - Computes the expected distance to the next waypoint based on current position belief.
    - Subtracts the distance to the next point from the current TOF reading to calculate the necessary target distance for the next motion.
    - Sends a PID control command over Bluetooth to move the robot forward to the calculated target. (`CMD.PID_CONTROL` , described in artemis section)

4. Update Robot Location:
    - If localization is enabled for the current waypoint, the robot turns to zero degrees and then performs an observation scan (function `perform_observation_loop()` from Lab 11) and sends the data through BLE. Then, an update step is performed to estimate the robot’s current position.
    - If localization is disabled, the robot’s position is assumed to be exactly at the waypoint.

## Arduino Implementation 

Note that all data sent over from the Artemis to the computer over BLE is processed and piped into a CSV file by a notification handler. This CSV file is reset at specific points throughout the code before new ToF and Gyro data comes in.

#### CMD.PID_TURN_CONTROL

This case is responsible for using orientation PID control to arrive at a desired target angle. 

```c++
// Get PID parameters over BLE
success = robot_cmd.get_next_value(Kp_turn);
if (!success) return;
// Same for Kd & Ki
...

success = robot_cmd.get_next_value(target_turn);
if (!success) return;
tx_estring_value.clear();

// Variables used for cutoff timer (5 s)
...

int turn_cutoff = 5000;
while ( (millis() - turn_starttime) < turn_cutoff ) { 
    getDMP();
    runPIDRot();
    counter_turn += 1;

}
stop();
pidTurn_start = 0;
clearVariables();
break;
```

Initially, an `arrived` boolean flag (triggered when the robot arrived within a threshold of the target angle) was used to end the PID control. However, this meant that if we slightly overshot the target, the flag would still say that we arrived as we did pass the target. At this point, the robot has overshot but the arrived flag is true and the PID control will no longer run. We tried to severly overdampen the system to fix this, however it left us with too much steady state error. Our solution was implementing the 5 second cutoff timer before stopping the PID; this gave us plenty of time to approach angles and eliminated the issue of early PID stoppage. The `getDMP()` and `RunPIDRot` variables are explain in great detail in lab 6. The `clearVariables()` function is seen in all cases used in this lab; it clears all varaibles in the entire arduino script at the end of cases so the next case is ready to run without any memory overflow. 

#### CMD.SEND_CURRENT_POSE

This case is responsible for sending a ToF reading to the computer over BLE to help inform the distance needed to travel. 

```c++
distanceSensor1.startRanging();

while(!distanceSensor1.checkForDataReady()) {
    delay(10); // delay while not ready 
}

collectTOF();

// Send Data
tx_estring_value.clear();
tx_estring_value.append("T:");
tx_estring_value.append(TOF1[0]);
tx_characteristic_string.writeValue(tx_estring_value.c_str());

clearVariables();
break;
```

`collectTOF`is explained in many previous labs; it is responsible for collecting a TOF value and appending it to the `TOF1` array.

#### CMD.PID_CONTROL

This case has a similar purpose to `CMD.PID_TURN_CONTROL`, except it is responsible for linear PID control rather than rotational. 

```c++
distanceSensor1.startRanging();
pid_start = 0;
// Get PID parameters over BLE
success = robot_cmd.get_next_value(Kp);
if (!success) return;
// Same for Kd & Ki
...

success = robot_cmd.get_next_value(target_tof);
if (!success) return;
tx_estring_value.clear();

// Variables used for cutoff timer (7 s)
...
int linear_cutoff = 7000; 

while ((millis() - linear_starttime) < linear_cutoff) { // hard time cutoff
    runPIDLin();
    counter_lin = counter_lin + 1;
}
    stop();
    clearVariables();
    run = 0;

    break;
```

`runPIDLin()` is the same function used in Lab 7 for integration of Kalman filtering onto the robot; it is responsible for running the linear PID control and is explained in Lab 7 and 5. Just as in orientation PID control, a cutoff timer is used to eliminate the early ending of PID control that occurs with am arrived flag.

#### CMD.SPIN

This command is called in the `perform_observation_loop()` function and is responsible for localization gyro and ToF data collection; it is explained in lab 11. The only change is the swapping out of an arrived flag for a timer just as for orientation and linear PID cases. 

## Results

Below are a series of trials from our lab! In the last two runs you can see a live belief map and logging in jupyter notebook which ouputs critical information such as if we are localizing at that specific waypoint, our current belief pose, our calculated target heading, etc. To try and speed up our runs, we did not localize at every point since our waypoint setting algorithm and PID control was fairly good at getting us between shorter distances.

### Early Run

<iframe width="450" height="315" src="https://www.youtube.com/embed/_luMDGDUHVw"allowfullscreen></iframe>
<figcaption>Early Run</figcaption>

We saw much success with the structure of our high-level planning on the first run, but ran into trouble with inconsistencies in lower level control and angle targeting. When we localized, our planner output was correct based on position belief, but we struggled to tune the orientation PID loop in such a way that it responded (similarly) well to changes in angle between 10 and 180 degrees, while settling within a reasonable time. We changed our approach to add a time cutoff to both control loops, sacrificing a bit of accuracy for longer-term operation.

### Nearly There

<iframe width="450" height="315" src="https://www.youtube.com/embed/ufMTsQO14IY"allowfullscreen></iframe>
<figcaption>Almost Perfect</figcaption>

During this run we had to nudge robot at (-5,-3) due to a ToF underestimation. Also, the derivative term blew up at (5,3) while localizing so we had to manually face it at the temporary target angle. Also apologies as we dont scroll down on the jupyter lab script until later in the video so some of the outputs are hidden. 

### Final Run!

<iframe width="450" height="315" src="https://www.youtube.com/embed/LJK30JyzM_U"allowfullscreen></iframe>
<figcaption>Final Run</figcaption>

It was awesome to see everything from the semester really come together perfectly during this run! You can see the localization do a great job of correcting for ToF underestimating and overall noise; coordinate (5,3) is a great example of this where it is short of the marker but accounts for it by angling slightly upward to approach the next waypoint where it then arrives with complete precision. 

## Collaboration

Thank you to all the course TAs for all of the time and support during this entire semester! I worked completely with [Jack Long](https://jack-d-long.github.io/) and [Trevor Dales](https://trevordales.github.io/). ChatGPT was used for checking specifics of implementing math functions in python.