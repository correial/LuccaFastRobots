+++
title = "Lab 6: Orientation Control"
date = 2025-03-19
weight = 7
[taxonomies]
tags = ["Robotics", "C++", "Sensors", "Python", "Embedded Software", "Microcontroller" ]
+++

## Bluetooth Communication and PID/DMP Code

PID code is handeled and sent over bluetooth in a very similar fasion to lab 5!

### Main Function

Here is the turning PID part of the main function. The PID turning code waits until the `start_time_pid_turn` flag is set to 1 via the `PID_TURN_CONTROL` command that is responsible for sending the Kp, Ki, and Kd gains as well as sending the starting flag: `ble.send_command(CMD.PID_TURN_CONTROL, "1.5|0|40|90") # P|I|D`. The `getDMP` and `runPIDROT()` functions are covered in detail in the next sections. After the turning PID arrays are filled, the robot is then instructed to stop and the `sendPIDTURNData()` function is used to loop through all arrays and send them over to be piped into a CVS via my notification hander.

**Main Loop**

```c++
if (pidTurn_start) {
  start_time_pid_turn = millis();
  while (counter_turn < pidTurn_len) {
    getDMP();
    runPIDRot();
    counter_turn += 1;
  }
  stop();
  pidTurn_start = 0;
  sendPIDTURNData();
}
```

**Notification Handler**

```c++
def notification_handler(sender, data):

  try:
      entry = data.decode("utf-8").strip()

      # Split by "," then extract values using fast unpacking
      T, X, S, P, I, D, F = entry.split(",")
      T = float(T[2:])
      X = float(X[2:])  
      S = float(S[2:])  
      P = float(P[2:])  
      I = float(I[2:])  
      D = float(D[2:])  
      F = float(F[2:])
      
      # Check if file is empty, then write headers
      with open("PID_data2.csv", "a+") as f:
          f.seek(0)
          if f.read(1) == "":
              f.write("Time,Gyro,Speed,P,I,D,Filtered_D\n")
          f.write(f"{T},{X},{S},{P},{I},{D},{F}\n")

  except Exception as e:
      print(f"Error parsing data: {e}")```
```

### getDMP() Function

The `getDMP` function is responsible for updating the global `yaw_gy` varaible whenever there is a new value, and if there is not (pretty rare) the previous value is used to avoid storing zeros in the yaw array. Also, it is worth noting that if we were deriving the yaw angle by taking the integral of the gyroscope's angular velocity over time it would not make sense to then take the derivative. However, since the DMP is providing discrete angle readings, we do need to take these derivatives still. 

```c++
void getDMP() {

  icm_20948_DMP_data_t dmp;
  myICM.readDMPdataFromFIFO(&dmp);

    if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail)  // If more data is available then we should read it right away - and not delay
  {
    delay(10);
    yawData[counter_turn] = yawData[counter_turn-1];
  }

  // Check if the IMU has data
  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
    if ((dmp.header & DMP_header_bitmap_Quat6) > 0) {

      // Scale to +/- 1
      qx = ((double)dmp.Quat6.Data.Q1) / 1073741824.0;  // Convert to double. Divide by 2^30
      qy = ((double)dmp.Quat6.Data.Q2) / 1073741824.0;  // Convert to double. Divide by 2^30
      qz = ((double)dmp.Quat6.Data.Q3) / 1073741824.0;  // Convert to double. Divide by 2^30

      // Start converting quaternion to Euler angles
      qw = sqrt(1.0 - min(((qx * qx) + (qy * qy) + (qz * qz)), 1.0));

      // Yaw (z-axis rotation)
      siny = 2.0 * (qw * qz + qx * qy);
      cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
      yaw_gy = (float)(atan2(siny, cosy) * 180.0 / PI);

      yawData[counter_turn] = yaw_gy;
    }
  }
}
```

### runPIDRot() Function

The `runPIDRot()` function is responsible for calling the PID turning function (`pid_turn_Gyro()`) and ensuring that it has the current distance fed in as a parameter. The function also populates the PID arrays

```c++
void runPIDRot() {

  timesROT[counter_turn] = millis() - start_time_pid_turn;

  curDistance_turn = yaw_gy;

  pidTurn_speed[counter_turn] = pid_turn_Gyro(Kp_turn, Ki_turn, Kd_turn, curDistance_turn, target_turn);

  pidTurn_p[counter_turn] = Kp_turn * pos_error_turn;
  pidTurn_i[counter_turn] = Ki_turn * pos_error * dt_turn;
  pidTurn_d[counter_turn] = Kd_turn * d_term_turn;
  pidTurn_d_filt[counter_turn] = Kd_turn * d_term_turn_filtered;
}
```

### pid_turn_Gyro() Function

There are three key aspects of the PID turning control loop:
1. When I first was testing with proportional and PD control, I noticed that what was really happening was that the speed was just jumping between the floor value on the negative and positve side as seen in the below image. To account for this I decided that I needed to map the calculated speeds (in the range of 0-255) into the range scaled to my robot's floor and ceiling. This was done using the Arduino map command `speedTurn_set_mapped = map(speedTurn_set, 0, 255, minSpeedTurn, maxSpeedTurn)`. The result is that my robot is now using real PID control (which will be shown later) rather than just going between the PWM floor.

<img src="/Fast Robots Media/Lab 6/NoMappingP.png" alt="Alt text" style="display:block;">
<figcaption>No Mapping Control</figcaption>

2. A low pass filter was used on the derivative term to reduce noise and the effect of derivative kick. The alpha value was calculated through a slight trial and error method where I found a balance between reducing noise (from the filtered_d_term) but also reducing speed/amplitude as fast as possible when needing to slow down (from the current d_term). The alpha that optimized this trade off was **0.1**.

3. While testing for hours I noticed that regardless of my gains (I tried Kd terms on the order of 1000), my robot would not be able to stop in time and over shot its target. I realized that it was not a coding issue but rather the cars inability to change directions fast enough. I also noticed that the overshoot only happened on the first approach– if I picked the car up and replaced it at the beginning location after it had been running for a while the car was able to stop perfectly. To account for this, I have the car delay for 1 second at a low PWM before starting. This solved the issue completely!

```c++
int pid_turn_Gyro(float Kp_turn, float Ki_turn, float Kd_turn, float pos_cur_turn, float targ_turn) {

  // Varaible Init
  time_cur_turn = millis();

  dt_turn = time_cur_turn - time_prev_turn;

  time_prev_turn = time_cur_turn;

  pos_error_turn = pos_cur_turn - targ_turn;  // Position error

  pid_ori_recip = targ_turn - (targ_turn < 0 ? -1 : 1) * 180;

  if (abs(pos_error_turn) > 180.0) {
      pos_error_turn += (pid_ori_recip - pos_cur_turn) / abs(pid_ori_recip - pos_cur_turn) * 360.0;
  }

  d_term_turn = (pos_error_turn - prev_error_turn) / dt_turn;  // Derivative term

  // LPF on derivative term
  prev_filt_D_term_turn = d_term_turn_filtered;
  d_term_turn_filtered = D_LPF_ALPHA * d_term_turn + (1-D_LPF_ALPHA) * prev_filt_D_term_turn;

  integral_error_turn = integral_error_turn + (pos_error_turn * dt_turn);  // Integral Term

  // Set Speed PID Calculation
  int speedTurn_set = ((Kp_turn * pos_error_turn) + (Ki_turn * pos_error * dt_turn) + (Kd_turn * d_term_turn_filtered));  

  // Mapping original PWM speed to new bound
  if (speedTurn_set > 0){
  speedTurn_set_mapped = map(speedTurn_set, 0, 255, minSpeedTurn, maxSpeedTurn);
  } else {
  speedTurn_set_mapped = map(speedTurn_set, -255, 0, -maxSpeedTurn , -minSpeedTurn);
  }

  if (abs(pos_error_turn) < 2) {
    stop();
    speedTurn_set_mapped = 0;
  }

  // Delay to avoid overshoot
  if((time_cur_turn - start_time_pid_turn) < 1000 ){
    speedTurn_set_mapped = minSpeedTurn - 20;
    time_cur_turn = millis();
  }

  // Set forward speed
  else if (speedTurn_set_mapped > 0) {
    turnRight(speedTurn_set_mapped);
  }

  // Set backwards speed
  else if (speedTurn_set_mapped < 0) {
    turnLeft(-1*speedTurn_set_mapped);
  }

  prev_error_turn = pos_error_turn;
  return speedTurn_set_mapped;
}
```

## PID Control and Gains Discussion

### PID Gains

After much testing, I decided that the best final gains were **Kp=1.65** and **Kd=130**. The Kp term is high enough to adjust and continue moving even when at small angles from the target. At the same time, it is not so big that it overshoots a large angle from the target. The Kd term is responsible for slowing down the car as it approaches the target angle. It is large enough to help slow the car down and reduce overshoot. With hours of tuning, I realized that if I continued to increase the Kd term, there would be more instability as I osccilate quickly about the target distance. The progression of choosing and testing gains is shown later. I found that my robot was able to arrive within a degree or two of the target distance and thus without the presence of considerable external noise, I did not see the need to implement integral control as well.

## Testing

### Disturbance Correction (PD)

Here are my two final disturbance tests. The first is with **Kp=1.8** and **Kd=50**. You can see that the car slightly overshoots but is able to come within a few degrees of the 90 degree target each time it restabalizes.

<iframe width="450" height="315" src="https://www.youtube.com/embed/pXpwFAQoIhk"allowfullscreen></iframe>
<figcaption>Disturbance Correction Kp=1.8 | Kd=50</figcaption>

Here is the other final disturbance test with **Kp=1.65** and **Kd=130**. You can see that for these gains, the robot overshoots less but is just a little worst at re-aligning exactly at 90 degrees. Thus while it overshoots less, it is slower when moving to a set angle (with the increaed Kd gain that helps it slow down earlier). 

<iframe width="450" height="315" src="https://www.youtube.com/embed/S883NBawxys"allowfullscreen></iframe>
<figcaption>Disturbance Correction Kp=1.65 | Kd=130</figcaption>

Here is the plotted data after being sent over bluetooth:

<img src="/Fast Robots Media/Lab 6/FinalBumper.png" alt="Alt text" style="display:block;">
<figcaption>Disturbance Correction Kp=1.65 | Kd=130</figcaption>

You can certainly see some derivative kick with the increased Kd term, however, the LPF does take care of it for the most part.

## Turning Way Points

To start thinking about future applications of the PID controller I added waypoints that the robot would turn to. During navigation this can be implemented along side of the straightline PID controller; the robot doesn't need to turn in an arc while it drives, instead it can drive straight, pivot, then drive straight again. In this example, the robot turns from 0 degrees to 90 degrees, then back to 0 degrees, then ends at 120 degrees:

<iframe width="450" height="315" src="https://www.youtube.com/embed/RQPXhLDtzWo"allowfullscreen></iframe>
<figcaption>Waypoints</figcaption>

Here is the graph for my waypoint setting:

<img src="/Fast Robots Media/Lab 6/WayPoint Setting.png" alt="Alt text" style="display:block;">
<figcaption>Waypoint Data</figcaption>

You can certianly see some more derivative kick, but again, it is filtered out well by the LPF.

## Sampling Time Discussion

The IMU polling rate is set to 1.1 kHz in the line: `success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);`. The sampling time for our system is the DMP output rate ODR which is set in my setup function: `success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 2) == ICM_20948_Stat_Ok);`. The 2 sets the ODR to an overall 549 Hz, plenty fast for the PID turning loop.

$$ ODR = \frac{\text{DMP running rate}}{\text{ODR setting}} - 1 $$

**Polling rate** is 1.1 kHz (1100 Hz)

$$ ODR = \frac{1100}{2} - 1 = 550 - 1 = 549 \text{ Hz} $$


## Collaboration

I collaborated extensively on this project with [Jack Long](https://jack-d-long.github.io/) and [Trevor Dales](https://trevordales.github.io/). I referenced [Stephan Wagner's site](https://fast.synthghost.com/) for much of the lab but most specifically for implementing the DMP! ChatGPT was used to help plot graphs.



