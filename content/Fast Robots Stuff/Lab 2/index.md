+++
title = "Lab 2: IMU"
date = 2025-02-20
weight = 11
[taxonomies]
tags = ["Robotics", "C++", "Sensors", "Python", "Embedded Software", "Microcontroller" ]
+++

### IMU Setup

#### AD0_VAL & Initial Data Observations

The AD0_VAL represents the last bit of the I2C address. In our case, 1 is set as the default and that shouldn’t be changed unless the ADR on the board is closed via solder and then should be set to 0. 

After testing with the example code as well as the lecture 4 code, it appears that the accelerometer and gyroscope data print as expected. Three axis are printed for both sensor as well as the corresponding unit (mg for acceleration and DPF for the gyroscope). As discussed in lecture, in the data you can see accelerations and rotations being tracked but not absolute position; changes in the printed values only occur with movement. Additionally, you can see that the changing values are dependent on the axis being rotated about and the sensor being observed. When rotating around the z-axis you can see that the accelerometer data does not change, however the gyroscope does. When accelerating the board along the x-axis you can see a change in value for the accelerometer x-axis but not the gyroscope. 

<img src="/Fast Robots Media/Lab 2/IMUandArtemis.png" height = 400 alt="Alt text" style="display:block;">
<figcaption>Artemis and IMU Connection with BLUE LED Indicator</figcaption>

### Accelerometer

#### Accelerometer Data to Pitch and Roll Conversion

``` c++
pitch_a = atan2(myICM.accX(), myICM.accZ()) * 180 / M_PI; 
roll_a  = atan2(myICM.accY(), myICM.accZ()) * 180 / M_PI; 
```

<iframe width="450" height="315" src="https://www.youtube.com/embed/JZWDCYHxN64" allowfullscreen></iframe>
<figcaption>Video of IMU Testing - full screen to reduce blur</figcaption>

<div style="display: flex; gap: 10px; justify-content: center; align-items: center; white-space: nowrap; overflow-x: auto;">
    <figure style="text-align: center; margin: 0;">
        <img src="/Fast Robots Media/Lab 2/0 Ouput.png" alt="Alt text">
        <figcaption>0°</figcaption>
    </figure>
    <figure style="text-align: center; margin: 0;">
        <img src="/Fast Robots Media/Lab 2/P -90.png" alt="Alt text">
        <figcaption>Pitch @ -90°</figcaption>
    </figure>
    <figure style="text-align: center; margin: 0;">
        <img src="/Fast Robots Media/Lab 2/P 90.png" alt="Alt text">
        <figcaption>Pitch @ 90°</figcaption>
    </figure>
    <figure style="text-align: center; margin: 0;">
        <img src="/Fast Robots Media/Lab 2/R -90.png" alt="Alt text">
        <figcaption>Roll @ -90°</figcaption>
    </figure>
    <figure style="text-align: center; margin: 0;">
        <img src="/Fast Robots Media/Lab 2/R 90.png" alt="Alt text">
        <figcaption>Roll @ 90°</figcaption>
    </figure>
</div>

As you can see from the frozen frames, the accelerometer is very accurate with vary little variation in angle from the expected value. As a result I do not think it is nessesary to do a two-point calibration.

#### Data Collection and Plotting Code

The following code was used to collect the data in arrays and then use Juypter to pipe the data from the Artemis into a CSV file and graph. Note that as I added more arrays to store more data (LPF, Gyro, Complementary Filter) I simply added more columns to the csv via the notification handler. The graphs will be shown later in the lab for analysis.

Artemis Aruino Code:
```c++
case GET_ACC_READINGS: {
    
    float Millis_Cur2 = 0;
    float pitch_a = 0;
    float roll_a = 0;

    if (!success) {
        return;
    }

    // First, collect all data
    for (int i = 0; i < lenarr; i++) { 
    
    // Ensure new data is available before recording
    while (!myICM.dataReady()) {
        // Wait for new data to be ready
    }
    myICM.getAGMT();

    Millis_Cur2 = millis();
    pitch_a = atan2(myICM.accX(), myICM.accZ()) * 180 / M_PI; 
    roll_a  = atan2(myICM.accY(), myICM.accZ()) * 180 / M_PI; 
    
    millisArray[i] = Millis_Cur2;
    RollArray[i] = roll_a;
    PitchArray[i] = pitch_a;
}

// Then, send the collected data
for (int x = 0; x < lenarr; x++) { 
    tx_estring_value.clear();
    tx_estring_value.append("T:");
    tx_estring_value.append(millisArray[x]);
    tx_estring_value.append(",");
    tx_estring_value.append("P:");
    tx_estring_value.append(PitchArray[x]);
    tx_estring_value.append(",");
    tx_estring_value.append("R:");
    tx_estring_value.append(RollArray[x]);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
}

break;
}
```

Jupyter Code: 
```python
def notification_handler(sender, data):
   
    try:
        # Decode bytes to string & split by ";" (separating blocks)
        entry = data.decode("utf-8").strip()

        # Split by "," then extract values using fast unpacking
        p, r, t = entry.split(",")
        time_value = float(p[2:])  # Skip "P:"
        pitch = float(r[2:])   # Skip "R:"
        roll = float(t[2:])  # Skip "T:"

        # Check if file is empty, then write headers
        with open("data.csv", "a+") as f:
            f.seek(0)
            if f.read(1) == "":
                f.write("Time,Pitch,Roll\n")
            f.write(f"{time_value},{pitch},{roll}\n")

    except Exception as e:
        print(f"Error parsing data: {e}")

```

#### Fourier Transform and Low Pass Filter Plotting

The raw data shows some noise in the higher frequencies however it is fairly negligable. This is due to the fact that the IMU has a low pass filter implemented already. Regardless, I will add a low pass filter 


When collecting dat in the proximity of the running car, the most noise appeared to be in the range of 0 Hz and 5 Hz; I will make the cutoff at 5 Hz. The lowpass filter effects the output by limiting faster frequencies (which we are defining as noise) from being shown in the data. If the frequency chosen is too small, you will still have unwanted noise in the smaller frequency range. However, if you pick too high of a cutoff frequency you run the risk of ignoring data points that may actually be important and a correct reflection of the robot's movement (maybe a sharp turn on a flip). 

In order to apply a low pass filter I had to calculate my alpha value as **0.0876** using following equations:

$$\alpha = \frac{T}{T + RC}$$

$$ f_c = \frac{1}{2\pi RC} $$

- T = sampling rate
- $f_c$ = cutoff frequency 


<img src="/Fast Robots Media/Lab 2/LPF_FFT_RAW.png" alt="Alt text" style="display:block;">
<figcaption>Raw and Fourier Transform Data with Low Pass Filter - Car in Proximity</figcaption>

<br /> 

<img src="/Fast Robots Media/Lab 2/LPF_FFT_RAW_OSS.png" alt="Alt text" style="display:block;">
<figcaption>Raw and Fourier Transform Data with Low Pass Filter - Hand Osscilations</figcaption>

<br /> 

<img src="/Fast Robots Media/Lab 2/TableBang.png" alt="Alt text" style="display:block;">
<figcaption>Raw and Fourier Transform Data with Low Pass Filter - Hitting Table</figcaption>

You can see that the low pass filter is successful in reducing unwanted noise in the accelerometer data. This is especially clear in the final graphic (Raw and Fourier Transform Data with Low Pass Filter - Hand Osscilations) where the LPF works to ignore the spikes in magnitude that comes from hitting the table.

### Gyroscope
Equations to compute pitch, roll, and yaw angles from the gyroscope:

```c++
dt = (micros()-last_time)/1000000.;
last_time = micros();
pitch_g = pitch_g + myICM.gyrX()*dt;
roll_g = roll_g + myICM.gyrY()*dt;
yaw_g = yaw_g + myICM.gyrZ()*dt;
```

#### Gyroscope vs. Accelerometer Data

When first collecting the gyroscope readings I noticed that the data did not match the accelerometer data. I realized that due to the default axis of the gyroscope, the pitch and roll for the gyroscope really corresponded to the roll and pitch of the accelerometer respectively (and make the pitch negative).

<img src="/Fast Robots Media/Lab 2/initialFlip.png" alt="Alt text" style="display:block;">
<figcaption>Initial Gyro vs. Accelerometer Readings (Flipping Needed) </figcaption>

After making those changes, I noticed that there was still drift from the gyroscope over time, likely due from integrating the error in each step. However, I did find it interesting that the gyroscop provided cleaner and smoother data during quick direction changed (going from -90 to 90 degrees and back). Thus while the gyroscope alone may not be highly accurate, it is still stable. 

<img src="/Fast Robots Media/Lab 2/RawGyro.png" alt="Alt text" style="display:block;">
<figcaption>Raw Gyro Data vs. Accelerometer Readings </figcaption>

To observe the effects of changing the sampling rate, I added delays in my case `GET_ACC_READINGS` command code FOR loop to slow down the data collection. I noticed that a delay of 10 ms added some choppiness to the plotting without a significant increase it collection time. However, adding a 100 ms delay significantly increased the data collection time as well as the choppiness in the plot. The gyroscope, which I found was especially good at tracking quick changes of direction smoothly is now not nearly as clear. Additionally, you can see the plot jumping around for smoother IMU movements as the gaps between time intervals is increased. 

#### Complementary Filter Implementation

The following code was used to imlpement my complementary filter:

```c++
roll_comp[i] = (roll_comp[i-1] + roll_G[i]*dt)*(1-alpha) + (rollLPF[i]*alpha);
pitch_comp[i] = (pitch_comp[i-1] + pitch_G[i]*dt)*(1-alpha) + (pitchLPF[i]*alpha);
```
<img src="/Fast Robots Media/Lab 2/CompResults.png" alt="Alt text" style="display:block;">
<figcaption>Complementary Filter Gyro vs. Low Pass Filter Accelerometer</figcaption>

From the results you can that with an alpha value of 0.0876 the combined measurements from the accelerometer and gyroscope significantly increases stability (which comes from the gyroscope) and accuracy (from the low pass filter accelerometer).

### Sampling Data

#### Speed Up
I took a few measures to speed up the execution time for my main loop:

1. Removed the part in my code where I wait for IMU data to be ready (for example checking`(myICM.dataReady())` to move through the command loop. Instead I check if data is ready in the main loop and if it is I call the function `collectIMU()` to compute the pitch, roll and yaw. After computing I add them to their respective arrays and iterate through those arrays with a different command (that does not affect the resolution as data is already collected).
2. Removed debugging print statments in my command to get IMU data
3. I use flags to start/stop data recording

While my IMU was able to sample new values farily quickly (~ 350 Hz) after cleaning up my code, the main loop runs significantly faster than my IMU produces new data. This is evident when comparing the `IMU_Count` variable (which only runs when data is collected) to the `Total_Loops` (which counts the number of times cycled through the main). The `Total_Loops` is larger on the magnitude of 10-100x which means that the IMU is the holdup.

Code of main loop function:

```c++
void
loop()
{
    // Listen for connections
    BLEDevice central = BLE.central();

    // If a central is connected to the peripheral
    if (central) {
      // comment for speed
        // Serial.print("Connected to: ");
        // Serial.println(central.address());

        // While central is connected
        while (central.connected()) {
          write_data();
          
          if (myICM.dataReady() && start) {
              collectIMU();
              myICM.getAGMT();
          }

          Total_Loops = Total_Loops + 1;

            // Read data
            read_data();
        }
    }
}
```

collectIMU() function:

```c++
void
collectIMU(){
        
        int i = IMU_Count;

        float Millis_Cur2 = 0;
        float pitch_a = 0;
        float roll_a = 0;

        // LPF Var
        double pitch_a_LPF[] = {0, 0};
        double roll_a_LPF[] = {0, 0};
        const int n = 1;
        const float alpha = 0.0876;

        // Gyro Var
        float dt = 0;
        unsigned long last_time = micros();
        float pitch_g =0;
        float roll_g = 0;
        float yaw_g = 0;

        myICM.getAGMT();

        Millis_Cur2 = millis();

        // Raw Pitch and Roll
        pitch_a = atan2(myICM.accX(), myICM.accZ()) * 180 / M_PI; 
        roll_a  = atan2(myICM.accY(), myICM.accZ()) * 180 / M_PI; 
        
        millisArray[i] = Millis_Cur2;
        RollArray[i] = roll_a;
        PitchArray[i] = pitch_a;

        // LPF
        pitch_a_LPF[n] = alpha*pitch_a + (1-alpha)*pitch_a_LPF[n-1];
        pitch_a_LPF[n-1] = pitch_a_LPF[n];
        pitchLPF[i] = pitch_a_LPF[n];
        roll_a_LPF[n] = alpha*roll_a + (1-alpha)*roll_a_LPF[n-1];
        roll_a_LPF[n-1] = roll_a_LPF[n];
        rollLPF[i] = roll_a_LPF[n];

        //Gyro Data

        dt = (micros()-last_time)/1000000.;
        last_time = micros();
        pitch_g = pitch_g + myICM.gyrX()*dt;
        roll_g = roll_g + myICM.gyrY()*dt;
        yaw_g = yaw_g + myICM.gyrZ()*dt;

        roll_G[i] = pitch_g; //pitch and roll are flipped
        pitch_G[i] = -1*roll_g; //pitch and roll are flipped
        yaw_G[i] = yaw_g;

        // Complementary Filter Data

        roll_comp[i] = (roll_comp[i-1] + roll_G[i]*dt)*(1-alpha) + (rollLPF[i]*alpha);
        pitch_comp[i] = (pitch_comp[i-1] + pitch_G[i]*dt)*(1-alpha) + (pitchLPF[i]*alpha);

      IMU_Count++;
    }
```

Jupyter code to start/stop data collection via setting a global start variable to `1` or `0` within the `START_DATA_COLLECTION` and `START_DATA_COLLECTION` commands:
```c++
import time

ble.send_command(CMD.START_DATA_COLLECTION, "")
time.sleep(5)
ble.send_command(CMD.STOP_DATA_COLLECTION, "")
```

The old `case GET_ACC_READINGS` command was called in Jupyter after stopping data collection to then re-popoulate the csv file with new values.

<img src="/Fast Robots Media/Lab 2/CSVPOP.png" alt="Alt text" height = 250 style="display:block;">
<figcaption>CSV proving population of time-stamped IMU data in arrays</figcaption>

#### Data Storage

I decided that it would be best to have seperate arrays for storing accelerometer and gyroscope data rather than one large one. This was partially because I decided that it would be easier to organize and parse through the data using different arrays to compartmentalize the data before sending them over bluetooth. I also found it easier to create a CSV from the seperate arrays in Jupyter. 

Each of these arrays contain floats as the gyroscope and acceleration naturally output decimal values. With a double data type being twice the size of a float (64 vs 32 bits), I decided that a float was the best data type for these sensor arrays. 

I have a total of 10 floats arrays for a total of 40 bytes at a time:
- 1 for time
- 2 for accelerometer roll and pitch
- 2 for LPF roll and pitch
- 3 for gyroscope roll, pitch, and yaw
- 2 for complementary filter data

In lab 1b global variables use 30,648 bytes. This lab we added the above arrays to send IMU data. If the Artemis board has 384 kB of RAM, then 353,352 bytes of dynamic memor remain which allow us to store 353,352/40 = 8833 data points. With an average step time of 2.86 ms (shown below) we get a sample rate of 349.65 Hz. This corresponds to **25.26 seconds of IMU data collection**.

#### 5 Seconds of IMU Data

<img src="/Fast Robots Media/Lab 2/5SecSample.png" alt="Alt text" style="display:block;">
<figcaption>Proving 5 Seconds of IMU Data</figcaption>

I used one of my CSV files as an example of collecting at least 5 seconds of data and sending it over bluetooth. To do this I took the difference between the first time stamp and the last time stamp in my `proximityFinal.csv` file:

```c++
import pandas as pd

# Load the CSV file
df = pd.read_csv("proximityFinal.csv")

# Get the first and last time values
first_time = df["Time"].iloc[0]
last_time = df["Time"].iloc[-1]

# Compute the difference
time_difference = last_time - first_time
time_difference_sec = time_difference/1000
time_difference_avg = (last_time - first_time)/len(df["Time"])

print(f"Time difference: {time_difference_sec} seconds")
print(f"Average step time difference: {time_difference_avg} ms")
```

### RC Stunts

<iframe width="450" height="315" src="https://www.youtube.com/embed/Hd9J3dOaupc" allowfullscreen></iframe>
<figcaption>Stunt 1</figcaption>

<iframe width="450" height="315" src="https://www.youtube.com/embed/fK9v2iRqGfE" allowfullscreen></iframe>
<figcaption>Stunt 2</figcaption>

The car is quick at direction changing and accelerating. When spinning it is able to hold its position on the ground without drifting much. Note that the car's speed can not be changed while moving, it can only stop and change directions. 

### Collaboration
I collaborated extensively on this project with [Jack Long](https://jack-d-long.github.io/) and [Trevor Dales](https://trevordales.github.io/). I referenced [Daria's site](https://pages.github.coecis.cornell.edu/dak267/dak267.github.io/#page-top) for code debugging in my complementary filter as well as visually understanding how to effectively display my plots. ChatGPT was heavily used to write plotting code for the Raw, FFT and LPF data. It also helped me write my FFT function as the provided link had some syntax error and missing pictures.