+++
title = "Lab 3: Time of Flight Sensors"
date = 2025-02-20
weight = 10
[taxonomies]
tags = ["Robotics", "C++", "Sensors", "Python", "Embedded Software", "Microcontroller" ]
+++

### I2C Address and Time-of-Flight Sensor Discussion

Two time of flight sensors will used and ultimately mounted on the car to provide different points of view to help the robot naviate; one of my ToF sensors will be mounted on the front center of the car and the other between the two wheels on one side. This will allow me to keep a set distance from walls to the side as well as avoid obstacles and navigate with vision at the front.

Since we want to use two ToF flight sensors which have the same I2C address, we use the XSHUT pin on one of the sensors to shut off one ToF which modifying the I2C address of the other. It can then be restarted again to function simultaniously with the other. Below is the code to execute this initialization:

```c++
  pinMode(SHUTDOWN_PIN, OUTPUT);
  
  // Set the address of TOF1 to 0xf5
  digitalWrite(SHUTDOWN_PIN, LOW); // Shut down TOF2
  distanceSensor1.begin();
  distanceSensor1.setI2CAddress(0xf5);
  if (distanceSensor1.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("1 Initialized");
    distanceSensor1.setDistanceModeLong();
      
  }
  digitalWrite(SHUTDOWN_PIN, HIGH); // Restart TOF2

  if (distanceSensor2.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("2 Initialized");
    distanceSensor2.setDistanceModeLong();
  }
```

Per the data ToF [data sheet](https://cdn.sparkfun.com/assets/8/9/9/a/6/VL53L0X_DS.pdf), the default I2C address is 0x52 (`0b 0101 0010`). 


Using the `Example05_Wire_I2C.ino` example sketch, I was able to scan for connected I2C devices:

<img src="/Fast Robots Media/Lab 3/I2C Scanning.png" alt="Alt text" style="display:block;">
<figcaption>I2C Device Scanning</figcaption>

From the scan we can see the Tof device address as 0x29 (`0b 0010 1001`). Since the least significant bit is used to indicate read/write, it is soley a matter of shifting the 7-bit 0x29 address left by one bit to make space for the read/write bit that appears in the data sheet. Thus, the scanned address matches the data sheet as it is just a shift left away from matching the 0x52 8-bit address.

### Physical Connection

In the below images you can see the battery leads soldered to the QWICC connect cables and the ToF sensors wired as outlined in the schematic:

<img src="/Fast Robots Media/Lab 3/wiring.png" alt="Alt text" style="display:block;">
<figcaption>Wiring Schematic</figcaption>

<img src="/Fast Robots Media/Lab 3/quikk connections.png" height = 350 alt="Alt text" style="display:block;">
<figcaption>ToF sensor connected to QWIIC breakout board</figcaption>

<img src="/Fast Robots Media/Lab 3/Battery Pack.png" height = 300 alt="Alt text" style="display:block;">
<figcaption>Soldered Battery Pack</figcaption>

### Two ToF Sensors In Parallel Test 

From the following video we can see the two ToF sensors working in parallel. I noticed that when the clearance for my ToF sensor drops below 20 mm, they tend to output 0 mm. However, this is to be expected since according to section 3.3 of the sensor [data sheet](https://cdn.sparkfun.com/assets/8/9/9/a/6/VL53L0X_DS.pdf), the minimum ranging distance is 4 cm. Overall, the sensors are accurate outside of this range.

I decided to use the long ToF mode `distanceSensor1.setDistanceModeLong()`. According to the [data sheet](https://cdn.sparkfun.com/assets/8/9/9/a/6/VL53L0X_DS.pdf): "long distance mode allows the longest possible ranging distance of 4 m to be reached." In a classroom environment I believe that this longer distance will prove more useful for mapping even though it is at the cost of higher resolution at shorter distances.  

<iframe width="450" height="315" src="https://www.youtube.com/embed/vdYf5x8ExiM" allowfullscreen></iframe>
<figcaption>Two ToF Sensor Testing</figcaption>

### ToF Sensor Speed

I tested the speed of my ToF sensors in two ways. The first was by printing the Artemis clock to the Serial as fast as possible and printing new ToF sensor data from both sensors only when available. The second was by testing the speed when called over bluetooth and compare it to the IMU. 

1. I used the following function to visualize and quantify the gap between each ToF data collection:

```c++
void loop() {
  distanceSensor1.startRanging(); 
  distanceSensor2.startRanging();

  if (distanceSensor1.checkForDataReady())
  {
  int distance1 = distanceSensor1.getDistance();
  distanceSensor1.clearInterrupt();
  distanceSensor1.stopRanging();
  Serial.print("Distance1(mm): ");
  Serial.print(distance1);
  Serial.println("   ");
  }
  if (distanceSensor2.checkForDataReady())
  {
  int distance2 = distanceSensor2.getDistance(); 
  distanceSensor2.clearInterrupt();
  distanceSensor2.stopRanging();
  Serial.print("Distance2(mm): ");
  Serial.print(distance2);
  Serial.println("   ");

  }
  Serial.print("T: ");
  Serial.print(millis());
  Serial.print("   ");
  Serial.println();
}
```

<div style="display: flex; justify-content: center; gap: 10px;">
    <figure>
        <img src="/Fast Robots Media/Lab 3/Speed2.png" alt="Speed Test Frame 1" style="width: 150px;">
        <figcaption style="text-align: center;">Speed Test Frame 1</figcaption>
    </figure>
    <figure>
        <img src="/Fast Robots Media/Lab 3/Speed1.png" alt="Speed Test Frame 2" style="width: 150px;">
        <figcaption style="text-align: center;">Speed Test Frame 2</figcaption>
    </figure>
    <figure>
        <img src="/Fast Robots Media/Lab 3/Speed3.png" alt="Speed Test Frame 3" style="width: 150px;">
        <figcaption style="text-align: center;">Speed Test Frame 3</figcaption>
    </figure>
</div>

From the above three images you can see that the average gap between ToF sensor collection is 90 ms, corresponding to a rate of **11 Hz**. Both sensor 1 and sensor two print at roughly the same rate.

2. I created a function `collectTOF()` to collect my ToF data in order to ensure the code doesn't hang while it waits for the sensor to finish a measurement. It is called from the main loop when one of the two ToF sensors are ready AND my `start` flag is true. The function populates a time and two ToF arrays with data. The data is parsed through via case `GET_TOF_READINGS` to be sent over bluetooth.

```c++
void 
collectTOF(){

  int i = TOF_Count;

  float Millis_TOF = 0;
  Millis_TOF = millis();
  millisTOF[i] = Millis_TOF;

  distanceSensor1.startRanging(); 
  distanceSensor2.startRanging();

  // TOF1 distance
  int distance1 = distanceSensor1.getDistance();
  distanceSensor1.clearInterrupt();
  distanceSensor1.stopRanging();
  TOF1[i] = distance1;

  // TOF2 distance
  int distance2 = distanceSensor2.getDistance(); 
  distanceSensor2.clearInterrupt();
  distanceSensor2.stopRanging();
  TOF2[i] = distance2;

  TOF_Count = TOF_Count + 1; 
}
```

After collecting data for 5 seconds (using my `START_DATA_COLLECTION` and `STOP_DATA_COLLECTION` command flags) the following number data points were collected from the IMU (function shown in Lab 2) and ToF sensors relative to the number of cycles through my main loop. This method helps eliminate the delay from print statments in the first speed test:

<img src="/Fast Robots Media/Lab 3/ToF and IMU Counter.png" height=75 alt="Alt text" style="display:block;">
<figcaption>IMU and ToF Counters</figcaption>

From the counters we can see that the ToF sensors were considerably slower than the IMU at recieving data and thus would be the limiting factor. After 5 seconds of data collection this corresponds **10 Hz for the ToF and 98 Hz for the IMU**. 

### ToF Sensor Data (Over Bluetooth)

Here I created a Time vs. Distance graph of ToF data collected on the Artemis then sent over bluetooth:

<img src="/Fast Robots Media/Lab 3/Mid Range Parallel.png" alt="Alt text" style="display:block;">
<figcaption>Distance Testing via Bluetooth</figcaption>

Here is another graph where I simultaniously collected ToF and IMU data for 5 seconds over bluetooth and graphed them on the same time axis. The complementary filter from Lab 2 was used for the IMU data:

<img src="/Fast Robots Media/Lab 3/AngleandDistance.png" alt="Alt text" style="display:block;">
<figcaption>Angle and Distance Collection</figcaption>

### ToF Accuracy

To quantify the accuracy of the ToF sensors I collected 10 seconds of data at three distances and graphed the data vs. time as well as the set target distance:  

<img src="/Fast Robots Media/Lab 3/100mm.png" alt="Alt text" style="display:block;">
<figcaption>100 mm Test</figcaption>

<img src="/Fast Robots Media/Lab 3/150mm.png" alt="Alt text" style="display:block;">
<figcaption>150 mm Test</figcaption>

<img src="/Fast Robots Media/Lab 3/200mm.png" alt="Alt text" style="display:block;">
<figcaption>200 mm Test</figcaption>


All around from the graphs you can see that the time of flight sensors are fairly accurate at a range of distances. While the testing setup was fairly precise, a few milimeters of variance is likely due to the human error in trying to perfectly align the placement of the sensors along my ruler. Nonetheless there is certainly some higher frequency noise in the ToF sensors which could likely be reduced by a LPF.

### Collaboration
I collaborated extensively on this project with [Jack Long](https://jack-d-long.github.io/) and [Trevor Dales](https://trevordales.github.io/). I referenced [Wenyi's site](https://mavisfu.github.io/lab3.html) for my wiring setup, ToF sensor initiation, and sketch used to  print out distance sensors in my serial monitor to test speed. ChatGPT was used to help plot CSV data and format graphs. 