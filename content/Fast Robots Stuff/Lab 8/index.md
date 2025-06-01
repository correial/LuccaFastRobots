+++
title = "Lab 8: Stunts"
date = 2025-04-08
weight = 5
[taxonomies]
tags = ["Robotics", "C++", "Sensors", "Python", "Embedded Software", "Microcontroller" ]
+++

<img src="/Fast Robots Media/Lab 8/FRCars.png" alt="Alt text" style="display:block;">
<figcaption>Cars 3 Way Tie</figcaption>

## Flip Implementation

### Bluetooth Data Transmission

The entire flip program starts when called over bluetooth via the Jupyter notebook line: `ble.send_command(CMD.STUNT, "1200")`. The parameter passed in is the distance away from the wall that the car should begin to reverse at. Note that 1200 mm is larger than the distance away from the wall of the mat (300 mm) as the car needs time to slow down before ultimately flipping on the mat. This parameter allows for easy flip tuning for various different starting positions and flip locations.

### Arduino Code
The `STUNT` command is responsible for ensuring the first kalman filter value is stored and  calling the essential `Flip()` and `sendStuntData()` functions. Here is the command:

```c++
stuntStartTime = millis();

// Loop until first kalman value is recieved
while (!kalmanReady) {
Serial.print("waiting for ToF");
if (distanceSensor1.checkForDataReady()) {
    collectTOF();
    getKalmanData();
    mu = { TOF1[0], 0 };

    kalmanReady = 1;
    // stuntSpeed[stuntCounter] = 255;
    stuntCounter = stuntCounter + 1;
}
delay(10);
}

// Stunt logic
Flip();
sendStuntData();
stop();
```

The functions to note are:

-  `Flip()`runs the logic to drive at the wall then flip the robot. While the robot has yet to reach the flip distance, it drives forward at full speed. Once it reaches the target distance, it runs in reverse at full speed for 2300 ms. Note that during both loops, ToF and Kalman data is being collected.

```c++
if (!flipNow) {
    while (kf[stuntCounter-1] > (stunt_distance)) {
     
      stuntTime[stuntCounter] = millis() - stuntStartTime;
      collectTOF();
      getKalmanData();

      forward(255);
      stuntSpeed[stuntCounter] = 255;
      stuntCounter = stuntCounter + 1;
    }
    flipNow = 1;
  }

  if (flipNow) {

    Serial.println("flipping");

    flipStartTime = millis();
    while ((millis() - flipStartTime) < 2300) {

      stuntTime[stuntCounter] = millis() - stuntStartTime;
      collectTOF();
      getKalmanData();

      reverse(255);
      stuntSpeed[stuntCounter] = -255;
      stuntCounter = stuntCounter + 1;
    }
    flipNow = 0;
  }

    stop();
```

- `collectTOF()` responsible for collecting current ToF values and updating the global `TOF1` array with distances
- `getKalmanData()` reads in the current `TOF1` array value (updated by the collectTOF() function). In return it updates the global `kf` array with the kalman filter calculated distance which is used for tracking distance to the wall for the stunt. In the below code, you can see the use of an update flag that flips based on whether new sensor data is ready– this helps to speed up the process. The following A and B matrices were used post tuning: 

<div id="math-A"></div>
<div id="math-B"></div>

<script>
  document.addEventListener("DOMContentLoaded", function () {
    // Matrix A (tight rows)
    katex.render(String.raw`
      A = \begin{bmatrix}
        0 & 1 \\[-0.0em]
        0 & -\frac{d}{m}
      \end{bmatrix}
      = 
      \begin{bmatrix}
        0 & 1 \\[-0.0em]
        0 & -4.07
      \end{bmatrix}
    `, document.getElementById("math-A"), { displayMode: true });

    // Matrix B (tight rows)
    katex.render(String.raw`
      B = \begin{bmatrix}
        0 \\[-0.0em]
        \frac{1}{m}
      \end{bmatrix}
      = 
      \begin{bmatrix}
        0 \\[-0.0em]
        10
      \end{bmatrix}
    `, document.getElementById("math-B"), { displayMode: true });
  });
</script>

```c++
if (update) {
    KalmanResult result = kalman(mu, sigma, Matrix<1, 1>{ (pid_speed[stuntCounter] / 255.0) * 1000 }, Matrix<1, 1>{ TOF1[stuntCounter] });

    mu = result.mu;
    sigma = result.sigma;

    kf[stuntCounter] = mu(0, 0);
    update = 0;
  }

  else {
    // Same as above except pass in false for fifth kalman() parameter to flip update to false
  }

```

- `sendStuntData()` loops through crucial arrays and sends them over bluetooth to be processed by the notification hanlder and piped into a csv for graphing and post prosessing.

## Results

### Stunt Videos 

The time elapsed for each stunt trial are in the image captions; the timer starts once the car passes the blue line and ends once it retuns and crosses it.

<iframe width="450" height="315" src="https://www.youtube.com/embed/nGscvDKEgaM"allowfullscreen></iframe>
<figcaption>Flip Trial 1 | Time: 2.38s</figcaption>

<iframe width="450" height="315" src="https://www.youtube.com/embed/f26MszdWCjk"allowfullscreen></iframe>
<figcaption>Flip Trial 2 | Time: 2.36s</figcaption>

<iframe width="450" height="315" src="https://www.youtube.com/embed/3NoplLiZmkU"allowfullscreen></iframe>
<figcaption>Flip Trial 3 | Time: 2.30s</figcaption>

### Graphs for Trial 3

From the graph you can see Kalman Filter Position vs. Time as well as Speed vs. Time. Note that the speed flips from 255 to -255 once the robots hits the target distance of 1200 mm. This distance was played around with until finally deciding that 1200 mm was just enough to have the robot flip on the mat (300 mm from the wall) without hitting the wall or flipping too early. You can see that the Kalman filter could use some further tuning as the model predicts too slow of a velocity (as seen from the lack of steepness in the bunched up data). Returning to the state space model, this could be improved by reducing m (to increase the B matrix) or increasing U (less intuitive fix). ToF sensor data is visible from the KF jumps but aren't shown to reduce graph clutter.

<img src="/Fast Robots Media/Lab 8/StuntGraph.png" alt="Alt text" style="display:block;">
<figcaption>Trial 3 Graph</figcaption>

### Blooper Video

Here is my blooper video! ([Here is the original](https://www.youtube.com/shorts/y1TS2CTmxV8) but vote on the edited cars version)
<iframe width="450" height="315" src="https://www.youtube.com/embed/rXx_Q_o5dlI"allowfullscreen></iframe>
<figcaption>Blooper</figcaption>

## Summary and Challenges

1. You can see that in two of the trials, an **added weight** is mounted to the front of the robot to help it nosedive and flip about its front. By the end I realized that the added mass did not help as much as a fully charged battery thus you can see that the trials with the added weight (1 and 2) are nearly identical to those without. The weight was made out of taped together washers and was shared between Trevor and I.

<img src="/Fast Robots Media/Lab 8/car.png" alt="Alt text" style="display:block;">
<figcaption>Robot with (right) and without (left) weight</figcaption>

2. Throughout the hours of testing, I had to use **correction factors** to straighted the car's trajectory toward the mat and also help to slow the wheels down at the same time, ensuring that when flipped, the robot was oriented straight. My correction terms scale the passed in PWM speed. When driving at the wall, a correction term of 0.95 scales the right side motor and when driving in reverse (slowing down) a correction factor of 0.90 scales the left side motor. The blooper is an example of the robot before tuning. You can see it arc left when approaching the wall and then spinning as it slowed down which made it ultimately return at the completely wrong angle.

## Collaboration

I collaborated extensively on this project with [Jack Long](https://jack-d-long.github.io/) and [Trevor Dales](https://trevordales.github.io/). ChatGPT was used to help plot graphs.



