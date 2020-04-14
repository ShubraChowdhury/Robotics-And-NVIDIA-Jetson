# Estimation Project #


Project outline:

 - [Step 1: Sensor Noise](#step-1-sensor-noise)
 - [Step 2: Attitude Estimation](#step-2-attitude-estimation)
 - [Step 3: Prediction Step](#step-3-prediction-step)
 - [Step 4: Magnetometer Update](#step-4-magnetometer-update)
 - [Step 5: Closed Loop + GPS Update](#step-5-closed-loop--gps-update)
 - [Step 6: Adding Your Controller](#step-6-adding-your-controller)



### Step 1: Sensor Noise ###

For the first step, you will collect some simulated noisy sensor data and estimate the standard deviation of the quad's sensor.


1. Choose scenario `06_NoisySensors`.  In this simulation, the interest is to record some sensor data on a static quad, so you will not see the quad move.  You will see two plots at the bottom, one for GPS X position and one for The accelerometer's x measurement.  The dashed lines are a visualization of a single standard deviation from 0 for each signal. The standard deviations are initially set to arbitrary values (after processing the data in the next step, you will be adjusting these values).  If they were set correctly, we should see ~68% of the measurement points fall into the +/- 1 sigma bound.  When you run this scenario, the graphs you see will be recorded to the following csv files with headers: `config/log/Graph1.txt` (GPS X data) and `config/log/Graph2.txt` (Accelerometer X data).

2. Plug in your result into the top of `config/6_Sensornoise.txt`.  Specially, set the values for `MeasuredStdDev_GPSPosXY` and `MeasuredStdDev_AccelXY` to be the values you have calculated.

		MeasuredStdDev_GPSPosXY = 0.704 #0.869261 #2 
		MeasuredStdDev_AccelXY =  0.502 #0.552092 #.1 

3. Run the simulator. If your values are correct, the dashed lines in the simulation will eventually turn green, indicating you’re capturing approx 68% of the respective measurements (which is what we expect within +/- 1 sigma bound for a Gaussian noise model)

		Simulation #24 (../config/06_SensorNoise.txt)
		PASS: ABS(Quad.GPS.X-Quad.Pos.X) was less than MeasuredStdDev_GPSPosXY for 70% of the time
		PASS: ABS(Quad.IMU.AX-0.000000) was less than MeasuredStdDev_AccelXY for 69% of the time
		Simulation #25 (../config/06_SensorNoise.txt)
		PASS: ABS(Quad.GPS.X-Quad.Pos.X) was less than MeasuredStdDev_GPSPosXY for 70% of the time
		PASS: ABS(Quad.IMU.AX-0.000000) was less than MeasuredStdDev_AccelXY for 69% of the time
		Simulation #26 (../config/06_SensorNoise.txt)
		PASS: ABS(Quad.GPS.X-Quad.Pos.X) was less than MeasuredStdDev_GPSPosXY for 70% of the time
		PASS: ABS(Quad.IMU.AX-0.000000) was less than MeasuredStdDev_AccelXY for 69% of the time
## Output Reference files : 
#### 1. Step 1-Sensor Noise-Scenerio_6.gif
#### 2. Step 1-Sensor Noise-Scenerio_6_1.PNG

<p align="center">
<img src="video_screenshots/Step 1-Sensor Noise-Scenerio_6.gif" width="500"/>
</p>

<p align="center">
<img src="video_screenshots/Step 1-Sensor Noise-Scenerio_6_1.PNG" width="500"/>
</p>


### Step 2: Attitude Estimation ###

In this step, you will be improving the complementary filter-type attitude filter with a better rate gyro attitude integration scheme.

1. Run scenario `07_AttitudeEstimation`.  For this simulation, the only sensor used is the IMU and noise levels are set to 0 (see `config/07_AttitudeEstimation.txt` for all the settings for this simulation).  There are two plots visible in this simulation.
   - The top graph is showing errors in each of the estimated Euler angles.
   - The bottom shows the true Euler angles and the estimates.
Observe that there’s quite a bit of error in attitude estimation.

## Output Reference files : 
#### 1. Step 2-Attitude Estimation-Scenerio_7_SMALL_ANGLE_GYRO_INTEGRATION.gif
#### 2. Step 2-Attitude Estimation-SAcenerio_7_Small_Angle_Gyro_Integration.PNG

<p align="center">
<img src="video_screenshots/Step 2-Attitude Estimation-Scenerio_7_SMALL_ANGLE_GYRO_INTEGRATION.gif" width="500"/>
</p>

<p align="center">
<img src="video_screenshots/Step 2-Attitude Estimation-SAcenerio_7_Small_Angle_Gyro_Integration.PNG" width="500"/>
</p>

2. In `QuadEstimatorEKF.cpp`, you will see the function `UpdateFromIMU()` contains a complementary filter-type attitude filter.  To reduce the errors in the estimated attitude (Euler Angles), implement a better rate gyro attitude integration scheme. 

			PASS: ABS(Quad.Est.E.MaxEuler) was less than 0.100000 for at least 3.000000 seconds
			Simulation #8 (../config/07_AttitudeEstimation.txt)
			PASS: ABS(Quad.Est.E.MaxEuler) was less than 0.100000 for at least 3.000000 seconds
			Simulation #9 (../config/07_AttitudeEstimation.txt)
			PASS: ABS(Quad.Est.E.MaxEuler) was less than 0.100000 for at least 3.000000 seconds
			Simulation #10 (../config/07_AttitudeEstimation.txt)
			PASS: ABS(Quad.Est.E.MaxEuler) was less than 0.100000 for at least 3.000000 seconds
			Simulation #11 (../config/07_AttitudeEstimation.txt)
			PASS: ABS(Quad.Est.E.MaxEuler) was less than 0.100000 for at least 3.000000 seconds
			Simulation #12 (../config/07_AttitudeEstimation.txt)
			PASS: ABS(Quad.Est.E.MaxEuler) was less than 0.100000 for at least 3.000000 seconds
			Simulation #13 (../config/07_AttitudeEstimation.txt)
			PASS: ABS(Quad.Est.E.MaxEuler) was less than 0.100000 for at least 3.000000 seconds
			Simulation #14 (../config/07_AttitudeEstimation.txt)
			PASS: ABS(Quad.Est.E.MaxEuler) was less than 0.100000 for at least 3.000000 seconds
			Simulation #15 (../config/07_AttitudeEstimation.txt)
			PASS: ABS(Quad.Est.E.MaxEuler) was less than 0.100000 for at least 3.000000 seconds
			Simulation #16 (../config/07_AttitudeEstimation.txt)
			PASS: ABS(Quad.Est.E.MaxEuler) was less than 0.100000 for at least 3.000000 seconds
			Simulation #17 (../config/07_AttitudeEstimation.txt)
			PASS: ABS(Quad.Est.E.MaxEuler) was less than 0.100000 for at least 3.000000 seconds

## Output Reference files : 
#### 1. Step 2-Attitude Estimation-Scenerio_7_SMALL_ANGLE_GYRO_INTEGRATION.gif
#### 2. Step 2-Attitude Estimation-SAcenerio_7_Small_Angle_Gyro_Integration.PNG

<p align="center">
<img src="video_screenshots/Step 2-Attitude Estimation-Scenerio_7_full.gif" width="500"/>
</p>

<p align="center">
<img src="video_screenshots/Step 2-Attitude Estimation-SAcenerio_7_Final.PNG" width="500"/>
</p>



			// SMALL ANGLE GYRO INTEGRATION: === ORIGINAL ==============
			// (replace the code below)
			// make sure you comment it out when you add your own code -- otherwise e.g. you might integrate yaw twice

			float predictedPitch = pitchEst + dtIMU * gyro.y;
			float predictedRoll = rollEst + dtIMU * gyro.x;
			ekfState(6) = ekfState(6) + dtIMU * gyro.z;	// yaw
			// normalize yaw to -pi .. pi
			if (ekfState(6) > F_PI) ekfState(6) -= 2.f*F_PI;
  			if (ekfState(6) < -F_PI) ekfState(6) += 2.f*F_PI;
  			// CALCULATE UPDATE
			accelRoll = atan2f(accel.y, accel.z);
			accelPitch = atan2f(-accel.x, 9.81f);
			  
			// FUSE INTEGRATION AND UPDATE
			rollEst = attitudeTau / (attitudeTau + dtIMU) * (predictedRoll)+dtIMU / (attitudeTau + dtIMU) * accelRoll;
			pitchEst = attitudeTau / (attitudeTau + dtIMU) * (predictedPitch)+dtIMU / (attitudeTau + dtIMU) * accelPitch;
			lastGyro = gyro;
			
			
			=================   UPDATED -- better rate gyro attitude integration scheme ===================
			//use the Quaternion<float> class, which has a handy FromEuler123_RPY function for creating a quaternion from Euler Roll/PitchYaw
  			//       (Quaternion<float> also has a IntegrateBodyRate function, though this uses quaternions, not Euler angles)
  			
  			Quaternion<float> quat = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6));
			    quat.IntegrateBodyRate(gyro, dtIMU);
			
			    float predictedPitch = quat.Pitch();
			    float predictedRoll = quat.Roll();
			    ekfState(6) = quat.Yaw();
			
			  // normalize yaw to -pi .. pi
			  if (ekfState(6) > F_PI) ekfState(6) -= 2.f*F_PI;
  			  if (ekfState(6) < -F_PI) ekfState(6) += 2.f*F_PI;
  			 // CALCULATE UPDATE
			  accelRoll = atan2f(accel.y, accel.z);
			 accelPitch = atan2f(-accel.x, 9.81f);
			  
			// FUSE INTEGRATION AND UPDATE
			rollEst = attitudeTau / (attitudeTau + dtIMU) * (predictedRoll)+dtIMU / (attitudeTau + dtIMU) * accelRoll;
			pitchEst = attitudeTau / (attitudeTau + dtIMU) * (predictedPitch)+dtIMU / (attitudeTau + dtIMU) * accelPitch;
			lastGyro = gyro;  
  


### Step 3: Prediction Step ###

In this next step you will be implementing the prediction step of your filter.


1. Run scenario `08_PredictState`.  This scenario is configured to use a perfect IMU (only an IMU). Due to the sensitivity of double-integration to attitude errors, we've made the accelerometer update very insignificant (`QuadEstimatorEKF.attitudeTau = 100`).  The plots on this simulation show element of your estimated state and that of the true state.  At the moment you should see that your estimated state does not follow the true state.

2. In `QuadEstimatorEKF.cpp`, implement the state prediction step in the `PredictState()` functon. If you do it correctly, when you run scenario `08_PredictState` you should see the estimator state track the actual state, with only reasonably slow drift, as shown in the figure below:

			Simulation #27 (../config/08_PredictState.txt)
			
			  //PredictState()
			  gPrime(0, 3) = dt;
			  gPrime(1, 4) = dt;
			  gPrime(2, 5) = dt;
			
			  gPrime(3, 6) = (RbgPrime(0) * accel).sum() * dt;
			  gPrime(4, 6) = (RbgPrime(1) * accel).sum() * dt;
			  gPrime(5, 6) = (RbgPrime(2) * accel).sum() * dt;
			
  			 ekfCov = gPrime * ekfCov * gPrime.transpose() + Q;

## Output Reference files : 
#### 1. Step 3-Prediction Step-Scenerio_8-Attitude Estimation.gif
#### 2. Step 3-Prediction -Scenerio_8.PNG

<p align="center">
<img src="video_screenshots/Step 3-Prediction Step-Scenerio_8-Attitude Estimation.gif" width="500"/>
</p>

<p align="center">
<img src="video_screenshots/Step 3-Prediction -Scenerio_8.PNG" width="500"/>
</p>


3. Now let's introduce a realistic IMU, one with noise.  Run scenario `09_PredictionCov`. 

4. In `QuadEstimatorEKF.cpp`, calculate the partial derivative of the body-to-global rotation matrix in the function `GetRbgPrime()`.  Once you have that function implement, implement the rest of the prediction step (predict the state covariance forward) in `Predict()`.



5. Run your covariance prediction and tune the `QPosXYStd` and the `QVelXYStd` process parameters in `QuadEstimatorEKF.txt` to try to capture the magnitude of the error you see. Note that as error grows our simplified model will not capture the real error dynamics (for example, specifically, coming from attitude errors), therefore  try to make it look reasonable only for a relatively short prediction period (the scenario is set for one second).  A good solution looks as follows:





			  //GetRbgPrime()
			  RbgPrime(0, 0) = -cos(pitch) * sin(yaw);
			  RbgPrime(0, 1) = -sin(roll) * sin(pitch) * sin(yaw) - cos(pitch) * cos(yaw);
			  RbgPrime(0, 2) = -cos(roll) * sin(pitch) * sin(yaw) + sin(roll) * cos(yaw);
			  RbgPrime(1, 0) = cos(pitch) * cos(yaw);
			  RbgPrime(1, 1) = sin(roll) * sin(pitch) * cos(yaw) - cos(roll) * sin(yaw);
    			  RbgPrime(1, 2) = cos(roll) * sin(pitch) * cos(yaw) + sin(roll) * sin(yaw);



                        
			Simulation #2 (../config/09_PredictCovariance.txt)
			Simulation #3 (../config/09_PredictCovariance.txt)
			Simulation #4 (../config/09_PredictCovariance.txt)
			Simulation #5 (../config/09_PredictCovariance.txt)
			Simulation #6 (../config/09_PredictCovariance.txt)
			Simulation #7 (../config/09_PredictCovariance.txt)
			Simulation #8 (../config/09_PredictCovariance.txt)
			Simulation #9 (../config/09_PredictCovariance.txt)
			Simulation #10 (../config/09_PredictCovariance.txt)
			Simulation #11 (../config/09_PredictCovariance.txt)
			
			

## Output Reference files : 
#### 1. Step 3-Prediction Step-Scenerio_9-PredictionCov.gif
#### 2. Step 3-Prediction Step-Scenerio_9.PNG

<p align="center">
<img src="video_screenshots/Step 3-Prediction Step-Scenerio_9-PredictionCov.gif" width="500"/>
</p>

<p align="center">
<img src="video_screenshots/Step 3-Prediction Step-Scenerio_9.PNG" width="500"/>
</p>




### Step 4: Magnetometer Update ###

In this step, you will be adding the information from the magnetometer to improve your filter's performance in estimating the vehicle's heading.

1. Run scenario `10_MagUpdate`.  This scenario uses a realistic IMU, but the magnetometer update hasn’t been implemented yet. As a result, you will notice that the estimate yaw is drifting away from the real value (and the estimated standard deviation is also increasing).  Note that in this case the plot is showing you the estimated yaw error (`quad.est.e.yaw`), which is drifting away from zero as the simulation runs.  You should also see the estimated standard deviation of that state (white boundary) is also increasing.

2. Tune the parameter `QYawStd` (`QuadEstimatorEKF.txt`) for the QuadEstimatorEKF so that it approximately captures the magnitude of the drift, as demonstrated here:

                        //Pre implementation of UpdateFromMag()

			Simulation #4 (../config/10_MagUpdate.txt)
			FAIL: ABS(Quad.Est.E.Yaw) was less than 0.120000 for 0.000000 seconds, which was less than 10.000000 seconds
			FAIL: ABS(Quad.Est.E.Yaw-0.000000) was less than Quad.Est.S.Yaw for 89% of the time
			Simulation #5 (../config/10_MagUpdate.txt)
			FAIL: ABS(Quad.Est.E.Yaw) was less than 0.120000 for 0.000000 seconds, which was less than 10.000000 seconds
			FAIL: ABS(Quad.Est.E.Yaw-0.000000) was less than Quad.Est.S.Yaw for 89% of the time


## Output Reference files : 
#### 1. Step 4-Magnetometer Update-Scenerio_10_pre.gif
#### 2. Step 4-Magnetometer Update-Scenerio_10_PRE_IMPL.PNG

<p align="center">
<img src="video_screenshots/Step 4-Magnetometer Update-Scenerio_10_pre.gif" width="500"/>
</p>

<p align="center">
<img src="video_screenshots/Step 4-Magnetometer Update-Scenerio_10_PRE_IMPL.PNG" width="500"/>
</p>

3. Implement magnetometer update in the function `UpdateFromMag()`.  Once completed, you should see a resulting plot similar to this one:

			
			//UpdateFromMag()
			
			  zFromX(0) = ekfState(6);
			  float diff = magYaw - ekfState(6);
			  if (diff > F_PI) {
			      zFromX(0) += 2.f * F_PI;
			  }
			  else if (diff < -F_PI) {
			      zFromX(0) -= 2.f * F_PI;
			  }
 			  hPrime(0,6) = 1;
			
			
			// Post implementation of UpdateFromMag()
			Simulation #20 (../config/10_MagUpdate.txt)
			PASS: ABS(Quad.Est.E.Yaw) was less than 0.120000 for at least 10.000000 seconds
			PASS: ABS(Quad.Est.E.Yaw-0.000000) was less than Quad.Est.S.Yaw for 69% of the time
			Simulation #21 (../config/10_MagUpdate.txt)
			PASS: ABS(Quad.Est.E.Yaw) was less than 0.120000 for at least 10.000000 seconds
			PASS: ABS(Quad.Est.E.Yaw-0.000000) was less than Quad.Est.S.Yaw for 69% of the time
			Simulation #22 (../config/10_MagUpdate.txt)
			PASS: ABS(Quad.Est.E.Yaw) was less than 0.120000 for at least 10.000000 seconds
			PASS: ABS(Quad.Est.E.Yaw-0.000000) was less than Quad.Est.S.Yaw for 69% of the time

## Output Reference files : 
#### 1. Step 4-Magnetometer Update-Scenerio_10_Post.gif
#### 2. Step 4-Magnetometer Update-Scenerio_10_POST_IMPL.PNG

<p align="center">
<img src="video_screenshots/Step 4-Magnetometer Update-Scenerio_10_Post.gif" width="500"/>
</p>

<p align="center">
<img src="video_screenshots/Step 4-Magnetometer Update-Scenerio_10_POST_IMPL.PNG" width="500"/>
</p>


### Step 5: Closed Loop + GPS Update ###

1. Run scenario `11_GPSUpdate`.  At the moment this scenario is using both an ideal estimator and and ideal IMU.  Even with these ideal elements, watch the position and velocity errors (bottom right). As you see they are drifting away, since GPS update is not yet implemented.

2. Let's change to using your estimator by setting `Quad.UseIdealEstimator` to 0 in `config/11_GPSUpdate.txt`.  Rerun the scenario to get an idea of how well your estimator work with an ideal IMU.

3. Now repeat with realistic IMU by commenting out these lines in `config/11_GPSUpdate.txt`:
```
#SimIMU.AccelStd = 0,0,0
#SimIMU.GyroStd = 0,0,0
```

4. Tune the process noise model in `QuadEstimatorEKF.txt` to try to approximately capture the error you see with the estimated uncertainty (standard deviation) of the filter.

5. Implement the EKF GPS Update in the function `UpdateFromGPS()`.

6. Now once again re-run the simulation.  Your objective is to complete the entire simulation cycle with estimated position error of < 1m (you’ll see a green box over the bottom graph if you succeed).  You may want to try experimenting with the GPS update parameters to try and get better performance.

***Success criteria:*** *Your objective is to complete the entire simulation cycle with estimated position error of < 1m.*

**Hint: see section 7.3.1 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj) for a refresher on the GPS update.**

At this point, congratulations on having a working estimator!

### Step 6: Adding Your Controller ###

Up to this point, we have been working with a controller that has been relaxed to work with an estimated state instead of a real state.  So now, you will see how well your controller performs and de-tune your controller accordingly.

1. Replace `QuadController.cpp` with the controller you wrote in the last project.

2. Replace `QuadControlParams.txt` with the control parameters you came up with in the last project.

3. Run scenario `11_GPSUpdate`. If your controller crashes immediately do not panic. Flying from an estimated state (even with ideal sensors) is very different from flying with ideal pose. You may need to de-tune your controller. Decrease the position and velocity gains (we’ve seen about 30% detuning being effective) to stabilize it.  Your goal is to once again complete the entire simulation cycle with an estimated position error of < 1m.

**Hint: you may find it easiest to do your de-tuning as a 2 step process by reverting to ideal sensors and de-tuning under those conditions first.**

***Success criteria:*** *Your objective is to complete the entire simulation cycle with estimated position error of < 1m.*


## Tips and Tricks ##

 - When it comes to transposing matrices, `.transposeInPlace()` is the function you want to use to transpose a matrix

 - The [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj) document contains a helpful mathematical breakdown of the core elements on your estimator

## Submission ##

For this project, you will need to submit:

 - a completed estimator that meets the performance criteria for each of the steps by submitting:
   - `QuadEstimatorEKF.cpp`
   - `config/QuadEstimatorEKF.txt`

 - a re-tuned controller that, in conjunction with your tuned estimator, is capable of meeting the criteria laid out in Step 6 by submitting:
   - `QuadController.cpp`
   - `config/QuadControlParams.txt`

 - a write up addressing all the points of the rubric

## Authors ##

Thanks to Fotokite for the initial development of the project code and simulator.
