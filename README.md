# State-Estimation
Include codes: Kalman_Filter, Sensors.

# Study:

## ORB_SLAM2:
### input:
	* camera image of current frame
	* timestamp of the current image frame

### output: 
	* 2D pointcloud of key features
	* the last frame that was captured

### method:
	searches for only key features in the frame. 
	Tries to close loops over the images to build a map and localize in respect to earlier images.

### pros: 
	* doesn't use a lot of computing time or need heavyduty hardware due to only using key features in a frame and not every pixel in frame

### cons:
	* doesn't work well at high speeds due to high percentage of key features changing between frames 
  
  
## Rovio
### input:
	* camera image of current frame
	* IMU measurments

### output: 
	* angle to each key feature from current location
	* distance vector to each key feature from current location 

### method:
	by adding the error in position of features in the stream to an Extended Kalman Filter's update stage 
	it adds stability in motion and movments when location cannot be determind from camera due to loss of key features. 
	when this happenes estimation is not is accurate but isn't lost completely.  

### pros: 
	* IMU measurements make the system more robust at high speeds.

### cons:
	* requires accurate dynamic model.
	* requires high accuracy IMU sensors.
  
  
 ## Servo:
 ### input:
	* camera image of current frame
	* timestamp of the current image frame
	* IMU measurments

### output: 
	* 2D pointcloud of key features
	* the last frame that was captured

### method:
	this is an extention of ORB_SLAM2. 
	the ORB_SLAM2 is added IMU measurments to better approximate the location and avoid losing tracking 
	and spacial awareness when no similar key features are found between frames.
		very similar to rovio and even uses it in current version but the idea is to use the camera adn finetune with IMU
		as apposed to rovio that dose the oposite. 

### pros: 
	* doesn't use a lot of computing time or need heavyduty hardware due to only using key features in a frame and not every pixel in frame
	* doesn't lose localization that easaly at high speeds due to coupling with IMU data.

### cons:
	* requires at current implementation high accuracy sensing and precise dynamic model 
