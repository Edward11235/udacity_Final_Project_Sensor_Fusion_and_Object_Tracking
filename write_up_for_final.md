## Write a short recap of the four tracking steps and what you implemented there (EKF, track management, data association, camera-lidar sensor fusion). Which results did you achieve? Which part of the project was most difficult for you to complete, and why?
EKF is relatively easy for since it is well covered in the lecture
and I practiced a lot in exercises. It basically consists two procedures
prediction and update, and it is used to fused the result of estimation
and sensor data to improve the accuracy of data. Track management and 
Data association are used together to match track with measurement correctly. 
This is a important problem since on real road, there might be new cars detected by our
sensor, and they may confuse our system. I mainly give each track a 
score and decide if I discard it based on this score in track management. 
In data association part, I wrote the code that can calculate track score with
MHD. Track management is the most difficult part since I never 
heard about it, and it is totally new to me. 
Camera-lidar sensor fusion is what I want to achieve with EKF algorithm. It is a
method to fuse measurements from both camera and lidar in order to take advantage
of both camera and lidar. For this part, I use normal kalman filter for lidar and 
use EKF for camera since the system function for camera is non-linear.
The result is that I built a complete program that can track cars on
the road.

## Do you see any benefits in camera-lidar fusion compared to lidar-only tracking (in theory and in your concrete results)?
Yes. In theory, if I only use lidar, it is not good at recognization.
For example, a cyclist maybe recognized as a pedestrain since they
look alike from behind. With camera, a picture is added to increase 
the accuracy of recognization and it's unlikely this time to recognize
a cyclist as a pedestrain. In practice, it is also good. With lidar and
camera, my RMSE is about 0.15 while initially, it was about 0.3 with only
lidar.

## Which challenges will a sensor fusion system face in real-life scenarios? Did you see any of these challenges in the project?
One challenge I noticed is that sensor fusion requires huge computation
power. When my laptop run loop_over_dataset.py, the fan of the laptop made
a huge noise and it's getting hot. This means, the refresh rate of sensor
fusion system may be low, which is dangerous since in real life, a car
may rush out of a crossroad fast. So I guess we need to improve the speed
of sensor fusion algorithm.

## Can you think of ways to improve your tracking results in the future?
As I found in my program, the result of camera is actually good since it has lower
variance. However, the result of lidar is not good since it has a higher variance (I found
this when I write gating funciton in association.py.) So I think the camera is good enough,
but we need a better lidar to reduce variance.