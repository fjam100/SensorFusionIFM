# SensorFusionIFM
Kalman filtering with technically incorrect covaraiance propagation

Start roscore,
Source the files,
rosrun sensor_fusion hello,
rosbag play ifm_z_data.bag from the correct directory.

Covariance propagation:
To correct the covariance propagation in the code, the state is defined as follows:
[euler angles (3X1), angular velocities (3X1)]
A_angle=[I(3,3) I(3,3).dt; zeros(3,6)]
Use A_angle in the 2nd step of the prediction step and in the measurement update step

Similar for [x, xdot, xddot]
