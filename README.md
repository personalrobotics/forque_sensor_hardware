# Forque Sensor Hardware

Reads and manages the Force/Torque sensor of the Forque.

The Hardware Interface can connect to the F/T sensor via IP, publish its data as a ROS topic and trigger a re-tare (or disable any biasing).

## Sample Usage
`ros2 run forque_sensor_hardware forque_sensor_hardware —ros-args -p host:=xxx.xxx.x.xx` (replace the x's with the F/T sensor's IP address)

(See [the code](https://github.com/personalrobotics/forque_sensor_hardware/blob/2cfafda1f5aed56a0656895aaa47e7d58d98acad/src/main.cpp#L58) for a complete list of parameters)
