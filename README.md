# Forque Sensor Hardware

Reads and manages the Force/Torque sensor of the Forque.

The Hardware Interface can connect to the F/T sensor via IP, publish its data as a ROS topic and trigger a re-tare (or disable any biasing).

## Sample Usage
`ros2 run forque_sensor_hardware forque_sensor_hardware —ros-args -p host:=xxx.xxx.x.xx` (replace the x's with the F/T sensor's IP address)

(See [the code](https://github.com/personalrobotics/forque_sensor_hardware/blob/b18c53c65557c9c37e235d307e40a5a14d349f6f/src/main.cpp#L67) for a complete list of parameters)

## Troubleshooting

Note that the F/T transmitter sometimes has an idiosynchracy where, if you request it starts streaming data to an IP address at a UDP port, and the UDP port is different from the one it started streaming to upon startup, it will close the old socket but not reopen a new one. Thus, if you launch this code and aren't receiving any data, the first thing to do is to stop the code, turn off the F/T transmitter, and then turn it back on. When it turns back on, it should automatically open a socket to the previously-requested IP address and port, which this code can then listen on.
