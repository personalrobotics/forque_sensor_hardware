# Forque Sensor Hardware

Reads and manages the Force/Torque sensor of the Forque.

The Hardware Interface can connect to the F/T sensor via IP, publish its data as a ROS topic and trigger a basic calibration routine.

### Example

1. Connect the Forque Sensor to power and to your computer via ethernet.
2. Make sure the sensor is reachable by ```ping 192.168.1.1```
3. Clone this repo as well as the [net-ft driver dependency](https://github.com/epfl-lasa/net-ft-ros)
3. ```caktin build forque_sensor_hardware```
4. ```roslaunch forque_sensor_hardware forque.launch```

You can display sensor data by ```rostopic echo /forqueSensor```.

You can trigger a calibration process by<br/>
```rostopic pub rostopic pub /bias_controller/trigger/goal pr_control_msgs/TriggerActionGoal ...```<br/>
(Replace the ... by pressing tab). You can see the update of the calibration in /rosout.

### Troubleshooting

If you can't connect to the sensor, consider running ```sudo ifconfig eth1 192.168.1.2 netmask 255.255.255.0``` to setup networking temporarily or configure your ```/etc/hosts``` file like this to set it up permanently:

```
auto enx0023576c5936
  iface enx0023576c5936 inet static
  address 192.168.1.2
  netmask 255.255.255.0
  broadcast 192.168.1.255
  gateway 192.168.1.1
  dns-nameservers 8.8.8.8
```

These instructions are for Ubuntu 16.04 and may differ depending on your current networking setup. ```enx0023576c5936``` is the name of the appropriate network interface.

After changing the settings, you may need to run
```
sudo ip addr flush enx0023576c5936
sudo systemctl restart networking.service
```

