nmea_converter
==========

A ros package that converts gnss nmea messages to navsatfix messages


# Launch

~~~
source $HOME/catkin_ws/devel/setup.bash
roslaunch nmea_converter nmea_converter.launch
~~~

# Node

## Subscribed Topics
 - /f9p/nmea_sentence (nmea_msgs/Sentence)

## Published Topics

 - /f9p/fix (sensor_msgs/NavSatFix) (This topic will not be published unless its location has been estimated.)

 - /gga (nmea_msgs/Gpgga)


# Parameter description

The parameters are set in `launch/nmea_converter.launch` .

|Name|Type|Description|Default value|
|:---|:---|:---|:---|
|sub_topic_name|bool|Topic name of nmea_msgs/Gpgga to subscribe|/f9p/nmea_sentence|
|pub_fix_topic_name|double|Topic name of nmea_msgs/Gpgga to publish|/f9p/fix|
|pub_gga_topic_name|bool|Topic name of nmea_msgs/Gpgga to publish|/gga|
|output_gga|bool|Whether to output nmea_msgs/Gpgga|false|


# Related packages
- [nmea_comms](https://github.com/MapIV/nmea_comms)