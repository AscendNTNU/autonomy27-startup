Documentation of the takeoff node


###### PX4 Docs #####

Use the topic OffboardControlMode to specify setpoints. It has to be streamed as a keep-alive signal? 2Hz

The OffboardControlMode message needs to be in the format:
uint64 timestamp (microseconds since system start)
bool position (1)



In order to arm the drone in offboardmode these requirments needs to be met:
* mode_req_angular_velocity - gyroscope? Maybe this just happenes internally and I don't need to think about it?
* mode_req_attitude - IMU handles this
* mode_req_offboard_signal - Heartbeat signal that is used to keep the offboard signal alive. Can be done thorough