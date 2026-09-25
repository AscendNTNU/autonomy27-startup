Documentation of the takeoff node


###### PX4 Docs #####

Use the topic OffboardControlMode to specify setpoints. It has to be streamed as a keep-alive signal? 2Hz

The OffboardControlMode message needs to be in the format:
uint64 timestamp (microseconds since system start)
bool position (1)



In order to arm the drone in offboard