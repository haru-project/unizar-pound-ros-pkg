/** TOPIC / SERVICES DEFINITION
*
*  TOPIC(type, topic, source, dest, priority, period, time_to_live)
*  QOSTOPIC(type, topic, source, dest, priority, period, time_to_live, queue_length)
*  TFTOPIC(topic, source, dest, priority, period, time_to_live)
*
*  SERVICE(type, topic, source, priority, time_to_live)
*
*/

// This is an example topic to be send from the roscore on 192.168.1.2 to 
// the roscore on 192.168.1.3, assuming you have not modified the default base_ip (192.168.1.1)
// in the config YAML file. This line must be in both remote ros_pound nodes
TOPIC(std_msgs::String, "chatter", 1, "2", 100, 5, 10000)

// These are the topics for the README example using 2 remote machines with different roscores 
// through a VPN
// TOPIC(std_msgs::String, "chatter_1", 0, "1", 100, 5, 10000)
// TOPIC(std_msgs::String, "chatter_2", 1, "0", 100, 5, 10000)
// TOPIC(sensor_msgs::CompressedImage, "usb_cam/image_raw/compressed", 1, "0", 100, 25, 100)