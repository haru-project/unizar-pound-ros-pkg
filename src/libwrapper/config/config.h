/** TOPIC / SERVICES DEFINITION
*
*  TOPIC(type, topic, source, dest, priority, period, time_to_live)
*  QOSTOPIC(type, topic, source, dest, priority, period, time_to_live, queue_length)
*  TFTOPIC(topic, source, dest, priority, period, time_to_live)
*
*  SERVICE(type, topic, source, priority, time_to_live)
*
*/

// This is an example topic to be send from the roscore on 192.168.1.1 to 
// the roscore on 192.168.1.2, assuming you have not modified the base_ip
// in the config YAML file. This line must be in both remote ros_pound nodes
TOPIC(std_msgs::String, "chatter", 1, "2", 100, 5, 10000)