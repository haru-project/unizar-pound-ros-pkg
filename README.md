# unizar-pound-ros-pkg
The Pound ROS node enables topic prioritization and supports multiple ROS cores in addition to the options of compression and configurations. It is mainly designed to improve communication performance (in terms of delay and jitter) in wireless multi-hop networks.

Usage:
`rosrun ros_pound ros-pound --node-id 0 --num-of-nodes 3`

Parameter `node-id` specifies the identifier of the local node, parameter `num-of-nodes` specifies the total number of nodos in the robotic network

By default, ros-pound uses UDP/IP communication. The network must have an address of the type `192.168.XX.YY`. XX is the network identificator while YY is related with the `node-id` parameter. Suppose the network is `192.168.1.YY`:

* Node with `--node-id 0` *must* have IP `192.168.1.1`
* Node with `--node-id 1` *must* have IP `192.168.1.2`

etc.

The base IP is by default `192.168.1.1`; this can be changed in the YAML configuration file (`~/.rospound/config.yaml`)specifying the parameter:

`base_ip = 192.168.2.1`

for example.

The topics to be transported must be specified in the `config/config.h` file with the format:

`TOPIC(type, topic_name, source, dest, priority, period, time_to_live)`

where

* `type` specifies the data type for example `std_msgs::Float64` 
* `topic_name` specifies the name of the topic, for example `"float_number"` 
* `source` is the source node of the topic, for example `0` 
* `dest` specifies the destination node(s) as text for example `"0,1,2"` however for the moment Pound only support unicast (only one node should be specified) 
* `priority` specifies the priority associated to the topic for example `56`. Must be between 0 and 127 
* `period` the expected period of the topic in ms, for example `100` 
* `time_to_live` period during the which the message is considered valid in ms, for example `500` 

The data type used must be specified in the `config/data_types.h` file. For example:

`#include <std_msgs/Float64.h>`


The complete list of options for the YAML file (`~/.rospound/config.yaml`) is:

```     
use_ip: true
device: wlan0
base_ip: 192.168.1.1
nodes:
 - id: 3
   mac: 22:44:55:FF:00:01
 - id: 2
   mac: 18:19:20:21:22:23
routes:
 - dest: 3
   next: 5
 - dest: 4
   next: 5
feedback: false
auto_tuning: false
port: 32000
queue: 50
delay: 2500
rate_mbps: 6.0
use_discard: false
quiet: false
```

The Pound node is similar to RT-WMP package available at http://wiki.ros.org/ros-rt-wmp. While the RT-WMP assigns global priorities, the Pound assigns local (node or machine level) priorities.
For more information, please contact dantard@unizar.es


## Testing on multiple machines with different roscores

Assuming you have 2 computers connected to the same network, make sure the IPs of them are 192.168.1.2, for the **PC1**, and 192.168.1.3, for the **PC2**.

Clone this repository (currently on the branch `haru`) on your workspace and compile it using `catkin_make` in both computers. Take a look to the file [config.h](src/libwrapper/config/config.h):

```cpp
TOPIC(std_msgs::String, "chatter", 1, "2", 100, 5, 10000)
```

It mainly means this node is configured to send the topic `chatter` from the node 1 (192.168.1.2) to the node 2 (192.168.1.3).

So, execute the following in the **PC1** (192.168.1.2):

```bash
# Terminal 1
roscore 
# Terminal 2
rosrun ros_pound ros-pound --node-id 1 --num-of-nodes 3
```

And the following in the **PC2** (192.168.1.3):

```bash
# Terminal 1
roscore 
# Terminal 2
rosrun ros_pound ros-pound --node-id 2 --num-of-nodes 3
```

Check that a topic called `/R1/chatter` exists in the PC1. In the same way, check that a topic called `/R2/rx/R1/chatter` exists in the PC2. The first one is the *publisher* topic from the PC1 and the second one is the *subscriber* topic on the PC2, where the `/R1/chatter` topic will be automatically published.

To test it, let's publish some data into the first topic on the **PC1**:

```bash
rostopic pub -r 5 /R1/chatter std_msgs/String "data: 'Hello my friend'"
```

And let's read it on **PC2**:

```bash
rostopic echo /R2/rx/R1/chatter
```

you should get the following output:

```bash
data: "Hello my friend"
---
data: "Hello my friend"
---
data: "Hello my friend"
---
(...)
```
