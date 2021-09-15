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


## Testing on multiple machines with different roscores in the same local network

Clone this repository (currently on the branch `haru`) on your workspace and compile it using `catkin_make` in both computers. Take a look to the file [config.h](src/libwrapper/config/config.h):

```cpp
TOPIC(std_msgs::String, "chatter", 1, "2", 100, 5, 10000)
```

It mainly means this node is configured to send data from a topic called `chatter` (notice it will not be the final name, some namespaces are added) from the node 1 (192.168.1.2) to the node 2 (192.168.1.3). Also notice that it assumes you have 2 computers connected to the same network and their IPs are 192.168.1.2, for the **PC1**, and 192.168.1.3, for the **PC2**. In case they have other IPs, please modify the previous node IDs (1 and "2" in the previous snippet) and the corresponding ones in the next commands, although take in account the [valid range for these IPs](#found-issues-in-the-original-repository).

Notice also that we have not added any config.yaml file, so the `base_ip` is the default one 192.168.1.**1**, so the node_id of the a PC with the IP 192.168.1.N is N-**1**. In our case this base IP is the router IP.

Then, execute the following in the **PC1** (192.168.1.2):

```bash
# Terminal 1
roscore 
# Terminal 2
rosrun ros_pound ros-pound --node-id 1 --num-of-nodes 1
```

And the following in the **PC2** (192.168.1.3):

```bash
# Terminal 1
roscore 
# Terminal 2
rosrun ros_pound ros-pound --node-id 2 --num-of-nodes 1
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

## Testing on multiple machines with different roscores in different local networks using a VPN

Clone this repository (currently on the branch `haru`) on your workspace and compile it using `catkin_make` in both computers. 

Connect both computers to the same VPN and get their IPs to the network interface corresponding to this VPN. In our case, we will supose PC1 is 10.0.0.34 and PC2 is 10.0.0.35. 

Then, we will create a config file for our setup. Creates the file `~/.rospound/config.yaml` in both computers and set the following configuration:

```yaml
use_ip: true
base_ip: 10.0.0.34
```

Notice that the `base_ip` must be the IP of the PC with lover IP number, in this case the PC1. So the PC1's node_id is 0 (its IP minus the base_ip) and the PC2's node_id is 1 (that is the result of 35 minus the base_ip). 

Now, we have to modify in both computers the file [config.h](src/libwrapper/config/config.h) with the topics we want to send and receive in each computer. In this case we will test with the following topics:

```cpp
TOPIC(std_msgs::String, "chatter_1", 0, "1", 100, 5, 10000)
TOPIC(std_msgs::String, "chatter_2", 1, "0", 100, 5, 10000)
TOPIC(sensor_msgs::CompressedImage, "usb_cam/image_raw/compressed", 1, "0", 100, 25, 100)
```

It means the PC1 will send string msgs through the topic `/chatter_1` to the PC2 (as you know, actually the topic will `/R0/chatter_1` in the PC1 and `/R1/rx/R0/chatter_1` in the PC2) and the PC2 will send strings msgs through the topic `/chatter_2` and compressed images through the topic `usb_cam/image_raw/compressed` to the PC1 (as you know, actually they will be `/R1/chatter_2` and `/R1/usb_cam/image_raw/compressed` in the PC2 and they will be received as`/R0/rx/R1/chatter_2` and `/R0/rx/R1/usb_cam/image_raw/compressed` in the PC1).

Due to we are going to also send compressed images, it is necessary to also modify the file [data_types.h](src/libwrapper/config/data_types.h) adding this new message type:

```cpp
// This is the msg include for the example topic
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/String.h>
```

Finally, compile the package in both computers executing `catkin_make` in the root of your workspace. We are ready to run these nodes:

Execute the following in the **PC1** (Rememeber it has the IP 10.0.0.34, that is the current base_ip, so it is the node_id 0):

```bash
# Terminal 1
roscore 
# Terminal 2
rosrun ros_pound ros-pound --node-id 0 --num-of-nodes 2
```

And the following in the **PC2** (Rememeber it has the IP 10.0.0.35, that is the current base_ip + 1, so it is the node_id 1):

```bash
# Terminal 1
roscore 
# Terminal 2
rosrun ros_pound ros-pound --node-id 1 --num-of-nodes 2
```

Check that the previous topics have been created and feel free to send messages through them and receive the messages in the remote computer.

## Found Issues in the original repository

1. The `ros_pound` node doesn't find some ROS shared libraries. **Fixed** in the `CMakeLists.txt`adding a post install instructions to set execution permissions to every user, but it should be fixed in other way.

2. The range of valid IPs seem to be from `base_ip` to `base_ip + 31`. **Fixed** modifying the function `mcast_to_vector()` of the node `ros_pound`, that was limiting the destination node IDs to 31.

3. It is not possible to send the same topic name in both directions. It means that if you want to send the topic `/chatter` from PC1 to PC2, you will not can send it from PC2 to PC1, although it seems it should be possible due to the automatic namespaces that the rospound node add to the topics.