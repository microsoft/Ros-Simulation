# Architecture

## Robot Simulation

Simulating multiple robots can be a challenge due to resource constraints. On one end, it is very compute intensive to calculate the physics of robot movement and generate realistic sensor data--especially as the number of robots in a space increases. On the other end, the robots themselves also need compute power to run algorithms for navigation.

For these reasons, running robot simulation at scale is challenging without support for multi-threading and leveraging distributed compute. For this project, we initially explored using several available simulators:

- [Stage](http://rtv.github.io/Stage/)
- [Gazebo](http://gazebosim.org/)
- [ARGoS](https://www.argos-sim.info/)

We chose ARGoS as the simulator for its multi-threaded design, which allowed us to scale the simulator to take advantage of many cores.

Another problem to solve was how to orchestrate the robots when running them across a distributed cluster of machines. We chose [Kubernetes](https://kubernetes.io/) for its ease of use, and so we could also leverage [Helm](https://helm.sh/) for its flexibility and go-template constructs.

## Containers

Containers allow us to easily manage multiple robots in a lightweight and scalable way.

We created two docker images for the simulator and robots and pushed them to an [Azure Container Registry](https://azure.microsoft.com/en-us/services/container-registry/). The simulator image contains the code for running ARGoS as well as the bridge to talk to ROS. The robot images contain ROS controllers for the robot, and various navigation libraries.

Both of these images are pulled down and used in our Kubernetes cluster.

The Simulator and Robot containers communicate over four topics:

- /clock
  - Simulator sends /clock to robot -> Used for syncing up the simulation clock tick with the ros::Time object
- /scan
  - Simulator sends /scan to robot -> Generated LaserScan sensor data from Simulator
- /odom
  - Simulator sends /odom to robot -> Generated Odometry sensor data (change in position and velocity) from Simulator
- /cmd_vel
  - Robot sends /cmd_vel messages to simulator ->  Kinesmatics messages to move the robot at particular velocity

## ROS

The common pattern for running multiple robots in simulation for ROS is to share one ROS master (roscore) and add namespaces and tf_prefixes for each robot. Because they share one ROS master, they can easily pub/sub to ros topics. This is done in various Gazebo/Stage sample projects for multi-robot setups.

### Common configuration

![Single Ros Core](../images/arch_SingleRosCore.png =500x)

This design has four inherent challenges:

1. Many open source ROS packages do not consider proper namespacing when utilizing parameters, topics and services, so there is risk for global namespacing to be used on accident.
1. TF2 does not support tf_prefixes, and no suitable replacement strategy has been suggested. So this usually means that all TF frames need to be manually mapped to a namespaced pattern.
1. The robots share the same TF tree, which can cause performance degrades, and requires extraneous traffic for simple transforms.
1. The ROS code is configured different to run with namespacing, and this does not match how it will be run on a real robot.

To avoid these issues, we ran each robot with its own ros master, which meant that each robot also had its own TF tree.

### Our configuration

![Multiple Ros Cores](../images/arch_MultipleRosCore.png =500x)

Each of the robots maintains their own ROS master, and their topics are mapped to namespaced topics on the simulator. The topic transport and mapping is accomplished using Nimbro (see [Networking](#Networking)).

### Networking

We used an open source project called [Nimbro](https://github.com/AIS-Bonn/nimbro_network) to serve as a communication bus between the simulator and each of the robots. This opened up the possibility of having multiple ROS masters.

`nimbro_topic_sender` is a ROS node that subscribes to particular topics on one ros master, and sends them to another receiving `nimbro_topic_receiver` ROS node on a different ros master. The receiver then publishes the messages on an analogous topic. This set up allows us to remove the need for namespacing on the robot side, although namespacing is still needed on the simulator side (we accomplish this with `remap`).

The networking protocol between the simulation container and the robot containers is configurable to TCP or UDP, and there are various configurations, such as rate limits and compression that can assist in boosting performance.

The addressing and dns resolution between Nimbro sender and receiver is handled by Kubernetes. There is a Kubernetes service for every nimbro receiver, so that it can be reached through a simple name and port, such as `robot-nimbro-0:17000`.

## Simulator

[ARGoS](https://www.argos-sim.info/) is the simulator that we chose to use for this project. It can support multiple robots in simulation (using various physics engines to generate sensor data and perform robot movement). ARGoS does not have built in support for interacting with ROS. Most samples projects include a robot controller written in C++.

ARGoS has a plugin architecture, so we leveraged an existing ARGoS-ROS bridge and added it as a controller plugin for ARGoS. This component does not contain business logic for how the robot behaves, but rather, serves as a translator between ARGoS and ROS. This allowed us to use ARGoS to test our ROS robot controllers in the ARGoS simulation environment.

The existing bridge did not have message mappings for [LaserScan](http://wiki.ros.org/navigation/Tutorials/RobotSetup/Sensors#Publishing_LaserScans_over_ROS) and [Odom](http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom#The_nav_msgs.2BAC8-Odometry_Message) messages. Since LaserScan and Odom are needed for the ROS navigation stack, we added the translation functionality to the existing bridge.

Another plugin we added was to publish a [simulation clock](http://wiki.ros.org/Clock) on each iteration of the simulation cycle. This allows our simulation to stay in sync, so long as ROS packages are using ros::Time with the rosparam /use_sim_time=true. This is the same mechanism that Gazebo and Stage use today for simulation clocks. The clock plugin also publishes an [Real Time Factor (RTF)](./ValidateAndTroubleshoot.md#Real-Time-Factor-(RTF)).

## Robot

The robot container contains various open source packages for navigation. The existing robot controller can be swapped out for your own controller.

The only difference between what runs in simulation and what runs on the actual robot should be the hardware drivers for various components (LIDAR, wheels, etc.). These will be replaced by the simulator, which will provide generated sensor messages.
