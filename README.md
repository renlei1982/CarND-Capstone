
## Self driving car ND - Capstone Project Submission
### Students  
This submission was prepared by the **True North** team (cohort start date 2017-07-27)

|Role           | Name                | Email                   |
|:--------------|:--------------------|:------------------------|
| Team Lead     | Tamas Panyi         |  panyitamas@gmail.com   |
| Team Member 1 | Toby Coleman        |  tobycoleman@gmail.com  |
| Team Member 2 | Giancarlo Colmenares|  giancarlo.cs@gmail.com |
| Team Member 3 | Ahmed Elshaarany    |  sh3rany@gmail.com      |
| Team Member 4 | Lei Ren             |  renlei1982@gmail.com   |

The project repository can be accessed on Github:
[True North - Capstone GitHub repo](https://github.com/psnx/CarND-Capstone)

### AWS instance
The code was tested on various AWS instances. The Team Lead used `p2.xlarge` instance type with the recommended AMI type: Deep Learning AMI Ubuntu Linux - 2.3_Sep2017 (ami-cd67a4b4)

#### Running the code
On virtual machine in file `~/CarND-Capstone/ros/src/styx/server.py` the monkey patch may need to be activated.
```python
# eventlet.monkey_patch()
```
For AWS it must be _deactivated_, as it is now.

## Original Project Specification
This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

### Installation 

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop). 
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space
  
  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```

