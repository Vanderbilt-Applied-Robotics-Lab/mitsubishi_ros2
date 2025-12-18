# Installation

## Requirements

The following software is required for execution of this ROS2 Humble package

- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [Git](https://docs.ros.org/en/humble/Installation.html)
- [Rosdep2](https://zoomadmin.com/HowToInstall/UbuntuPackage/python-rosdep2)
- [Ubuntu 22.04 Realtime](https://ubuntu.com/blog/real-time-ubuntu-released) *(Optional but recommended)*

## Setup

1. Source your ROS2 Humble install with the following command

> ROS2 only supports `sh`, `bash`, and `zsh` shells by default. If you are using `sh` or `zsh` instead of `bash`, please source the corresponding file as needed

```bash
source /opt/ros/humble/setup.bash
```

2. Determine a directory/path to install this ROS2 workspace and then clone this repository there with the following command

```bash
git clone https://github.com/Vanderbilt-Applied-Robotics-Lab/mitsubishi_ros2.git
```

3. `cd` into the cloned directory and run the following command to source ROS2 dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

4. Build the program by running the following command

```bash
colcon build
```

There should not be any errors or warnings from the build of this package. If there are, please contact the maintainer of this repository.

# Usage

In order to run any of the launch files within this repository, a hardware setup followed by a software setup must be performed.

## Hardware Setup

Perform the following steps:

1. Plug in the robot's power
2. Connect the controlling/host computer to the robot controller via an ethernet (RJ45) cable
3. Turn on the controller via its power switch
4. Wait for the controller to boot
5. Switch the controller's 'MODE' key to 'AUTOMATIC' if it is not already in it
6. Press the 'START' button on the controller

## Software Setup

Perform the following steps:

1. Open a terminal
2. `cd` to the installed ROS2 workspace
3. Source ROS2 commands with the following command

> ROS2 only supports `sh`, `bash`, and `zsh` shells by default. If you are using `sh` or `zsh` instead of `bash`, please source the corresponding file as needed
```bash
source /opt/ros/humble/setup.bash; source ./install/setup.bash
```


4. Run **one** of the following commands

```bash
# Just RViz
ros2 launch mitsubishi_robot_bringup view.launch.py
# RViz and MoveIt
ros2 launch mitsubishi_robot_bringup moveit.launch.py
```

Additional launch parameters are available for each launch files as follows:

| Parameter             | Default value                | Description                               |
| --------------------- | ---------------------------- | ----------------------------------------- |
| `robot_name`          | `rv_3s`                      | Mitsubishi robot name (rv_3s or rv_6sdl_s15) |
| `simulation`          | `false`                      | Use simulation or real hardware.          |
| `robot_ip`            | `192.168.0.1`                | IP Address of the robot                   |
| `robot_port`          | `10000`                      | Realtime control port of the robot        |
| `control_port`        | `10003`                      | Program control port of the robot         |
| `robot_password`      | `ARMALABS`                   | Robot password for initialization program |
| `use_fake_hardware`   | `false`                      | Whether or not to run in simulation       |
| `description_package` | `mitsubishi_robot_description`    | URDF robot description package            |
| `description_file`    | `mitsubishi_robot.xacro`          | URDF robot descripition file              |
| `moveit_package`      | `mitsubishi_robot_moveit`         | Moveit configuration file package         |
| `controller_package`  | `mitsubishi_robot_bringup`        | Package with the controllers yaml         |
| `controllers_file`    | `ros2_controllers.yaml`      | ROS2 Control controllers yaml file        |
| `robot_controller`    | `joint_trajectory_controller` | The controller for the robot              |

In order to utilize any of the launch parameters above, put them following one of the above launch commands as follows

```bash
ros2 launch mitsubishi_robot_bringup moveit.launch.py robot_name:=rv_3s simulation:=false
```

There will be an error regarding `octomap` when this command is run, it can be safely ignored.

# FTP

The Mitsubishi controller has support for FTP file transfers if you need to add a new program. Please note that it uses a proprietary encoding, so copying files should only really be done between robots, just copy a text file saved as a `.MB5` will not work. The following involves steps on both the mitsubishi computer and the localhost.

## Requirements

- `telnet`
- `pyftpdlib`
    - It's a python package, you can just pip install it (May also need to `sudo pip install` it)

## Hardware setup steps

1. Connect your computer to the Mitsubishi controller over ethernet. There should be a RJ45 cable already with the robot, if not then one can be plugged into its back panel.

2. Ensure you are properly connect to the robot by pinging, the response should be simiar to the following

```bash
➜  ~ ping 192.168.0.1
PING 192.168.0.1 (192.168.0.1) 56(84) bytes of data.
64 bytes from 192.168.0.1: icmp_seq=1 ttl=64 time=1.31 ms
64 bytes from 192.168.0.1: icmp_seq=2 ttl=64 time=1.32 ms
64 bytes from 192.168.0.1: icmp_seq=3 ttl=64 time=1.65 ms
64 bytes from 192.168.0.1: icmp_seq=4 ttl=64 time=1.06 ms
^C
--- 192.168.0.1 ping statistics ---
4 packets transmitted, 4 received, 0% packet loss, time 3015ms
rtt min/avg/max/mdev = 1.063/1.337/1.653/0.209 ms
```

## Computer setup

Open a terminal and input the following commands

1. Run `ip a` to determine the IP address of the net device connecting to the Mitsubishi controller. An example output is provided below wherein `192.168.0.3` is the IP of the utilized network device. This will be different for every user, and will be referenced as `<LOCAL_IP>` in future steps

```bash
➜  ~ ip a            
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
    inet 127.0.0.1/8 scope host lo
       valid_lft forever preferred_lft forever
    inet6 ::1/128 scope host 
       valid_lft forever preferred_lft forever
2: wlp2s0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc noqueue state UP group default qlen 1000
    <REDACTED>
7: enx3c18a0d4f87c: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel state UP group default qlen 1000
    link/ether <MAC ADDRESS> brd ff:ff:ff:ff:ff:ff
    inet 192.168.0.3/24 brd 192.168.0.255 scope global noprefixroute enx3c18a0d4f87c
       valid_lft forever preferred_lft forever
```


2. After finding the IP of the net device connecting to the robot controller, run the following command to start an FTP server.

```bash
➜  ~ sudo python3 -m pyftpdlib -p 21 -i <LOCAL_IP> -u mitsubishi -P armalabs
[I 2024-09-20 17:54:02] concurrency model: async
[I 2024-09-20 17:54:02] masquerade (NAT) address: None
[I 2024-09-20 17:54:02] passive ports: None
[I 2024-09-20 17:54:02] >>> starting FTP server on <LOCAL_IP>:21, pid=14376 <<<
```

## Mitsubishi setup

Once a connection has been made to the controller, and the FTP server started, perform the following steps to connect to the server from the Mitsubishi controller.

1. Telnet into the controller

```bash
➜  ~ telnet 192.168.0.1 23
Trying 192.168.0.1...
Connected to 192.168.0.1.
Escape character is '^]'.

-> 
```

2. After connecting, you will have a VXWorks shell with some basic functionality. You can use the `help` command to look through commands.

```bash
-> help

help                           Print this list
ioHelp                         Print I/O utilities help info
dbgHelp                        Print debugger help info
nfsHelp                        Print nfs help info
netHelp                        Print network help info
spyHelp                        Print task histogrammer help info
timexHelp                      Print execution timer help info
h         [n]                  Print (or set) shell history
i         [task]               Summary of tasks' TCBs
ti        task                 Complete info on TCB for task
sp        adr,args...          Spawn a task, pri=100, opt=0x19, stk=20000
taskSpawn name,pri,opt,stk,adr,args... Spawn a task
td        task                 Delete a task
ts        task                 Suspend a task
tr        task                 Resume a task
d         [adr[,nunits[,width]]] Display memory
m         adr[,width]          Modify memory
mRegs     [reg[,task]]         Modify a task's registers interactively
pc        [task]               Return task's program counter

Type <CR> to continue, Q<CR> to stop: 

iam       "user"[,"passwd"]  	Set user name and passwd
whoami                         Print user name
devs                           List devices
ld        [syms[,noAbort][,"name"]] Load stdin, or file, into memory
                               (syms = add symbols to table:
                               -1 = none, 0 = globals, 1 = all)
lkup      ["substr"]         List symbols in system symbol table
lkAddr    address              List symbol table entries near address
checkStack  [task]             List task stack sizes and usage
printErrno  value              Print the name of a status value
period    secs,adr,args... Spawn task to call function periodically
repeat    n,adr,args...    Spawn task to call function n times (0=forever)
version                        Print VxWorks version info, and boot line

NOTE:  Arguments specifying 'task' can be either task ID or name.

value = 1 = 0x1
-> 
```

3. Add the computer hosting the FTP server as a host

```bash
-> hostAdd "computer", "192.168.0.3"
value = 0 = 0x0
->
```

4. Create an FTP network device

```bash
-> netDevCreate "targetftp:","computer",1
value = 0 = 0x0
->
```

5. Set the local user credentials

```bash
-> iam "mitsubishi", "armalabs"
value = 0 = 0x0
->
```

6. `ls` the FTP server to confirm functionality. `<DIRECTORY>` is relative to where the FTP server was started on the host machine. For this command, it is recommended to just leave it blank so that the command becomes `ls "targetftp:/"`

```bash
-> ls "targetftp:/<DIRECTORY>"
<FILES>
value = 0 = 0x0
->
```

7. The `copy` command on VxWorks can be used to transfer files bidirectionally between the mitsubshi and local computers. Similar to how Windows gives separate drives a label (Like `C:/` or `D:/`), by adding the FTP as a net device, it is essentially treating it as a separate drive. Therefore, to copy a file to the mitsubishi, the `<FROM>` would be prefixed with the 'label' `targetftp:/` and the `<TO>` would not be prefixed. Similarly, to copy to the mitsubishi, the `<TO>` woul be prefixed with the 'label' `targetftp:/` and the `<FROM>` would not be prefixed

```bash
-> copy <FROM>, <TO>
<FILE CONTENTS>
value = 0 = 0x0
->
```

8. The file will not be on the robot controller. The default telnet directory is where all the robot programs are stored; however, if you want to go elsewhere on the device, the `cd` command must be used. It's syntax is very different from standard UNIX. An example is given below.

```bash
-> pwd
/robprg/dat
value = 12 = 0xc
-> cd("/robprg")
value = 0 = 0x0
-> pwd
/robprg
value = 8 = 0x8
->
```
