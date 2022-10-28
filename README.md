# Teleop Web
This project is designed to work with the ROS Bridge Suite to enable teleoperation from a webserver hosted on a Robot. 

# Setup

* Ensure that you have ros-<distro>-rosbridge-server installed on your robot. If you are using Microsoft's ROS distribution, this is included.
* Clone this repository into your robot's workspace.
* Setup Certificates for the web server. These are required for both ROS Web Bridge and ROS Teleop Web as the Gamepad APIs and rendering system require them.

## Linux Certificate Setup

``` bash
wget https://github.com/FiloSottile/mkcert/releases/download/v1.4.4/mkcert-v1.4.4-linux-arm64 -O /usr/bin/mkcert
sudo chmod +x /usr/bin/mkcert
mkdir -P /ws/certs
cd /ws/certs
mkcert –install
mkcert <external ip> localhost
```

External IP is the IP address of the robot, not the Docker container.

## Windows Certificate Setup

``` cmd
choco install wget
wget https://github.com/FiloSottile/mkcert/releases/download/v1.4.4/mkcert-v1.4.4-windows-amd64.exe -O %USERPROFILE%\mkcert.exe
mkdir c:\ws\certs
cd c:\ws\certs
mkcert –install
mkcert <external ip> localhost
```

# Launch ROS Bridge Suite and Teleop Web

## Linux
Replace /ws with your workspace directory
``` bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml certfile:=/ws/certs/localhost+1.pem keyfile:=/ws/certs/localhost+1-key.pem ssl:=true
ros2 launch teleop_web teleop_web.launch.py certfile:=/ws/certs/localhost+1.pem keyfile:=/ws/certs/localhost+1-key.pem
```
## Windows
``` bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml certfile:=c:\ws\certs\localhost+1.pem keyfile:=c:\ws\certs\localhost+1-key.pem ssl:=true
ros2 launch teleop_web teleop_web.launch.py certfile:=c:\ws\certs\localhost+1.pem keyfile:=c:\ws\certs\localhost+1-key.pem
```

# Setting up a Steamdeck
Follow [these instructions](https://support.microsoft.com/en-us/topic/xbox-cloud-gaming-in-microsoft-edge-with-steam-deck-43dd011b-0ce8-4810-8302-965be6d53296) for setting up a Microsoft Edge Browser with a Gamepad.

# Using on any Web Browser which supports WebGL and Web Sockets
You can now connect a Gamepad.
> NOTE: On Screen Gamepad coming soon.

Navgiate to `https://<IP of your robot>:8088`.


