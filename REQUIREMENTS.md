# Hardware requirements

- Storage: at least 30 GB of free space
- GPU: recommended, but not required
  - Any integrated graphics cards should suffice
- A display device (monitor)
  - Gazebo simulator requires a display device to run on. Thus, headless
    servers cannot be used.


# Software environments

- Operating system: Ubuntu 20.04
- X window system (e.g., xorg)
  - `echo $DISPLAY` should show a valid display name
- Docker
  - `docker-ce` is recommended (https://docs.docker.com/engine/install/ubuntu/)


# Robots

We tested four robotic applications, and for two of them we utilized both the
physical robots as well as the virtual models. If you intend to test the
actual robots, you can purchase the following hardware and assemble them on
your own:
- TurtleBot3 burger
  - https://www.robotis.us/turtlebot-3-burger-us/
- PX4 drone - X500 kit by Holybro
  - https://shop.holybro.com/x500-kit_p1180.html

Otherwise, you can still perform tests with only the virtual models in the
simulator, i.e., do a SITL (simulator-in-the-loop) testing. Everything for the
SITL testing is included in the artifact.

