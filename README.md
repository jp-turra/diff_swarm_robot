# Differential Swarm Robot

This is a graduation final work project to explore swarm capabilities and low cost electronic pieces to build a robot. Also the simulation of robot behavior is included.

## Prepare environment

In the root directory of the project, run the following command to prepare the environment.

### Docker compose

To build docker image, run

`docker compose build`

To run docker container, run

`docker compose up`

Access the container using VSCode Dev Container extension or the command `docker exec -it <container_id> bash` to access the container shell.

### Extra linux setup

* Run the following command to allow docker access X server
    * `xhost +si:localuser:$USER`
    * In the the last command don't work, run `xhost +si:localuser:root`

## Start CoppeliaSim inside the container

```bash
cd $HOME/CoppeliaSim
./coppeliaSim
```

## Build ROS packages

```bash
cd $HOME/project
colcon build
```
