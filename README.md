# Gazebo Worlds

Some Gazebo worlds created to benchmark path planning algorithms.

## Worlds

### CPR Agriculture + Solar Panel
![CPR Agriculture + Solar Panel](https://github.com/gelardrc/gazebo_worlds/blob/main/img/worlds/solar_farm.jpg)

### Plain Solar Farm
![Plain Solar Farm](https://github.com/gelardrc/gazebo_worlds/blob/main/img/worlds/plain_solar_farm.jpg)

### Olival

![Olival](https://github.com/gelardrc/gazebo_worlds/blob/main/img/worlds/olival.jpg)

### Substation_2

![Substation_2](https://github.com/gelardrc/gazebo_worlds/blob/main/img/worlds/substation_2.jpg)



## How to Use

```bash
cd <your_catkin>/src
git clone https://github.com/gelardrc/gazebo_worlds.git
source devel/setup.bash
roslaunch gazebo_worlds example.world
```

⚠️ Copy all models to the ~/.gazebo/models directory, or export the Gazebo model path to your project's model directory. Example:

```bash
export GAZEBO_MODEL_PATH=<path_to_your_project_models>:$GAZEBO_MODEL_PATH
```
