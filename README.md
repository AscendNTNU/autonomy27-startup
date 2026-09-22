# autonomy27-startup

Vi skal lage det jeg vil kalle q-art 

<img src="bilder/image.png" alt="bad path in q" width="500"/>


## Docker image

### i VS code :

### i terminalen

build image
```bash
source /opt/ros/jazzy/setup.bash
docker build -f docker/Dockerfile -t q-art .
```
start container

```bash
docker run -it --rm \ -v "$(pwd):/workspaces/ros2-px4" \ -w /workspaces/ros2-px4 \ q-art
```

## ROS 2 workspace

bygg ved å kjøre 
```bash
colcon build 
```
then source workspace 
```bash
source install/setup.bash
```

# selve oppgaven
q-art er inspirert fra strava art (se [link](https://www.strav.art/)). målet er å bli kjent med flere verktøy vi kommer til å bruke i løpet av året. 

lage et program som gjør at en drone kan:
- lette
- fly i et mønster bestemt av oss
- lande 
autonomt 

### former

#### firkant 

<img src="bilder/square.webp" alt="drawing" width="200"/>


#### sirkel
<img src="bilder/circle.jpg" alt="circle" width="200"/>

#### hjerte 
<img src="bilder/heart.webp" alt="heart" width="200"/>

#### ascend

<img src="bilder/ascend.JPG" alt="ascend logo" width="200"/>

## steg

1. designe FSM/BT veldig enkelt design jeg ville gått for FSM
2. lage koden
3. ut og fly lære logging osv


# Læringsmål
- ROS2 
- PX4
- Qgroundcontrol
- 

