# Drive and Park

Parking a vehicle is a complex task that involves potential risks, such as endangering pedestrians or damaging the vehicle itself. The self-park system is designed to address these challenges by offering an autonomous parking solution capable of handling a variety of parking scenarios with high precision, regardless of the parking space type. This feature significantly enhances both safety and convenience by enabling the vehicle to park without driver input. \
When the user intends to park in a specific location, they can simply press a button, prompting the vehicle to locate an available parking space. The system identifies the type of parking space—whether straight, parallel or angled—and adapts its approach accordingly. During this process, the vehicle continuously monitors its surroundings in real-time to prevent accidents. If an obstacle is detected, the system will automatically stop the vehicle. Once parked, the system notifies the user and shuts off the engine. Additionally, by pressing another button, the user can command the vehicle to exit the parking space safely, ensuring proper alignment with other parked vehicles. \
In summary, the self-park system offers a comprehensive, autonomous parking solution designed to enhance both safety and convenience. By utilising advanced sensors, state machines, and decision tree algorithms, the system ensures precise parking in various scenarios while minimising risks. With real-time monitoring and obstacle detection, it provides a seamless parking experience, making driving safer and more efficient.

![image](https://github.com/user-attachments/assets/c4e13992-61f3-432e-8499-8389f5311ebb)

## Launch the system

### 1. Start the car
1. **Press the red button on the car**
2. **Press the start button**
3. **Switch on the remote control**

*The battery level should be higher than 10V*

### 2. Jetson Nano
1. **Connect to the Raspberry PI** : ```ssh pi@10.105.1.169``` (password : geicar)
2. **Connect to the Jetson Nano** : ```ssh jetson@192.168.1.10``` (password : geicar)
3. **Launch the docker** : ```sudo docker start -ai ros-humble```
4. **Launch ros2 launch file** : ```ros2 launch geicar_start_jetson geicar.jetson.launch.py```

### 3. RaspberryPi
1. **Connect to the Raspberry PI** : ```ssh pi@10.105.1.169``` (password : geicar)
2. **Go into the ros2_WS directory** : ```cd ros2_ws```
3. **Launch ros2 launch file** : ```ros2 launch geicar_start geicar.launch.py```

## XboX controller

> **Commands :**
>
> **start** : start the system
>
> **Left trigger** : drive backward
> **Right trigger** : drive forward
>
> **Left stick** : direction
> 
> **Y button** : manual mode
> **A button** : autonomous mode
> **B button** : emergency stop
>
> **Directional path (Down) + Start** : calibration

## Drive and Park feature
When turning on the autonomous mode the car execute a sequence of action to looking for an empty parking space and park inside. The steps are the following one :

*At the beginning the car should be aligned with the parked cars*

1. **Initialize the LiDar**
The car stops around 5 seconds. \
*It initialize the distance between itself and the parked cars with the LiDar.*

2. **Search a parking space**
The car drives straight. \everal margin areas :
- **0-35cm**: stops
- **35-70cm**: slows down
- **70-100cm**: detection
The distances are evaluated with the *ultrasonic sensors*.

### Parking and leaving mode
**20 cm margins** all around the car. 
The distances are evaluated with the *LiDar*.

*The parking space research is done with the LiDar and the odometer.*

3. **Parking**
*If a parking space is found and identify (straight/parallel) :*
The car waits 5 seconds, then parks  and stops.

The maneuver performed depends of the parking space size :
- **Parallel** : length > **220cm** and depth > **95cm**
- **Straight** : **140cm** < length < **180cm** annd depth > **145cm**

4. **Leaving parking space**
***The user must switches to autonomous mode (switch the mode to manual and then autonomous).***
The car leaves the parking space, stops and switches to manual mode. 

If the car does not found a parking space it stops after **3,5m**.

## Security feature
The car slows down or stops when an obstacle is detected

### Drive mode
Several margin areas :
- **0-35cm**: stops
- **35-70cm**: slows down
- **70-100cm**: detection
The distances are evaluated with the *ultrasonic sensors*.

### Parking and leaving mode
**20 cm margins** all around the car. 
The distances are evaluated with the *LiDar.
