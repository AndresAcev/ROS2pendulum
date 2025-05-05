# ROS2 pendulo

[Your Commands] → [ROS 2 Topics] → [Controller Manager] → [Gazebo Plugin] → [Gazebo Physics]

```bash
ros2 launch scara view.launch.py
ros2 launch scara_gz sim.launch.py
```

Paquetes para controlar un pendulo, maqueta de EAFIT

## Compilar los paquetes

Deberemos instalar una libreria en ubuntu para poder compilar los paquetes:

```bash
sudo apt install libserial-dev 
```

y luego en el workspace compilar los paquetes:

```bash
colcon build
```

## Usar el paquete de descripción

Lanzar primero el pkg scara, contiene una representacion rViz del urdf model
```bash
colcon build --packages-select scara && source install/setup.bash && ros2 launch scara view.launch.py
```

Lanza la simulacion en gazebo
```bash
colcon build --packages-select scara_gz && source install/setup.bash && ros2 launch scara_gz sim.launch.py
```

## Para usar los paquetes

1. Cambiamos los permisos de los puertos

```bash
    sudo chmod 777 /dev/ttyACM0
```

2. Creamos una conexión entre el puerto real y uno virtual	

```bash
    socat -d -d pty,rawer,echo=0,link=/tmp/scara  /dev/ttyACM0,b115200,raw
```	

3. En otro terminal, lanzamos el sistema
	
```bash
    ros2 launch servo_hardware_moveit_config demo.launch.py
```	


## Manually Load Controllers

    1.Check if /joint_states is being published
```bash
    ros2 topic echo /joint_states
```	

    2. if not, After launching your simulation, in a new terminal, try:
```bash
    ros2 control list_controllers
    ros2 control load_controller --set-state active joint_state_broadcaster
```	

## Pasos testeo consola

1. Check the control manager has the correct data
```bash
    ros2 control list_controllers
```	
2. Run the rqt graph to se the position of the carro
```bash
    ros2 run rqt_plot rqt_plot /joint_states/position[0]  # For corredera joint
```	

3. Pub a position of the carro (change the value as pleace 0 is the center +- 4 are the ends)

```bash
    ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['corredera'], points: [{positions: [0.2], time_from_start: {sec: 2, nanosec: 0}}]}" --once
```	

4. Echo the states to the pos vel and effort
```bash
    ros2 topic echo /joint_states
```	