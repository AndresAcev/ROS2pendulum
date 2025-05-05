# ROS2 SCARA

```bash
ros2 launch scara view.launch.py
ros2 launch scara_gz sim.launch.py
ros2 launch scara_moveit demo.launch.py
```

Paquetes para controlar un robot SCARA Epson que tenemos en la universidad usando ROS 2

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

```bash
ros2 launch scara view.launch.py
```
```bash
colcon build --packages-select scara && source install/setup.bash && ros2 launch scara view.launch.py
```

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

## Github Pages

Para usar el github-pages necesitamos instalar:

```bash
pip install mkdocs pymdown-extensions
```

Para correr en local:

```bash
mkdocs serve
```

Para desplegar la página en github

```bash
mkdocs gh-deploy
```

Luego de desplegar hacer el _commit_ y _push_ del repositorio para subir los cambios.