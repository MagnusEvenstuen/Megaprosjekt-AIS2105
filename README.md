# Megaprosjekt
# AIS2105 - Mekatronikk og robotikk

## Innhold
- [Gruppe](#Gruppe)
- [Om prosjektet](#Om-prosjektet)
- [Hva kan roboten gjøre](#Hva-kan-roboten-gjøre)
- [Viktige komandoer](#Viktige-komandoer)
- [IP Oppsett](#IP-Oppsett)
# Gruppe
Prosjektgruppe 163

Eldar Helseth &amp;
Oliver Steinnes Gundersen &amp;
Nikolai Dworacek &amp;
Magnus Evenstuen

# Om prosjektet
I dette prosjektet skal vi kombinere bildebehandling og robotstyring. Vi bruker en UR5e-robot med et xxxx kamera og flytter rundt på kuber. 

# Hva kan roboten gjøre
Gå til hjemmeposisjon, scanne arbeisområde, gjennkjenne kuber etter farger, plukke opp kuber i denne rekkefølgen. 

# Viktige komandoer
```
colcon build # Bygger alle pakkene i workspace
```
```
colcon build --packages-select qube_bringup  # Bygger kun den spesifiserte pakken
```
```
source install/setup.bash  # Kilde oppsett etter bygging
```
```
ros2 launch ros2 launch ur_robot_driver ur_control.launch.py ur_type:=urX ​robot_ip:=yyy.yyy.yyy.yyy use_mock_hardware:=false
initial_joint_controller:=joint_trajectory_controller headless_mode:=true
```
```
ros2 launch ros2 launch ur_robot_driver ur_control.launch.py ur_type:=urX robot_ip:=yyy.yyy.yyy.yyy use_mock_hardware:=false
initial_joint_controller:=joint_trajectory_controller
```
```
ros2 launch ros2 launch ur_robot_driver ur_control.launch.py ur_type:=urX robot_ip:=yyy.yyy.yyy.yyy use_mock_hardware:=false
initial_joint_controller:=joint_trajectory_controller
```
```
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=urX launch_rviz:=true
```

# IP Oppsett
IP address: 143.25.150.72

Subnet mask: 255.255.252.0

Default gateway 143.25.151.0
