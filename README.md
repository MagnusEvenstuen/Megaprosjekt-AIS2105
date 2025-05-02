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


# Styring fra extern laptop

Før launch filene kjøres på surface nettbrettet så setter man domain ID med kommandoen:
```
export ROS_DOMAIN_ID=X
```

Og setter samme ID-en i begge terminalene samt terminalen på laptoppen.

Sjekk at 'topics' kan leses på laptoppen med:
```
ros2 topic list
```

Deretter kan man kjøre:
```
ros2 run rviz2 rviz2
```

Og legge til 'motionPlanning' med ADD funksjonen. Man kan så gå inn på 'joints', gjøre endringer, så trykke 'plan' og 'execute'. Da skal roboten flytte seg. 





# Build uten crash!
```
`export MAKEFLAGS="-j 1"`

`colcon build --executor sequential`
```


# Kamera

Kjør integrert kamera:
```
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0
```

Kjør tilkobla usb-kamera:
```
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video2
```

Kjør kamera med kalibreringfil:
```
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0 -p camera_info_url:=file:///home/user/ost.yaml # Pass på riktig plassering / filnamn
```

Kalibrering (kameraet må kjøre i en egen terminal for å kalibrere): 
```
ros2 run camera_calibration cameracalibrator   --size 7x9   --square 0.02 --no-service-check   --ros-args   -r image:=/image_raw   -r camera_info:=/camera_info   -r set_camera_info:=/usb_cam/set_camera_info  # Husk å sjekk riktig antal ruter/ størrelse på rutene
```
