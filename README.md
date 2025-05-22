# Megaprosjekt -
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
IP address: 143.25.150.7

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

Om kamera ikkje køyrer, bruk denne kommandoen for å finne alle kamera tilkopla PC-en:
```
v4l2-ctl --list-devices
```

# Om pakkene

### camera_detector
camera_detector pakken bruker openCV til å detektere firkanter av forskjellige farger. Pakken detekterer farger med å gjøre om RGB verdier til HSV verdier, så ser den på om kulør, metning og lysstyrke er innenfor de satte verdiene, og hvis det er det er det riktig farge. Etter at forskjellige objekter er detektert sjekkes det hvor mange kanter det er på hver av de formene. Den står da bare igjen med former med 4 kanter og ingen kurver på de kantene. Så sees det på hvor kvadratisk de forskjellige firkantene er. 

Denne måten å løse detektering av kuber på har både fordeler og ulemper. En fordel er at den er veldig god til å detektere firkanter. Den er også ganske robust på kvadrat i forskjellige høyder og størrelser så lenge de er større enn den minste tillatte størrelsen i koden. Ulempa er at den ikke er så robust når det kommer til forskjellig orientasjon på kubene. Hvis kubene står diagonalt på bildet (balanserer på en kant) vil ikke pakken klare å detektere at det er en kube der. Det kan også skje at fargedeteksjonen får en kube til å se litt for rund ut, og derfor ikke se på det som en kube.

### simple_mover
Denne pakken har vi hentet en del inspirasjon til fra https://github.com/dominikbelter/ros2_ur_moveit_examples/blob/main/src/hello_moveit.cpp. Den går ut på at vi setter posisjonen og orientasjonen til armen, og bruke moveit sin inverkinematikkløser til å gå til den riktige posisjonen. Dette gir alltid endestykke riktig posisjon og orientasjon, men har en utfordring med at ikke alle joints blir satt, så noen ganger kan deler av hånda til robotarmen komme i veien for kamera. Noen ganger vil også armen ta runder for å komme til riktig posisjon, dette kommer nok av hvordan inverskinematikken er laget.
