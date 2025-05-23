# Megaprosjekt -
# AIS2105 - Mekatronikk og robotikk

## Innhold
- [Gruppe](#gruppe)
- [Om prosjektet](#om-prosjektet)
- [Hva kan roboten gjøre](#hva-kan-roboten-gjøre)
- [Viktige komandoer](#viktige-komandoer)
- [Oppsett](#veiledning-til-oppsett-av-prosjektet)
  - [ROS 2](#ros-2)
  - [Nedlastinger](#nedlastinger)
  - [Bugging](#bygging)
  - [Klargjøring av robot](#klargjøring-av-robot)
  - [IP-oppsett](#ip-oppsett)
  - [Surface oppsett](#surface-oppsett)
  - [Oppsett på egen maskin](#Oppsett-på-egen-maskin)
- [Styring fra ekstern laptop](#styring-fra-extern-laptop)
- [Build uten crash!](#build-uten-crash)
- [Kamera](#kamera)
- [Om pakkene](#om-pakkene)

# Gruppe
Prosjektgruppe 163

Eldar Helseth &amp;
Oliver Steinnes Gundersen &amp;
Magnus Evenstuen

# Om prosjektet
I dette prosjektet skal vi kombinere bildebehandling og robotstyring. Vi bruker en UR5e-robot med et xxxx kamera og flytter rundt på kuber. 

# Hva kan roboten gjøre
Gå til hjemmeposisjon, scanne arbeisområde, gjennkjenne kuber etter farger, plukke opp kuber i denne rekkefølgen. 

# Veiledning til oppsett av prosjektet
Dersom du ønsker å kjøre prosjektet, kan du følge denne veiledningen som vil ta deg gjennom alle stegene du er nødt å gjøre. Veiledningen er laget for de som bruker Linux operativsystem og vi bruker en UR5e-robot. Som opreativsystem har vi valgt å gjøre denne veiledningen for linux der vi bruker Ubuntu 20.04. Merk også at for å kunne følge følge denne veiledningen trenger du tilgang til Adam Leon Kleppe (foreleser i AIS2105) sin Microsoft Surface nettbrett som er satt opp for dette prosjektet. Link til github prosjektet finner du her (sett inn link).

# Veiledning til oppsett av prosjektet

Dersom du ønsker å kjøre prosjektet, kan du følge denne veiledningen som tar deg gjennom alle nødvendige steg. Veiledningen er laget for brukere av Linux (Ubuntu 20.04) og omhandler en UR5e-robot. Merk også at du må ha tilgang til Adam Leon Kleppe (foreleser i AIS2105) sin Microsoft Surface-nettbrett, som er forhåndskonfigurert for dette prosjektet. Link til GitHub-prosjektet finner du her (sett inn link).

## ROS 2

Dette prosjektet krever at du har ROS 2 installert. Du kan følge denne veiledningen:  
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

## Nedlasting

Etter at du har installert ROS 2, må du laste ned prosjektet. Gå til mappen der du vil ha prosjektet, høyreklikk og velg **Åpne i terminal**. Lim inn kommandoen:

```bash
git clone https://github.com/MagnusEvenstuen/Megaprosjekt-AIS2105.git
```

Dette vil laste ned prosjektet til mappen du åpnet terminalen i. Denne mappen blir ditt workspace for prosjektet.

## Bygging

Gå til workspace-mappen, høyreklikk og velg **Åpne i terminal**. Lim inn følgende kommando for å bygge:

```bash
colcon build
```

Når du kjører kommandoen, starter byggeprosessen. Colcon identifiserer først alle avhengigheter og ROS 2-pakker, kompilerer kildekoden og genererer deretter de nødvendige konfigurasjons- og oppsettfiler.

Etter at byggingen er fullført, lim inn denne kommandoen i samme terminal:

```bash
source install/setup.bash
```

Dette oppdaterer miljøvariablene slik at systemet finner de nylig kompilert pakkene.

## Klargjøring av robot

Vi bruker en UR5e-robot og et Sandberg USB-kamera (andre USB-kameraer fungerer også). For å feste kameraet har Adam Leon Kleppe designet og 3D-printet en brakett med “custom quick removal”, som gjør det enkelt å montere både kamera og griper. Braketten festes til roboten med fire M5-skruer.

<p align="center">
  <img src="https://github.com/user-attachments/assets/ca00e5b0-8544-4d61-baa7-097da6e12eba" alt="Robot uten feste" width="300" />
  <img src="https://github.com/user-attachments/assets/d0a31847-35da-477d-a555-6f6326f17163" alt="Kamera feste" width="300" />
</p>

## IP-oppsett

Etter oppstart av roboten er det viktig at roboten, Surface-nettbrettet og PC-en din har riktige nettverksinnstillinger og ligger i samme nettverk.

1. Trykk på de tre linjene øverst til høyre på robotens skjerm.  
   ![Bilde tre strek robotskjerm](https://github.com/user-attachments/assets/a034907f-a56b-432a-b73b-590183218c36)

2. Velg **Settings** i menyen som dukker opp.  
   ![Meny for Settings](https://github.com/user-attachments/assets/2b8b9a71-5a73-415b-9035-4cc9cd3fa0a9)

3. Velg **Network** fra menyen til venstre.  
   ![Nettverksinnstillinger](https://github.com/user-attachments/assets/fff002c2-9549-42c0-8c8a-7c7dfa1bec46)

Under prosjektet har vi brukt disse innstillingene:

- **IP address:** 143.25.150.7  
- **Subnet mask:** 255.255.252.0  
- **Default gateway:** 143.25.151.0  

Det er ikke nødvendig å bruke akkurat disse, men viktig at default gateway er identisk for PC, Surface og robot, og at hver enhet har en unik IP-adresse.

4. Sjekk at roboten er koblet til nettverket – en grønn hake vises når den er tilkoblet.  
   ![Sjekk nettverk](https://github.com/user-attachments/assets/60be46bd-ec5c-4143-a47d-b866828f2ce5)

5. Bytt modus fra **Lokal** til **Remote** på roboten.  
   ![Lokal/Remote](https://github.com/user-attachments/assets/63d2118e-f250-4130-b8ab-54633cfa4819)

## Surface-oppsett

Etter at Surface-maskinen har startet opp i Ubuntu og nettverksinnstillingene er verifisert, åpner du to terminaler (Ctrl+Alt+T).

- I den ene terminalen kjører du:
  ```bash
  <lim inn kommando>
  ```
- I den andre terminalen kjører du:
  ```bash
  <lim inn kommando>
  ```

Disse to launch-filene starter alle nødvendige tjenester for å kjøre prosjektet. Nå vil RViz starte, og du kan styre roboten via GUI.

## Oppsett på egen maskin

<!-- Fortsett her med egen maskin-oppsettet -->

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
