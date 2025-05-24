# Megaprosjekt -
# AIS2105 - Mekatronikk og robotikk

![forside](https://github.com/user-attachments/assets/471e2185-74b4-4740-a66b-cd0663d9b0d9)

## Innhold
- [Gruppe](#gruppe)
- [Om prosjektet](#om-prosjektet)
- [Hva kan roboten gjøre](#hva-kan-roboten-gjøre)
- [Viktige komandoer](#viktige-komandoer)
- [Oppsett](#veiledning-til-oppsett-av-prosjektet)
  - [ROS 2](#ros-2)
  - [Nedlastinger](#nedlastinger)
  - [Building](#building)
  - [Build uten crash!](###Build-uten-crash!) 
  - [Klargjøring av robot](#klargjøring-av-robot)
  - [IP-oppsett](#ip-oppsett)
  - [Surface oppsett](#surface-oppsett)
  - [Oppsett på egen maskin](#Oppsett-på-egen-maskin)
  - [Problemer?](#problemer?)
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
Roboten kan gå til hjemmeposisjon. Så kan den gå til kameraposisjon, så leter den etter kuber, også kan den  gå til hver av kubene i denne rekkefølgen:
1. Rød
2. Gul
3. Blå
4. Grønn

Dersom den ikke ser noe kube starter den en sekvens for å lete etter kuber.

# Veiledning til oppsett av prosjektet
Dersom du ønsker å kjøre prosjektet, kan du følge denne veiledningen som vil ta deg gjennom alle stegene du er nødt å gjøre. Veiledningen er laget for de som bruker Linux operativsystem og vi bruker en UR5e-robot. Som opreativsystem har vi valgt å gjøre denne veiledningen for linux der vi bruker Ubuntu 20.04. Merk også at for å kunne følge følge denne veiledningen trenger du tilgang til Adam Leon Kleppe (foreleser i AIS2105) sin Microsoft Surface nettbrett som er satt opp for dette prosjektet. Link til github prosjektet finner du her (sett inn link).

# Veiledning til oppsett av prosjektet

Dersom du ønsker å kjøre prosjektet, kan du følge denne veiledningen som tar deg gjennom alle nødvendige steg. Veiledningen er laget for brukere av Linux (Ubuntu 20.04) og omhandler en UR5e-robot. Merk også at du må ha tilgang til Adam Leon Kleppe (foreleser i AIS2105) sin Microsoft Surface-nettbrett, som er forhåndskonfigurert for dette prosjektet. Link til GitHub-prosjektet finner du her (sett inn link).

## ROS 2

Dette prosjektet krever at du har ROS 2 installert. Du kan følge denne veiledningen:  
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

## Nedlasting

Etter at du har installert ROS 2, må du laste ned prosjektet. Gå til mappen der du vil ha prosjektet, høyreklikk og velg **Åpne i terminal**. Lim inn kommandoen:

```
git clone https://github.com/MagnusEvenstuen/Megaprosjekt-AIS2105.git
```

Dette vil laste ned prosjektet til mappen du åpnet terminalen i. Denne mappen blir ditt workspace for prosjektet.

## Building

Gå til workspace-mappen, høyreklikk og velg **Åpne i terminal**. Lim inn følgende kommando for å bygge:

```
colcon build
```

Når du kjører kommandoen, starter byggeprosessen. Colcon identifiserer først alle avhengigheter og ROS 2-pakker, kompilerer kildekoden og genererer deretter de nødvendige konfigurasjons- og oppsettfiler.

Etter at byggingen er fullført, lim inn denne kommandoen i samme terminal:

```
source install/setup.bash
```

Dette oppdaterer miljøvariablene slik at systemet finner de nylig kompilert pakkene.

### Build uten crash!
Når man skal bygge for første gang så kan pc-en kræsje. For å unngå dette kjør følgende kommandoer:

```
`export MAKEFLAGS="-j 1"`

`colcon build --executor sequential`
```
Dette trenger man bare å gjøre en gang. Det gjør at man bygge pakkene sekvensielt (én og én) i stedet for i parallell. Etter det kan man bygge vanlig med:

```
colcon build
```


## Klargjøring av robot

Vi bruker en UR5e-robot og et Sandberg USB-kamera (andre USB-kameraer fungerer også). For å feste kameraet har Adam Leon Kleppe designet og 3D-printet en brakett med “custom quick removal supports”, som gjør det enkelt å montere både kamera og griper samtidig. Braketten festes til roboten med fire M5-skruer.

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

Etter at Surface-maskinen har startet opp i Ubuntu og nettverksinnstillingene er verifisert, åpner du en terminaler (Ctrl+Alt+T).

Start med å sette den samme doimain ID-en som på surfacen tidligere:

```
export ROS_DOMAIN_ID=X
```
Etterpå kan du kopiere denne lauch filen I terminale:

```
  ros2 launch ros2 launch ur_robot_driver ur_control.launch.py ur_type:=urX ​robot_ip:=yyy.yyy.yyy.yyy use_mock_hardware:=false
initial_joint_controller:=scaled_joint_trajectory_controller headless_mode:=true
```

Åpne en ny terminal (Ctrl+ALT+T) og kopier følgende inn i terminale:

```
  ros2 launch ros2 launch ur_robot_driver ur_control.launch.py ur_type:=urX robot_ip:=yyy.yyy.yyy.yyy use_mock_hardware:=false
initial_joint_controller:=scaled_joint_trajectory_controller
```

Disse to launch-filene starter alle nødvendige tjenester for å kjøre prosjektet. Nå vil RViz starte, og du kan styre roboten via GUI.

![Robot](https://github.com/user-attachments/assets/67df50bb-ed99-40b8-9fdc-eb3f27eaca6e)

Etter kjøring av launch filene, skal rviz dukke opp med noe lignende bildet over. Posisjonen på roboten i Rviz skal være lik den faktiske posisjonen på roboten som er tilkoblet

![robotEndra](https://github.com/user-attachments/assets/a437ffec-1c5c-4b81-890f-91aa5fcf3997)

For å teste om Moveit kan bevege roboten så kan man dra i den "blå ballen" pil (1), så trykke på "Plan &amp; execute" pil(2) og så skal roboten flytte seg til den valgte posisjonen. Start gjerne med en liten men merkbar bevegelse for å forsikre at bevegelsen ikke treffer på en singularitet. Om alt fungerer så langt så er du på god vei!


## Kjøring fra egen maskin

Start med å sette den samme doimain ID-en som på surfacen tidligere:

```
export ROS_DOMAIN_ID=X
```

Sjekk at 'topics' kan leses på laptoppen med:

```
ros2 topic list
```

Om du er tilkoblet roboten og surfacen så skal det dukke opp mange topics i terminalen. Om du bare får 2-3 stk, så er det ikke sikkert du har kontakt.

Deretter kjører man launch fila i simple_mover:

```
ros2 launch simple_mover robot.launch.py
```

Då skal roboten gå til "camera_home" hvor den tar et bilde og leter etter 4 bokser. En rød, en gul en blå og en grønn. Så vil roboten gå å nært og "se" på alle boksene i den rekkefølgen. Alle bevegelsene til roboten er delt to, så først beveger den seg langs x og y, før den skifter mellom z 0.2 og 0.6. Alle bevegelsene har også eitt delay på 3 sekunder for å sikre at roboten får fullført bevegelsen sin før den begynner på en ny en. Om roboten ikke ser alle fire boksene på bildet som er tatt så vil den gå inn i søkemodus.

Når man kjører launch fila så kan man også legge til eit parameter for å kjøre roboten heim eller i "sleep":

```
ros2 launch simple_mover robot.launch.py home:=true
```

Om home parameteren som har standardverdien "false" blir satt til "true" så vil roboten kjøre til den valgte hjem posisjonen, så stopper programmet.


Og legge til 'motionPlanning' med ADD funksjonen. Man kan så gå inn på 'joints', gjøre endringer, så trykke 'plan' og 'execute'. Da skal roboten flytte seg. 

## Kjøring
Du kan nå plassere kuber på bordet. Når du kjører launch filen vil kamera gjennkjenne kubene, roboten vil først flytte seg til rød, så til gul, så til blå også videre til grønn, til slutt vil den gå tilbake til home posisjon. 

## Problemer?

Om du får problemer, er det ikke nødvendigvis din feil, roboten kan bare ha en dårlig dag. Vi anbefaler å snakke rolig med den, spørre pent og stryke den forsiktig før og under kjøring. Dette kan hjelpe roboten til å føle seg bedre, og plutselig begynne å fungere.

Dersom dette ikke løser problemet, kan det være lurt å sjekke nettverkstilkoblingene. Prøv å pinge roboten fra Surface-nettbrettet først. Dersom dette fungerer, prøv å pinge både roboten og Surface fra din egen PC.

Åpne en terminal (Ctrl+Alt+T):

```
ping 143.25.XXX.X
```

Fungerer ikke dette, er neste steg å starte alle enhetene på nytt. Dette løser problemet i omtrent 50 % av tilfellene (men er tidkrevende).

Har du fortsatt problemer? Da må det tilkalles ekspert-hjelp: https://www.ntnu.no/ansatte/adam.l.kleppe.



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

Om kamera ikkje kjører, bruk denne kommandoen for å finne alle kamera tilkopla PC-en:

```
v4l2-ctl --list-devices
```

# Om pakkene

### camera_detector
camera_detector pakken bruker openCV til å detektere firkanter av forskjellige farger. Pakken detekterer farger med å gjøre om RGB verdier til HSV verdier, så ser den på om kulør, metning og lysstyrke er innenfor de satte verdiene, og hvis det er det er det riktig farge. Etter at forskjellige objekter er detektert sjekkes det hvor mange kanter det er på hver av de formene. Den står da bare igjen med former med 4 kanter og ingen kurver på de kantene. Så sees det på hvor kvadratisk de forskjellige firkantene er. 

Denne måten å løse detektering av kuber på har både fordeler og ulemper. En fordel er at den er veldig god til å detektere firkanter. Den er også ganske robust på kvadrat i forskjellige høyder og størrelser så lenge de er større enn den minste tillatte størrelsen i koden. Ulempa er at den ikke er så robust når det kommer til forskjellig orientasjon på kubene. Hvis kubene står diagonalt på bildet (balanserer på en kant) vil ikke pakken klare å detektere at det er en kube der. Det kan også skje at fargedeteksjonen får en kube til å se litt for rund ut, og derfor ikke se på det som en kube.

Posisjonen til kubene i kamera koordinat gjøres om til robotkoordinat og publiseres når det kommer en kommando om at robotkontrolleren (simple_mover) vil ha posisjonen til kubene. Utregningen for kamerakoordinat til robotkoordinat tar utgangspunkt i at kamera er i kameraets hjemmeposisjon, så når det skal tas bilder i andre posisjoner hvis roboten ikke finner alle kubene må denne offsetten legges på verdien.

### simple_mover
Denne pakken har vi hentet en del inspirasjon til fra https://github.com/dominikbelter/ros2_ur_moveit_examples/blob/main/src/hello_moveit.cpp. Den går ut på at vi setter posisjonen og orientasjonen til armen, og bruke moveit sin inverkinematikkløser til å gå til den riktige posisjonen. Dette gir alltid endestykke riktig posisjon og orientasjon, men har en utfordring med at ikke alle joints blir satt, så noen ganger kan deler av hånda til robotarmen komme i veien for kamera. Noen ganger vil også armen ta runder for å komme til riktig posisjon, dette kommer nok av hvordan inverskinematikken er laget.

### move_it
Prosjektet bruker move_it til å planlegge og utføre bevegelser. Du kan lese om move_it her https://moveit.picknik.ai/main/index.html


Vår gud:

![memeTemplateAdamAdam](https://github.com/user-attachments/assets/3dcbdf57-e4b3-4326-ab92-95255afd9012)

# Video fra prosjeket:




https://github.com/user-attachments/assets/ace8ee86-3726-42e3-93b0-7016f1b89c96




https://github.com/user-attachments/assets/9757e7fc-8c5f-481d-886a-b7b0235f3ff2

