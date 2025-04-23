# Megaprosjekt-AIS2105
# AIS2105 - Mekatronikk og robotikk

## Innhold
- [Gruppe](#Gruppe)
- [Om prosjektet](#Om-prosjektet)
- [Viktige komandoer](#Viktige-komandoer)

# Gruppe
Prosjektgruppe 163

Eldar Helseth &amp;
Oliver Steinnes Gundersen &amp;
Nikolai Dworacek &amp;
Magnus Evenstuen

# Om prosjektet
I dette prosjektet skal vi kombinere bildebehandling og robotstyring. Vi bruker en UR5e-robot med et xxxx kamera og flytter rundt på kuber. 

# Viktige komandoer
```
colcon build # Bygger alle pakkene i workspace
```
```
colcon build --packages-select qube_bringup  # Bygger kun den spesifiserte pakken
```
```
source install/setup.bash  # Kilde oppsett etter bygging
