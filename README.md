# prosjektoppgave
Prosjektoppgave på NTNU. 

# Viktig!

# Benytte bridge mellom ROS og OpenCV

For å få cv_bridge til å linke korrekt, må repoet klones lokalt og cv_bridge mappa må legges i catkin_ws. Det må deretter bygges på nytt ved hjelp av: "catkin_make --pkg cv_bridge"

[ros-perception github repo](https://github.com/ros-perception/vision_opencv)

# Samkjøre ROS, OpenCV, RPC og AirSim/AirLib

Airlib og RPC er i utgangspunktet kompilert med clang 5.0, hvor libc++ er brukt som standard library (std::). Hvis du insstallerer ROS direkte i Ubuntu vill ikke disse bibliotekene linkes riktig sammen, og du vil få linker errors som inneholder std::\_\_cxx1, std::\_\_c++ eller lignende. Dette betyr stort sett at koden er kompilert med forskjellig standard library.

Jeg har kun fått alt til å fungere sammen med å bruke g++ (GNU compiler).