**Oryginalna dokumentacja repozytorium:**
[Original README](docs/README.md)

## Spis treści
1. [Opis projektu](#opis-projektu)
2. [Przygotowanie środowiska symulacyjnego](#przygotowanie-środowiska-symulacyjnego)
3. [Implementacja algorytmu](#implementacja-algorytmu)
4. [Uruchomienie](#uruchomienie)
5. [Przydatne komendy / informacje](#przydatne-komendy--informacje)

## Opis projektu
Projekt miał na celu zaprojektowanie i implementację autonomicznego samochodu działającego w środowisku symulacyjnym ROS + Gazebo. W ramach projektu stworzono system umożliwiający pojazdowi samodzielne poruszanie się po drodze, wykrywając i interpretując elementy otoczenia na podstawie obrazu z kamery. Algorytm analizuje obraz z kamery zamontowanej na samochodzie, aby wykrywać linię na środku jezdni. Na tej podstawie samochód dynamicznie wyznacza trajektorię ruchu, utrzymując się w granicach wyznaczonego pasa. Detekcja znaków drogowych odbywała się za pomocą wytrenowanej wcześniej sieci neuronowej, dzięki czemu system jest w stanie rozpoznać różne znaki, takie jak ograniczenia prędkości, znaki stopu czy ostrzeżenia.

## Przygotowanie środowiska symulacyjnego
W projekcie zostały wykorzystane projekty gotowy znaleziony w internecie:

- [Nathan Benson Park](https://app.gazebosim.org/OpenRobotics/fuel/models/nathan_benderson_park) - Gotowy świat zawierający układ miasta z drogami z teskturami pasów.

- [Prius](https://app.gazebosim.org/OpenRobotics/fuel/models/Prius%20Hybrid%20with%20sensors) - Model pojazdu, który będzie testowany. 

- [Znak Stop](https://app.gazebosim.org/OpenRobotics/fuel/models/Stop%20Sign) - Model znaku, który posłużył jako model tekstury, w którym zmieniano grafikę, aby stworzyć inne znaki.

Wczytanie świata następuje w pliku custom_city.sdf w następujący sposób:
```bash
<include> 
  <uri> 
    model://nathan_benderson_park 
  </uri> 
</include>
```

W ten sposób, odnosimy się do lokalizacji modelu samego modelu miasta, które zostało przez nas zmodyfikowane. Każda tekstura wczytywana jest w ten sam sposób, podając jej siatkę punktów, w których ma się znaleźć, teksturę oraz inne parametry takie jak fizyczną kolizję z innymi elementami otoczenia: 

```bash
<visual name="Road_visual"> 
                <geometry> 
                    <mesh> 
                        <uri>meshes/nb_park.dae</uri> 
                        <submesh> 
                            <name>Road</name> 
                            <center>false</center> 
                        </submesh> 
                    </mesh> 
                </geometry> 
                <material> 
                    <diffuse>1.0 1.0 1.0</diffuse> 
                    <specular>0.06 0.06 0.06</specular> 
                    <pbr> 
                        <metal> 
                            <albedo_map>materials/textures/Road_Albedo.png</albedo_map> 
                            <normal_map>materials/textures/Road_Normal.png</normal_map> 
                            <roughness_map>materials/textures/Road_Roughness.png</roughness_map> 
                            <environment_map>materials/textures/EnvMap.dds</environment_map>
                        </metal> 
                    </pbr> 
                </material> 
            </visual>  
```
Do mapy również dodano znaki, tak aby przybrały możliwie realne rozmieszczenie znaków w mieście. Wśród dodanych znaków są: 

- Znak stopu 
- Znaki dopuszczalnej prędkości 
- Nakazy jazdy, skrętu 
- Znaki pierwszeństwa 
- Znak przejścia dla pieszych 

Znaki są wczytywane w ten sam sposób jak mapa oraz zmienione, tak aby miały różne grafiki. 

```bash
<include> 
      <uri>model://models/Signs/Priority_sign</uri> 
      <name>Priority Sign</name> 
      <pose>-134.9530029296875 973.00592041015625 5 0 0 0</pose> 
    </include> 
 
<?xml version="1.0" ?> 
<sdf version="1.5"> 
  <model name="Priority Sign"> 
    <static>true</static> 
    <link name="link"> 
      <collision name="collision"> 
        <geometry> 
          <mesh> 
            <scale>10 10 10</scale> 
            <uri>model://models/Signs/Priority_sign/meshes/priority_sign.dae</uri> 
          </mesh> 
        </geometry> 
      </collision> 
      <visual name="visual"> 
        <geometry> 
          <mesh> 
            <scale>10 10 10</scale> 
            <uri>model://models/Signs/Priority_sign/meshes/priority_sign.dae</uri> 
          </mesh> 
        </geometry> 
      </visual> 
    </link> 
  </model> 
</sdf> 
```
Ważnym elementem jest podanie poprawnej ścieżki do mapy punktów modelu określonym w sekcji <uri> i podanie niej odpowiedniej grafiki:

```bash
<image id="Speed_30_tga">
      <init_from>../materials/textures/priority.png</init_from>
</image> 
```
Model samochodu nie został w żaden sposób zmodyfikowany, tylko wczytany wprost z pliku wraz z określeniem miejsca, w którym ma się pojawiać na mapie względem centrum.

## Implementacja algorytmu

## Uruchomienie
Po sklonowaniu repozyturium i zbudowaniu dockera należy wykonać poniższe komendy.

Zbudowanie paczek:
```bash
colcon build --symlink-install
colcon build --packages-select autonomous_vehicle
```

Uruchomienie środowiska:
```bash
source install/setup.bash
```

Uruchomienie pliku launch:
```bash
ros2 launch autonomous_vehicle/launch/auto.launch.py
```

Uruchomienie pliku node do wyświetlania obrazów z kamer:
```bash
ros2 run autonomous_vehicle helper_node
```

## Przydatne komendy / informacje:
Bridge ROS-Gazebo:
```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=ros_gz_bridge.yaml
```
Ścieżka do modeli gazebo pobieranych z internetu:
```bash
$HOME/.gz/fuel/
```
RViz:
```bash
ros2 run rviz2 rviz2
```
Przy wizualizacji chmury punktów lidaru trzeba to umieścić w "Global Options / Fixed Frame"
```bash
prius_hybrid_sensors/sensors/center_laser_sensor
```
