**Oryginalna dokumentacja repozytorium:**
[Original README](docs/README.md)

## Spis treści
1. [Opis projektu](#opis-projektu) 

## Opis projektu
Projekt miał na celu zaprojektowanie i implementację autonomicznego samochodu działającego w środowisku symulacyjnym ROS + Gazebo. W ramach projektu stworzono system umożliwiający pojazdowi samodzielne poruszanie się po drodze, wykrywając i interpretując elementy otoczenia na podstawie obrazu z kamery. Algorytm analizuje obraz z kamery zamontowanej na samochodzie, aby wykrywać linię na środku jezdni. Na tej podstawie samochód dynamicznie wyznacza trajektorię ruchu, utrzymując się w granicach wyznaczonego pasa. Detekcja znaków drogowych odbywała się za pomocą wytrenowanej wcześniej sieci neuronowej, dzięki czemu system jest w stanie rozpoznać różne znaki, takie jak ograniczenia prędkości, znaki stopu czy ostrzeżenia.


### Przydatne komendy / informacje:
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

### Potencjalnie przydatne źródła:
- [ROS2-Self-Driving-Car-AI-using-OpenCV](https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV/tree/main/self_driving_car_pkg/worlds)
- [r2s: interfejs CLI](https://github.com/mjcarroll/r2s)
