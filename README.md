**Oryginalna dokumentacja repozytorium:**
[Original README](docs/README.md)

### TODO:
- [ ] Dodać znaki do świata.
- [ ] Launch file.

### Przydatne komendy / informacje
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
