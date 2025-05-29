# ur_mover

`ur_mover` er en ROS 2-node som mottar posisjonsdata og sender dem som bevegelsesmål til en UR-robot via MoveIt sitt Action-grensesnitt. Noden fungerer som siste steg i et større pekesystem, og tar inn koordinater for målposisjon og utfører bevegelsen ved hjelp av MoveIt-planlegging. Den er foreløpig ufullstendig. 

## Innhold

- `ur_mover_node.py`: Hovednoden som håndterer bevegelsesinstruksjoner og MoveGroup-action
- `config/`: Inneholder eventuelle konfigurasjonsfiler
- `launch/ur_mover_launch.py`: ROS 2 launch-fil for å starte noden
- `setup.py` og `package.xml`: ROS 2-pakkedefinisjon

## Avhengigheter

- ROS 2 (Jazzy)
- `moveit_msgs`
- `geometry_msgs`
- `shape_msgs`
- `rclpy`
- `std_msgs`

## Funksjonalitet

Noden lytter på følgende topics:
- `/next_pos` (`Float64MultiArray`): Inneholder en posisjon som roboten skal bevege seg til (x, y, z)
- `/movement_state` (`Int32`): Startsignal med verdi `1` for å initiere bevegelse

Og publiserer:
- `/movement_state` (`Int32`): Sender tilbake `2` når bevegelse er fullført, eller feilmelding ved avbrudd

## Eksempel på oppstart

Bygg pakken:
```bash
colcon build --packages-select ur_mover
source install/setup.bash
```

Start noden med launch-fil:
```bash
ros2 launch ur_mover ur_mover_launch.py
```

## Eksempel på melding

Publiser et mål manuelt:
```bash
ros2 topic pub /next_pos std_msgs/msg/Float64MultiArray "{data: [0.4, -0.2, 0.2]}"
ros2 topic pub /movement_state std_msgs/msg/Int32 "data: 1"
```
