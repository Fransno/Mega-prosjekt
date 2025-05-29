# Robotisk kubepeker – ROS2-basert system

Dette systemet bruker ROS2 og et kamera til å detektere fargede kuber, planlegge posisjoner og styre en robotarm til å peke på dem i gitt rekkefølge. Systemet er delt inn i fem ROS2-pakker:

## Pakker

- **`camera_pipeline`** – Utfører bildebehandling og fargedeteksjon
- **`error_handler`** – Sjekker at riktige kuber er funnet, og styrer systemtilstand
- **`movement_planner`** – Planlegger posisjoner basert på kube-koordinater
- **`movement_controller`** – Styrer robotens bevegelser mellom posisjoner
- **`cube_bringup`** – Samler alle nodene i ett samlet launch-oppsett

## Systemoversikt

1. Kamera strømmer bilder.
2. `camera_pipeline` utfører HSV-fargebasert segmentering og publiserer kube-koordinater.
3. `error_handler` sjekker om alle ønskede kuber er funnet, og sender tilstandsbeskjeder.
4. `movement_planner` bygger ønsket posisjonsliste for robotarmen.
5. `movement_controller` tolker tilstanden og sender bevegelser til roboten.
6. `cube_bringup` starter hele systemet med én launchfil.

## Kontrolltilstander

- `0` = Avventer kubeinput
- `1` = Klar for planlegging og bevegelse (alle kuber funnet)
- `2` = Rescan (ikke alle kuber funnet)
- `3` = Returner til hjemposisjon (f.eks. ved oppstart)
- `4` = Abort (for mange mislykkede forsøk)
- `5` = Init (startverdi satt i `config.json`)

## Konfigurasjon

En felles `config.json` brukes for å sette:
- HSV-fargegrenser
- Minimumskrav for kontur-område
- Kube-rekkefølge (`cube1`, `cube2`, `cube3`)
- Startposisjon og zoom-parameter
- Kamerainnstillinger
- Debug-modus

Eksempel:
```json
{
  "cube1": "blue",
  "cube2": "red",
  "cube3": "yellow",
  "home_position": [0.0, 0.0, 50.0],
  "z": 10.0,
  "zoom_value": 10.0,
  "controll_state": 5,
  "debug_mode": true,
  "color_boundary": {
    "red_lower": [0, 50, 50],
    "red_upper": [15, 255, 255],
    "red2_lower": [155, 50, 50],
    "red2_upper": [190, 255, 255],
    "yellow_lower": [20, 50, 50],
    "yellow_upper": [40, 255, 255],
    "blue_lower": [95, 70, 70],
    "blue_upper": [140, 255, 255],
    "green_lower": [35, 50, 50],
    "green_upper": [90, 255, 255]
  },
  "contour_min_area": 100
}
```

## Oppstart

For å starte hele systemet:

```bash
ros2 launch cube_bringup cube_bringup_launch.py
```

Alternativt kan du starte hver node individuelt:

```bash
ros2 launch camera_pipeline pipeline_launch.py
ros2 launch error_handler error_handler_launch.py
ros2 launch movement_planner movement_planner_launch.py
ros2 launch movement_controller movement_controller_launch.py
```

## Fargekoder i meldinger

- `1.0` = Rød
- `2.0` = Gul
- `3.0` = Blå
- `4.0` = Grønn

## Avhengigheter

- ROS2 (f.eks. Humble)
- `rclpy`, `std_msgs`, `sensor_msgs`
- `OpenCV`, `NumPy`, `cv_bridge`
- USB-kamera med støtte for V4L2

## Lisens

Apache-2.0
