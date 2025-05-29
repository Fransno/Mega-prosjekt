# Kubepeker – ROS2-basert system

Dette systemet bruker ROS2 og et kamera til å detektere fargede kuber, planlegge posisjoner og styre en UR robotarm til å peke på dem i gitt rekkefølge. Systemet er delt inn i seks hovedpakker:

- `camera_pipeline`: Utfører fargedeteksjon med OpenCV
- `error_handler`: Overvåker at riktige kuber er funnet
- `movement_planner`: Planlegger posisjoner basert på kube-koordinater
- `movement_controller`: Styrer selve bevegelsen basert på systemets kontrolltilstand
- BRINGUP
- URMOVER

## Systemoversikt

1. Kamera strømmer bilder.
2. `camera_pipeline` utfører fargebasert segmentering i HSV og publiserer kube-koordinater.
3. `error_handler` sjekker om alle ønskede kuber er funnet, og styrer videre flyt.
4. `movement_planner` bygger ønsket posisjonsliste for robotarmen.
5. `movement_controller` styrer roboten til hjemposisjon, zoomer, eller peker på kuber.

## Konfigurasjon

Systemet bruker en felles `config.json`-fil som definerer fargegrenser, posisjoner og systemoppførsel. Eksempel:

```json
{
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
  "contour_min_area": 100,
  "home_position": [0.0, 0.0, 50.0],
  "zoom_value": 10.0,
  "controll_state": 5,
  "cube1": "blue",
  "cube2": "red",
  "cube3": "yellow",
  "z": 10.0,
  "debug_mode": true,
  "pipeline.launch.py": {
    "autoexposure": false,
    "exposure_absolute": 200,
    "auto_white_balance": false,
    "white_balance_temperature": 55000,
    "brightness": 5,
    "contrast": 20,
    "saturation": 30,
    "gain": 2
  }
}
```



## Kontrolltilstander

Systemet opererer med enkle tilstandskoder for koordinering:

- `0` = Avventer kubeinput
- `1` = Klar for planlegging og bevegelse
- `2` = Rescan (ikke alle kuber funnet)
- `3` = Returner til hjemposisjon (starttilstand)
- `4` = Avbryt etter for mange feil
- `5` = Init (fra config)

## Oppstart

Start pakkene enkeltvis med f.eks.:
```bash
ros2 launch camera_pipeline pipeline_launch.py
ros2 launch error_handler error_handler_launch.py
ros2 launch movement_planner movement_planner_launch.py
ros2 launch movement_controller movement_controller_launch.py -- blir feil

andre greiene

```

## Fargekoder

Farger representeres som tall i `detected_cubes`:
- `1.0` = Rød
- `2.0` = Gul
- `3.0` = Blå
- `4.0` = Grønn

## Ekstra funksjonalitet

- **Debug-modus**: Slår på logging for feilsøking
- **Zoom-effekt**: Flytter robotarmen opp ved rescan
- **Sirkulær tilstandsmaskin**: Returnerer til tilstand 0 etter handlinger
- legge til for flere kuber av samme farge osv. 


## Avhengigheter

- ROS2 (Jazzy)
- OpenCV (`cv2`)
- NumPy
- `cv_bridge`, `usb_cam`, `std_msgs`, `sensor_msgs`
