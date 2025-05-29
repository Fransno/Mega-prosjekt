# camera_pipeline

Dette ROS2-pakken håndterer bildeinnhenting fra USB-kamera og fargedeteksjon i HSV-fargerommet. Den publiserer både visuelle hjelpebilder og koordinater til detekterte fargede objekter.

## Innhold

- `color_detector.py`: Hovednoden for fargedeteksjon.
- `launch/pipeline_launch.py`: Starter både `usb_cam` og `color_detector` med tilpassede kamerainnstillinger.
- `config/default_cam.yaml`: Standard kamerakalibreringsfil.
- `config.json`: Inneholder terskler for fargedeteksjon og kamerainnstillinger.

## Funksjonalitet

- Detekterer fargede objekter (rød, gul, blå, grønn) i sanntid.
- Bruker HSV-fargerom og konfigurerbare grenser for robusthet mot lysvariasjon.
- Publiserer:
  - Maskebilder for hver farge
  - Annotert originalbilde med konturer og sentre
  - Liste over objekter som `[fargekode, x, y, fargekode, x, y, ...]` på `/detected_cubes`
  - Et spotlight-bilde med kun fargede områder markert

## Avhengigheter

- ROS2 (testet med Humble)
- `cv_bridge`, `sensor_msgs`, `std_msgs`, `usb_cam`
- OpenCV og NumPy

## Oppsett og kjøring

1. Installer avhengigheter:

   ```bash sudo apt install ros-humble-usb-cam ros-humble-cv-bridge```
3. Legg til pakken i ditt ROS2-workspace og bygg:

   ```colcon build --packages-select camera_pipeline```
5. Kjør launch-filen:

   ```ros2 launch camera_pipeline pipeline_launch.py```

