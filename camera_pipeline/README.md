# camera_pipeline

Dette ROS2-pakken håndterer bildeinnhenting fra USB-kamera og fargedeteksjon i HSV-fargerommet. Den publiserer både visuelle hjelpebilder og koordinater til detekterte fargede objekter.

## Innhold

- `color_detector.py`: Hovednoden for fargedeteksjon.
- `launch/pipeline_launch.py`: Starter både `usb_cam` og `color_detector` med tilpassede kamerainnstillinger.
- `config/default_cam.yaml`: Standard kamerakalibreringsfil.

## Funksjonalitet

- Detekterer fargede objekter (rød, gul, blå, grønn) i sanntid.
- Bruker HSV-fargerom og konfigurerbare grenser for robusthet mot lysvariasjon.
- Publiserer:
  - Maskebilder for hver farge
  - Annotert originalbilde med konturer og sentre
  - Liste over objekter som `[fargekode, x, y, fargekode, x, y, ...]` på `/detected_cubes`
  - Et spotlight-bilde med kun fargede områder markert

## Avhengigheter

- `cv_bridge`, `sensor_msgs`, `std_msgs`, `usb_cam`
- OpenCV og NumPy

## Oppsett og kjøring

1. Installer avhengigheter:

   ```sudo apt install ros-jazzy-usb-cam ros-jazzy-cv-bridge```
3. Legg til pakken i ditt ROS2-workspace og bygg:

   ```colcon build --packages-select camera_pipeline```
5. Kjør launch-filen:

   ```ros2 launch camera_pipeline pipeline_launch.py```

### Eksempel på melding

`/detected_cubes: Float64MultiArray
data: [1.0, 320.0, 240.0, 2.0, 400.0, 250.0, 3.0, 100.0, 150.0]`

## Konfigurasjon

`config.json` inneholder fargegrenser og kamerainnstillinger (eksponering, kontrast osv.).

Du kan tilpasse `default_cam.yaml` for kamerakalibrering hvis nødvendig.

