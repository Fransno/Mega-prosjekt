# cube_bringup

`cube_bringup` er en ROS2-pakke som samler og starter hele kubepekersystemet i én launch-fil. Den inkluderer kamerapipeline, feilhåndtering, planlegging og bevegelseskontroll.

## Innhold

- `launch/cube_bringup_launch.py` – Hoved-launchfil som inkluderer launch-filene fra:
  - `camera_pipeline`
  - `error_handler`
  - `movement_planner`
  - `movement_controller`
- `package.xml`, `setup.py`, `setup.cfg` – ROS2-metadata og installasjonsskript

## Avhengigheter

Denne pakken forutsetter at følgende ROS2-pakker er installert og tilgjengelige:

- `camera_pipeline`
- `error_handler`
- `movement_planner`
- `movement_controller`

## Bruk

Start hele systemet med én kommando:

```bash
ros2 launch cube_bringup cube_bringup_launch.py
```

Dette vil automatisk:
1. Starte kameranoden og sette parametere fra `config.json`
2. Starte `error_handler_node` for kubevalidering og kontrolltilstand
3. Starte `movement_planner_node` for generering av posisjonssekvens
4. Starte `movement_controller_node` for bevegelsesstyring
