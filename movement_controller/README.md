# movement_controller

Denne ROS2-pakken inneholder noden `movement_controller_node`, som styrer hvilken posisjon roboten skal bevege seg til basert på systemets kontrolltilstand og innkommende bevegelser.

## Innhold

- `movement_controller_node.py` – ROS2-node for å styre bevegelsesflyt
- `launch/movement_controller_launch.py` – Launch-fil for noden
- `package.xml` og `setup.py` – ROS2-metadata og installasjonsskript

## Avhengigheter

- `rclpy`
- `std_msgs`

## Abonnement og publisering

**Abonnerer på:**
- `/controll_state` (`Int32`) – Gjeldende kontrolltilstand
- `/movement_state` (`Int32`) – Status for pågående bevegelse
- `/planned_pos` (`Float64MultiArray`) – Ferdig planlagte kube-koordinater

**Publiserer til:**
- `/next_pos` (`Float64MultiArray`) – Neste posisjon roboten skal bevege seg til
- `/controll_state` (`Int32`) – Tilstand etter bevegelse
- `/movement_state` (`Int32`) – Intern status (0 = klar, 1 = kjører, 2 = ferdig)

## Kontrollflyt

Noden reagerer på `controll_state`-verdier:

- `3` – Returner til hjemposisjon
- `2` – Zoom ut og ta nytt bilde
- `1` – Beveg deg til 3 målposisjoner i rekkefølge
- Alle tilstander koordineres med `movement_state` (0 → 1 → 2)

## Tilstandsmaskin (forenklet)

- Hjemposisjon brukes som start og fallback (justert med `zoom_value` ved rescan).
- Når planlagte posisjoner er mottatt, sendes de i rekkefølge ved bevegelsesstart (`movement_state == 0`).
- Etter hver bevegelse (`movement_state == 2`) trigges neste steg.

## Bruk

Start noden med:

```bash
ros2 launch movement_controller movement_controller_launch.py
```
