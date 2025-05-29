# error_handler

Dette ROS2-pakken inneholder noden `error_handler_node`, som overvåker deteksjonsdata og bestemmer systemets neste kontrolltilstand. Systemet er designet for å oppdage feil i kube-deteksjon og styre prosessen videre avhengig av suksess eller gjentatte feil.

## Innhold

- `error_handler_node.py` – ROS2-node for feilhåndtering basert på innkommende deteksjonsdata
- `launch/error_handler_launch.py` – Launch-fil for å starte noden
- `package.xml` og `setup.py` – ROS2-metadata og installasjonsskript

## Avhengigheter

- `rclpy`
- `std_msgs`
- `collections`
- `time`

## Beskrivelse av funksjon

Noden abonnerer på:
- `/detected_cubes` (`Float64MultiArray`): Liste over detekterte kuber (fargekode, x, y)
- `/controll_state` (`Int32`): Gjeldende kontrolltilstand

Noden publiserer til:
- `/controll_state` (`Int32`): Ny kontrolltilstand basert på feilhåndtering

### Kontrolltilstander

- `0` = Avventer kubeinput
- `1` = Klar for planlegging og bevegelse (alle kuber funnet)
- `2` = Rescan (ikke alle kuber funnet)
- `3` = Returner til hjemposisjon (start-/resettilstand, håndteres av controller)
- `4` = Abort (flere feilforsøk enn tillatt)

### Logikk

1. Initialiseres med ønsket kube-sekvens fra `config.json`, og publiserer kontrolltilstand `3` for å flytte roboten til hjem posisjon.
2. Teller antall forekomster av hver farge i `detected_cubes`.
3. Hvis alle ønskede kuber er funnet: Publiser kontrolltilstand `1`.
4. Hvis noen mangler: Publiser `2` og øk intern feil-teller.
5. Etter 3 mislykkede forsøk: Publiser `4` (abort).

## Bruk

For å starte noden:
```bash
ros2 launch error_handler error_handler_launch.py
```
