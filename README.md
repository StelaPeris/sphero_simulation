# sphero_simulation

MuJoCo simulacija sfernog robota s njihalom, s ROS 2 sučeljem.

## Struktura

```
sphero_simulation/
├── package.xml, setup.py, setup.cfg, resource/
├── sphero_simulation/
│   ├── sim_core.py      MuJoCo (bez ROS-a): model, aktuatori (vx/vy -> ESC/servo), telemetrija, IMU
│   ├── sim_node.py      ROS 2 čvor simulacije: glumi serijski port (sim/cmd, sim/telemetry) i kameru
│   ├── sim_bridge.py    ROS 2 čvor: robot_bridge za simulaciju (iste petlje, iste teme /robot<ID>/...)
│   └── control/         regulatori i filtri iz robot_bridge.py, bez ROS-a
│       ├── pid.py               YawPID, RollPID, SpeedPID
│       ├── kalman.py            KalmanCV, SpeedEstimator (brzina iz poze kamere, kao on_pose)
│       └── util.py              clamp, wrap180, heading_from_quat
├── models/
│   ├── pendulum/        njihalo iz CAD-a bez ljuske (ispitni stol)
│   │   ├── pendulum.xml         osovina učvršćena, momentni aktuatori (kao u radu)
│   │   ├── pendulum_drive.xml   osovina učvršćena, ESC + servo (oba aktuatori brzine), kao na robotu
│   │   ├── pendulum_bench.xml   njihalo na stolu, vrti se osovina
│   │   └── meshes/*.stl
│   ├── sphere/          cijeli robot: ljuska + njihalo (bez okoline)
│   │   ├── sphere.xml           momentni aktuatori (kao u radu)
│   │   ├── sphere_drive.xml     ESC + servo (oba aktuatori brzine), kao na robotu
│   │   └── meshes/*.stl
│   └── worlds/          okoline za sphere (ukljucuju se s <include>, bira se world:=<ime>)
│       ├── arena.xml            pod + zidovi arene + world osi
│       └── empty.xml            samo pod
├── config/sim.yaml         parametri simulacije (aktuatori, telemetrija, kamera)
├── config/sim_bridge.yaml  parametri sim_bridgea (pojačanja, timeouti)
├── start.sh, session.yml   tmuxinator sesija
├── launch/sim.launch.py
├── tools/               CAD -> MJCF skripte, drive_standalone.py (bez ROS-a)
└── test/test_sim_core.py
```

Novi model ide u `models/<ime>/` (XML + `meshes/`). `setup.py` automatski instalira
cijeli folder `models/`. Čvor ga bira parametrom `model:=<ime>` i učitava
`models/<ime>/<ime>_drive.xml`.

## Instalacija (u containeru)

```bash
pip install mujoco numpy            # ako već nije
cd ~/ros2_ws/src && unzip sphero_simulation.zip   # ili kopiraj folder
cd ~/ros2_ws && colcon build --packages-select sphero_simulation --symlink-install
source install/setup.bash
```

## Pokretanje

```bash
ros2 launch sphero_simulation sim.launch.py robot_id:=5 model:=sphere               # robot u areni + sim_bridge
ros2 launch sphero_simulation sim.launch.py robot_id:=5 model:=sphere world:=empty  # bez zidova
ros2 launch sphero_simulation sim.launch.py robot_id:=5 model:=sphere bridge:=false # samo simulacija
ros2 launch sphero_simulation sim.launch.py robot_id:=5 variant:=bench              # njihalo na stolu
```

### Kao s pravim robotom (preko sim_bridgea)

Iste teme kao `robot_bridge.py` (Float32, s timeoutom, pa se šalju stalno s `-r 10`):
```bash
ros2 topic pub -r 10 /robot5/speed_target std_msgs/msg/Float32 "{data: 0.25}"   # petlja brzine [m/s]
ros2 topic pub -r 10 /robot5/roll_target  std_msgs/msg/Float32 "{data: 15.0}"   # ručni nagib [deg]
ros2 topic pub -r 10 /robot5/cmd_vx       std_msgs/msg/Float32 "{data: 0.09}"   # gas bez petlje
ros2 topic pub -1 /robot5/yaw_target std_msgs/msg/Float32 "{data: 90.0}"      # smjer [deg], treba yaw_loop:=true
ros2 topic echo /robot5/speed_fwd
```
Debug teme su iste kao na robotu: `speed_meas`, `speed_fwd`, `speed_ref`, `vx_out`, `vy_out`, `roll_rel`,
`roll_resp`, `roll_tgt`, `yaw`, `yaw_ref`, `yaw_err_pub`, `rod_speed`.

### Odbijanje od zidova (wall_bounce_corners + bouncing_driver)

Tvoji čvorovi rade nepromijenjeni nad simulacijom (koriste `/b0/pose`, `/robot5/yaw`, `yaw_target`, `cmd_vx`):
```bash
ros2 launch sphero_simulation sim.launch.py robot_id:=5 model:=sphere yaw_loop:=true yaw_offset:=72 x0:=-1.08 y0:=-0.60 yaw0:=20
ros2 topic pub -r 10 /people_stop std_msgs/msg/Bool "{data: false}"
python3 .../wall_bounce_corners.py
python3 .../bouncing_driver.py
```
`robot_id` i `yaw_offset` moraju odgovarati prvom aktivnom robotu u `robot_config.json` (kugla b0).
Pravi `robot_bridge` ne smije raditi istovremeno (iste teme).

### Izravno simulaciji (bez bridgea)
```bash
ros2 topic pub -1 /robot5/cmd/vx std_msgs/msg/Float32 "{data: 0.09}"   # gas
ros2 topic pub -1 /robot5/cmd/vy std_msgs/msg/Float32 "{data: -0.1}"   # servo za BRZINU: roll_rel raste dok je vy < 0
ros2 topic pub -1 /robot5/cmd/vy std_msgs/msg/Float32 "{data: 0.0}"    # stani
ros2 service call /robot5/reset std_srvs/srv/Trigger
```

## Kako je povezano

```
 sim_bridge (= robot_bridge)                         sim_node (= robot + kamera)
 speed_target ─► SpeedPID ─┐                         ┌─► ESC:   omega = esc_gain * mz(vx) (kašnjenje esc_tau)
 roll_target  ─► RollPID ──┼─ /robot5/sim/cmd [vx,vy]┤
 (yaw_target ─► YawPID)    │                         └─► servo: gdot  = servo_gain * mz(vy)
       ▲  ▲                                                     │ MuJoCo
       │  └── /robot5/sim/telemetry [main_roll, pend_roll, main_pitch, pend_pitch, main_yaw, rod_speed]
       └───── /b0/pose (kamera, 33 Hz) ──► SpeedEstimator (Kalman, speed_fwd)
```
`sim/cmd` i `sim/telemetry` zamjenjuju serijski port (`send_control` i `TelemetryParser`).
Pravi `robot_bridge.py` se ne mijenja; može uvesti iste klase iz `sphero_simulation.control`.

## Teme simulacije (relativne, pod namespaceom robot<ID>)

| Tema | Tip | Smjer | Opis |
|---|---|---|---|
| `sim/cmd` | Float32MultiArray | ulaz | `[vx, vy]` kao `send_control` (RC 1500 ± 500 µs) |
| `cmd/vx`, `cmd/vy` | Float32 | ulaz | ručno, isto značenje |
| `cmd/omega` | Float64 | ulaz | izravno brzina glavčine [rad/s] (zaobilazi model ESC-a) |
| `sim/telemetry` | Float32MultiArray | izlaz | `[main_roll, pend_roll, main_pitch, pend_pitch, main_yaw, rod_speed]`, 50 Hz |
| `/b<ball>/pose` | PoseStamped | izlaz | poza kugle kao s kamere, 33 Hz, šum `camera_noise` |
| `joint_states` | JointState | izlaz | `drive`, `servo`: kut, brzina, moment |
| `imu/main`, `imu/pend` | Imu | izlaz | orijentacija, kutna brzina, akceleracija (s gravitacijom) |
| `roll_rel` | Float64 | izlaz | main_roll − pend_roll [°] |
| `pose`, `heading_deg`, `speed` | | izlaz | samo `sphere`: točna poza, smjer ψ, brzina (bez šuma) |

U prozoru: gornja polovica ljuske je u grupi 0 (tipka **0** je skriva/prikazuje),
prozirna vanjska sfera r = 185 mm (kontakt) u grupi 2 (tipka **2**). Kamera prati kuglu.
Svijet je odvojen od robota: `models/worlds/<ime>.xml` (pod, zidovi, prepreke, svjetla). Novi svijet =
nova datoteka u tom folderu (kopija `empty.xml` + svoje geomove), pokreće se s `world:=<ime>`.
Arena: zidovi oko X ∈ [−2.4, 2.4] m, Y ∈ [−3.04, 3.04] m (visina 0.25 m - pretpostavka; `ARENA_X/Y`, `WALL_H`
u `tools/build.py`). World koordinatni sustav u ishodištu: x crvena, y zelena, z plava (0.5 m).

## Pretpostavke u modelu (provjeriti)

- Baterije 4 × 652 g (CNHL 8000 mAh 4S), servo 2 × 58 g (TD-8125MG); motor 5065 340 g je procjena.
- Ostali dijelovi su skalirani na izvaganu masu njihala s osovinom (4594.9 g).
- **Kalibracija težišta:** CAD daje L = 54.8 mm (njihalo s osovinom), izmjereno je 64 mm, a izmjereni period
  ljuljanja 2.5 s u simulaciji odgovara L ≈ 65 mm. Zato je težište nagibnog dijela spušteno za 12.4 mm
  (`L_TARGET = 64.0` u `tools/massprops.py`; `None` = čisti CAD). Masa i tromost oko težišta ostaju iz CAD-a.
  S kalibracijom: period ljuljanja 2.54 s; krug γ = 20° pri 0.2 / 0.3 m/s: ΔR/R = 31 / 62 %
  (eksperiment, fit 591 v²: 23 / 53 %). Bez kalibracije: 2.77 s i 36 / 77 %.
- Ljuska: printani dio iz CAD-a (do r = 174.2 mm, PLA 1.24 g/cm³ - pretpostavka) + keramički sloj
  174.2–185 mm s ostatkom izvagane mase (6914.9 g ukupno) -> keramika 4.9 kg, 1.12 g/cm³.
  J₁ = 0.142 kg m² (tanka ljuska bi dala 0.160).
- Kontakt: condim 6 (torzijsko trenje 0.005 m), kotrljajuće trenje 0, bez `noslip`
  (`noslip` s condim 6 blokira i kotrljanje).
- **Servo (TD-8125MG continuous) je servo za brzinu:** brzina nagiba = 58 °/s po jedinici vy, mrtva zona 0.01
  (5 µs). Iz baga `MSU_test_cet_006` (R² = 0.70; pozicijski model R² = 0.16). Granica momenta 2.45 N m.
- **ESC:** ω = 28 rad/s · mz(vx, 0.04), kašnjenje prvog reda 1 s - PROCJENA (gas 0.09 → ~0.26 m/s;
  mrtva zona prema `U0_SPEED = 0.05`). Granica momenta pogona 10 N m (procjena). Za točnije: test T0.
- **Konvencije kao na robotu:** yaw 0° = +x, raste suprotno od kazaljke (+90° = +y, ±180° = −x, −90° = −y).
  `vy < 0` → `roll_rel` raste, pa je `vy = −RollPID(...)` (kao u robot_bridge) negativna povratna veza;
  `roll_rel > 0` → kugla skreće ulijevo (yaw raste). Zato YawPID radi s `yaw_invert: false`.
  U `config/sim.yaml`: `servo_sign: -1`, `roll_sign: 1`, `yaw_sign: 1`.
  S tim: skok smjera 90° unutar 5° za ~6 s; veći skokovi pretjeraju ~10–15° i njišu se oko cilja
  (spora unutarnja petlja nagiba). Odstupanje IMU-a i kamere (~70°) se može dodati s `yaw_offset_deg`.
- **Petlja brzine:** s pojačanjima s robota (kp 0.15, ki 0.1) u simulaciji raspiruje mod ljuljanja (2.5 s,
  ζ ≈ 0.04) i postaje nestabilna. U `config/sim_bridge.yaml` su stabilna: kp 0.01, ki 0.03, k_ff 0.15
  (do 90 % za ~2 s, točno ±0.003 m/s).
- Osi u modelu su kao u CAD-u: x = osovina, z = gore.