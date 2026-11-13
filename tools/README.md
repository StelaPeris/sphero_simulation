# CAD -> MuJoCo

Regeneracija modela iz STEP-a cijelog robota (ljuska + njihalo, Fusion export).
Treba: `pip install cadquery-ocp trimesh`.

    cd tools
    STEP=/put/do/IsThisLife_Robot_V3.5+Shell.step python leaves.py   # ~1-2 min: cita STEP, tesselira -> leaves.pkl
    python massprops.py      # mase, tezista, tromosti -> leaves_m.pkl, massprops.json (ispis provjera)
    python build.py          # STL-ovi + ../models/pendulum/*.xml i ../models/sphere/*.xml
    python make_bench.py     # ../models/pendulum/pendulum_bench.xml (njihalo na stolu, vrti se osovina)

Ocekivana struktura STEP-a: korijen sadrzi sklop `Robot` (njihalo s osovinom) i ljusku
(neimenovana tijela). Dijelovi se svrstavaju u tijela `shell`, `hub`, `tilt` u `massprops.py`
(funkcija `body`) - provjeriti ako se CAD bitno promijeni.

Pretpostavke (vrh `massprops.py`): baterije 4 x 652 g, motor 340 g, servo 2 x 60 g,
njihalo + osovina 4594.9 g (izvagano), ljuska 6914.9 g (izvagano), vanjski polumjer 185 mm
(CAD ide do 174.2 mm; razlika = jednolik vanjski sloj iste gustoce).

`drive_standalone.py` - voznja njihala s tipkovnice bez ROS-a.
