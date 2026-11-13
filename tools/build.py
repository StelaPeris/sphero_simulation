import os, sys, json, pickle, collections
import numpy as np, trimesh

PKG = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
L = pickle.load(open("leaves_m.pkl", "rb"))
MP = json.load(open("massprops.json"))
R_OUT = 0.185                       # izmjereni vanjski polumjer [m]
# arena (MUJOCO_HANDOFF.md): unutarnji rub zidova
ARENA_X, ARENA_Y = 2.4, 3.04        # pola sirine / duljine [m]: X u [-2.4, 2.4], Y u [-3.04, 3.04]
WALL_H, WALL_T = 0.25, 0.05         # visina i debljina zida [m] - PRETPOSTAVKA


def arena_xml(ind=4):
    """Zidovi (kolizija) + world koordinatni sustav u ishodistu (samo prikaz)."""
    sp = " " * ind; hx, hy, h, t = ARENA_X, ARENA_Y, WALL_H / 2, WALL_T / 2
    walls = [("wall_xp", f"{hx + t} 0 {h}", f"{t} {hy + 2 * t} {h}"), ("wall_xn", f"{-hx - t} 0 {h}", f"{t} {hy + 2 * t} {h}"),
             ("wall_yp", f"0 {hy + t} {h}", f"{hx} {t} {h}"), ("wall_yn", f"0 {-hy - t} {h}", f"{hx} {t} {h}")]
    out = [f"{sp}<!-- arena: zidovi oko X u [-{hx}, {hx}], Y u [-{hy}, {hy}] m (visina {WALL_H} m - pretpostavka) -->"]
    out += [f'{sp}<geom name="{n}" type="box" pos="{p}" size="{z}" rgba=".55 .6 .68 1" contype="1" conaffinity="1" condim="3" friction="0.6 0.005 0"/>'
            for n, p, z in walls]
    out += [f"{sp}<!-- world koordinatni sustav (samo prikaz): x crvena, y zelena, z plava, duljina 0.5 m -->",
            f'{sp}<geom name="world_x" type="capsule" fromto="0 0 0.003 0.5 0 0.003" size="0.008" rgba="1 0 0 1"/>',
            f'{sp}<geom name="world_y" type="capsule" fromto="0 0 0.003 0 0.5 0.003" size="0.008" rgba="0 .8 0 1"/>',
            f'{sp}<geom name="world_z" type="capsule" fromto="0 0 0.003 0 0 0.5" size="0.008" rgba="0 0 1 1"/>',
            f'{sp}<geom name="world_origin" type="sphere" pos="0 0 0.003" size="0.02" rgba=".1 .1 .1 1"/>']
    return "\n".join(out)

COL = {"battery": ".15 .15 .15 1", "motor": ".25 .25 .3 1", "servo": ".1 .1 .1 1", "steel": ".7 .7 .72 1",
       "al": ".75 .75 .78 1", "elec": ".1 .4 .15 1", "print": ".85 .15 .15 1",
       "shell_lower": ".9 .25 .25 1", "shell_upper": ".9 .25 .25 1", "shell_mount": ".85 .15 .15 1"}


def mesh_key(l):
    if l["kind"] == "shell":   # polovice ljuske odvojeno, da se gornja moze sakriti
        if l["vol"] > 100e3:
            return "shell_lower" if l["com"][2] < 0 else "shell_upper"
        return "shell_mount"
    return l["kind"]


def export_meshes(outdir, bodies):
    os.makedirs(os.path.join(outdir, "meshes"), exist_ok=True)
    groups = collections.defaultdict(list)
    for l in L:
        if l["body"] in bodies and len(l["F"]):
            groups[(l["body"], mesh_key(l))].append(trimesh.Trimesh(l["V"] / 1000.0, l["F"], process=False))
    assets, geoms = [], collections.defaultdict(list)
    for (b, k), ms in sorted(groups.items()):
        trimesh.util.concatenate(ms).export(os.path.join(outdir, "meshes", f"{b}_{k}.stl"))
        assets.append(f'    <mesh name="{b}_{k}" file="{b}_{k}.stl" inertia="shell"/>')
        grp = ' group="0"' if k == "shell_upper" else ""          # gornja polovica: vidljiva (grupa 0), tipka 0 u vieweru je skriva
        geoms[b].append(f'<geom type="mesh" mesh="{b}_{k}" rgba="{COL[k]}"{grp}/>')
    return assets, geoms


def inertial(b):
    d = MP[b]; I = np.array(d["I"]); c = d["com"]
    return (f'<inertial pos="{c[0]:.5f} {c[1]:.5f} {c[2]:.5f}" mass="{d["mass"]:.4f}" '
            f'fullinertia="{I[0,0]:.6g} {I[1,1]:.6g} {I[2,2]:.6g} {I[0,1]:.6g} {I[0,2]:.6g} {I[1,2]:.6g}"/>')


VISUAL = '''  <visual>
    <global offwidth="1600" offheight="1000"/>
    <headlight ambient=".35 .35 .35" diffuse=".55 .55 .55" specular=".1 .1 .1"/>
    <rgba haze=".85 .88 .92 1"/>
  </visual>'''
TEXTURES = '''    <texture type="skybox" builtin="gradient" rgb1=".95 .96 .98" rgb2=".70 .75 .82" width="512" height="3072"/>
    <texture name="grid" type="2d" builtin="checker" rgb1=".92 .92 .92" rgb2=".82 .82 .82" width="300" height="300" mark="edge" markrgb=".65 .65 .65"/>
    <material name="grid" texture="grid" texrepeat="10 10" texuniform="true" reflectance=".05"/>'''


def actuators(kind):
    if kind == "torque":
        return '''  <actuator>
    <!-- momenti kao u radu: gear=-1 -> +u djeluje na ljusku/osovinu, -u na njihalo (B = [1, -1]) -->
    <motor name="u_x" joint="drive" gear="-1" ctrlrange="-2.5 2.5"/>
    <motor name="u_y" joint="servo" gear="-1" ctrlrange="-2.4 2.4"/>
  </actuator>'''
    return '''  <actuator>
    <!-- kao na pravom robotu, oba su aktuatori BRZINE (ctrl = zadana brzina zgloba [rad/s]):
         ESC: brzina glavcine prema osovini; servo (TD-8125MG continuous): brzina nagiba.
         Pretvorbu naredbe vx, vy [-1, 1] (RC impuls 1500 +/- 500 us) u brzinu radi sim_core.py
         (pojacanja i mrtve zone). Granice momenta: pogon iza reduktora 10 N m - PRETPOSTAVKA;
         servo 25 kg cm = 2.45 N m. kv servoa je velik: reduktor servoa se tesko vrti unatrag. -->
    <velocity name="esc" joint="drive" kv="20" ctrlrange="-6 6" forcerange="-10 10"/>
    <velocity name="servo" joint="servo" kv="300" ctrlrange="-1.5 1.5" forcerange="-2.45 2.45"/>
  </actuator>'''


def pendulum_chain(geoms, ind):
    sp = " " * ind
    G = lambda b, k: ("\n" + " " * k).join(geoms[b])
    return f'''{sp}<!-- glavcina: motor 5065 + cikloidni reduktor + stozasti zupcanik, rotira oko osovine (x) -->
{sp}<body name="hub">
{sp}  <joint name="drive" type="hinge" axis="1 0 0"/>
{sp}  {inertial("hub")}
{sp}  {G("hub", ind + 2)}
{sp}  <site name="main_imu" pos="0 0 0.035"/>
{sp}  <!-- nagibni dio: bocne ploce, 4 baterije, 2 servo motora; okrece se oko osi y (lezaj 40x52) -->
{sp}  <body name="tilt">
{sp}    <joint name="servo" type="hinge" axis="0 1 0" range="-0.6 0.6"/>
{sp}    {inertial("tilt")}
{sp}    {G("tilt", ind + 4)}
{sp}    <site name="pend_imu" pos="0 0.05 0.02"/>
{sp}  </body>
{sp}</body>'''


HEADER = '''<mujoco model="{name}">
  <!-- GENERIRANO: tools/build.py iz {step}. Ne uredjivati rucno.
       Mase: baterije 4 x 652 g, motor 340 g i servo 2 x 60 g (PRETPOSTAVKA), celik 7.85 g/cm3,
       ostali dijelovi skalirani na izvaganu masu njihala s osovinom (4594.9 g).
       {calib}
       Ljuska: izvagane polovice 6914.9 g = printana ljuska iz CAD-a (PLA 1.24 g/cm3, do r = 174.2 mm)
       + keramicki sloj 174.2-185 mm (ostatak mase, jednoliko). -->
  <compiler angle="radian" meshdir="meshes" autolimits="true" inertiafromgeom="false"/>
  <option timestep="0.001" integrator="implicitfast" gravity="0 0 -9.81"{opt}/>
{visual}
  <default><geom contype="0" conaffinity="0" group="1"/></default>
  <asset>
{tex}
{assets}
  </asset>'''


CAL = MP.get("calibration")
CALIB = (f"KALIBRACIJA: tezište nagibnog dijela spusteno za {-CAL['tilt_com_shift_mm']:.1f} mm, L = {CAL['L_target_mm']:.0f} mm "
         f"(CAD {CAL['L_cad_mm']:.1f} mm); vidi L_TARGET u massprops.py.") if CAL else "Bez kalibracije tezista (cisti CAD)."


def pendulum_xml(act, assets, geoms):
    G = lambda b, k: ("\n" + " " * k).join(geoms[b])
    return HEADER.format(name="pendulum_drive" if act == "drive" else "pendulum", step=STEP, opt="",
                         visual=VISUAL, tex=TEXTURES, assets="\n".join(assets), calib=CALIB) + f'''
  <worldbody>
    <light pos="0.3 -0.6 0.8" dir="-0.3 0.6 -0.8" diffuse=".8 .8 .8"/>
    <!-- pod na visini gdje bi bio pravi pod (r = 0.185 m ispod osovine); samo vizualno -->
    <geom name="floor_vis" type="plane" pos="0 0 -{R_OUT}" size="1 1 0.01" material="grid"/>
    <!-- osovina + izlazni zupcanik, ucvrsceni (ispitni stol) -->
    <body name="shaft">
      {inertial("shaft_only")}
      {G("shell", 6)}
{pendulum_chain(geoms, 6)}
    </body>
  </worldbody>
{actuators(act)}
  <sensor>
    <jointpos name="gamma" joint="servo"/>
    <jointvel name="omega_x" joint="drive"/>
    <framequat name="main_q" objtype="site" objname="main_imu"/>
    <framequat name="pend_q" objtype="site" objname="pend_imu"/>
  </sensor>
</mujoco>
'''


def sphere_xml(act, assets, geoms):
    G = lambda b, k: ("\n" + " " * k).join(geoms[b])
    opt = ' cone="elliptic" impratio="10"'   # bez noslip: s condim 6 blokira i kotrljanje
    return HEADER.format(name="sphere_drive" if act == "drive" else "sphere", step=STEP, opt=opt,
                         visual=VISUAL, tex="", assets="\n".join(assets), calib=CALIB) + f'''
  <!-- SVIJET (pod, zidovi, svjetla) je u zasebnoj datoteci; sim_node ga mijenja parametrom world:=<ime> -->
  <include file="../worlds/arena.xml"/>
  <worldbody>
    <!-- ljuska (s osovinom i nosacima) -->
    <!-- pocetna orijentacija: zakret -90 deg oko z -> osovina duz -y, pozitivan gas vozi u +x (psi = 0) -->
    <body name="shell" pos="0 0 {R_OUT}" quat="0.7071068 0 0 -0.7071068">
      <freejoint name="root"/>
      {inertial("shell")}
      <geom name="shell_contact" type="sphere" size="{R_OUT}" contype="1" conaffinity="1" condim="6"
            friction="0.8 0.005 0" rgba="1 1 1 0.12" group="2"/>
      {G("shell", 6)}
      <site name="shell_imu" pos="0 0 0"/>
{pendulum_chain(geoms, 6)}
    </body>
  </worldbody>
{actuators(act)}
  <sensor>
    <jointpos name="gamma" joint="servo"/>
    <jointvel name="omega_x" joint="drive"/>
    <framequat name="shell_q" objtype="site" objname="shell_imu"/>
    <framequat name="main_q" objtype="site" objname="main_imu"/>
    <framequat name="pend_q" objtype="site" objname="pend_imu"/>
    <framepos name="shell_pos" objtype="body" objname="shell"/>
    <framelinvel name="shell_v" objtype="body" objname="shell"/>
  </sensor>
</mujoco>
'''


STEP = os.environ.get("STEP_NAME", "IsThisLife_Robot_V3.5+Shell.step")


def world_xml(name, with_arena):
    return f'''<mujoco model="world_{name}">
  <!-- SVIJET za modele kugle (models/sphere/*.xml ga ukljucuju s <include>). GENERIRANO: tools/build.py.
       Smije sadrzavati samo okolinu: pod, zidove, prepreke, svjetla, teksture. Ishodiste = world koordinatni sustav. -->
  <asset>
{TEXTURES}
  </asset>
  <worldbody>
    <light pos="0 0 4" dir="0 0 -1" diffuse=".7 .7 .7" castshadow="false"/>
    <light pos="1 -1 2" dir="-1 1 -2" diffuse=".4 .4 .4"/>
    <!-- pod (beton). size = POLA sirine/duljine prikaza [m]; za kontakt je ravnina ionako beskonacna.
         friction = klizno, torzijsko [m], kotrljanje [m] -->
    <geom name="floor" type="plane" size="25 25 0.1" material="grid" contype="1" conaffinity="1"
          condim="6" friction="0.8 0.005 0" solref="0.004 1"/>
{arena_xml(4) if with_arena else ""}
  </worldbody>
</mujoco>
'''


os.makedirs(os.path.join(PKG, "models", "worlds"), exist_ok=True)
for wname, wa in [("arena", True), ("empty", False)]:
    open(os.path.join(PKG, "models", "worlds", f"{wname}.xml"), "w").write(world_xml(wname, wa))
    print("zapisano", os.path.join(PKG, "models", "worlds", f"{wname}.xml"))
# tijelo samo osovine (+ izlazni zupcanik) za model njihala bez ljuske
sh = [l for l in L if l["body"] == "shell" and l["kind"] != "shell"]
m = sum(l["mass"] for l in sh); c = sum(l["mass"] * l["com"] for l in sh) / m
I = np.zeros((3, 3))
for l in sh:
    d = l["com"] - c; I += l["Ic"] * (l["mass"] / l["vol"]) + l["mass"] * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
MP["shaft_only"] = dict(mass=m / 1e3, com=(c / 1e3).tolist(), I=(I * 1e-9).tolist())

for model, bodies, fn in [("pendulum", ("hub", "tilt", "shell"), pendulum_xml), ("sphere", ("hub", "tilt", "shell"), sphere_xml)]:
    out = os.path.join(PKG, "models", model)
    if model == "pendulum":   # u modelu njihala nema ljuske: samo osovina i zupcanik
        Lsave = L[:]; L[:] = [l for l in L if l["kind"] != "shell"]
    assets, geoms = export_meshes(out, bodies)
    if model == "pendulum":
        L[:] = Lsave
    for act, name in [("torque", f"{model}.xml"), ("drive", f"{model}_drive.xml")]:
        open(os.path.join(out, name), "w").write(fn(act, assets, geoms))
        print("zapisano", os.path.join(out, name))