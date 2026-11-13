import os, copy, xml.etree.ElementTree as ET
D = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models", "pendulum")
tree = ET.parse(os.path.join(D, "pendulum_drive.xml")); root = tree.getroot()
root.set("model", "pendulum_bench")
wb = root.find("worldbody")
shaft = wb.find("body[@name='shaft']"); hub = shaft.find("body[@name='hub']"); tilt = hub.find("body[@name='tilt']")
wb.remove(shaft)
new_hub = ET.SubElement(wb, "body", name="hub")
for el in hub:
    if el.tag == "joint" or el is tilt: continue
    new_hub.append(copy.deepcopy(el))
new_shaft = ET.SubElement(new_hub, "body", name="shaft")
ET.SubElement(new_shaft, "joint", name="drive", type="hinge", axis="-1 0 0", armature="0.01")
for el in shaft:
    if el.tag != "body": new_shaft.append(copy.deepcopy(el))
# zute oznake na krajevima osovine, da se vidi vrtnja (samo vizualno, bez mase)
for x in (0.12, -0.12):
    ET.SubElement(new_shaft, "geom", type="box", pos=f"{x} 0 0.008", size="0.012 0.003 0.004", rgba="1 .8 0 1", mass="0")
new_hub.append(copy.deepcopy(tilt))
# ESC: na stolu nema gravitacijskog opterecenja, pa manji kv (vremenska konstanta I/kv = 0.01/0.5 = 20 ms);
# s kv=20 iz pendulum_drive.xml petlja bi bila brza od koraka 1 ms i oscilirala bi.
esc = root.find("actuator/velocity[@name='esc']"); esc.set("kv", "0.5")
ET.indent(tree, "  ")
out = os.path.join(D, "pendulum_bench.xml")
txt = ET.tostring(root, encoding="unicode")
txt = txt.replace('<mujoco model="pendulum_bench">', '<mujoco model="pendulum_bench">\n  <!-- GENERIRANO: tools/make_bench.py iz pendulum_drive.xml. Njihalo ucvrsceno za stol, okrece se osovina.\n       armature 0.01 kg m2 na zglobu drive: tromost rotora kroz reduktor - PRETPOSTAVKA (osovina sama ima ~1e-5).\n       ESC kv = 0.5 (vremenska konstanta 20 ms). -->', 1)
open(out, "w").write(txt)
print("zapisano", out)
