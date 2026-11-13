import pickle, numpy as np, json, sys
L=pickle.load(open("leaves.pkl","rb"))
M_TOTAL=4594.9      # g, izvagano: njihalo + osovina + baterije
L_TARGET=64.0       # mm, KALIBRACIJA: izmjereni L (njihalo s osovinom); potvrden periodom ljuljanja 2.5 s.
                    # CAD daje ~55 mm. Tezište nagibnog dijela spusta se tako da m*L = M_TOTAL*L_TARGET
                    # (masa i tromost oko tezista ostaju). None = bez kalibracije (cisti CAD).
M_BATT=652.0        # g, CNHL 8000 mAh 4S (deklarirano, +-5 g)
M_MOTOR=340.0       # g, 5065 270KV - PRETPOSTAVKA
M_SERVO=58.0        # g po servu, TD-8125MG (specifikacija)
M_SHELL=3562.7+3352.2   # g, izvagane polovice ljuske (s nosacima osovine)
R_OUT=185.0         # mm, izmjereni vanjski polumjer (CAD ljuska je samo do 174.2 mm)
R_CAD=174.2         # mm, vanjski polumjer printane ljuske u CAD-u; od R_CAD do R_OUT je keramicki sloj
RHO_SHELL_PRINT=1.24e-3   # g/mm3, printana ljuska (PLA, geometrija iz CAD-a) - PRETPOSTAVKA
RHO_STEEL=7.85e-3; RHO_AL=2.70e-3; RHO_PCB=1.9e-3   # g/mm3
SERVO_PARTS=("ZK_122","XG-ZIJI","SG-ZIJI","Component3","MOTOR-1723","PCB-CHAZUO","GE_27","金属")
STEEL=("Screw","Nut","Closed-Bearing","Ball Bearing","Rod RVS","DIN ","Hexagon","Washer","91290A","90593A","4481N")
ELEC=("Board","PCB","dc-005","X2564","User_Library","C_0603","R_0603","R0603","SOT","SOD","TO-223","Type_C","LED","Q1","U1","U2","U3","U4","D1","D2","D3","D4","R1","R2","R3","R4","R5","R6","R7","R8","R9","C1","C2","C3","C4","C5","C6","C7","C8","C9","C10","H1","H2","H3","H4","H5","M1","DC1","Extruded","SOLID","1653","8407","7361","2_1_200","3_1_2","5_1_2")
def is_shaft(l):
    e=l["V"].max(0)-l["V"].min(0); return l["top"].startswith("=>") and e[0]>250 and e[1]<15 and e[2]<15
def body(l):
    t,n,(x,y,z)=l["top"],l["name"],l["com"]
    if t=="SHELL" or t.startswith("Output Gear") or is_shaft(l): return "shell"
    if t.startswith(("CyclodialDrive","Driver Gear")): return "hub"
    if t.startswith(("Battery","Servo Gear","Bus Servo")): return "tilt"
    if abs(y)<12 and abs(z)<12 and abs(x)>25: return "hub"          # lezajevi osovine u glavcini
    if abs(y)<40 and abs(x)<30: return "hub"
    return "tilt"
def kind(l):
    t,n=l["top"],l["name"]
    if t.startswith("Battery"): return "battery"
    if n.startswith("Motor 5065"): return "motor"
    if t.startswith("Servo Gear") and n.startswith(SERVO_PARTS): return "servo"
    if t=="SHELL": return "shell"
    if is_shaft(l): return "al"
    if any(k in n for k in STEEL): return "steel"
    if t.startswith("Bus Servo") or n.startswith(ELEC): return "elec"
    return "print"
for l in L: l["body"]=body(l); l["kind"]=kind(l)
vol=lambda k:sum(l["vol"] for l in L if l["kind"]==k)
# keramicki vanjski sloj (CAD ga nema): R_CAD..R_OUT, ostatak izvagane mase ljuske
V_COAT=4/3*np.pi*(R_OUT**3-R_CAD**3)
rho_shell=RHO_SHELL_PRINT
m_coat=M_SHELL-rho_shell*vol("shell")
I_coat=2/5*m_coat*(R_OUT**5-R_CAD**5)/(R_OUT**3-R_CAD**3)      # g mm2, oko sredista
print(f"ljuska: printano {vol('shell')/1e3:.0f} cm3 x {rho_shell*1e3:.2f} g/cm3 = {rho_shell*vol('shell'):.0f} g; keramika {V_COAT/1e3:.0f} cm3, {m_coat:.0f} g -> {m_coat/V_COAT*1e3:.2f} g/cm3")
nserv=2
L_p=[l for l in L if l["kind"]!="shell"]
fixed={"battery":M_BATT*4,"motor":M_MOTOR,"servo":M_SERVO*nserv,
       "steel":vol("steel")*RHO_STEEL,"al":vol("al")*RHO_AL,"elec":vol("elec")*RHO_PCB}
rho_p=(M_TOTAL-sum(fixed.values()))/vol("print")   # njihalo + osovina = M_TOTAL
dens={"battery":M_BATT*4/vol("battery"),"motor":M_MOTOR/vol("motor"),"servo":M_SERVO*nserv/vol("servo"),
      "steel":RHO_STEEL,"al":RHO_AL,"elec":RHO_PCB,"print":rho_p,"shell":rho_shell}
for l in L: l["mass"]=l["vol"]*dens[l["kind"]]
print("mase po vrsti [g]:",{k:round(sum(l['mass'] for l in L if l['kind']==k)) for k in dens})
print(f"gustoca printanih/neimenovanih dijelova: {rho_p*1e3:.2f} g/cm3  (V={vol('print')/1e3:.0f} cm3)")
out={}
for b in ["shell","hub","tilt"]:
    B=[l for l in L if l["body"]==b]; m=sum(l["mass"] for l in B)
    c=sum(l["mass"]*l["com"] for l in B)/m
    I=np.zeros((3,3))
    for l in B:
        d=l["com"]-c; I+=l["Ic"]*dens[l["kind"]]+l["mass"]*(np.dot(d,d)*np.eye(3)-np.outer(d,d))
    if b=="shell":   # vanjski sloj: centriran u ishodistu
        mt=m+m_coat; ct=c*m/mt
        d=c-ct; I=I+m*(np.dot(d,d)*np.eye(3)-np.outer(d,d)) + m_coat*np.dot(ct,ct)*np.eye(3)-m_coat*np.outer(ct,ct) + I_coat*np.eye(3)
        m,c=mt,ct
    out[b]=dict(mass=m/1e3,com=(c/1e3).tolist(),I=(I*1e-9).tolist())   # kg, m, kg m2
    print(f"{b:5s} m={m:7.1f} g  tezište={np.round(c,1)} mm  Idiag={np.round(np.diag(I)*1e-9,5)} kg m2")
if L_TARGET is not None:   # kalibracija tezista
    Lw0=[l for l in L if l["kind"]!="shell"]; mL_cad=-sum(l["mass"]*l["com"][2] for l in Lw0)/1e6   # kg m
    dz=(M_TOTAL*L_TARGET/1e6 - mL_cad)/out["tilt"]["mass"]                                        # m
    out["tilt"]["com"][2]-=dz
    out["calibration"]=dict(L_target_mm=L_TARGET, L_cad_mm=mL_cad/M_TOTAL*1e6, tilt_com_shift_mm=-dz*1e3)
    print(f"KALIBRACIJA: L iz CAD-a {mL_cad/M_TOTAL*1e6:.1f} mm -> {L_TARGET} mm; tezište nagibnog dijela spusteno za {dz*1e3:.1f} mm")
pend=[l for l in L if l["body"] in ("hub","tilt")]; mp=sum(l["mass"] for l in pend); cp=sum(l["mass"]*l["com"] for l in pend)/mp
Lw=[l for l in L if l["kind"]!="shell"]   # kao vaganje: njihalo + osovina (+ izlazni zupcanik)
allm=sum(l["mass"] for l in Lw); allc=sum(l["mass"]*l["com"] for l in Lw)/allm
print(f"njihalo (hub+tilt): {mp:.0f} g, tezište z={cp[2]:.1f} mm, x={cp[0]:.1f}, y={cp[1]:.1f}")
print(f"CAD, s osovinom (kao vaganje): m*L = {allm*-allc[2]/1e6:.4f} kg m  -> L={-allc[2]:.1f} mm; izmjereno 4.5949*0.064 = {4.5949*0.064:.4f} kg m")
Io=sum(l["Ic"][0,0]*dens[l["kind"]]+l["mass"]*(l["com"][1]**2+l["com"][2]**2) for l in pend)*1e-9
print(f"tromost njihala oko osovine I_O={Io:.4f} kg m2, oko tezista {Io-mp/1e3*(cp[1]**2+cp[2]**2)*1e-6:.4f}; tockasta masa daje {mp/1e3*(cp[2]/1e3)**2:.4f}")
print(f"period slobodnog njihanja na nepomicnoj osovini: {2*np.pi*np.sqrt(Io/(mp/1e3*9.81*np.hypot(cp[1],cp[2])/1e3)):.3f} s")
J1=out['shell']['I'][0][0]; print(f"ljuska ukupno {out['shell']['mass']:.3f} kg, Ixx={J1:.4f} Iyy={out['shell']['I'][1][1]:.4f} Izz={out['shell']['I'][2][2]:.4f} kg m2; tanka 2/3 m r^2 = {2/3*out['shell']['mass']*0.185**2:.4f}")
json.dump(out,open("massprops.json","w"),indent=1)
pickle.dump(L,open("leaves_m.pkl","wb"))
