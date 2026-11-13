import os
import sys, time, pickle
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_Label
from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
t0=time.time()
doc=TDocStd_Document(TCollection_ExtendedString("doc"))
r=STEPCAFControl_Reader(); r.SetNameMode(True)
r.ReadFile(os.environ.get("STEP","robot.step")); r.Transfer(doc)
st=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
print("read",time.time()-t0, file=sys.stderr)
def name(l):
    n=TDataStd_Name()
    return n.Get().ToExtString() if l.FindAttribute(TDataStd_Name.GetID_s(),n) else "?"
def vol(s):
    p=GProp_GProps(); BRepGProp.VolumeProperties_s(s,p); return p.Mass()
def bb(s):
    b=Bnd_Box(); BRepBndLib.Add_s(s,b); return (b.GetXMin(),b.GetYMin(),b.GetZMin(),b.GetXMax(),b.GetYMax(),b.GetZMax())
roots=TDF_LabelSequence(); st.GetFreeShapes(roots)
def walk(l,depth,maxd):
    shp=st.GetShape_s(l)
    ref=TDF_Label()
    lab=l
    if st.IsReference_s(l): st.GetReferredShape_s(l,ref); lab=ref
    kids=TDF_LabelSequence(); st.GetComponents_s(lab,kids)
    v=vol(shp); b=bb(shp)
    print("  "*depth+f"{name(l)} [{name(lab)}] V={v/1e3:.1f}cm3 bb=({b[0]:.0f},{b[1]:.0f},{b[2]:.0f})..({b[3]:.0f},{b[4]:.0f},{b[5]:.0f}) n={kids.Length()}")
    if depth<maxd:
        for i in range(1,kids.Length()+1): walk(kids.Value(i),depth+1,maxd)
for i in range(1,roots.Length()+1): walk(roots.Value(i),0,int(sys.argv[1]))
