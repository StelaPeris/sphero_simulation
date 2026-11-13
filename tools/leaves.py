import os
exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)),'tree.py')).read().split('roots=TDF_LabelSequence()')[0])
import numpy as np, pickle
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID, TopAbs_FACE
from OCP.BRep import BRep_Tool
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.BRepMesh import BRepMesh_IncrementalMesh
def tess(shape):
    BRepMesh_IncrementalMesh(shape,1.0,False,0.5,True)
    V=[];F=[];off=0
    e=TopExp_Explorer(shape,TopAbs_FACE)
    while e.More():
        f=TopoDS.Face(e.Current()); loc=TopLoc_Location()
        tri=BRep_Tool.Triangulation_s(f,loc)
        if tri is not None:
            tr=loc.Transformation()
            for i in range(1,tri.NbNodes()+1):
                p=tri.Node(i).Transformed(tr); V.append((p.X(),p.Y(),p.Z()))
            rev=f.Orientation()==1
            for i in range(1,tri.NbTriangles()+1):
                a,b,c=tri.Triangle(i).Get()
                F.append((off+a-1,off+c-1,off+b-1) if rev else (off+a-1,off+b-1,off+c-1))
            off+=tri.NbNodes()
        e.Next()
    return np.array(V),np.array(F,dtype=int)
def props(s):
    p=GProp_GProps(); BRepGProp.VolumeProperties_s(s,p); c=p.CentreOfMass(); M=p.MatrixOfInertia()
    return p.Mass(),np.array([c.X(),c.Y(),c.Z()]),np.array([[M.Value(i,j) for j in (1,2,3)] for i in (1,2,3)])
leaves=[]
def walk(l,path,top,ploc=None):
    from OCP.TopLoc import TopLoc_Location as _L
    ploc = ploc if ploc is not None else _L()
    ref=TDF_Label(); lab=l
    if st.IsReference_s(l): st.GetReferredShape_s(l,ref); lab=ref
    kids=TDF_LabelSequence(); st.GetComponents_s(lab,kids)
    s=st.GetShape_s(l).Moved(ploc); n=name(l)
    cloc=ploc.Multiplied(st.GetLocation_s(l))
    if kids.Length()==0:
        e=TopExp_Explorer(s,TopAbs_SOLID); j=0
        while e.More():
            so=e.Current(); v,c,I=props(so)
            if v>1e-3:
                V,F=tess(so); leaves.append(dict(name=n if not n.startswith("=>") else f"anon_{j:02d}",path=path,top=top,vol=v,com=c,Ic=I,V=V,F=F))
            j+=1; e.Next()
    else:
        for i in range(1,kids.Length()+1): walk(kids.Value(i),path+"/"+n,top if top else name(kids.Value(i)),cloc)
roots=TDF_LabelSequence(); st.GetFreeShapes(roots)
kids=TDF_LabelSequence(); st.GetComponents_s(roots.Value(1),kids)
for i in range(1,kids.Length()+1):
    k=kids.Value(i)
    if name(k).startswith("Robot"):
        ref=TDF_Label(); st.GetReferredShape_s(k,ref); rk=TDF_LabelSequence(); st.GetComponents_s(ref,rk)
        rloc=st.GetLocation_s(k)
        for j in range(1,rk.Length()+1): walk(rk.Value(j),"",name(rk.Value(j)),rloc)
    else:
        walk(k,"","SHELL")
pickle.dump(leaves,open("leaves.pkl","wb"))
print(len(leaves),"solida, V uk",sum(l["vol"] for l in leaves)/1e3)
