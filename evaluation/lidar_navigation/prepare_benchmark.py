"""Convert captured body scans to binary input for the C++ benchmark."""
import argparse,numpy as np
from scipy.spatial.transform import Rotation
from pathlib import Path
parser=argparse.ArgumentParser();parser.add_argument('capture',type=Path);parser.add_argument('output',type=Path);parser.add_argument('--initial-acceleration',type=float,nargs=3,required=True);args=parser.parse_args();b=args.capture;args.output.mkdir(parents=True,exist_ok=True)
o=np.genfromtxt(b/'odometry.csv',delimiter=',',names=True)
a=np.array(args.initial_acceleration);a/=np.linalg.norm(a);v=np.cross(a,[0,0,1]);skew=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]]);level_world=np.eye(3)+skew+skew@skew/(1+a[2]);records=[]
for f in sorted(b.glob('scan*.npz')):
 d=np.load(f);i=np.argmin(abs(o['stamp_s']-float(d['stamp'])))
 if abs(o['stamp_s'][i]-float(d['stamp']))>.02:raise ValueError('Missing matching odometry at scan end')
 r=level_world@Rotation.from_quat([o[k][i] for k in ['qx','qy','qz','qw']]).as_matrix();yaw=np.arctan2(r[1,0],r[0,0]);rotation=Rotation.from_euler('z',-yaw).as_matrix()@r
 with (args.output/(f.stem+'.bin')).open('wb') as out:out.write(rotation.astype('<f4').tobytes());out.write(d['points'].astype('<f4').tobytes())
 p=level_world@np.array([o[k][i] for k in ['x','y','z']]);records.append([f.stem,float(d['stamp']),float(d['elapsed']),*p,yaw])
np.save(args.output/'poses.npy',np.array(records,dtype=object))
