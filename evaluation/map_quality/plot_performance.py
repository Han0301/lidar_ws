"""Static performance and observed-space figures, with no accuracy labels."""
import argparse
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args()
j=json.loads((a.root/'performance.json').read_text());core=j['mapping_core'];out=a.root/'figures';out.mkdir(exist_ok=True)
names=['full','cache','prune_cache','native_cache','ray_budget_2000','half_integration'];labels=['Full','Cache','Prune + cache','Native + cache','Ray budget 2000','Half integration']
colors=['#73839a','#86a9b9','#aa8d83','#319a91','#bba254','#ae82a9']
fig,axes=plt.subplots(1,3,figsize=(14,4.6))
for ax,values,title in zip(axes,[[core[n]['resources']['cpu_s'] for n in names],[core[n]['resources']['peak_rss_mib'] for n in names],[core[n]['final_geometry']['free_cells'] for n in names]],['Process CPU (s), same 445 observations','Peak RSS (MiB), kernel high-water mark','Final ground-supported free cells']):
 ax.barh(labels,values,color=colors);ax.invert_yaxis();ax.set_title(title,fontsize=10);ax.grid(axis='x',alpha=.2)
 for i,v in enumerate(values):ax.text(v,i,' '+('%0.1f'%v if v<1000 else str(int(v))),va='center',fontsize=8)
 ax.set_xlim(0,max(values)*1.18)
fig.suptitle('Core replay: exact grids for implementation changes; coverage loss for budgets',fontsize=12)
fig.tight_layout();fig.savefig(out/'core_tradeoffs.png',dpi=160);plt.close(fig)
fig,axes=plt.subplots(1,3,figsize=(12,5))
for ax,n,title in zip(axes,['full','ray_budget_2000','half_integration'],['Full endpoints / full integration','Ray budget 2000','Half integration']):
 rows=list(csv.DictReader((a.root/'mapping_core'/n/'metrics.csv').open()));r=rows[-1]
 g=np.fromfile(a.root/'mapping_core'/n/(r['stamp_ns']+'.grid'),np.int8).reshape(int(r['height']),int(r['width']))
 colors=np.full((*g.shape,3),.6);colors[g==0]=1;colors[g==100]=[.8,.15,.15]
 x,y=float(r['origin_x']),float(r['origin_y']);ax.imshow(colors,origin='lower',extent=[x,x+.1*g.shape[1],y,y+.1*g.shape[0]])
 geo=core[n]['final_geometry'];ax.set_title(title+'\nfree %d / largest component %d'%(geo['free_cells'],geo['largest_free_component_cells']),fontsize=9)
 ax.set_xlabel('odom x (m)');ax.set_ylabel('odom y (m)')
fig.suptitle('White: ground-supported free / red: occupied / gray: unknown',fontsize=11)
fig.tight_layout();fig.savefig(out/'map_budget_comparison.png',dpi=160);plt.close(fig)
