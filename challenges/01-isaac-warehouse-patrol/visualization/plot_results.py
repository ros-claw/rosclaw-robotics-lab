"""Export independent visit measurements; no simulated/fabricated plot inputs."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

p=argparse.ArgumentParser()
p.add_argument('--reports',type=Path,nargs='+',required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
runs=[json.loads(path.read_text()) for path in a.reports]
if any(run['status']!='PASS' for run in runs):
    raise ValueError('This success comparison requires independently PASS reports')
fig, axes=plt.subplots(1,2,figsize=(12,4),layout='constrained')
sites=['entry','shelf','aisle','home']
x=np.arange(4)
width=.75/len(runs)
for i, run in enumerate(runs):
    by_site={v['site_id']:v for v in run['visits']}
    xx=x+(i-(len(runs)-1)/2)*width
    axes[0].bar(xx,[by_site[s]['position_error_m'] for s in sites],width,label=run['run_id'])
    axes[1].bar(xx,[by_site[s]['stable_dwell_sim_seconds'] for s in sites],width,label=run['run_id'])
axes[0].axhline(.4,color='#d44',linestyle='--',label='0.4 m acceptance bound')
axes[1].axhline(2,color='#d44',linestyle='--',label='2 SIM seconds minimum')
for ax in axes:
    ax.set_xticks(x,sites)
    ax.grid(axis='y',alpha=.2)
    ax.set_axisbelow(True)
    ax.legend(fontsize=8)
axes[0].set_ylabel('Independent PhysX position error (m)')
axes[0].set_ylim(0,.45)
axes[1].set_ylabel('Post-Nav2 stable dwell (SIM seconds)')
fig.suptitle('Measured Native patrol visits · SIM evidence only · failed runs retained separately')
fig.savefig(a.output,dpi=160)
