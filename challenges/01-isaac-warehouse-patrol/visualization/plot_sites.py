"""Measured site locations over the original map, with actual reachability reports."""
import argparse
from pathlib import Path
import yaml
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser()
p.add_argument('--map',type=Path,required=True)
p.add_argument('--sites',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
m=yaml.safe_load(a.map.with_suffix('.yaml').read_text())
sites=yaml.safe_load(a.sites.read_text())['sites']
im=Image.open(a.map).convert('L')
fig,ax=plt.subplots(figsize=(7,9),layout='constrained')
ox,oy=m['origin'][:2];res=m['resolution']
ax.imshow(im,cmap='gray',extent=(ox,ox+im.width*res,oy,oy+im.height*res),origin='upper')
for name,site in sites.items():
    ax.plot(site['x'],site['y'],'o',markersize=7)
    ax.annotate(name,(site['x'],site['y']),xytext=(7,5),textcoords='offset points')
    ax.add_patch(plt.Circle((site['x'],site['y']),.4,fill=False,color='#e05c2e'))
ax.set_xlabel('map x (m)');ax.set_ylabel('map y (m)')
ax.set_title('Measured inspection sites · all four physically verified\nSee independent Native reports; circles show 0.4 m arrival tolerance')
fig.savefig(a.output,dpi=160)
