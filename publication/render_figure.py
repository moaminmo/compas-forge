from pathlib import Path
import json, csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
HERE=Path(__file__).resolve().parent
C=json.loads((HERE/'figure-config.json').read_text(encoding='utf-8'))
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,'text.color':'#eaf2f8','axes.labelcolor':'#bfd0df','xtick.color':'#bfd0df','ytick.color':'#bfd0df','axes.edgecolor':'#385067','svg.fonttype':'none'})
fig=plt.figure(figsize=(12,6.75),facecolor='#091622')
fig.text(.055,.92,C['title'].upper(),fontsize=13,fontweight='bold',color=C['accent'])
fig.text(.055,.835,C['headline'],fontsize=25,fontweight='bold')
fig.text(.055,.778,C['subtitle'],fontsize=10,color='#bfd0df')
def panel():
    ax=fig.add_axes([.075,.20,.57,.48],facecolor='#102232')
    ax.spines[['top','right']].set_visible(False)
    ax.grid(alpha=.14,color='#b9cfe0',axis='y');ax.set_axisbelow(True)
    return ax
for i,(value,label) in enumerate(C['cards']):
    y=.535-i*.175
    fig.add_artist(FancyBboxPatch((.70,y),.255,.135,boxstyle='round,pad=0.012',facecolor='#152c3e',edgecolor='#284459',transform=fig.transFigure))
    fig.text(.714,y+.073,value,fontsize=18 if len(value)<15 else 13,fontweight='bold',color=C['accent'])
    fig.text(.714,y+.025,label,fontsize=8.1,color='#bfd0df')
fig.text(.055,.085,'HEADLESS DEMO  /  SOURCE + INPUTS + OUTPUTS INCLUDED',fontsize=9,color=C['accent'])
fig.text(.055,.046,'Recorded fixture results. See publication/README.md for reproduction and scope.',fontsize=8,color='#a8bdce')
d=json.loads((HERE/'demo.json').read_text());ax=fig.add_axes([.05,.16,.60,.55],projection='3d',facecolor='#091622')
for offset,result,color in [(0,d['results'][0],'#ff977d'),(1.7,d['results'][1],C['accent'])]:
    v=np.array(result['vertices'],dtype=float);v[:,0]+=offset
    ax.add_collection3d(Poly3DCollection([v[f] for f in result['faces']],facecolor=color,edgecolor=color,alpha=.35,linewidth=1.4))
    ax.text(offset+.15,-.25,0,result['name'].upper(),color=color,fontsize=11)
ax.set(xlim=(-.2,2.9),ylim=(-.4,1.2),zlim=(0,1.1));ax.set_box_aspect((2.7,1.1,1.1));ax.view_init(23,-61);ax.set_axis_off()

fig.savefig(HERE/'evidence.png',dpi=160,facecolor=fig.get_facecolor())
fig.savefig(HERE/'evidence.svg',facecolor=fig.get_facecolor(),metadata={'Date':None})
plt.close(fig)
