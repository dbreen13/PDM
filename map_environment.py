import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.patches import RegularPolygon

#define Matplotlib figure and axis
fig, ax = plt.subplots()

#create simple line plot
ax.plot([0,100],[0, 0],
[100,100],[0,80],
[100,0],[80,80],
[0,0],[80,0],
color='blue')

#add rectangle to plot
w = 40
h = 4
for s in [0,50]:
    ax.add_patch(Rectangle((s, 0), w, h))

w2 = 4
h2 = 60
for x in [0,15,30,45,60,75,90]:
    ax.add_patch(Rectangle((x, 10), w2, h2, color='brown'))

w3 = 15
h3 = 4
for s in [0,20,40,60,80]:
    ax.add_patch(Rectangle((s, 75), w3, h3, color='green'))

#add oxagon to plot
for x in [10,55,70]:
    for y in [10,30,60]:
        ax.add_patch(RegularPolygon((x, y), 6 , 2, color='red'))

#display plot
plt.show()