# Libraries
import matplotlib.pyplot as plt
import pandas as pd
from math import pi

# Set data
df = pd.DataFrame({
'group': ['LiCoO2','LiMn2O2','LiFePO4','LiNiMnCoO2','LiNiCoAlO2','Li4Ti5O12'],
'Specific \n Energy': [100,75,50,100,100,50],
'Specific \n Power': [50,75,75,75,100,75],
'Energy \n Density': [100,75,75,100,100,75 ],
'Power \n Density': [50,100,75,100,100,75],
'Safety': [50,75,100,75,50,100 ],
'Performance': [75,50,50,75,75,100 ],
'Life Span': [50,50,100,75,100,100],
'Cost': [75,75,75,75,50,25 ]
})

# ------- PART 1: Create background

# number of variable
categories=list(df)[1:]
N = len(categories)

# What will be the angle of each axis in the plot? (we divide the plot / number of variable)
angles = [n / float(N) * 2 * pi for n in range(N)]
angles += angles[:1]

fig = plt.figure()
fig.set_size_inches(16, 12)
plt.rcParams["font.family"] = "serif"
plt.rcParams["font.serif"] = "Times New Roman"

# Initialise the spider plot
ax = plt.subplot(111, polar=True)

# If you want the first axis to be on top:
ax.set_theta_offset(pi / 2)
ax.set_theta_direction(-1)

# Draw one axe per variable + add labels labels yet
plt.xticks(angles[:-1], categories, size= 20)
ax.tick_params(axis="x" , pad= 40)
plt.box(on=None)

# Draw ylabels
ax.set_rlabel_position(0)
plt.yticks([20,40,60,80,100], ["20","40","60","80","100"], color="black", size=16)
plt.ylim(0,100) 

# ------- PART 2: Add plots 
# Ind1
lw = 4 

values=df.loc[0].drop('group').values.flatten().tolist()
values += values[:1]
ax.plot(angles, values, color='blue', linewidth=lw, linestyle='solid', label="LiCoO2")  
ax.fill(angles, values, color='blue', alpha=0.2)

# Ind2
values=df.loc[1].drop('group').values.flatten().tolist()
values += values[:1]
ax.plot(angles, values, color='red', linewidth=lw, linestyle='solid', label="LiMn2O2")  
ax.fill(angles, values, color='red', alpha=0.2)

# Ind2
values=df.loc[2].drop('group').values.flatten().tolist()
values += values[:1]
ax.plot(angles, values, color='greenyellow', linewidth=lw, linestyle='solid', label="LiFePO4")  
ax.fill(angles, values, color='greenyellow', alpha=0.2)

# Ind2
values=df.loc[3].drop('group').values.flatten().tolist()
values += values[:1]
ax.plot(angles, values, color='purple', linewidth=lw, linestyle='solid', label="LiNiMnCoO2")  
ax.fill(angles, values, color='purple', alpha=0.2)

# Ind2
values=df.loc[4].drop('group').values.flatten().tolist()
values += values[:1]
ax.plot(angles, values, color='lightseagreen', linewidth=lw, linestyle='solid', label="LiNiCoAlO2")  
ax.fill(angles, values,  color='lightseagreen', alpha=0.2)

# Ind2
values=df.loc[5].drop('group').values.flatten().tolist()
values += values[:1]
ax.plot(angles, values, color='darkorange', linewidth=lw, linestyle='solid', label="Li4Ti5O12")  
ax.fill(angles, values, color='darkorange', alpha=0.2)

# Add legend
plt.legend(loc='center left', prop={'size': 20}, bbox_to_anchor=(1.2, 0.5)) 
plt.show()