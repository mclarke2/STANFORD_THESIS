# ----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------- 
import matplotlib.pyplot as plt
# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():
    group_names = ['Light-Duty Vehicles - 59%', 'Medium- and Heavy-Duty Trucks - 23%', 'Aircraft - 9%', 'Other - 4%', 'Rail - 2%', 'Ships and Boats - 3%']   
    sizes = [59,23,9,4,2,3]
    fig, axes = plt.subplots(figsize=(18, 14), subplot_kw=dict(aspect="equal"))
    explode = (0, 0, 0, 0.1, 0.2 ,0.3 ) 
    legend_properties = {'size': 20}    
    colors = ['darkgreen','forestgreen','mediumspringgreen','turquoise','mediumaquamarine','teal']
    axes.pie(sizes,explode = explode, startangle=90, colors=colors)
    plt.legend(labels=group_names, loc="center left",bbox_to_anchor=(1, 0, 0.5, 1),frameon=False,prop=legend_properties) 
    plt.show()     
 
if __name__ == '__main__': 
    main()     