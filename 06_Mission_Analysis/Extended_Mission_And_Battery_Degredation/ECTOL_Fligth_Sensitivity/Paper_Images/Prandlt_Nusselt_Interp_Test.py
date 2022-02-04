import numpy as np
from matplotlib import pyplot as plt  
from   scipy.interpolate import interp1d, interp2d, RectBivariateSpline, RegularGridInterpolator
# ----------------------------------------------------------------------
#   Main
# ---------------------------------------------------------------------- 
def main():     
    raw_Pr = np.array([[-173.2,0.780 ], [-153.2,0.759 ], [-133.2,0.747 ], [-93.2,0.731  ], [-73.2,0.726  ], [-53.2,0.721  ], 
                       [-33.2,0.717  ], [-13.2,0.713  ], [0.0,0.711    ], [6.9,0.710    ],[15.6,0.709   ], [26.9,0.707   ],
                       [46.9,0.705   ], [66.9,0.703   ], [86.9,0.701   ], [106.9,0.700  ], [126.9,0.699  ], [226.9,0.698  ], 
                       [326.9,0.703  ], [426.9,0.710  ], [526.9,0.717  ], [626.9,0.724  ], [  726.9,0.730 ],[826.9,0.734],[1226.9,0.743], [1626.9,0.742]]) 
   
    z1 = np.polyfit(raw_Pr[:,0],raw_Pr[:,1],4)
    f1 = np.poly1d(z1)    
   
    raw_nu = np.array([[-75	,7.40E-6  ],[-50	,9.22E-6  ],[-25	,11.18E-6 ],[-15	,12.01E-6 ],[-10	,12.43E-6 ],[-5	,12.85E-6 ],[0	,13.28E-6 ],[5	,13.72E-6 ],
                       [10	,14.16E-6 ],[15	,14.61E-6 ],[20	,15.06E-6 ],[25	,15.52E-6 ],[30	,15.98E-6 ],[40	,16.92E-6 ],
                       [50	,17.88E-6 ],[60	,18.86E-6 ],[80	,20.88E-6 ],[100	,22.97E-6 ],[125	,25.69E-6 ],[150	,28.51E-6 ],
                       [175	,31.44E-6 ],[200	,34.47E-6 ],[225	,37.60E-6 ],[300	,47.54E-6 ],[412	,63.82E-6 ],[500	,77.72E-6 ],
                       [600	,94.62E-6 ],[700	,112.6E-6 ],[800	,131.7E-6 ],[900	,151.7E-6 ],[1000,172.7E-6    ],[1100,194.6E-6    ]])
   
    z2 = np.polyfit(raw_nu[:,0],raw_nu[:,1], 2)
    f2 = np.poly1d(z2)    
    
    temp  = np.linspace(-200, 1600, 30)
    Pr_new = f1(temp)
    Nu_new = f2(temp) 
        
    
    # Exact Solution 
    fig  = plt.figure(1) 
    fig.set_size_inches(8, 8)     
    axes = fig.add_subplot(1,1,1)   
    axes.plot(raw_Pr[:,0], raw_Pr[:,1], 'bo-' , label = 'raw data')
    axes.set_ylim([0.69,0.79])
    axes.plot(temp,Pr_new, 'rs-' , label = 'interpolated')
    axes.set_ylabel("Pr") 
    axes.legend(loc='upper right', prop={'size': 16}) 
    axes.set_xlabel("Temperature")   
    
    
    # Exact Solution 
    fig  = plt.figure(2) 
    fig.set_size_inches(8, 8)     
    axes = fig.add_subplot(1,1,1)   
    axes.plot(raw_nu[:,0], raw_nu[:,1], 'bo-' , label = 'raw data')
    axes.plot(temp,Nu_new, 'rs-' , label = 'interpolated')
    axes.set_ylabel("Nu") 
    axes.legend(loc='upper left', prop={'size': 16}) 
    axes.set_xlabel("Temperature")    
     
    return     

if __name__ == '__main__': 
    main()   
    plt.show() 

