# Multiprocessing test 

# imports  
from SUAVE.Core import Data
import numpy as np
import time 
import multiprocessing
import concurrent.futures
from functools import partial

# function
def main(): 
    #num_cpu = multiprocessing.cpu_count()
    #print("Number of CPUs: " + str(num_cpu) )
        
    #wait_time              = 1 
    #number_of_computations = 8
    
    
    #start = time.perf_counter()  
    
    
    #num_mic   = 8
    #data_list = list(np.arange(8))
    #pool   = multiprocessing.Pool(processes=number_of_computations) 
    #C_togo = np.ones(len(data_list))
    #prod_x = partial(prod_xy,y= 10,C= C_togo)
    #res    = pool.map(prod_x, data_list)
     
    #A_res = np.zeros(num_mic)
    #B_res = np.zeros(num_mic)
    
    #for i in range(num_mic):
        #A_res[i] = res[i].A 
        #B_res[i] = res[i].B  
        
    #finish = time.perf_counter()
    #print(f'Multi Process Method Finished in {round(finish-start,2)} second(s)')  
    
    #print(A_res)
    #print(B_res)

                   
    a  = np.random.rand(30,20,10,200,10,20,2)
    b  = np.random.rand(30,20,10,200,10,20,2)
    

    ti2_2 = time.time()    
    c1 = np.multiply(a, b)
    c2 = np.multiply(a, b)
    c3 = np.multiply(c1, c2) 
    tf2_2          = time.time()
    elapsed_time = round((tf2_2-ti2_2),2)
    print('  Elapsed Time: ' + str(elapsed_time) + ' secs')
    
    
    ti2_2 = time.time()    
    c = np.multiply(np.multiply(a, b), np.multiply(a, b))
    tf2_2          = time.time()
    elapsed_time = round((tf2_2-ti2_2),2)
    print('  Elapsed Time: ' + str(elapsed_time) + ' secs')
    

    ti2_2 = time.time()     
    c = (a*b)*(a*b)
    tf2_2          = time.time()
    elapsed_time = round((tf2_2-ti2_2),2)
    print('  Elapsed Time: ' + str(elapsed_time) + ' secs')
    
    

    ti2_2 = time.time()    
    d = np.divide(np.divide(a, b), np.divide(a, b))
    tf2_2          = time.time()
    elapsed_time = round((tf2_2-ti2_2),2)
    print('  Elapsed Time: ' + str(elapsed_time) + ' secs')
    

    ti2_2 = time.time()     
    d = (a/b)/(a/b)
    tf2_2          = time.time()
    elapsed_time = round((tf2_2-ti2_2),2)
    print('  Elapsed Time: ' + str(elapsed_time) + ' secs')
    
    
    ## start timer 
    #start = time.perf_counter()     
    #for _ in range(number_of_computations):
        #new_do_something(wait_time)
    
    ## end timer 
    #finish = time.perf_counter() 
    #print(f'Sequential Method Finished in {round(finish-start,2)} second(s)')  

    
    ## MULTI PROCESS METHOD 
    ## start timer 
    #start = time.perf_counter()  
    
    #multiprocess_method_1(wait_time,number_of_computations,num_cpu) 
    ## end timer 
    #finish = time.perf_counter()
    
    #print(f'Multi Process Method 1 Finished in {round(finish-start,2)} second(s)')    
    

    #start = time.perf_counter()  
     
    #multiprocess_method_2(wait_time,number_of_computations,num_cpu) 
    ## end timer 
    #finish = time.perf_counter()
    
    #print(f'Multi Process Method 2 Finished in {round(finish-start,2)} second(s)')  
    
    return  


def prod_xy(x,y,C):
 
    time.sleep(4)    
    A  =  (x+1) * y + C[x]
    B  =  (x+1) + y + C[x]
    
    D = Data()
    D.A = A
    D.B = B
    return A

#def parallel_runs(data_list):
    #pool = multiprocessing.Pool(processes=4)
    #prod_x=partial(prod_xy, y=10) # prod_x has only one argument x (y is fixed to 10)
    #result_list = pool.map(prod_x, data_list)
    
    #return result_list


# ----------------------------------------------
# New Multiprocessing Method
# ----------------------------------------------
def multiprocess_method_1(wait_time,number_of_computations,num_cpu):
   
    with concurrent.futures.ProcessPoolExecutor(max_workers = num_cpu) as executor:  
        results = [executor.submit(new_do_something, wait_time) for i in range(number_of_computations)]
         
        for f in concurrent.futures.as_completed(results): 
            print(f.result()) 
        
    return  


def multiprocess_method_2(wait_time,number_of_computations,num_cpu):
    wait_time = list(np.ones(number_of_computations))
    with concurrent.futures.ProcessPoolExecutor(max_workers = num_cpu) as executor:  
        results = executor.map(new_do_something, wait_time)
         
        for result in results:
            print(result) 
        
    return  

def new_do_something(seconds):
    print(f"Sleeping for {seconds} second...")
    time.sleep(seconds)
    return f'Done sleeping...{seconds}' # returing a value (string)  



if __name__ == '__main__':
    main()