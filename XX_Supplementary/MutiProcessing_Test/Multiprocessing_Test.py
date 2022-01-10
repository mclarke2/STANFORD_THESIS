# Multiprocessing test 

# imports  
import numpy as np
import time 
import multiprocessing
import concurrent.futures

# function
def main(): 

    num_cpu = multiprocessing.cpu_count()
    print("Number of CPUs: " + str(num_cpu) )
        
    wait_time              = 1 
    number_of_computations = 24
    
    # start timer 
    start = time.perf_counter()     
    for _ in range(number_of_computations):
        new_do_something(wait_time)
    
    # end timer 
    finish = time.perf_counter() 
    print(f'Sequential Method Finished in {round(finish-start,2)} second(s)')  

    
    # MULTI PROCESS METHOD 
    # start timer 
    start = time.perf_counter()  
    
    multiprocess_method(wait_time,number_of_computations,num_cpu)

    # end timer 
    finish = time.perf_counter()
    
    print(f'Multi Process Method Finished in {round(finish-start,2)} second(s)')    
    return  

# ----------------------------------------------
# New Multiprocessing Method
# ----------------------------------------------
def multiprocess_method(wait_time,number_of_computations,num_cpu):
   
    with concurrent.futures.ProcessPoolExecutor(max_workers = num_cpu) as executor:  
        results = [executor.submit(new_do_something, wait_time) for i in range(number_of_computations)]
         
        for f in concurrent.futures.as_completed(results): 
            print(f.result()) 
        
    return  

def new_do_something(seconds):
    print(f"Sleeping for {seconds} second...")
    time.sleep(seconds)
    return f'Done sleeping...{seconds}' # returing a value (string)  


if __name__ == '__main__':
    main()