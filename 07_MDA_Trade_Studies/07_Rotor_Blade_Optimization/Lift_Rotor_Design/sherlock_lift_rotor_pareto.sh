#!/bin/bash 
#SBATCH -p jjalonso
#SBATCH --mem-per-cpu 8GB
#SBATCH -N 1
#SBATCH -n 32
#SBATCH --cpus-per-task=2
#SBATCH -t 4-00:00
module load python/3.6.1
module load py-mpi4py
module load swig
module load fltk
module load libxml2
module load eigen
module load gcc/10.1.0
 
mpiexec -n 32 python3  Lift_Rotor_Design.py
