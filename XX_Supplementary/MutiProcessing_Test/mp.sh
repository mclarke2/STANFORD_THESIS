#!/bin/bash
#SBATCH -N 2
#SBATCH -n 12
mpiexec -n 24 /opt/python/3.7/bin/python  Multiprocessing_Test.py
