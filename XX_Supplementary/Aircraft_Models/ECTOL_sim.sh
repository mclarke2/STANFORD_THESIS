#!/bin/bash
#SBATCH -N 1
#SBATCH -n 12
mpiexec -n 1 /opt/python/3.7/bin/python  ECTOL_2P.py
