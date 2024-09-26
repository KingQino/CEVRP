# CEVRP

## Usage :dog:

1. First step - compile

   ```shell
   mkdir build
   cd build
   cmake ..
   make
   ```

2. Second step - run

   ```shell
   ./Run 0 E-n22-k4.evrp 1 0 1
   
   # Explanation
   # ./Run <algorithm: 0-CBMA, 1-LAHC> <problem_instance_filename> <enable_logging: 0-no, 1-yes> <stop_criteria: 0 max-evals, 1 max-time> <multithreading: 0 no, 1 yes> 
   ```
3. Third step - HPC run 

   ```shell
   ml load CMake/3.18.4 GCC/13.2.0
   ```

   `./build/script.slurm`
   ```shell
   #!/bin/bash
   
   # Slurm job options (job-name, compute nodes, job time)
   #SBATCH --job-name=Ma-Original
   #SBATCH --output=path/log/slurm-%A_%a.out
   #SBATCH --time=48:0:0                   # Request 48 hours of compute time
   #SBATCH --nodes=1                       # Request 1 node
   #SBATCH --tasks-per-node=1              # One task per node
   #SBATCH --cpus-per-task=10              # Each task uses 10 CPUs (threads)
   #SBATCH --mem-per-cpu=1G                # Memory per CPU
   #SBATCH --account=su008-exx866
   #SBATCH --array=0-16

   module load GCC/13.2.0

   mapfile -t cases < parameters.txt
   CASE="${cases[$SLURM_ARRAY_TASK_ID]}"

   srun ./Run 0 "$CASE" 1 0 1 
   ```
   
   `./build/parameters.txt`
   ```shell
   E-n22-k4.evrp
   E-n23-k3.evrp
   E-n30-k3.evrp
   E-n33-k4.evrp
   E-n51-k5.evrp
   E-n76-k7.evrp
   E-n101-k8.evrp
   X-n143-k7.evrp
   X-n214-k11.evrp
   X-n351-k40.evrp
   X-n459-k26.evrp
   X-n573-k30.evrp
   X-n685-k75.evrp
   X-n749-k98.evrp
   X-n819-k171.evrp
   X-n916-k207.evrp
   X-n1001-k43.evrp
   ```