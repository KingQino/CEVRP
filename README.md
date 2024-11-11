# CEVRP

## Usage :dog:

1. First step - compile

   ```shell
   mkdir build
   cd build
   cmake -DCMAKE_BUILD_TYPE=Release ..
   make
   ```
   In case running `Valgrind` for memory leak check
   
   ```shell
   ml load valgrind cmake gcc
   cmake -DCMAKE_BUILD_TYPE=Debug ..
   make valgrind_tests
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
   sbatch script.slurm 
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

4. Fourth step - collect experiment results

   Objective values - `.stats/algorithm/`

   ```shell
   ls | grep -v objective.sh | awk -F'-' '{print $0"\t"substr($2, 2)}' | sort -t $'\t'  -k2n | awk -F'\t' '{print "tail -n 3 "$1"/stats.*"}' > objective.sh
   sh objective.sh
   ```

   convergence curve - 

   ```shell
   
   ```

## Experiments :deer:

- Experiment design, start mulltiple programs in one go. 

  Git clone the experimental branch to HPC server. Note: naming the directory name with the exclusive word. 

  ```shell
  # git checkout -b exp-[group]-[name]
  # git clone -b exp-[group]-[name] git@github.com:KingQino/CEVRP.git [name]
  
  git checkout -b exp-g2-adaptive
  git clone -b exp-g2-adaptive git@github.com:KingQino/CEVRP.git Adaptive
  ```

  After getting multiple experimental branches/directories, we can run the script below:

  `setup.sh`

  ```shell
  #!/bin/bash
  
  # Target directories
  directories=$(ls -d */ | grep -v '^Original/')
  
  # Check if any directories were found
  if [ -z "$directories" ]; then
      echo "No directories found."
      exit 1
  fi
  
  # Loop through each directory to set up build structure and create script.slurm
  for dir in $directories; do
      # Remove trailing slash from directory name
      dir=${dir%/}
  
      # Check if the directory exists
      if [ -d "$dir" ]; then
          # Define paths for build and log folders
          build_dir="$dir/build"
          log_dir="$build_dir/log"
  
          # Create the build and log directories if they don't exist
          mkdir -p "$log_dir"
          echo "Created '$log_dir'."
  
          # Copy parameters.txt into the build folder
          if [ -f "parameters.txt" ]; then
              cp parameters.txt "$build_dir"
              echo "Copied 'parameters.txt' to '$build_dir'."
          else
              echo "'parameters.txt' not found in the current directory."
          fi
  
          # Create script.slurm with dynamic content
          cat > "$build_dir/script.slurm" <<EOL
  #!/bin/bash
  
  # Slurm job options (job-name, compute nodes, job time)
  #SBATCH --job-name=Ma-$dir                             # Job name set to the parent directory name
  #SBATCH --output=$(pwd)/$log_dir/slurm-%A_%a.out       # Output log file path in the log folder
  #SBATCH --time=48:0:0                                  # Request 48 hours of compute time
  #SBATCH --nodes=1                                      # Request 1 node
  #SBATCH --tasks-per-node=1                             # One task per node
  #SBATCH --cpus-per-task=10                             # Each task uses 10 CPUs (threads)
  #SBATCH --mem-per-cpu=1G                               # Memory per CPU
  #SBATCH --account=su008-exx866
  #SBATCH --array=0-16
  
  # Load necessary modules
  module load GCC/13.2.0
  
  # Load cases from parameters.txt
  mapfile -t cases < "parameters.txt"        # Load parameters.txt from the build directory
  CASE="\${cases[\$SLURM_ARRAY_TASK_ID]}"
  
  # Run the specified command with case argument
  srun ./Run 0 "\$CASE" 0 0 1
  EOL
  
          echo "Generated 'script.slurm' in '$build_dir'."
  
          # Navigate to build_dir, run cmake and make commands
          (
              cd "$build_dir" || exit
              cmake -DCMAKE_BUILD_TYPE=Release ..
              make
              sbatch script.slurm
          )
      else
          echo "Directory '$dir' does not exist."
      fi
  done
  ```

  Then waiting the experimental results, and collect.

- There are a number of conditions that affect our experiment results, including `stop_criteria`, `adaptive selection`, `filters`, `search acceleration`, `neighborhood expanding`, `local search operators combination & sequence`, `fine-tunning`, etc. 

  > How to design our experiments and organize our Excel table to collect results?

- 