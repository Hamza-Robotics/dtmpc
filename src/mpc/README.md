### Steps to Install acados:


clone the repo in the home/user directory and cd into the capracontrol directory. 

#### 1. Initialize and update the acados submodule:
```bash
cd ~/dtmpc/src/mpc
git clone https://github.com/acados/acados.git
cd acados
git submodule update --recursive --init
```

#### 2. Move to the acados directory and build it:
```bash

cd ~/dtmpc/src/mpc/trailer_local_planner/acados
mkdir -p build
cd build
cmake -DACADOS_WITH_QPOASES=ON ..
make install -j4
```

#### 3. Install the acados_template interface python libary.
```bash
cd ~/dtmpc/src/trailer_local_planner/acados/
pip install -e interfaces/acados_template
```

#### 4. set environment variables and run the python simulation:

Very important note. You must source each time you run the simulation. Each time you start up there is no need to build and install the code. Just run these lines and the libary is setup. 
```bash
cd ~/dtmpc/src/trailer_local_planner/acados/
export acados_dir="$(pwd)"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"$acados_dir/lib"
export ACADOS_SOURCE_DIR="$acados_dir"