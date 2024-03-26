### Steps to Install acados:


clone the repo in the home/user directory and cd into the capracontrol directory. 

#### 1. Initialize acados submodule:
```bash
git submodule update --recursive --init
```

#### 2. Move to the acados directory and build it:
```bash
cd ~/dtmpc/src/mpc/acados
mkdir -p build
cd build
cmake -DACADOS_WITH_QPOASES=ON ..
make install -j4
cd ~/dtmpc/
```

#### 3. Install the acados_template interface python libary.
```bash
cd ~/dtmpc/src/mpc/acados/
pip install -e interfaces/acados_template
cd ~/dtmpc/
```

#### 4. set environment variables and run the python simulation:

Very important note. You must source each time you run the simulation. Each time you start up there is no need to build and install the code. Just run these lines and the libary is setup. 
```bash
cd ~/dtmpc/src/mpc/acados/
export acados_dir="$(pwd)"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"$acados_dir/lib"
export ACADOS_SOURCE_DIR="$acados_dir"
cd ~/dtmpc/
```
#### Error codes for NLP solver:
0. ACADOS_SUCCESS,
1. ACADOS_FAILURE,
2. ACADOS_MAXITER,
3. ACADOS_MINSTEP,
4. ACADOS_QP_FAILURE,
5. ACADOS_READY,

#### Error codes for QP Solver:
0. SUCCESS, // found solution satisfying accuracy tolerance
1. MAX_ITER, // maximum iteration number reached
2. MIN_STEP, // minimum step length reached
3. NAN_SOL, // NaN in solution detected
4. INCONS_EQ, // unconsistent equality constraints




