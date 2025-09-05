# mppi_controller

## Dependancy

 - Eigen 3.3.9
 - EigenRand
 - OpenMP

## Map Download and Modification

```bash
git clone https://github.com/king-uk/mppi_controller.git
cd mppi_controller
```

## Please download BARN_dataset/grid_files from [BARN_dataset](https://www.cs.utexas.edu/~xiao/BARN/BARN.html)
```bash
mkdir BARN_dataset
cd BARN_dataset
python3 npy_to_txt.py
```

## Please set dependacy
```bash
mkdir include
cd include
git clone https://github.com/bab2min/EigenRand.git
git clone https://github.com/lava/matplotlib-cpp.git
```

---
## Usage


build with this

```bash
cd GuideMPPI
mkdir build && cd build
cmake .. -Dquadrotor=0 && make
./guide_mppi
```

## Result
<p align="center">
<img width="640" height="480" alt="Figure_1" src="figure/Bic_MPPI.png>
</p>

