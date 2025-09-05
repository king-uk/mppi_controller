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
cd BARN_dataset
python3 npy_to_txt.py
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
