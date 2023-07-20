# idbastar

By Joaquim Ortiz-Haro and Wolfgang Honnig



The first version was
[kinodynamic-motion-planning-benchmark](https://github.com/imrCLab/kinodynamic-motion-planning-benchmark).



## Robotic Systems



## Problem Description 


## How to use

we provide several exectuables and libraries to fit different use cases



## Planners

- Pure Optimization-Base
- RRT* (ompl) + Optimization
- Idbastar (Iterative search and Optimization)
- Dbrrt, AO-dbrrt and DBrrtConnect (coming soon!)
- SST* (ompl)

## Building

Dependencies:

fcl (we use   )
yaml-cpp
Eigen 3
Crocoddyl (we use 1.8)


You will need OMPL 1.6 -- we recommend to install OMPL in a local directory with -DCMAKE_INSTALL_PREFIX, and use -DCMAKE_PREFIX_PATH here

## Citing

If you use or work for academic research, please cite:

```
@online{hoenigDbADiscontinuityboundedSearch2022,
  title = {Db-A*: Discontinuity-Bounded Search for Kinodynamic Mobile Robot Motion Planning},
  author = {Hoenig, Wolfgang and Ortiz-Haro, Joaquim and Toussaint, Marc},
  year = {2022},
  eprint = {2203.11108},
  eprinttype = {arxiv},
  url = {http://arxiv.org/abs/2203.11108},
  archiveprefix = {arXiv}
}
```

