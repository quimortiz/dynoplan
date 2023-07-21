# Dynoplan 🦖

By Joaquim Ortiz-Haro and Wolfgang Honnig

The first version [kinodynamic-motion-planning-benchmark](https://github.com/imrCLab/kinodynamic-motion-planning-benchmark) is now deprecated. 

## Robots and Problem Description 

Kinodynamic motion planning problem are defined in [dynobench](https://github.com/quimortiz/dynobench)

## How to use

we provide several exectuables and libraries to fit different use cases

## Testing

Check the tests to learn how to use the code! 

## Planners

- Pure Optimization-Base
- RRT* (ompl) + Optimization
- Idbastar (Iterative search and Optimization)
- Dbrrt, AO-dbrrt and DBrrtConnect (coming soon!)
- SST* (ompl)

## Building

Dependencies:

fcl (0.7)
yaml-cpp
Eigen 3
Crocoddyl (1.8)
OMPL (1.6)

Yes, you need OMPL 1.6 for planners RRT + TO and  SST  . You will need OMPL 1.6 -- we recommend to install OMPL in a local directory with -DCMAKE_INSTALL_PREFIX, and use -DCMAKE_PREFIX_PATH here

## Motion Primitives

You will find a small set of motion primitives for each system in  [dynobench](https://github.com/quimortiz/dynobench).
I you want more primitives, e.g. to run our planner, use
wget ... 


## Benchmark 

Results of the benchmark in TRO are in folder XX. 
To replicate the results, use branch XX
and run the script : 

Benchmark between planners
```
python3
```

Study of heuristic functions
```
python3
```

Study of strategy for trajectoy optimization with free terminal time 
```
python3
```



## Citing

If you use or work for academic research, please cite:

```
COOL TRO paper
```


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

