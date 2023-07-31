# Dynoplan 🦖



<p align="center">
<img src="assets/example1.png" width=60% height=auto>
</p >





The first version [kinodynamic-motion-planning-benchmark](https://github.com/imrCLab/kinodynamic-motion-planning-benchmark) is now deprecated.

## Robots and Problem Description

Kinodynamic motion planning problem are defined in [Dynobench](https://github.com/quimortiz/dynobench)


<p align="center">
<img src="assets/dynobench.png" width=30% height=auto>
</p >


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

Results of reported in our TRO paper are in folder XX. To replicate the results use commit: `xxxxx`


First, download primitives with:

```
bash -x download_primitives.bash
```

Primitvies are stored in a new `dynomotions_full` directory. Next, move to the `build` directory and run commands:

Benchmark between planners

```
python3 ../benchmark/benchmark.py -m bench -bc    ../benchmark/config/compare.yaml
```

Study of heuristic functions

```
python3 ../benchmark/benchmark.py -m bench_search -bc    ../benchmark/config/bench_search.yaml
```

Study of strategy for trajectoy optimization with free terminal time
```
python3 ../benchmark/benchmark.py -m bench_time -bc    ../benchmark/config/bench_time.yam
```

Study of time spent in each component

```
python3   ../benchmark/benchmark.py -m study  -bc ../benchmark/config/bench_abblation_study.yaml
```

You can modify each config file to change the number of runs, the evaluated problems and the maximum time.
The configurations files we used for `TRO` have prefix `TRO`.

The paramteres for each algorithm are in `.yaml` files inside the `benchmark/config/algs` directory, severalfro example `idbastar_v0.yaml`.







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
