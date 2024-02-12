"""__init__
License: BSD 3-Clause License
Copyright (C) 2023, New York University

Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""

import pathlib
import os
import numpy as np
import mim_solvers

python_path = pathlib.Path(".").absolute().parent / "python"
print(python_path)
os.sys.path.insert(1, str(python_path))

from py_mim_solvers import SQP

from problems import create_double_pendulum_problem, create_quadrotor_problem
import pinocchio as pin

# Solver params
MAXITER = 10
TOL = 1e-4
CALLBACKS = True
FILTER_SIZE = MAXITER

# Create 1 solver of each type for each problem

problems = [create_double_pendulum_problem(), create_quadrotor_problem()]

for problem in problems:
    x0 = problem.x0.copy()

    # Create solver SQP (MS)
    solverSQP = mim_solvers.SolverSQP(problem)
    solverSQP.xs = [solverSQP.problem.x0] * (solverSQP.problem.T + 1)
    solverSQP.us = solverSQP.problem.quasiStatic(
        [solverSQP.problem.x0] * solverSQP.problem.T
    )
    solverSQP.termination_tolerance = TOL
    solverSQP.use_filter_line_search = True
    solverSQP.filter_size = MAXITER
    solverSQP.with_callbacks = CALLBACKS
    solverSQP.reg_min = 0.0  # This turns of regularization completely.
    reginit = 0.0

    # Create python solver
    pysolverSQP = SQP(problem, VERBOSE=CALLBACKS)
    pysolverSQP.termination_tolerance = TOL

    # SQP
    solverSQP.xs = [x0] * (problem.T + 1)
    solverSQP.us = problem.quasiStatic([x0] * problem.T)
    solverSQP.solve(solverSQP.xs.copy(), solverSQP.us.copy(), MAXITER, False, reginit)

    pysolverSQP.xs = [x0] * (problem.T + 1)
    pysolverSQP.us = problem.quasiStatic([x0] * problem.T)
    pysolverSQP.solve(pysolverSQP.xs.copy(), pysolverSQP.us.copy(), MAXITER)

##### UNIT TEST #####################################

set_tol = 1e-4

for t in range(problem.T - 1, 0, -1):
    assert (
        np.linalg.norm(pysolverSQP.L[t] + solverSQP.K[t], 1)
        < (np.size(pysolverSQP.L[t])) * set_tol
    )
    assert (
        np.linalg.norm(pysolverSQP.l[t] + solverSQP.k[t], 1) / (len(pysolverSQP.l[t]))
        < set_tol
    )

assert (
    np.linalg.norm(np.array(pysolverSQP.xs) - np.array(solverSQP.xs)) / (problem.T + 1)
    < set_tol
), "Test failed"
assert (
    np.linalg.norm(np.array(pysolverSQP.us) - np.array(solverSQP.us)) / problem.T
    < set_tol
), "Test failed"

assert pysolverSQP.KKT - solverSQP.KKT < set_tol, "Test failed"

print("ALL UNIT TEST PASSED .....")
