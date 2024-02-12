"""__init__
License: BSD 3-Clause License
Copyright (C) 2023, New York University

Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""

import numpy as np
import crocoddyl
import pinocchio as pin

import example_robot_data
from crocoddyl.utils.pendulum import (
    CostModelDoublePendulum,
    ActuationModelDoublePendulum,
)


def create_double_pendulum_problem():
    """
    Create shooting problem for the double pendulum model
    """
    print("Created double pendulum problem ...")
    # Loading the double pendulum model
    pendulum = example_robot_data.load("double_pendulum")
    model = pendulum.model
    state = crocoddyl.StateMultibody(model)
    actuation = ActuationModelDoublePendulum(state, actLink=1)
    nu = actuation.nu
    runningCostModel = crocoddyl.CostModelSum(state, nu)
    terminalCostModel = crocoddyl.CostModelSum(state, nu)
    xResidual = crocoddyl.ResidualModelState(state, state.zero(), nu)
    xActivation = crocoddyl.ActivationModelQuad(state.ndx)
    uResidual = crocoddyl.ResidualModelControl(state, nu)
    xRegCost = crocoddyl.CostModelResidual(state, xActivation, xResidual)
    uRegCost = crocoddyl.CostModelResidual(state, uResidual)
    xPendCost = CostModelDoublePendulum(
        state,
        crocoddyl.ActivationModelWeightedQuad(np.array([1.0] * 4 + [0.1] * 2)),
        nu,
    )
    dt = 1e-2
    runningCostModel.addCost("uReg", uRegCost, 1e-4 / dt)
    runningCostModel.addCost("xGoal", xPendCost, 5e-1 / dt)
    terminalCostModel.addCost("xGoal", xPendCost, 10.0)
    runningModel = crocoddyl.IntegratedActionModelEuler(
        crocoddyl.DifferentialActionModelFreeFwdDynamics(
            state, actuation, runningCostModel
        ),
        dt,
    )
    terminalModel = crocoddyl.IntegratedActionModelEuler(
        crocoddyl.DifferentialActionModelFreeFwdDynamics(
            state, actuation, terminalCostModel
        ),
        dt,
    )
    T = 100
    x0 = np.array([3.14, 0.0, 0.1, 0.0])
    pb = crocoddyl.ShootingProblem(x0, [runningModel] * T, terminalModel)
    return pb


def create_cartpole_problem():
    """
    Create shooting problem for Cartpole
    """
    print("Create cartpole problem ...")
    # Creating the DAM for the cartpole
    cartpoleDAM = DifferentialActionModelCartpole()
    # Using NumDiff for computing the derivatives. We specify the
    # withGaussApprox=True to have approximation of the Hessian based on the
    # Jacobian of the cost residuals.
    cartpoleND = crocoddyl.DifferentialActionModelNumDiff(cartpoleDAM, True)
    # Getting the IAM using the simpletic Euler rule
    timeStep = 5e-2
    cartpoleIAM = crocoddyl.IntegratedActionModelEuler(cartpoleND, timeStep)
    # Creating the shooting problem
    T = 50
    terminalCartpole = DifferentialActionModelCartpole()
    terminalCartpoleDAM = crocoddyl.DifferentialActionModelNumDiff(
        terminalCartpole, True
    )
    terminalCartpoleIAM = crocoddyl.IntegratedActionModelEuler(terminalCartpoleDAM, 0.0)
    terminalCartpole.costWeights[0] = 200
    terminalCartpole.costWeights[1] = 200
    terminalCartpole.costWeights[2] = 1.0
    terminalCartpole.costWeights[3] = 0.1
    terminalCartpole.costWeights[4] = 0.01
    terminalCartpole.costWeights[5] = 0.0001
    pb = crocoddyl.ShootingProblem(x0, [cartpoleIAM] * T, terminalCartpoleIAM)
    return pb


def create_quadrotor_problem():
    """
    Create shooting problem for quadrotor task
    """
    print("Create quadrotor problem ...")
    hector = example_robot_data.load("hector")
    robot_model = hector.model
    target_pos = np.array([1.0, 0.0, 1.0])
    target_quat = pin.Quaternion(1.0, 0.0, 0.0, 0.0)
    state = crocoddyl.StateMultibody(robot_model)
    d_cog, cf, cm, u_lim, l_lim = 0.1525, 6.6e-5, 1e-6, 5.0, 0.1
    tau_f = np.array(
        [
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0, 1.0],
            [0.0, d_cog, 0.0, -d_cog],
            [-d_cog, 0.0, d_cog, 0.0],
            [-cm / cf, cm / cf, -cm / cf, cm / cf],
        ]
    )
    actuation = crocoddyl.ActuationModelMultiCopterBase(state, tau_f)

    nu = actuation.nu
    runningCostModel = crocoddyl.CostModelSum(state, nu)
    terminalCostModel = crocoddyl.CostModelSum(state, nu)
    # Costs
    xResidual = crocoddyl.ResidualModelState(state, state.zero(), nu)
    xActivation = crocoddyl.ActivationModelWeightedQuad(
        np.array([0.1] * 3 + [1000.0] * 3 + [1000.0] * robot_model.nv)
    )
    uResidual = crocoddyl.ResidualModelControl(state, nu)
    xRegCost = crocoddyl.CostModelResidual(state, xActivation, xResidual)
    uRegCost = crocoddyl.CostModelResidual(state, uResidual)
    goalTrackingResidual = crocoddyl.ResidualModelFramePlacement(
        state,
        robot_model.getFrameId("base_link"),
        pin.SE3(target_quat.matrix(), target_pos),
        nu,
    )
    goalTrackingCost = crocoddyl.CostModelResidual(state, goalTrackingResidual)
    runningCostModel.addCost("xReg", xRegCost, 1e-6)
    runningCostModel.addCost("uReg", uRegCost, 1e-6)
    runningCostModel.addCost("trackPose", goalTrackingCost, 1e-2)
    terminalCostModel.addCost("goalPose", goalTrackingCost, 3.0)
    dt = 3e-2
    runningModel = crocoddyl.IntegratedActionModelEuler(
        crocoddyl.DifferentialActionModelFreeFwdDynamics(
            state, actuation, runningCostModel
        ),
        dt,
    )
    terminalModel = crocoddyl.IntegratedActionModelEuler(
        crocoddyl.DifferentialActionModelFreeFwdDynamics(
            state, actuation, terminalCostModel
        ),
        dt,
    )

    # Creating the shooting problem and the FDDP solver
    T = 33
    x0 = np.array(list(hector.q0) + [0.0] * hector.model.nv)
    pb = crocoddyl.ShootingProblem(x0, [runningModel] * T, terminalModel)
    return pb
