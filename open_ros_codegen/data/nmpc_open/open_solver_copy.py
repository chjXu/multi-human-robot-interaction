import casadi as cs
import opengen as og
import numpy as np

# make x(state) = [q, q.dot] 

JOINT_NUM = 6 #关节数
dis = 0.3 #安全距离
N = 3 # The MPC horizon length
NX = 2 # The number of elements in the state vector
NU = 1 # The number of elements in the control vector
NM = 6 # The number of elements in the obs and robot vector
sampling_time = 0.16 #sampling time

# avoidance
# R_k = cs.DM.eye(NU) * [3.0]
# Q_pos = cs.DM.eye(NX) * [10.0, 5.0]      #(大，小)->快，位置不准
# QN_pos = cs.DM.eye(NX) * [10.0, 5.0]     #（小， 大）->慢
# delta = cs.DM.eye(NM) * [0.1]
# miu = cs.DM.eye(NM) * [0.1]

# collaboration
R_k = cs.DM.eye(NU) * [8.0]
Q_pos = cs.DM.eye(NX) * [5.0, 9.0]      #(大，小)->快，位置不准
QN_pos = cs.DM.eye(NX) * [10.0, 5.0]     #（小， 大）->慢
delta = cs.DM.eye(NM) * [0.1]
miu = cs.DM.eye(NM) * [0.1]

#   [q.dot + 0.5*a*t]
#   [a]
def dynamics_ct(_x, _u):
    return cs.vcat([_x[1] + 0.5 * _u * sampling_time,
                    _u])


#  fc(x, u) = [q + q.dot * sample_time + 0.5 * q.dot.dot * sample_time^2]
#             [q.dot + q.dot.dot * sample_time]
def dynamics_dt(x, u):
    dx = dynamics_ct(x, u)
    return cs.vcat([x[i] + sampling_time * dx[i] for i in range(NX)])

# The stage cost for x and u
def stage_cost(_x, _u, _x_ref=None, _u_ref=None):
    if _x_ref is None:
        _x_ref = cs.DM.zeros(_x.shape)
        
    if _u_ref is None:
        _u_ref = cs.DM.zeros(_u.shape)
    
    dx = _x - _x_ref
    du = _u - _u_ref
    return cs.mtimes([dx.T, Q_pos, dx]) + cs.mtimes([du.T, R_k, du])

# The terminal cost for x
def terminal_cost(_x, _x_ref=None):
    if _x_ref is None:
        _x_ref = cs.DM.zeros(_x.shape)
    dx = _x - _x_ref
    return cs.mtimes([dx.T, QN_pos, dx])

# The stage cost for robot state
def state_cost():
    pass
    #for i in JOINT_NUM:
        #state_c = stage_cost()

# The stage cost for obs
def obs_cost(_obs, _x_obs):
    pass

# def control_cost(_u, _u_ref=None):
#     if _u_ref is None:
#         _u_ref = cs.DM.zeros(_u.shape)

#     du = _u - _u_ref
#     return cs.mtimes([du.T, R_k, du])

# def position_cost(_pos, _pos_ref=None):
#     if _pos_ref is None:
#         _pos_ref = cs.DM.zeros(_pos.shape)

#     dp = _pos - _pos_ref
#     return cs.mtimes([dp.T, Q_pos, dp])

# def rotation_cost(_rot, _rot_ref=None):
#     if _rot_ref is None:
#         _rot_ref = cs.DM.zeros(_rot.shape)

#     dr = _rot - _rot_ref
#     return cs.mtimes([dr.T, Q_rot, dr])

# init
x_0 = cs.MX.sym("x_0", NX)
x_ref = cs.MX.sym("x_ref", NX)
u_k = [cs.MX.sym('u_' + str(i), NU) for i in range(N)]


# create the cost function
x_t = x_0
total_cost = 0

for t in range(0, N):
    total_cost += stage_cost(x_t, u_k[t], x_ref) #update cost
    x_t = dynamics_dt(x_t, u_k[t])

total_cost += terminal_cost(x_t, x_ref)

# optimization
optimization_variables = []
optimization_parameters = []

optimization_variables += u_k
optimization_parameters += [x_0]
optimization_parameters += [x_ref]

optimization_variables = cs.vertcat(*optimization_variables)
optimization_parameters = cs.vertcat(*optimization_parameters)

#constraint
umin = [-5] * N
umax = [5] * N

bounds = og.constraints.Rectangle(umin, umax)

problem = og.builder.Problem(optimization_variables,
                             optimization_parameters,
                             total_cost) \
    .with_constraints(bounds)

ros_config = og.config.RosConfiguration() \
    .with_package_name("ur5_nmpc_controller") \
    .with_node_name("ur5_nmpc_controller_node") \
    .with_rate((int)(1.0/sampling_time)) \
    .with_description("Cool ROS node.")

build_config = og.config.BuildConfiguration()\
    .with_build_directory("optimization_engine")\
    .with_build_mode("release")\
    .with_build_c_bindings() \
    .with_ros(ros_config)

meta = og.config.OptimizerMeta()\
    .with_optimizer_name("mpc_controller")

solver_config = og.config.SolverConfiguration()\
    .with_tolerance(1e-5)

builder = og.builder.OpEnOptimizerBuilder(problem,
                                          meta,
                                          build_config,
                                          solver_config)

builder.build()