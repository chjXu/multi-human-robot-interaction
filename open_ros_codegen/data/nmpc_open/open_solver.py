import casadi as cs
import opengen as og
import numpy as np
from numpy import *
# import PyKDL
import math

# make x(state) = [q, q.dot] 

JOINT_NUM = 6 #关节数
ee_var = 12
end_var = 3
dis = 0.3 #安全距离
obs_num = 3
N = 5 # The MPC horizon length
NX = 2 # The number of elements in the state vector
NU = 1 # The number of elements in the control vector
NM = 6 # The number of elements in the obs and robot vector
sampling_time = 0.16 #sampling time

R_k = cs.DM.eye(JOINT_NUM) * [2.0]
Q_pos = cs.DM.eye(end_var) * [6.0]
Q_roa = cs.DM.eye(end_var) * [10.0]
QN_pos = cs.DM.eye(end_var) * [20.0]
QN_roa = cs.DM.eye(end_var) * [20.0]
miu = cs.DM.eye(NM) * [0.1]

# DH参数
pai = 3.1415926

d1 = 89.459
d2 = 0
d3 = 0
d4 = 109.15
d5 = 94.65
d6 = 82.3
a1 = 0
a2 = -425
a3 = -392.25
a4 = 0
a5 = 0
a6 = 0
alpha1 = pai / 2
alpha2 = 0
alpha3 = 0
alpha4 = pai / 2
alpha5 = -pai / 2
alpha6 = 0
theta_1 = 0
theta_2 = 0
theta_3 = 0
theta_5 = 0
theta_6 = 0

# base


# T_0_1
def T_0_1(theta_1):
    return [[cos(theta_1), 0, sin(theta_1),  0]
               [sin(theta_1), 0, -cos(theta_1), 0]
               [0           , 1,          0,   d1]
               [0           , 0,          0,    1]]

# T_1_2
def T_1_2(theta_2):
    return [[cos(theta_2), -sin(theta_2),  0, a2*cos(theta_2)]
            [sin(theta_2),  cos(theta_2),  0, a2*sin(theta_2)]
            [           0,             0,  1,               0]
            [           0,             0,  0,               1]]

# T_2_3
def T_2_3(theta_3):
    return [[cos(theta_3), -sin(theta_3),  0, a3*cos(theta_3)]
            [sin(theta_3),  cos(theta_3),  0, a3*sin(theta_3)]
            [           0,             0,  1,               0]
            [           0,             0,  0,               1]]

# T_3_4
def T_3_4(theta_4):
    return [[cos(theta_4), 0,  sin(theta_4), 0]
            [sin(theta_4), 0, -cos(theta_4), 0]
            [           0, 1,             0,d4]
            [           0, 0,             0, 1]]

# T_4_5
def T_4_5(theta_5):
    return [[cos(theta_5), 0, -sin(theta_5), 0]
            [sin(theta_5), 0,  cos(theta_5), 0]
            [           0,-1,             0,d5]
            [           0, 0,             0, 1]]

# T_5_6
def T_5_6(theta_6):
    return [[cos(theta_6), -sin(theta_6), 0, 0]
            [sin(theta_6),  cos(theta_6), 0, 0]
            [           0,             0, 1,d6]
            [           0,             0, 0, 1]]



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

def u_error(_u, _u_ref):
    for i in range(0, JOINT_NUM):
        du += _u[i] - _u_ref[i]
    return du                       #6x1

def end_error(_ee, _ee_ref):
    pos_error = [_ee[-1] - _ee_ref[-1], _ee[-2] - _ee_ref[-2], _ee[-3] - _ee_ref[-3]]

    cur_roa = [[_ee[0], _ee[1], _ee[2]],
               [_ee[3], _ee[4], _ee[5]],
               [_ee[6], _ee[7], _ee[8]]]

    print(cur_roa)
    ref_roa = [[_ee_ref[0], _ee_ref[1], _ee_ref[2]],
               [_ee_ref[3], _ee_ref[4], _ee_ref[5]],
               [_ee_ref[6], _ee_ref[7], _ee_ref[8]]]
    roa_error = np.diag(cur_roa.T * ref_roa) - 1.0
    return roa_error, pos_error                #3x1


# 误差2
# The stage cost for x and u
def stage_cost(_ee, _u, _ee_ref=None, _u_ref=None):
    if _ee_ref is None:
        _ee_ref = cs.DM.zeros(_ee.shape)
        
    if _u_ref is None:
        _u_ref = cs.DM.zeros(_u.shape)
    
    dr, dp = end_error(_ee, _ee_ref)
    du = u_error(_u, _u_ref)
    return cs.mtimes([dp.T, Q_pos, dp]) + cs.mtimes([du.T, R_k, du]) + cs.mtimes([dr.T, Q_roa, dr])

# 误差1
# The terminal cost for x
def terminal_cost(_ee, _ee_ref=None):
    if _ee_ref is None:
        _ee_ref = cs.DM.zeros(_x.shape)
    dr, dp = end_error(_ee, _ee_ref)
    return cs.mtimes([dr.T, QN_roa, dr]) + cs.mtimes([dp.T, QN_pos, dp])
 
# 误差3
# The stage cost for robot state
def state_cost(x_t, u_t):
    return cs.vcat([x_t[0] + x_t[1] * sampling_time + 0.5 * u_t * sampling_time * sampling_time,
                    x_t[1] + u_t * sampling_time])

# 范数
def __state_cost():
    pass

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
ee_0 = cs.MX.sym("ee_0", ee_var)
ee_ref = cs.MX.sym("ee_ref", ee_var)
obs = cs.MX.sym("obs", obs_num)
u_k = [cs.MX.sym('u_' + str(i), JOINT_NUM) for i in range(N)]
# x_k = [cs.MX.sym('x_' + str(i), NX) for i in range(N)]
# lamata_k = [cs.MX.sym('l_' + str(i), JOINT_NUM) for i in range(N)] 
# miu_k = [cs.MX.sym('m_' + str(i), JOINT_NUM) for i in range(N)]

# create the cost function
ee_t = ee_0
total_cost = 0

F = 0
for t in range(0, N):
    total_cost += stage_cost(ee_t, u_k[t], ee_ref) # update cost
    x_t = dynamics_dt(x_t, u_k[t]) # update state
    F += cs.fmax(0, 1- (ee_t[-1]-0.6)**2 - ee_t[-2]**2 - (ee_t[-3]-0.2)**2)
    # h_x_u = lamata * x_k
    # total_cost += h_x_u

total_cost += terminal_cost(x_t, x_ref)

# total_cost += __state_cost(x_t, u_k)
# total_cost += obs_cost(x_k, miu_k, u_k)
# total_cost += __obs_cost(x_k, miu_k, u_k)

# optimization
optimization_variables = []
optimization_parameters = []

optimization_variables += u_k
optimization_parameters += [ee_0]
optimization_parameters += [ee_ref]
#optimization_parameters += [obs]

optimization_variables = cs.vertcat(*optimization_variables)
optimization_parameters = cs.vertcat(*optimization_parameters)

#constraint
umin = [-0.3] * (JOINT_NUM * N)
umax = [0.3] * (JOINT_NUM * N)
point = [0.6, 0.0, 0.2]

# x_k_1 = None
# for i in range(0, N):
#     x_k_1 += cs.vertcat([x_t[0] + x_t[1] * sampling_time + 0.5 * u_k[i] * sampling_time * sampling_time,
#                     x_t[1] + u_k[i] * sampling_time])

# pos = PyKDL.

# safety_dis = cs.vertcat(cs.fmin(dis, pos-obs))
# safety_dis = og.constraints.Ball2(point, dis)
#C = og.constraints.BallInf(None, dis)

bounds = og.constraints.Rectangle(umin, umax)

problem = og.builder.Problem(optimization_variables,
                             optimization_parameters,
                             total_cost) \
    .with_constraints(bounds)
    #.with_aug_lagrangian_constraints(F, C)\
    

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
    .with_max_outer_iterations(10)\
    .with_tolerance(1e-5)\
    .with_penalty_weight_update_factor(8.0)

builder = og.builder.OpEnOptimizerBuilder(problem,
                                          meta,
                                          build_config,
                                          solver_config)

builder.build()