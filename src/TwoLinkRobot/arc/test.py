# Imports
import numpy as np
import altair as alt
import pydot
from IPython.display import display, SVG, clear_output, HTML
import matplotlib.pyplot as plt
from pydrake.all import *
from manipulation import running_as_notebook
from manipulation.meshcat_cpp_utils import(
    StartMeshcat, MeshcatJointSlidersThatPublish)
from manipulation.scenarios import AddMultibodyTriad

# Import pygmo for optimisation
import pygmo as pg
import pandas as pd
import time
from TwoLink_urdf_creator import *

# Define the number of actuators
na = 2

# GLOBAL PROBLEM SETUP

# Reference: https://wiki.ros.org/pr2_controller_manager/safety_limits
# Defining the actuator box
class ActuatorBox(LeafSystem):
    # Input values are for motor and NOT for the joint
    def __init__(self, tau_max_m, omega_max_m, gear_ratio, k_velocity):
        LeafSystem.__init__(self)
        # Both the values are motor specific and NOT joint specific
        # Get joint max torque
        self.tau_max_j = tau_max_m*gear_ratio
        self.k_vel = k_velocity
        # Convert RPM of motor to joint velocity in rad/sec
        self.omega_max_j = omega_max_m*(2*np.pi/60)/gear_ratio
        self.DeclareVectorInputPort('commanded_torque', BasicVector(na))
        self.DeclareVectorInputPort('joint_state', BasicVector(2*na))
        self.DeclareVectorOutputPort('realised_torque', BasicVector(na), self.OutputJointTorque)

    def OutputJointTorque(self, context, output):
        # Get the state values
        q = self.EvalVectorInput(context, 1).get_value()
        qvel = q[na:]
        u = self.EvalVectorInput(context, 0).get_value()
        # Compute the instantaneous torque limit
        # For the AK-70-10 motors
        kv = self.k_vel
        u_new = np.copy(u)
        # Using URDF safety controller rules
        for i in range(len(u)):
            if qvel[i]>=0 and u[i]>=0:
                u_bound = -kv*(qvel[i]-self.omega_max_j)
                u_new[i] = np.min([u[i], u_bound, self.tau_max_j])
            if qvel[i]>=0 and u[i]<=0:
                u_bound = -self.tau_max_j
                u_new[i] = np.max([u[i], u_bound])
            if qvel[i]<=0 and u[i]>=0:
                u_bound = self.tau_max_j
                u_new[i] = np.min([u[i], u_bound])
            if qvel[i]<=0 and u[i]<=0:
                u_bound = -(-kv*(np.abs(qvel[i])-self.omega_max_j))
                u_new[i] = np.max([u[i], u_bound, -self.tau_max_j])
#             print(u[i], u_bound, u_new[i])
        output.SetFromVector(u_new)

# Task definition
# Home position
carti = np.array([-0.22, 0.14, 1.00])
qi = np.array([0.93, 0.45])
cartd = np.array([0.18, 0.15, 1.00])
qd = np.array([0.01, 0.22])
qint = qd

# Simulation settings
time_step = 1e-4
sim_time = 5
error_threshold = 5e-3

# Robot URDF
urdf_path = './urdfs/TwoLinkRobot.urdf'
print(urdf_path)

# motor mass
m_m = 0.452
# Assumed values
tau_mu = 5.3e-3
b0 = 1.5e-5
# AK70-10 parameters
# gear ratio
gear_ratio = 10
# motor design variables
tau_max_m = 12/gear_ratio # Nm
omega_max_m = 475*gear_ratio   # RPM
Kt = 0.095        # Nm/A
Kv = 100          # RPM/V
Km = 0.19         # Nm/sqrt(W)
# Computed matrix for joule loss
K_joule = 1/np.sqrt(Km)*np.identity(na)
k_velocity = 8.33/(75*2*np.pi/60) # Nm/(rad/s)
# Control attributes
Kp = 15
Kd = 2*np.sqrt(Kp)

study = ["baseline_control", "baseline_motor", "baseline_codesign", "baseline_codesign_heuristics", "baseline"]

case = study[0]
if case=="baseline_control":
    # For baseline-control study
    pre_val_vals = [tau_mu, b0, Km, k_velocity, tau_max_m, omega_max_m, gear_ratio, m_m]
    pre_val_names = ["tau_mu", "b0", "Km", "k_velocity", "tau_max_m", "omega_max_m", "gear_ratio", "m_m"]
    # Define a dictionary for ease of usage and modulartiy of the dvs
    pre_val = dict(zip(pre_val_names, pre_val_vals))
    dv_names = ["Kp", "Kd"]
    print(case)
    print('pre_val: ', pre_val, 'dv_names: ', dv_names)
    sample_dv = [15, 2*np.sqrt(15)]
    print(sample_dv)
    dv_bounds = ([0,0],[100,100])
    print('dv_bounds: ', dv_bounds)

if case=="baseline_motor":
    # For baseline-control-motor study
    pre_val_vals = []
    pre_val_names = []
    print(case)
    # Define a dictionary for ease of usage and modulartiy of the dvs
    pre_val = dict(zip(pre_val_names, pre_val_vals))
    dv_names = ["tau_mu", "b0", "Km", "k_velocity", "tau_max_m", "omega_max_m", "gear_ratio", "Kp", "Kd", "m_m"]
    print('pre_val: ', pre_val, 'dv_names: ', dv_names)
    sample_dv = [0.0053, 1.5e-05, 0.19, 1.0606085407643906, 1.2, 4750, 10, 15, 2*np.sqrt(15), 0.452]
    print('Sample-DV: ', sample_dv)
    # Since we want to change the motor mass in the simulation here, we must include joint mass variation in the URDF
    # ['tau_mu', 'b0', 'Km', 'k_velocity', 'tau_max_m', 'omega_max_m', 'gear_ratio', 'Kp', 'Kd', 'm_m']
    dv_bounds = ([0,0,0,1e-2,1e-3,1000,0.1, 0, 0, 0],[1,1,5,1e2,5,9000,25,100,100,5])
    print('dv_bounds: ', dv_bounds)

if case=="baseline_codesign":
    # For baseline-codesign study
    pre_val_vals = []
    pre_val_names = []
    print(case)
    # Define a dictionary for ease of usage and modulartiy of the dvs
    pre_val = dict(zip(pre_val_names, pre_val_vals))
    dv_names = ["tau_mu", "b0", "Km", "k_velocity", "tau_max_m", "omega_max_m", "gear_ratio", "Kp", "Kd", "m_m", "l1", "l2"]
    print('pre_val: ', pre_val, 'dv_names: ', dv_names)
    sample_dv = [0.0053, 1.5e-05, 0.19, 1.0606085407643906, 1.2, 4750, 10, 15, 2*np.sqrt(15), 0.452, 0.36, 0.36]
    print('Sample-DV: ', sample_dv)
    # Since we want to change the motor mass in the simulation here, we must include joint mass variation in the URDF
    # ['tau_mu', 'b0', 'Km', 'k_velocity', 'tau_max_m', 'omega_max_m', 'gear_ratio', 'Kp', 'Kd', 'm_m']
    dv_bounds = ([0,0,0,1e-2,1e-3,1000,0.1, 0, 0, 0,1e-2,1e-2],[1,1,5,1e2,5,9000,25,100,100,5,2,2])
    print('dv_bounds: ', dv_bounds)

if case=="baseline_codesign_heuristics":
    # For baseline-codesign study
    pre_val_names = ["tau_mu", "b0", "k_velocity"]
    pre_val_vals = [0.0053, 1.5e-05, 1.0606085407643906]
    print(case)
    # Define a dictionary for ease of usage and modulartiy of the dvs
    pre_val = dict(zip(pre_val_names, pre_val_vals))
    dv_names = ["omega_max_m", "gear_ratio", "Kp", "Kd", "m_m", "l1", "l2"]
    print('pre_val: ', pre_val, 'dv_names: ', dv_names)
    sample_dv = [4750, 10, 15, 2*np.sqrt(15), 0.452, 0.36, 0.36]
    print('Sample-DV: ', sample_dv)
    # Since we want to change the motor mass in the simulation here, we must include joint mass variation in the URDF
    # ['tau_mu', 'b0', 'Km', 'k_velocity', 'tau_max_m', 'omega_max_m', 'gear_ratio', 'Kp', 'Kd', 'm_m']
    dv_bounds = ([1000,0.1, 0, 0, 0,1e-2,1e-2],[9000,25,100,100,5,2,2])
    print('dv_bounds: ', dv_bounds)

if case=="baseline":
    print(case)
    # For simulation study
    pre_val_vals = [tau_mu, b0, Km, k_velocity, tau_max_m, omega_max_m, gear_ratio, Kp, Kd, m_m]
    pre_val_names = ["tau_mu", "b0", "Km", "k_velocity", "tau_max_m", "omega_max_m", "gear_ratio", "Kp", "Kd", "m_m"]
    # Define a dictionary for ease of usage and modulartiy of the dvs
    pre_val = dict(zip(pre_val_names, pre_val_vals))
    dv_names = []
    sample_dv = []
    print(pre_val)


def compute_QoI(pre_val, dv_names, sample_dv, case, run_sim=False):
    # Construct the master dv list
    dv_dict = dict(zip(dv_names, sample_dv))
    if case==study[3]:
        # Map the entire design space from the sampled DVs using heuristics
        # Current heuristics used are from the jump paper
        dv_dict['tau_max_m'] = 5.48*dv_dict['m_m']**0.97
        dv_dict['Km'] = 0.15*dv_dict['m_m']**1.39
        # Later also fit k_velocity values
    dv = {**pre_val, **dv_dict}
    # Construct a new URDF if its such a case
    print(case)
    if (case==study[1]) or (case==study[2]) or (case==study[3]):
        urdf_path=urdf_creator(dv)
    else:
        urdf_path = './urdfs/TwoLinkRobot.urdf'
    # Construct the sim diagram
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
    model = Parser(plant, scene_graph).AddModelFromFile(urdf_path, 'diva_robot')
    X_R = RigidTransform(RotationMatrix.MakeYRotation(0), np.array([-0.1, 0.5, 1]))
    plant.WeldFrames(plant.world_frame(), plant.get_body(plant.GetBodyIndices(model)[0]).body_frame(), X_R)
    if run_sim:
        print(urdf_path)
        meshcat.Delete()
        meshcat.DeleteAddedControls()
        # Spawn table
        table = Parser(plant, scene_graph).AddModelFromFile('./urdfs/table/extra_heavy_duty_table_modified.sdf','table')

    # Due to the way the problem is formulated, spheres and the table can be safely ignored
    plant.Finalize()
    actuator = builder.AddSystem(ActuatorBox(dv['tau_max_m'], dv['omega_max_m'], dv['gear_ratio'], dv['k_velocity']))

    if run_sim:
        visualizer = MeshcatVisualizerCpp.AddToBuilder(builder, scene_graph, meshcat)

    # Initialise controller params, for now limited to Kp and Kd search
    Kp = np.full(na, dv['Kp'])
    Ki = np.full(na, 0)
    Kd = np.full(na, dv['Kd'])

    iiwa_controller = builder.AddSystem(InverseDynamicsController(plant, Kp, Ki, Kd, False))
    iiwa_controller.set_name("iiwa_controller");
    # Complete connections
    builder.Connect(plant.get_state_output_port(model),
                iiwa_controller.get_input_port_estimated_state())
    builder.Connect(iiwa_controller.get_output_port_control(),
                actuator.get_input_port(0))
    builder.Connect(plant.get_state_output_port(model),
                actuator.get_input_port(1))
    builder.Connect(actuator.get_output_port(), plant.get_actuation_input_port())

    # Connecting a data logger
    # Commanded torque
    logger1 = LogVectorOutput(iiwa_controller.get_output_port_control(), builder)
    # Joint state
    logger2 = LogVectorOutput(plant.get_state_output_port(model), builder)
    # Realised torque
    logger3 = LogVectorOutput(actuator.get_output_port(), builder)

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(context)
    gripper_frame = plant.GetBodyByName("eef").body_frame()

    # Now computing the QoIs
    # WORKSPACE
    # Workspace is always satisfied since its the same robot
    workspace = 0
    # DEFLECTION
    # Deflection is not applicable as we consider only rigid components
    deflection = 0
    # APPLIED FORCE
    # Applied force will also remain the same given that the tau_max is fixed
    applied_force = 0
    # PAYLOAD
    # same as applied force
    payload = 0
    # Setup the simulation
    xd = np.hstack((qint, 0*qint))
    plant.SetPositions(plant_context, qi)
    iiwa_controller.GetInputPort('desired_state').FixValue(
    iiwa_controller.GetMyMutableContextFromRoot(context), xd);
    # Simulation
    simulator = Simulator(diagram, context)
    simulator.AdvanceTo(sim_time);

    # Now we want to get the simulation data, i.e.,
    # the torque applied, final time of the joint variables
    log1 = logger1.FindLog(context)
    log2 = logger2.FindLog(context)
    log3 = logger3.FindLog(context)

    error = np.abs(log2.data()[:na,:].transpose() - np.tile(qint,(log3.data().shape[1],1)))
    e_norm = np.array([np.linalg.norm(val) for val in error])
    for i in range(len(e_norm)):
        if np.all(e_norm[i:]<error_threshold):
            break
    final_error = e_norm[-1]
    # CYCLE TIME
    ts = log2.sample_times()[i]

    omega_j = []
    for i in range(log2.data().shape[1]):
        omega_j.append(log2.data()[:,i][na:])
    omega_j = np.array(omega_j).transpose()
    omega_m = omega_j*dv['gear_ratio']
    # Convert the motor speed to RPM
    omega_m_rpm = omega_m*60/(2*np.pi)
    # Motor torque realised
    tau_m = log3.data()/dv['gear_ratio']

    K_joule = 1/np.sqrt(dv['Km'])*np.identity(na)
    # Total power losses
    inst_friction_power = []
    inst_joule_power = []
    for i in range(len(log3.sample_times())):
        omega_inst = omega_m[:, i]
        tau_f = dv['tau_mu']*np.sign(omega_inst)+dv['b0']*omega_inst
        P_f = np.dot(tau_f, omega_inst)
        inst_friction_power.append(P_f)
        tau_t = tau_f+tau_m[:, i]
        P_t = np.dot(K_joule.dot(tau_t), tau_t)
        inst_joule_power.append(P_t)

    # POWER LOSSES
    total_friction_loss = np.sum(inst_friction_power)*time_step
    total_joule_loss = np.sum(inst_joule_power)*time_step

    results = np.array([workspace, deflection, applied_force, payload, ts, total_friction_loss, total_joule_loss, final_error])
#     print(results)

    if run_sim:
        # Angular angular velocity of the motors in RPM
        motor_rpm = omega_j*gear_ratio*60/(2*np.pi)
        plt.plot(log2.sample_times(), motor_rpm[0, :], log2.sample_times(), motor_rpm[1, :])
        plt.title('motor RPM vs time')

        # Motor torque demanded
        fig = plt.figure()
        plt.plot(log1.sample_times(), log1.data()[0, :], log1.sample_times(), log1.data()[1, :])
        plt.title("motor torque demanded vs time")

        # Motor torque demanded
        fig = plt.figure()
        plt.plot(log1.sample_times(), log3.data()[0, :], log1.sample_times(), log3.data()[1, :])
        plt.title("motor torque realised vs time")

        print('Max realised torque', np.max(np.abs(log3.data())))

    return results


# Setting up a problem
class Baseline_Problem:
    def __init__(self, pre_val, dv_names, dv_bounds, case):
        self.pre_val = pre_val
        self.dv_names = dv_names
        self.dv_bounds = dv_bounds
        self.case = case
        pass

    def fitness(self,sample_dv):
#         print(sample_dv)
        # Write a function to output the QoI values
        results = compute_QoI(self.pre_val, self.dv_names, sample_dv, self.case)
        # Add weighting factor, K_w
        # workspace, deflection, applied_force, payload, ts, total_friction_loss, total_joule_loss, final_error
        K_w = np.diag([1,1,1,1,100,5,5,0])
        # Quadratic objective meta function with weights
        fitness_val = np.matmul(K_w, results).dot(results)
        return [fitness_val]

    def get_bounds(self):
        # Define bounds on the controller variables
        # Ideally from a csv file like before so that one can easily extend the optimisation problem
        return self.dv_bounds

    def get_name(self):
        # get the name of the problem also from the csv sheet
        return "Baseline_Control"

prob = pg.problem(Baseline_Problem(pre_val, dv_names, dv_bounds, case))
algo = pg.algorithm(pg.cmaes(gen = 1, sigma0=0.3))
archi = pg.archipelago(n=32, algo = algo, prob=prob, pop_size=500)
archi.evolve(); archi.wait()

a = archi.get_champions_f()
a2 = sorted(archi.get_champions_f(), key = lambda x: x[0])[0]
best_isl_idx = [(el == a2).all() for el in a].index(True)
x_best = archi.get_champions_x()[best_isl_idx]
f_best = archi.get_champions_f()[best_isl_idx]
print("Best relaxed solution, x: {}".format(x_best))
print("Best relaxed solution, f: {}".format(f_best))
