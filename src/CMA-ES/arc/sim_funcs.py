import numpy as np
from pydrake.all import *
# from problems.DIVAR3V.library.urdf_creator import *
from urdf_creator import *
import os
import pandas as pd

class ActuatorBox(LeafSystem):
    def __init__(self, tau_max_dv, inst_power_max):
        LeafSystem.__init__(self)
        self.tau_max = tau_max_dv
        self.inst_power_max = inst_power_max
        self.DeclareVectorInputPort('commanded_torque', BasicVector(7))
        self.DeclareVectorInputPort('joint_state', BasicVector(14))
        self.DeclareVectorOutputPort('realised_torque', BasicVector(7), self.OutputJointTorque)

    def OutputJointTorque(self, context, output):
        u = self.EvalVectorInput(context, 0).get_value()
        # Enforcing saturation with joint effort
        u = np.array(u)
        # Bound control input with tau_max
        u[u>self.tau_max]=self.tau_max
        u[u<-self.tau_max]=-self.tau_max
        # Also check for instantaneous power delivered
        q = self.EvalVectorInput(context, 1).get_value()
        # Extract joint velocity
        qvel = np.array(q)[7:]
        # We need Kt and Kv to compute the power
        # Assuming them to be constant for now
        Kt = 0.068
        Kv = 140
        # Compute inst power demanded
        inst_power = np.abs(np.multiply(u, qvel))/(Kt*Kv)
        # Find joints which violate power
        index = np.where(inst_power >= self.inst_power_max)
        for i in index:
            u[i] = self.inst_power_max/qvel[i]
        output.SetFromVector(u)

def compute_bottom_up_mappings(dv, path = 'None'):
    # print('Entering bottom-ups')
    # For ClassicOpt
    dv = list(dv)
    # Assign values into variables
    taumax = dv[6]
    pmax = dv[9]
    Kp_sample = dv[5]
    Kd_sample = dv[10]
    gear_ratio = dv[8]
    # Get final sim time, sim until upper bound of tcycle
    # qoi = pd.read_csv('../input/qoi_space.csv')
    # qoi = pd.read_csv('~/git/research/xray/src/problems/DIVAR3V/input/qoi_space.csv')
    qoi = pd.read_csv('./input/qoi_space.csv')
    sim_time = qoi[qoi.Variables.str.match('tcycle')].Upper[0]
    # Set the initial and final position of the robot
    carti = np.array([-0.65, 0.2, 0.8])
    cartd = np.array([-0.25, 0.05,0.45])
    error_threshold = 5e-3
    # Friction torque
    tau_mu = 5.3e-3
    b0 = 1.5e-5
    # Motor constant matrix
    km = 0.14
    # 1. Create the required URDF
    urdf_path = urdf_creator(dv, path)
    # 2. Setup pydrake simulation
    builder = DiagramBuilder()
    # Create plant
    time_step = 1e-4
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
    model = Parser(plant, scene_graph).AddModelFromFile(urdf_path, 'diva_robot')
    # Transform for the robot location
    X_R = RigidTransform(RotationMatrix.MakeYRotation(-np.pi/2), np.array([-0.1, 0.5, 1]))
    plant.WeldFrames(plant.world_frame(), plant.get_body(plant.GetBodyIndices(model)[0]).body_frame(), X_R)
    plant.Finalize()
    # Initialise the acutator module
    actuator = builder.AddSystem(ActuatorBox(taumax, pmax))
    # Add controller and build the system
    Kp = np.full(7, Kp_sample)
    Ki = np.full(7, 0)
    Kd = np.full(7, Kd_sample)
    iiwa_controller = builder.AddSystem(InverseDynamicsController(plant, Kp, Ki, Kd, False))
    iiwa_controller.set_name("iiwa_controller");
    builder.Connect(plant.get_state_output_port(model),
                    iiwa_controller.get_input_port_estimated_state())
    # Add a motor characteristics box here to define how the motor output would be based on the commanded control
    builder.Connect(iiwa_controller.get_output_port_control(),
                    actuator.get_input_port(0))
    builder.Connect(plant.get_state_output_port(model),
                    actuator.get_input_port(1))
    builder.Connect(actuator.get_output_port(),  plant.get_actuation_input_port())
    # Connecting a data logger
    # Joint state
    logger2 = LogVectorOutput(plant.get_state_output_port(model), builder)
    # Realised torque
    logger3 = LogVectorOutput(actuator.get_output_port(), builder)
    diagram = builder.Build()
    # Get end-effector
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(context)
    q0 = plant.GetPositions(plant_context)
    gripper_frame = plant.GetBodyByName("eef").body_frame()

    # REACHABILITY
    # Check the reachability for init_position
    # Create a program for ik computation
    # Compute IK
    ik = InverseKinematics(plant, plant_context)
    ik.AddPositionConstraint(
                gripper_frame, [0, 0, 0], plant.world_frame(),
                carti, carti)
    prog = ik.get_mutable_prog()
    q = ik.q()
    prog.AddQuadraticErrorCost(np.identity(len(q)), q0, q)
    prog.SetInitialGuess(q, q0)
    result = Solve(ik.prog())
    if result.is_success():
        qinit = result.GetSolution(ik.q())
        # Condition the final angles to be between -pi and pi
        qinit = (np.arctan2(np.sin(qinit), np.cos(qinit)))
        # Check the reachability for final_position
        ik = InverseKinematics(plant, plant_context)
        ik.AddPositionConstraint(
                    gripper_frame, [0, 0, 0], plant.world_frame(),
                    cartd, cartd)
        prog = ik.get_mutable_prog()
        q = ik.q()
        prog.AddQuadraticErrorCost(np.identity(len(q)), q0, q)
        prog.SetInitialGuess(q, q0)
        result = Solve(ik.prog())
        if result.is_success():
            qd = result.GetSolution(ik.q())
            # Condition the final angles to be between -pi and pi
            qd = (np.arctan2(np.sin(qd), np.cos(qd)))
            workspace = 1

        else:
            workspace = -1
            # Make sure some random final_joint_position is not selected
            qd = np.NaN
            # If reachability is not satisfied do not compute further
            applied_force = 1
            payload = 1
            total_joule_loss = 0
            total_friction_loss = 0
            ts = 1
            # Clean created URDFs so that it doesnt fill up memory
            os.remove(urdf_path)
            return workspace, applied_force, payload, ts, total_friction_loss, total_joule_loss
    else:
        workspace = -1
        # Make sure some random final_joint_position is not selected
        qinit = np.NaN
        qd = np.NaN
        # If reachability is not satisfied do not compute further
        applied_force = 1
        payload = 1
        total_joule_loss = 0
        total_friction_loss = 0
        ts = 1
        # Clean created URDFs so that it doesnt fill up memory
        os.remove(urdf_path)
        return workspace, applied_force, payload, ts, total_friction_loss, total_joule_loss

    # APPLIED_FORCE
    # If the code reaches here => reachability is True
    G = plant.GetBodyByName("eef").body_frame()
    W = plant.world_frame()
    # If we can reach the final position, set that as the robots position
    plant.SetPositions(plant.GetMyContextFromRoot(context), model, qd)
    J_G = plant.CalcJacobianSpatialVelocity(plant_context, JacobianWrtVariable.kQDot, G, [0,0,0], W, W)
    joint_torques = np.matmul(J_G.transpose(), np.array([0,0,-10,0,0,0]))
    # Verify required vs realised joint torques
    if np.max(np.abs(joint_torques)) > taumax:
        applied_force = -1
        # Do not proceed if applied force fails
        payload = 1
        total_joule_loss = 0
        total_friction_loss = 0
        ts = 1
        # Clean created URDFs so that it doesnt fill up memory
        os.remove(urdf_path)
        return workspace, applied_force, payload, ts, total_friction_loss, total_joule_loss
    else:
        applied_force = 1

    # CHECK STATIC PAYLOAD (Ref. to the old code)
    fully_stretched = np.array([0,-1.57,0,0,0,0,0])
    plant.SetPositions(plant.GetMyContextFromRoot(context), model,
                  fully_stretched)
    J_G = plant.CalcJacobianSpatialVelocity(plant_context, JacobianWrtVariable.kQDot, G, [0,0,0], W, W)
    # Lift a weight of 10N (1kg payload)
    joint_torques = np.matmul(J_G.transpose(), np.array([-10,0,0,0,0,0]))
    # Verify required vs realised joint torques
    if np.max(np.abs(joint_torques)) > taumax:
        payload = -1
        # Do not proceed if payload fails
        total_joule_loss = 0
        total_friction_loss = 0
        ts = 1
        # Clean created URDFs so that it doesnt fill up memory
        os.remove(urdf_path)
        return workspace, applied_force, payload, ts, total_friction_loss, total_joule_loss
    else:
        payload = 1

    # DYNAMIC MOVEMENT TO REACH
    # Set the initial and desired locations
    xd = np.hstack((qd, 0*qd))
    plant.SetPositions(plant_context, qinit)
    iiwa_controller.GetInputPort('desired_state').FixValue(
        iiwa_controller.GetMyMutableContextFromRoot(context), xd)

    # Run simulation
    simulator = Simulator(diagram, context)
    # print(dv)
    simulator.AdvanceTo(sim_time)
    # Read data from logs
    # log1 = logger1.FindLog(context)
    log2 = logger2.FindLog(context)
    log3 = logger3.FindLog(context)

    # Check if we reached the desired position
    error = np.abs(log2.data()[:7,:].transpose() - np.tile(qd,(log3.data().shape[1],1)))
    e_norm = np.array([np.linalg.norm(val) for val in error])
    if e_norm[-1] > error_threshold:
        ts = -1
        total_joule_loss = 0
        total_friction_loss = 0
        # Clean created URDFs so that it doesnt fill up memory
        os.remove(urdf_path)
        return workspace, applied_force, payload, ts, total_friction_loss, total_joule_loss
    else:
        for i in range(len(e_norm)):
            if np.all(e_norm[i:]<error_threshold):
                ts = log2.sample_times()[i]
                break

    # ACCOUNTING FOR POWER LOSSES if others are also okay to save comp time
    # Define constants for friction loss
    # Motor angular velocity
    omega = []
    for i in range(log2.data().shape[1]):
        omega.append(log2.data()[:,i][7:])
    omega_m = np.array(omega).transpose()*gear_ratio
    # Motor torque realised
    tau_m = log3.data()/gear_ratio
    # Costant matrix for tau->joule_loss
    K = 1/np.sqrt(km)*np.identity(7)
    # Total power loss
    inst_friction_power = []
    inst_joule_power = []
    for i in range(len(log3.sample_times())):
        omega_inst = omega_m[:, i]
        tau_f = tau_mu*np.sign(omega_inst)+b0*omega_inst
        P_f = np.dot(tau_f, omega_inst)
        inst_friction_power.append(P_f)
        tau_t = tau_f+tau_m[:, i]
        P_t = np.dot(K.dot(tau_t), tau_t)
        inst_joule_power.append(P_t)
    # Normalising to step size, since its an integration
    # If the step size is constant, this can be ignored
    total_friction_loss = np.sum(inst_friction_power)*time_step
    total_joule_loss = np.sum(inst_joule_power)*time_step
    # Clean created URDFs so that it doesnt fill up memory
    os.remove(urdf_path)
    return workspace, applied_force, payload, ts, total_friction_loss, total_joule_loss

# dv = [0.3085,0.3767,0.5785,0.3253,0.8768,2256,0.0707,0.0395,34.1140,13.3674,0.8,6]
# a1, lua, a23_distr, lfa, a45_distr, K, taumax, m_m, N, Pmax
# dv = [0.332,0.3655,0.5,0.335,0.5, 5.5, 22.775, 0.8, 10, 150]
# compute_bottom_up_mappings(dv)
