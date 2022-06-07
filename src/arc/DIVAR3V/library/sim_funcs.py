import numpy as np
from pydrake.all import (
    DiagramBuilder, AddMultibodyPlantSceneGraph, Parser,
    RigidTransform, RotationMatrix, InverseKinematics,
    Solve, JacobianWrtVariable, InverseDynamicsController,
    Simulator, LogVectorOutput
)
from joblib import Parallel, delayed
from problems.DIVAR3V.library.urdf_creator import *
# from urdf_creator import *
import os

# TODO:!!!
# Needs to be checked, the compuation is way too fast,
# something could be wrong

def compute_bottom_up_mappings(dv):
    # 1. Create the required URDF
    urdf_path = urdf_creator(dv)
    # urdf_path = './diva_teleop_boxy.urdf'
    # 2. Setup pydrake simulation
    builder = DiagramBuilder()
    # Create plant
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-4)
    model = Parser(plant, scene_graph).AddModelFromFile(urdf_path, 'diva_robot')
    # Transform for the robot location
    X_R = RigidTransform(RotationMatrix.MakeYRotation(-np.pi/2), np.array([-0.1, 0.5, 1]))
    plant.WeldFrames(plant.world_frame(), plant.get_body(plant.GetBodyIndices(model)[0]).body_frame(), X_R)
    plant.Finalize()
    # Build the diagram
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    # Get end-effector
    gripper = plant.GetBodyByName("eef")
    gripper_frame = gripper.body_frame()
    # Get plant_context
    plant_context = plant.GetMyMutableContextFromRoot(context)


    # REACHABILITY
    # Compute inverse kinematics for random initial and final samples
    # init_position = np.array([np.random.uniform(-0.17, -0.695), np.random.uniform(-0.381, 0.381), np.random.uniform(0.74, 0.80)])
    # Some points seem never to be reachable so the sampling space is reduced
    # init_position = np.array([np.random.uniform(-0.17, -0.695), np.random.uniform(-0.1, 0.381), np.random.uniform(0.74, 0.80)])
    # Simplify by fixing the initial position
    init_position = np.array([-0.35, 0.15, 0.8])
    # FINAL ALWAYS FAILS CHECK WHY
    # final_position = np.array([np.random.uniform(-0.17, 0), np.random.uniform(-0.17, 0.17), np.random.uniform(0.74, 0.90)])
    # Simplify by fixing the final position
    final_position = np.array([0,0,0.8])
    # angle_sample = np.array([np.random.uniform(np.pi/2.0, 3.0*np.pi/2.0), np.random.uniform(0, np.pi/2.0)])
    # R1 = RotationMatrix.MakeZRotation(angle_sample[0])
    # R2 = RotationMatrix.MakeYRotation(angle_sample[1])
    # final_orientation = R1.multiply(R2)
    # final_position = final_orientation.multiply(np.array([1, 0, 0])) + np.array([0,0,0.74])

    # Check the reachability for init_position
    # Get initial guess
    q0 = plant.GetPositions(plant_context)
    # Create a program for ik computation
    ik = InverseKinematics(plant, plant_context)
    # Add the position as constraint
    ik.AddPositionConstraint(gripper_frame, [0, 0, 0], plant.world_frame(),
            init_position, init_position)
    prog = ik.get_mutable_prog()
    q = ik.q()
    prog.AddQuadraticErrorCost(np.identity(len(q)), q0, q)
    prog.SetInitialGuess(q, q0)
    result = Solve(ik.prog())
    initial_position = result.GetSolution(ik.q())
    # if result.is_success():
    #     print("Init: IK success: ", result.GetSolution(ik.q()))
    # else:
    #     print("Init: IK failure")

    if result.is_success():
        # Check the reachability for final_position
        ik = InverseKinematics(plant, plant_context)
        ik.AddPositionConstraint(gripper_frame, [0, 0, 0], plant.world_frame(),
                final_position, final_position)
        # ik.AddOrientationConstraint(gripper_frame, RotationMatrix(),
        #         plant.world_frame(), final_orientation, 0.0)
        prog = ik.get_mutable_prog()
        q = ik.q()
        prog.AddQuadraticErrorCost(np.identity(len(q)), q0, q)
        prog.SetInitialGuess(q, q0)
        result = Solve(ik.prog())
        if result.is_success():
            # print("Final: IK success: ", result.GetSolution(ik.q()))
            reachability = 1
            final_joint_position = result.GetSolution(ik.q())
            # print(reachability, final_joint_position)
        else:
            # print("Final: IK failure")
            reachability = -1
    else:
        reachability = -1


    # APPLIED_FORCE
    G = plant.GetBodyByName("eef").body_frame()
    W = plant.world_frame()
    if reachability == 1:
        # If we can reach the final position, set that as the robots position
        plant.SetPositions(plant.GetMyContextFromRoot(context), model, final_joint_position)
        # Now hopefully the computed jacobian is at this pose

        J_G = plant.CalcJacobianSpatialVelocity(plant_context, JacobianWrtVariable.kQDot, G, [0,0,0], W, W)
        joint_torques = np.matmul(J_G.transpose(), np.array([0,0,-10,0,0,0]))
        # Verify required vs realised joint torques
        if np.max(joint_torques) > dv[-1]:
            applied_force = -1
        else:
            applied_force = 1
    else:
        # Since reachability is anyway bad in this case
        applied_force = 1


    # CHECK STATIC PAYLOAD (Ref. to the old code)
    if (reachability==1) and (applied_force==1):
        fully_stretched = np.array([0,-1.57,0,0,0,0,0])
        plant.SetPositions(plant.GetMyContextFromRoot(context), model,
                      fully_stretched)
        J_G = plant.CalcJacobianSpatialVelocity(plant_context, JacobianWrtVariable.kQDot, G, [0,0,0], W, W)
        # Lift a weight of 10N (1kg payload)
        joint_torques = np.matmul(J_G.transpose(), np.array([-10,0,0,0,0,0]))
        # Verify required vs realised joint torques
        if np.max(joint_torques) > dv[-1]:
            payload = -1
        else:
            payload = 1
    else:
        payload = 1

    # SETTLING TIME
    ts = 4/(np.sqrt(dv[-2]))


    # DYNAMIC MOVEMENT TO REACH
    if (reachability==1) and (applied_force==1) and (payload==1):
        builder = DiagramBuilder()
        # Create plant
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-4)
        model = Parser(plant, scene_graph).AddModelFromFile(urdf_path, 'diva_robot')
        # Transform for the robot location
        X_R = RigidTransform(RotationMatrix.MakeYRotation(-np.pi/2), np.array([-0.1, 0.5, 1]))
        plant.WeldFrames(plant.world_frame(), plant.get_body(plant.GetBodyIndices(model)[0]).body_frame(), X_R)
        plant.Finalize()

        # Get end-effector
        gripper = plant.GetBodyByName("eef")
        gripper_frame = gripper.body_frame()
        # Get plant_context
        plant_context = plant.GetMyMutableContextFromRoot(context)

        Kp = np.full(7, dv[-2])
        # Ki = 2 * np.sqrt(Kp)
        Ki = np.full(7, 0)
        Kd = 2 * np.sqrt(Kp)
        # Wire-up the controller
        controller = builder.AddSystem(InverseDynamicsController(plant, Kp, Ki, Kd, False))
        controller.set_name("ID_controller");
        builder.Connect(plant.get_state_output_port(model),
                    controller.get_input_port_estimated_state())
        builder.Connect(controller.get_output_port_control(),
                    plant.get_actuation_input_port())
        # Connecting a data logger
        logger1 = LogVectorOutput(controller.get_output_port_control(), builder)
        # logger2 = LogVectorOutput(plant.get_state_output_port(model), builder)
        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        plant_context = plant.GetMyMutableContextFromRoot(context)

        # Set the initial and desired locations
        qd = final_joint_position
        q0 = initial_position
        xd = np.hstack((qd, 0*qd))
        plant.SetPositions(plant_context, q0)
        controller.GetInputPort('desired_state').FixValue(
            controller.GetMyMutableContextFromRoot(context), xd)

        # Run simulation
        simulator = Simulator(diagram, context)
        simulator.AdvanceTo(ts)
        # Read data from logs
        log1 = logger1.FindLog(context)
        # Get max torque used for motion
        # print('simulated data, max torque: ', np.max(log1.data()))
        if np.max(log1.data()) > dv[-1]:
            motion_feasibility = -1
        else:
            motion_feasibility = 1
    else:
        # Since reachability is anyway bad in this case
        motion_feasibility = 1
    # motion_feasibility = 1

    # Clean created URDFs so that it doesnt fill up memory
    os.remove(urdf_path)

    return reachability, applied_force, payload, ts, motion_feasibility

    # Again, there is no dynamic simulation involved here,
    # which will tell us if max torque during simulation is reached

# dv = [0.3085,0.3767,0.5785,0.3253,0.8768,2256,0.0707,0.0395,34.1140,13.3674]
# compute_bottom_up_mappings(dv)
