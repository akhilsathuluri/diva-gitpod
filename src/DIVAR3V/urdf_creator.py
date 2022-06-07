# URDF creator
# Update: Adding joint and a simple sphere to represent it
# Only joint mass is a design variable and the rotor inertia and the peak torque
# are computed from that value
# Radius of the joint is also fixed as it doesnt affect the simulation in anyway
# Irrespective of the radius the kinematic length remains the same

# !!! This needs to be modified into the dictionary
# way of passing the variables

from odio_urdf import *
import numpy as np
import sys

# Utils
def list2space(com):
    return str(com[0])+' '+str(com[1])+' '+str(com[2])

# Sample incoming data dictionary
# {'carti': array([-0.1 ,  0.14,  0.85]),
#  'cartd': array([-0.1 ,  0.32,  1.29]),
#  'tau_mu': 0.0053,
#  'b0': 1.5e-05,
#  'Km': 0.19,
#  'k_velocity': 1.0606085407643906,
#  'tau_max_m': 1.2,
#  'omega_max_m': 4750,
#  'gear_ratio': 10,
#  'Kp': 15,
#  'Kd': 7.745966692414834,
#  'm_m': 0.452,
#  'a1': 0.36,
#  'lua': 0.36,
#  'a23_distr': 0.5,
#  'lfa': 0.36,
#  'a45_distr': 0.5,
#  'a7': 0.1}

# Density, side, length for a square cros-section link
# Assuming solid chunks of metal --> Which will need replacement
# from Lukas' meta models
def calc_mass(rho, s, l):
    return rho*s**2*l

# Assuming square cross-section links !!!
def calc_inertia(m, s, l):
    # About the CoM the link is along the Z-axis
    iab = np.zeros([3,3])
    iab[0,0] = (1/12.0)*m*(l**2+s**2)
    iab[1,1] = (1/6.0)*m*(s**2)
    iab[2,2] = (1/12.0)*m*(l**2+s**2)
    return iab

def urdf_creator(sampled_dv, path = 'None'):
    dv = sampled_dv.copy()
    pre_val = {'rho':1050, 'sw':0.05, 'sh':0.1}
    dv.update(pre_val)
    other_dvs = {'a2':dv['lua']*dv['a23_distr'], 'a3': dv['lua']*(1-dv['a23_distr']),'a4':dv['lfa']*dv['a45_distr'], 'a5': dv['lfa']*(1-dv['a45_distr']), 'a6': 0.001}
    dv.update(other_dvs)
    # print(dv)
    # materials
    joint_radius = 0.05
    materials = Group(
                Material("blue", Color(rgba="0 0 0.8 1.0")),
                Material("red", Color(rgba="0.8 0 0 1.0")))

    # mass (can be multiplied in common)
    m = [calc_mass(dv['rho'], dv['sh'], dv['a1']),
         calc_mass(dv['rho'], dv['sh'], dv['a2']),
         calc_mass(dv['rho'], dv['sh'], dv['a3']),
         calc_mass(dv['rho'], dv['sh'], dv['a4']),
         calc_mass(dv['rho'], dv['sh'], dv['a5']),
         calc_mass(dv['rho'], dv['sh'], dv['a6']),
         calc_mass(dv['rho'], dv['sh'], dv['a7']),]
    ixx = [calc_inertia(m[0], dv['sh'], dv['a1']),
           calc_inertia(m[1], dv['sh'], dv['a2']),
           calc_inertia(m[2], dv['sh'], dv['a3']),
           calc_inertia(m[3], dv['sh'], dv['a4']),
           calc_inertia(m[4], dv['sh'], dv['a5']),
           calc_inertia(m[5], dv['sh'], dv['a6']),
           calc_inertia(m[6], dv['sh'], dv['a7']),]

    name_tag = np.random.randint(11, 9999999999)
    robot_name = "divar3v"+str(name_tag)
    # Declare the robot architecture
    my_robot = Robot(robot_name, materials)

    my_robot(
        # base_link
        # No need of inertial for the base_link as it is fixed
        Link(
             Visual(
             Origin(rpy= "0 0 0",xyz= "0 0 0"),
             Material(name= "red"),
             Geometry(
             Cylinder(length=0.05,radius=0.1),
              )),
              name= "base_link"),

        # Joint-1
        Joint("joint1",
              Parent("base_link"),
              Child("link1"),
              Origin(xyz=list2space([0,0,0]), rpy=list2space([0,0,0])),
              Axis(xyz="0 0 1"),
              type="revolute"),

        # Link-1
        Link(
            Inertial(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,dv['a1']/2])),
             Mass(value= str(m[0])),
             Inertia(
                ixx= str(ixx[0][0, 0]),
                ixy= str(ixx[0][0, 1]),
                ixz= str(ixx[0][0, 2]),
                iyy= str(ixx[0][1, 1]),
                iyz= str(ixx[0][1, 2]),
                izz= str(ixx[0][2, 2])
                ),
            ),
             Visual(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,dv['a1']/2])),
             Material(name= "blue"),
             Geometry(
                 Box(size=list2space([dv['sw'], dv['sh'], dv['a1']])),
                  )),
              name = "link1"),

        # Joint-joint-1
        Joint("joint-joint-1",
              Parent("link1"),
              Child("joint-link-1"),
              Origin(xyz=list2space([0,0,dv['a1']]), rpy="1.5707 0 0"),
              Axis(xyz="1 0 0"),
              type="fixed"),

        # Joint-link-1
        Link(
            Inertial(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,0])),
             Mass(value= str(dv['m_m'])),
             Inertia(
                ixx= str(0),
                ixy= str(0),
                ixz= str(0),
                iyy= str(0),
                iyz= str(0),
                izz= str(0)
                ),
            ),
             Visual(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,0])),
             Material(name= "red"),
             Geometry(
                 Sphere(radius=joint_radius),
                  )),
              name = "joint-link-1"),

        # Joint-2
        Joint("joint2",
              Parent("link1"),
              Child("link2"),
              Origin(xyz=list2space([0,0,dv['a1']]), rpy="1.5707 0 0"),
              Axis(xyz="1 0 0"),
              type="revolute"),

        # Link-2
        Link(
            Inertial(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,dv['a2']/2])),
             Mass(value= str(m[1])),
             Inertia(
                ixx= str(ixx[1][0, 0]),
                ixy= str(ixx[1][0, 1]),
                ixz= str(ixx[1][0, 2]),
                iyy= str(ixx[1][1, 1]),
                iyz= str(ixx[1][1, 2]),
                izz= str(ixx[1][2, 2])
                ),
            ),
             Visual(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,dv['a2']/2])),
             Material(name= "blue"),
             Geometry(
                 Box(size=list2space([dv['sw'], dv['sh'], dv['a2']])),
                  )),
              name = "link2"),

        # Joint-joint-2
        Joint("joint-joint-2",
              Parent("link2"),
              Child("joint-link-2"),
              Origin(xyz=list2space([0,0, dv['a2']]), rpy="1.5707 0 0"),
              Axis(xyz="1 0 0"),
              type="fixed"),

        # Joint-link-2
        Link(
            Inertial(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,0])),
             Mass(value= str(dv['m_m'])),
             Inertia(
                ixx= str(0),
                ixy= str(0),
                ixz= str(0),
                iyy= str(0),
                iyz= str(0),
                izz= str(0)
                ),
            ),
             Visual(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,0])),
             Material(name= "red"),
             Geometry(
                 Sphere(radius=joint_radius),
                  )),
              name = "joint-link-2"),


        # Joint-3
        Joint("joint3",
              Parent("link2"),
              Child("link3"),
              Origin(xyz=list2space([0,0, dv['a2']]), rpy="0 0 0"),
              Axis(xyz="0 0 1"),
              type="revolute"),

        # Link-3
        Link(
            Inertial(
             Origin(rpy= "0 0 0",xyz=list2space([0,0, dv['a3']/2])),
             Mass(value= str(m[2])),
             Inertia(
                ixx= str(ixx[2][0, 0]),
                ixy= str(ixx[2][0, 1]),
                ixz= str(ixx[2][0, 2]),
                iyy= str(ixx[2][1, 1]),
                iyz= str(ixx[2][1, 2]),
                izz= str(ixx[2][2, 2])
                ),
            ),
             Visual(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,dv['a3']/2])),
             Material(name= "blue"),
             Geometry(
                 Box(size=list2space([dv['sw'], dv['sh'], dv['a3']])),
                  )),
              name = "link3"),

        # Joint-joint-3
        Joint("joint-joint-3",
              Parent("link3"),
              Child("joint-link-3"),
              Origin(xyz=list2space([0,0,dv['a3']]), rpy="1.5707 0 0"),
              Axis(xyz="1 0 0"),
              type="fixed"),

        # Joint-link-3
        Link(
            Inertial(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,0])),
             Mass(value= str(dv['m_m'])),
             Inertia(
                ixx= str(0),
                ixy= str(0),
                ixz= str(0),
                iyy= str(0),
                iyz= str(0),
                izz= str(0)
                ),
            ),
             Visual(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,0])),
             Material(name= "red"),
             Geometry(
                 Sphere(radius=joint_radius),
                  )),
              name = "joint-link-3"),

        # Joint-4
        Joint("joint4",
              Parent("link3"),
              Child("link4"),
              Origin(xyz=list2space([0,0,dv['a3']]), rpy="0 0 0"),
              Axis(xyz="0 1 0"),
              type="revolute"),

        # Link-4
        Link(
            Inertial(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,dv['a4']/2])),
             Mass(value= str(m[3])),
             Inertia(
                ixx= str(ixx[3][0, 0]),
                ixy= str(ixx[3][0, 1]),
                ixz= str(ixx[3][0, 2]),
                iyy= str(ixx[3][1, 1]),
                iyz= str(ixx[3][1, 2]),
                izz= str(ixx[3][2, 2])
                ),
            ),
             Visual(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,dv['a4']/2])),
             Material(name= "blue"),
             Geometry(
                 Box(size=list2space([dv['sw'], dv['sh'], dv['a4']])),
                  )),

              name = "link4"),

        # Joint-joint-4
        Joint("joint-joint-4",
              Parent("link4"),
              Child("joint-link-4"),
              Origin(xyz=list2space([0,0,dv['a4']]), rpy="1.5707 0 0"),
              Axis(xyz="1 0 0"),
              type="fixed"),

        # Joint-link-4
        Link(
            Inertial(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,0])),
             Mass(value= str(dv['m_m'])),
             Inertia(
                ixx= str(0),
                ixy= str(0),
                ixz= str(0),
                iyy= str(0),
                iyz= str(0),
                izz= str(0)
                ),
            ),
             Visual(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,0])),
             Material(name= "red"),
             Geometry(
                 Sphere(radius=joint_radius),
                  )),
              name = "joint-link-4"),

        # Joint-5
        Joint("joint5",
              Parent("link4"),
              Child("link5"),
              Origin(xyz=list2space([0,0, dv['a4']]), rpy="0 0 0"),
              Axis(xyz="0 0 1"),
              type="revolute"),

        # Link-5
        Link(
            Inertial(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,dv['a5']/2])),
             Mass(value= str(m[4])),
             Inertia(
                ixx= str(ixx[4][0, 0]),
                ixy= str(ixx[4][0, 1]),
                ixz= str(ixx[4][0, 2]),
                iyy= str(ixx[4][1, 1]),
                iyz= str(ixx[4][1, 2]),
                izz= str(ixx[4][2, 2])
                ),
            ),
             Visual(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,dv['a5']/2])),
             Material(name= "blue"),
             Geometry(
                 Box(size=list2space([dv['sw'], dv['sh'], dv['a5']])),
                  )),
              name = "link5"),

        # Joint-joint-5
        Joint("joint-joint-5",
              Parent("link5"),
              Child("joint-link-5"),
              Origin(xyz=list2space([0,0,dv['a5']]), rpy="1.5707 0 0"),
              Axis(xyz="1 0 0"),
              type="fixed"),

        # Joint-link-5
        Link(
            Inertial(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,0])),
             Mass(value= str(dv['m_m'])),
             Inertia(
                ixx= str(0),
                ixy= str(0),
                ixz= str(0),
                iyy= str(0),
                iyz= str(0),
                izz= str(0)
                ),
            ),
             Visual(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,0])),
             Material(name= "red"),
             Geometry(
                 Sphere(radius=joint_radius),
                  )),
              name = "joint-link-5"),


        # The wrist part
        # Joint-6
        Joint("joint6",
              Parent("link5"),
              Child("link6"),
              Origin(xyz=list2space([0,0,dv['a5']]), rpy="0 0 0"),
              Axis(xyz="0 1 0"),
              type="revolute"),

        # Link-6
        Link(
            Inertial(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,dv['a6']/2])),
             Mass(value= str(m[5])),
             Inertia(
                 ixx= str(ixx[5][0, 0]),
                 ixy= str(ixx[5][0, 1]),
                 ixz= str(ixx[5][0, 2]),
                 iyy= str(ixx[5][1, 1]),
                 iyz= str(ixx[5][1, 2]),
                 izz= str(ixx[5][2, 2])
                 ),
             ),
             Visual(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,dv['a6']/2])),
             Material(name= "blue"),
             Geometry(
                 Box(size=list2space([dv['sw'], dv['sh'], dv['a6']])),
                  )),
              name = "link6"),

        # Joint-7
        Joint("joint7",
              Parent("link6"),
              Child("link7"),
              Origin(xyz=list2space([0,0,dv['a6']]), rpy="0 0 0"),
              Axis(xyz="1 0 0"),
              type="revolute"),

        # Link-7
        Link(
            Inertial(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,dv['a7']/2])),
             Mass(value= str(m[6])),
             Inertia(
                  ixx= str(ixx[6][0, 0]),
                  ixy= str(ixx[6][0, 1]),
                  ixz= str(ixx[6][0, 2]),
                  iyy= str(ixx[6][1, 1]),
                  iyz= str(ixx[6][1, 2]),
                  izz= str(ixx[6][2, 2])
                  ),
              ),
             Visual(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,dv['a7']/2])),
             Material(name= "blue"),
             Geometry(
                 Box(size=list2space([dv['sw'], dv['sh'], dv['a7']])),
                  )),
              name = "link7"),

        # end-effector
        Joint("eef_joint",
              Parent("link7"),
              Child("eef"),
              Origin(xyz=list2space([0,0,dv['a7']]), rpy="0 0 0"),
              type="fixed"),

        # end-effector frame with 1kg payload
        Link(
            Inertial(
             Origin(rpy= "0 0 0",xyz="0 0 0"),
             Mass(value= str(1)),
             Inertia(
                ixx= str(0),
                ixy= str(0),
                ixz= str(0),
                iyy= str(0),
                iyz= str(0),
                izz= str(0)
                ),
            ),
              name = "eef"),

        # Adding transmission for pydrake simulation
        Transmission(Type("SimpleTransmission"),
            Actuator(Mechanicalreduction('1'), name = "motor1"),
            Transjoint("joint1"),
            name = "motor1"),

        Transmission(Type("SimpleTransmission"),
            Actuator(Mechanicalreduction('1'), name = "motor2"),
            Transjoint("joint2"),
            name = "motor2"),

        Transmission(Type("SimpleTransmission"),
            Actuator(Mechanicalreduction('1'), name = "motor3"),
            Transjoint("joint3"),
            name = "motor3"),

        Transmission(Type("SimpleTransmission"),
            Actuator(Mechanicalreduction('1'), name = "motor4"),
            Transjoint("joint4"),
            name = "motor4"),

        Transmission(Type("SimpleTransmission"),
            Actuator(Mechanicalreduction('1'), name = "motor5"),
            Transjoint("joint5"),
            name = "motor5"),

        Transmission(Type("SimpleTransmission"),
            Actuator(Mechanicalreduction('1'), name = "motor6"),
            Transjoint("joint6"),
            name = "motor6"),

        Transmission(Type("SimpleTransmission"),
            Actuator(Mechanicalreduction('1'), name = "motor7"),
            Transjoint("joint7"),
            name = "motor7"),
    )


    # Creating a URDF model from the python to be loaded to the
    # urdf_path = './urdfs/AutoGen/'+robot_name+'.urdf'
    # print(urdf_path)
    # sourcefile = open(urdf_path,'w')
    # print(my_robot, file=sourcefile)
    # sourcefile.close()
    # sys.stdout.flush()

    # with open(urdf_path,'w') as f:
    #     print(my_robot, file=f)
        # Also add the DV sample
        # f.write('<!-- ' + str(np.array(sampled_dv)) +  '-->')


    # return urdf_path, dv
    return str(my_robot), dv

# urdf_creator(sample_dv)
