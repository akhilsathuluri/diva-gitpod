# URDF creator
# Update: Adding joint and a simple sphere to represent it
# Only joint mass is a design variable and the rotor inertia and the peak torque
# are computed from that value
# Radius of the joint is also fixed as it doesnt affect the simulation in anyway
# Irrespective of the radius the kinematic length remains the same

from odio_urdf import *
import numpy as np
# Declare constants
# Xacroproperty(name= "pi",value= "3.14159265359")
# Utils
def list2space(com):
    return str(com[0])+' '+str(com[1])+' '+str(com[2])

# a1, lua, a23_distr, lfa, a45_distr, rho, s2, s4, K, taumax
# sample_dv = [0.332,0.3655,0.5,0.335,0.5,1292.5,0.07,0.0396, 5.5, 22.775]

# Map from sampled dv to actual urdf params
def dv_sampled2dv_urdf(dv):
    return [dv[0], dv[1]*dv[2], dv[1]*(1-dv[2]), dv[3]*dv[4], dv[3]*(1-dv[4]), dv[5], dv[6], dv[7], dv[8], dv[9]]

# Density, side, length for a square cros-section link
# Assuming solid chunks of metal --> Which will need replacement
# from Lukas' meta models
def calc_mass(rho, s, l):
    return rho*s**2*l

# Assuming square cross-section links
def calc_inertia(m, s, l):
    ixx = np.zeros([3,3])
    ixx[0,0] = 1/12.0*m*(l**2+s**2)
    ixx[1,1] = 1/6.0*m*(s**2)
    ixx[2,2] = 1/12.0*m*(l**2+s**2)
    return ixx

def urdf_creator(sampled_dv):
    dv = dv_sampled2dv_urdf(sampled_dv)
    # Setup robot parameters
    # materials
    materials = Group(
                Material("blue", Color(rgba="0 0 0.8 1.0")),
                Material("red", Color(rgba="0.8 0 0 1.0"))
    )

    rho = dv[5]
    s = dv[6:8]
    a = dv[:5]
    # Temporary lengths for link6 and 7
    templ = [0.01, 0.1]
    tempm = [calc_mass(rho, s[1], templ[0]), calc_mass(rho, s[1], templ[1])]
    tempixx = [calc_inertia(tempm[0], s[0], templ[0]), calc_inertia(tempm[1], s[1], templ[1])]
    # mass (can be multiplied in common)
    m = [calc_mass(rho, s[0], a[0]),calc_mass(rho, s[0], a[1]),calc_mass(rho, s[0], a[2]), \
            calc_mass(rho, s[1], a[3]),calc_mass(rho, s[1], a[4])]
    ixx = [calc_inertia(m[0], s[0], a[0]), calc_inertia(m[1], s[0], a[1]), \
            calc_inertia(m[2], s[0], a[2]), calc_inertia(m[3], s[1], a[3]), \
            calc_inertia(m[4], s[1], a[4])]

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
             Origin(rpy= "0 0 0",xyz=list2space([0,0,a[0]/2])),
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
             Origin(rpy= "0 0 0",xyz=list2space([0,0,a[0]/2])),
             Material(name= "blue"),
             Geometry(
                 Box(size=list2space([s[0], s[0], a[0]])),
                  )),
            Collision(
              Origin(rpy= "0 0 0",xyz=list2space([0,0,a[0]/2])),
              Geometry(
              Box(size=list2space([s[0], s[0], a[0]])),
               )),
              name = "link1"),

        # Joint-2
        Joint("joint2",
              Parent("link1"),
              Child("link2"),
              Origin(xyz=list2space([0,0,a[0]]), rpy="1.5707 0 0"),
              Axis(xyz="1 0 0"),
              type="revolute"),

        # Link-2
        Link(
            Inertial(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,a[1]/2])),
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
             Origin(rpy= "0 0 0",xyz=list2space([0,0,a[1]/2])),
             Material(name= "blue"),
             Geometry(
                 Box(size=list2space([s[0], s[0], a[1]])),
                  )),
            Collision(
              Origin(rpy= "0 0 0",xyz=list2space([0,0,a[1]/2])),
              Geometry(
              Box(size=list2space([s[0], s[0], a[1]])),
               )),
              name = "link2"),

        # Joint-3
        Joint("joint3",
              Parent("link2"),
              Child("link3"),
              Origin(xyz=list2space([0,0,a[1]]), rpy="0 0 0"),
              Axis(xyz="0 0 1"),
              type="revolute"),

        # Link-3
        Link(
            Inertial(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,a[2]/2])),
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
             Origin(rpy= "0 0 0",xyz=list2space([0,0,a[2]/2])),
             Material(name= "blue"),
             Geometry(
                 Box(size=list2space([s[0], s[0], a[2]])),
                  )),
            Collision(
              Origin(rpy= "0 0 0",xyz=list2space([0,0,a[2]/2])),
              Geometry(
              Box(size=list2space([s[0], s[0], a[2]])),
               )),
              name = "link3"),

        # Joint-4
        Joint("joint4",
              Parent("link3"),
              Child("link4"),
              Origin(xyz=list2space([0,0,a[2]]), rpy="0 0 0"),
              Axis(xyz="0 1 0"),
              type="revolute"),

        # Link-4
        Link(
            Inertial(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,a[3]/2])),
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
             Origin(rpy= "0 0 0",xyz=list2space([0,0,a[3]/2])),
             Material(name= "blue"),
             Geometry(
                 Box(size=list2space([s[1], s[1], a[3]])),
                  )),
            Collision(
              Origin(rpy= "0 0 0",xyz=list2space([0,0,a[3]/2])),
              Geometry(
              Box(size=list2space([s[1], s[1], a[3]])),
               )),
              name = "link4"),

        # Joint-5
        Joint("joint5",
              Parent("link4"),
              Child("link5"),
              Origin(xyz=list2space([0,0,a[3]]), rpy="0 0 0"),
              Axis(xyz="0 0 1"),
              type="revolute"),

        # Link-5
        Link(
            Inertial(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,a[4]/2])),
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
             Origin(rpy= "0 0 0",xyz=list2space([0,0,a[4]/2])),
             Material(name= "blue"),
             Geometry(
                 Box(size=list2space([s[1], s[1], a[4]])),
                  )),
            Collision(
              Origin(rpy= "0 0 0",xyz=list2space([0,0,a[4]/2])),
              Geometry(
              Box(size=list2space([s[1], s[1], a[4]])),
               )),
              name = "link5"),

        # The wrist part
        # Joint-6
        Joint("joint6",
              Parent("link5"),
              Child("link6"),
              Origin(xyz=list2space([0,0,a[4]]), rpy="0 0 0"),
              Axis(xyz="0 1 0"),
              type="revolute"),

        # Link-6
        Link(
            Inertial(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,templ[0]/2])),
             Mass(value= str(tempm[0])),
             Inertia(
                ixx= str(tempixx[0][0, 0]),
                ixy= str(tempixx[0][0, 1]),
                ixz= str(tempixx[0][0, 2]),
                iyy= str(tempixx[0][1, 1]),
                iyz= str(tempixx[0][1, 2]),
                izz= str(tempixx[0][2, 2])
                ),
            ),
             Visual(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,templ[0]/2])),
             Material(name= "blue"),
             Geometry(
                 Box(size=list2space([s[1], s[1], templ[0]])),
                  )),
            Collision(
              Origin(rpy= "0 0 0",xyz=list2space([0,0,templ[0]/2])),
              Geometry(
              Box(size=list2space([s[1], s[1], templ[0]])),
               )),
              name = "link6"),

        # Joint-7
        Joint("joint7",
              Parent("link6"),
              Child("link7"),
              Origin(xyz=list2space([0,0,templ[0]]), rpy="0 0 0"),
              Axis(xyz="1 0 0"),
              type="revolute"),

        # Link-7
        Link(
            Inertial(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,templ[1]/2])),
             Mass(value= str(tempm[1])),
             Inertia(
                ixx= str(tempixx[1][0, 0]),
                ixy= str(tempixx[1][0, 1]),
                ixz= str(tempixx[1][0, 2]),
                iyy= str(tempixx[1][1, 1]),
                iyz= str(tempixx[1][1, 2]),
                izz= str(tempixx[1][2, 2])
                ),
            ),
             Visual(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,templ[1]/2])),
             Material(name= "blue"),
             Geometry(
                 Box(size=list2space([s[1], s[1], templ[1]])),
                  )),
            Collision(
              Origin(rpy= "0 0 0",xyz=list2space([0,0,templ[1]/2])),
              Geometry(
              Box(size=list2space([s[1], s[1], templ[1]])),
               )),
              name = "link7"),

        # end-effector
        Joint("eef_joint",
              Parent("link7"),
              Child("eef"),
              Origin(xyz=list2space([0,0,templ[1]]), rpy="0 0 0"),
              type="fixed"),

        # end-effector frame
        Link(name = "eef"),

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
    urdf_path = './problems/DIVAR3V/library/urdfs/'+robot_name+'.urdf'
    # urdf_path = './urdfs/'+robot_name+'.urdf'
    # Reference URDF: divar3v6384210051
    # urdf_path = './urdfs/diva_boxy_transmission.urdf'
    with open(urdf_path,'w') as f:
        print(my_robot, file=f)

    return urdf_path

# urdf_creator(sample_dv)
