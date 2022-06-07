# TwoLink URDF creator

from odio_urdf import *
import numpy as np
import onnxruntime
import torch
from operator import itemgetter

# Utils
def list2space(com):
    return str(com[0])+' '+str(com[1])+' '+str(com[2])

def calc_mass(rho, sh, sw, l):
    return rho*sh*sw*l

def to_numpy(tensor):
    return tensor.detach().cpu().numpy() if tensor.requires_grad else tensor.cpu().numpy()

# Assuming square cross-section links
def calc_inertia(m, s, l):
    # About the CoM the link is along the Z-axis
    iab = np.zeros([3,3])
    iab[0,0] = (1/12.0)*m*(l**2+s**2)
    iab[1,1] = (1/6.0)*m*(s**2)
    iab[2,2] = (1/12.0)*m*(l**2+s**2)
    return iab

def urdf_creator(sampled_dv, fit_params, path = 'None'):
    # Prescibed values that dont change: Density, section width, and section height
    pre_val = {'rho':1050, 'sw':0.05, 'sh':0.1, 'l1':0.36, 'l2':0.36}
    # Mass estimation
    ort_session = onnxruntime.InferenceSession('./mass_estimator.onnx', providers=['CPUExecutionProvider'])
    mass_domain_scale = 4.0/9.0
    mass_mean = np.array(fit_params['mass_data']['mean'])
    mass_scale = np.array(fit_params['mass_data']['scale'])
    link_lengths = np.array(itemgetter('l1', 'l2')(sampled_dv))
    kvals = itemgetter('k1', 'k2', 'k3')(sampled_dv)
    input_list = np.c_[np.array([kvals,]*2), link_lengths*1000]
    input_list_torch = torch.from_numpy(input_list)
    norm_input_list = to_numpy((input_list_torch-mass_mean[:4])/mass_scale[:4])
    ort_pred_mass_frac_list = []
    for i in range(len(link_lengths)):
        ort_inputs_list = {ort_session.get_inputs()[0].name:norm_input_list[i].astype(np.float32).reshape(1,-1)}
        ort_pred_mass_frac_list.append(ort_session.run(None, ort_inputs_list))
    pred_mass_frac_list = (np.array(ort_pred_mass_frac_list)*mass_scale[4]+mass_mean[4]).flatten()
    m = (pred_mass_frac_list/100*pre_val['rho']*0.005*mass_domain_scale*link_lengths)

    # Combined dvs with pre_vals
    # If it is a co-design study then all the variables  link-lengths
    # are overwritten from the sampled DV. two dicts with same key results
    # in later one to be the selected one
    dv = {**pre_val, **sampled_dv}

    # materials
    joint_radius = 0.05
    materials = Group(
                Material("blue", Color(rgba="0 0 0.8 1.0")),
                Material("red", Color(rgba="0.8 0 0 1.0"))
    )

    # --------------------------------------------------------------
    # UNSCALABLE LINES!!!
    # m = [calc_mass(dv['rho'], dv['sh'], dv['sw'], dv['l1']), calc_mass(dv['rho'], dv['sh'], dv['sw'], dv['l1'])]
    ixx = [calc_inertia(m[0], dv['sw'], dv['l1']), calc_inertia(m[0], dv['sw'], dv['l2'])]
    # --------------------------------------------------------------

    name_tag = np.random.randint(11, 9999999999)
    robot_name = "TwoLinkStruct"+str(name_tag)
    # Declare the robot architecture
    my_robot = Robot(robot_name, materials)

    my_robot(
        # base_link
        # No need of inertial for the base_link as it is fixed
        Link(
             Visual(
             Origin(rpy= "0 1.5707 0",xyz= "0 0 0"),
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
              Axis(xyz="1 0 0"),
              type="revolute"),

        # Link-1
        Link(
            Inertial(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,dv['l1']/2])),
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
             Origin(rpy= "0 0 0",xyz=list2space([0,0,dv['l1']/2])),
             Material(name= "blue"),
             Geometry(
                 Box(size=list2space([dv['sw'], dv['sh'], dv['l1']])),
                  )),
              name = "link1"),

        # Joint-joint-1
        Joint("joint-joint-1",
              Parent("link1"),
              Child("joint-link-1"),
              Origin(xyz=list2space([0,0,dv['l1']]), rpy="1.5707 0 0"),
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
              Origin(xyz=list2space([0,0,dv['l1']]), rpy="1.5707 0 0"),
              Axis(xyz="1 0 0"),
              type="revolute"),

        # Link-2
        Link(
            Inertial(
             Origin(rpy= "0 0 0",xyz=list2space([0,0,dv['l2']/2])),
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
             Origin(rpy= "0 0 0",xyz=list2space([0,0,dv['l2']/2])),
             Material(name= "blue"),
             Geometry(
                 Box(size=list2space([dv['sw'], dv['sh'], dv['l2']])),
                  )),
            # Collision(
            #   Origin(rpy= "0 0 0",xyz=list2space([0,0,dv['l2']/2])),
            #   Geometry(
            #   Box(size=list2space([s[0], s[0], dv['l2']])),
            #    )),
              name = "link2"),

        # end-effector
        Joint("eef_joint",
              Parent("link2"),
              Child("eef"),
              Origin(xyz=list2space([0,0,dv['l2']]), rpy="0 0 0"),
              type="fixed"),

        # end-effector link
        # with a point mass for payload
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
    )

    urdf_path = './urdfs/TwoLinkAutoGen/'+robot_name+'.urdf'
    with open(urdf_path,'w') as f:

        print(my_robot, file=f)
        # Also add the DV sample
        f.write('<!-- ' + str(np.array(sampled_dv)) +  '-->')

    return urdf_path

# m_m
# sample_dv = {'m_m': 0.453}
# urdf_path = urdf_creator(sample_dv)
# print(urdf_path)
