#!/usr/bin/env python3

import rospy,rospkg
import argparse
from collections import namedtuple


import os,yaml
import numpy as np

from quartet_py import initialize
from harp_py import converter

from shared_control import Goal

from shared_control.arbitration import AssistanceDiscrete

from harp_arm_study_utils.loggers import RotatingLogFile,RosbagRecorder

from geometry_msgs.msg import Pose,Point,Quaternion

from shared_control import AdjSharedControlHdlr,load_arb_input_handlers


STUDY_VERSION_NAME = 'study_v1'
CONTROL_HZ = 50. # Shared control loop runs at this rate
INPUT_INTERFACE = 'generic_joystick'
NUM_INPUT_DOFS = 3 # How many axes of movement you can control at once with the input interface



ASSISTANCE_LEVELS = {
    0: np.array([0.4, 1.0, 0.0]),
    1: np.array([0.4, 0.8, 0.6]),
    2: np.array([0.4, 0.65, 0.75]),
    3: np.array([0.4, 0.55, 0.9]),
    4: np.array([0.4, 0.4, 1.0])
}



STUDY_ENV_FILE = os.path.join(rospkg.RosPack().get_path('assist_pref_grasp_demo'),'env_data/env_configs/pilot_v1.yaml')
STUDY_ENV_OFFSETS_FILE = os.path.join(rospkg.RosPack().get_path('assist_pref_grasp_demo'),'env_data/env_configs/pilot_v1_offsets.yaml')
STUDY_ENV_DIR = os.path.join(rospkg.RosPack().get_path('assist_pref_grasp_demo'),'env_data/models')


# Recorded data files live here after running
DEFAULT_LOG_DIR = os.path.join(rospkg.RosPack().get_path('assist_pref_grasp_demo'),'trial_data')


INPUT_OPTS = ['sim']
TELEOP_OPTS = ['max_linear_velocity','max_angular_velocity']
GOAL_OPTS = ['goal_objects']
OUTPUT_OPTS = ['record_data','log_dir','enable_vis']

class StudyConfig(namedtuple('StudyConfig',INPUT_OPTS + TELEOP_OPTS + GOAL_OPTS + OUTPUT_OPTS)):
    """
    Convenience class for storing trial running options

    
    Methods
    -------
    build_arg_parser(parser)
        Allows us to use the config keys as argument in an argument parser
    from_cmd_line_args(cls,args)
        Allows us to load config options in from command line
    from_yaml_file(fn)
        Allows us to load config options in from a yaml file
    from_parser(cls,args)
        Allows us to load config option in from the namespace of an argument parser
    create(cls, *args, **kwargs)
        Instatiation

    """
    __DEFAULT_OPTS__ = {
        'sim': True,
        'max_linear_velocity': None,
        'max_angular_velocity': None,
        'goal_objects': ['kinova_holder'],
        'record_data': False,
        'log_dir': DEFAULT_LOG_DIR,
        'enable_vis': True
    }

    @staticmethod
    def build_arg_parser(parser):
        """Allows us to use the config keys as arguments in an argument parser 
        
        Parameters
        ----------
        parser : argparse.ArgumentParser
            An argument parser that you probably instantiated in the main script

        Returns
        -------
        parser : argparse.ArgumentParser
            An argument parser with arguments added

        """
        parser.add_argument('--sim',help='run only in simulation',action='store_true')
        
        parser.add_argument('--max-linear-velocity',help='maximum linear velocity of end effector. used to scale the twist commands',type=float)
        parser.add_argument('--max-angular-velocity', help='maximum angular velocity of end effector. used to scale the twist commands',type=float)

        parser.add_argument('--goal-objects',help='specifies names of the goal objects. if using simulation, these names should match the model names used in gazebo. Will consider all objects in the environment goal objects if not set.',nargs='+',default=[])
        
        parser.add_argument('--record-data',help='enables data recording. specifically records robot states, user commands, auton commands, and applied commands. If relevant, also records goal distribution and/or blend factor',action='store_true')
        parser.add_argument('--log-dir',help='sets the directory in which to save recorded data. default is ' + DEFAULT_LOG_DIR,type=str,default=DEFAULT_LOG_DIR)
        parser.add_argument('--enable-vis',help='enables visualization of user, auton, and applied twists in rviz. If relevant, enables visualization of goal probabilities in rviz',action='store_true')

        return parser

    @classmethod
    def from_cmd_line_args(cls,args):
        """Allows us to load config options in from command line
        
        Parameters
        ----------
        args : Namespace
            Namespace probably returned from using your argument parser to parse args in the main script
        
        Returns
        -------
        config : StudyConfig
            An instance of StudyConfig

        """
        arg_dict = vars(args)

        config_dict = {k:v for k,v in arg_dict.items() if k in cls.__DEFAULT_OPTS__.keys() and v is not None}
        final_dict = {}
        for k,v in config_dict.items():
            if isinstance(v,(list,str)) and len(v) == 0:
                continue
            final_dict[k] = v
        config = cls.create(**final_dict)
        return config

    @staticmethod
    def from_yaml_file(fn):
        """Allows us to load config options in from a yaml file
        
        Parameters
        ----------
        fn : str
            Path of config file you're trying to load

        Returns
        -------
        dict
            Study config options as a dictionary

        """
        cfg = None
        if fn is not None and os.path.isfile(fn):
            with open(fn, 'r') as f:
                cfg = yaml.full_load(f)  # handle empty file
        if cfg is not None:
            for key,val in cfg['assistance_config'].items():
                if val == 'None':
                    cfg['assistance_config'][key] = None
            return cfg['assistance_config']
        else:
            return {}

    @classmethod
    def from_parser(cls,args):
        """Allows us to load config option in from the namespace of an argument parser
        
        Parameters
        ----------
        args : Namespace
            Namespace probably returned from using your argument parser to parse args in the main script
        
        Returns
        -------
        StudyConfig
            An instance of StudyConfig

        """
        arg_dict = vars(args)

        config_dict = {k:v for k,v in arg_dict.items() if k in cls.__DEFAULT_OPTS__.keys() and v is not None}
        final_dict = {}
        for k,v in config_dict.items():
            if isinstance(v,(list,str)) and len(v) == 0:
                continue
            final_dict[k] = v

        if len(args.config_file) > 0:
            yaml_dict = StudyConfig.from_yaml_file(args.config_file)
            if len(yaml_dict) > 0:
                for k,v in yaml_dict.items():
                    if not k in final_dict.keys():
                        final_dict[k] = v
                    elif v is not None:
                        final_dict[k] = v
        config = StudyConfig.create(**final_dict)
        args = config
        return config,args



    @classmethod
    def create(cls, *args, **kwargs):
        """Instatiation

        Parameters
        ----------
        *args : tuple, optional
            Name of the keys in __DEFAULT_OPTS__ to update
        **kwargs : dict, optional
            Dictionary of arguments and their values to update

        Returns
        -------
        config : StudyConfig
            An instance of StudyConfig

        """
        kwargs.update({k: v for k, v in zip(cls._fields, args)})
        vals = cls.__DEFAULT_OPTS__.copy()
        vals.update(kwargs)

        config = cls(**vals)

        return config

def compute_targets_from_offsets(obj_pose,offsets_dict):
    """Compute target (a.k.a. goal) poses by taking object poses and a dictionary of offsets

    Parameters
    ----------
    obj_pose : geometry_msgs.msg.Pose
        Pose of the object
    offsets_dict : dict
        Dictionary of offset values

    Returns
    -------
    targets : list
        List of target object poses as transformation matrices

    """
    og_position = np.array([obj_pose.position.x,obj_pose.position.y,obj_pose.position.z])

    targets = []

    for offset,angles in zip(offsets_dict['offsets'],offsets_dict['orientation']):
        new_position = og_position + np.array(offset)
        angles_rad = np.array(angles)*np.pi/180.
        new_quat = converter.eul_angles_to_quat([angles_rad[0],angles_rad[1],angles_rad[2]])

        target_pose = Pose()
        target_pose.position = Point(new_position[0],new_position[1],new_position[2])
        target_pose.orientation = Quaternion(new_quat[0],new_quat[1],new_quat[2],new_quat[3])

        target_transform = converter.to_transform_mat(target_pose)
        targets.append(target_transform)

    return targets

def load_goals(offsets_file,env,goal_object_names):
    """Load goal object information into Goal objects that the shared control code uses

    Parameters
    ----------
    offsets_file : str
        Path of file containing goal offset information
    env : Environment object from quartet_py code
        Contains information for the environment of the robot (e.g. objects with respect to robot base)
    goal_object_names : list of str
        Names assigned to each object in environment

    Returns
    -------
    goals : list of Goal objects
        List of Goal objects needed for shared control code

    """

    if offsets_file is not None and os.path.isfile(offsets_file):
        with open(offsets_file, 'r') as f:
            offsets_dict = yaml.full_load(f)
    obj_states = env.get_object_states()

    goals = []
    for name,pose,twist,frame in zip(*obj_states):
        if name not in goal_object_names:
            continue
        pose_as_transform_mat = converter.to_transform_mat(pose)

        target_poses = compute_targets_from_offsets(pose,offsets_dict[name])
        goals.append(Goal(name,pose_as_transform_mat,target_poses))

    return goals

if __name__ == "__main__":


    # Build argument parser
    parser = argparse.ArgumentParser(description="study_main_script")
    # Pass argument parser into StudyConfig class to add study option arguments to parser
    parser = StudyConfig.build_arg_parser(parser)

    # Add one more argument in case user passes study options in as a yaml file
    parser.add_argument('--config-file',help='path to config file (must be yaml). when set, will load arguments from the specified file',type=str,default='')

    argv = rospy.myargv()

    # Get namespace from argument parser
    args = parser.parse_args(argv[1:])

    # Load parsed arguments into study configuration class
    config,args = StudyConfig.from_parser(args)
    
    # Initialize this as a ROS node
    rospy.init_node(STUDY_VERSION_NAME,anonymous=True,log_level=rospy.INFO)
    
    # Get the robot and the environment objects
    robot,env = initialize('gen3',env_data_dir=STUDY_ENV_DIR,env_config_filename=STUDY_ENV_FILE,sim=args.sim,enable_rviz=True,enable_gazebo=False)
    
    # Send robot to home position
    robot.home_robot()

    # Load objects as instances of Goal class needed by shared control code
    goals = load_goals(STUDY_ENV_OFFSETS_FILE,env,goal_object_names=config.goal_objects)

    # Load input (e.g. joystick) handlers
    input_listeners,input_mapper = load_arb_input_handlers(INPUT_INTERFACE,NUM_INPUT_DOFS)
    
    # Set assistance levels
    init_assistance_state = AssistanceDiscrete(ASSISTANCE_LEVELS,level=0)

    # Set up rosbag recorder to record relevant data
    bag_record = RosbagRecorder(config.log_dir,['/my_gen3/joint_states','/joy','/arbitration_joy','/thetas'])

    loggers = [bag_record]


    # log the initial data if log directory is set
    if config.log_dir is not None:
        data = {
            'input_control_paradigm_map': rospy.get_param('/my_gen3/input_control_paradigm_map',{}),
            'input_interface_map': rospy.get_param('/my_gen3/input_interface_map',{}),
            'assistance_config': rospy.get_param('/assistance_config',{}), 
            'input_interfaces': {k: {'interface': v.input_interface, 'axes_map': v.interface_input_axes_map, 'buttons_map': v.interface_input_buttons_map} for k,v in input_listeners.items()},
            'input_mapper': input_mapper.as_dict(denumpify=True),
            'goals': [ [g.name, g.pose.tolist()] for g in goals ],  # list-of-lists to preserve order
            'assistance_data': {k:v.tolist() for k,v in init_assistance_state.assist_levels_dict.items()}
        }
        with open(os.path.join(config.log_dir, 'assistance_init.yaml'), 'w') as f:
            yaml.safe_dump(data, f)


        loop_log_file = RotatingLogFile(os.path.join(config.log_dir, 'assistance_data.csv'))
    else:
        loop_log_file = None


    # Load the object that makes all my fancy pieces of code work together in a control loop
    hdlr = AdjSharedControlHdlr(robot,env,goals,input_listeners,input_mapper,init_assistance_state,loggers,loop_log_file)

    # Start the shared control loop and have it run at a rate of CONTROL_HZ
    rospy.Timer(rospy.Duration(1./CONTROL_HZ),hdlr.main_loop)

    rospy.spin()

    # If keyboard stop hit, stop the robot and stop the data recording
    robot.send_twist_cmd(np.zeros(6))

    bag_record.stop()

    if loop_log_file is not None:
        loop_log_file.close()




