#!/usr/bin/env python3

from harp_arm_study_utils.data_processing.rosbag_exporter import export_bag,joy_bag_export,joint_states_bag_export

JOINT_NAMES = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','finger_joint','left_inner_knuckle_joint','left_inner_finger_joint','right_outer_knuckle_joint','right_inner_knuckle_joint','right_inner_finger_joint']

# Recorded topics: ['/my_gen3/joint_states','/joy','/arbitration_joy','/thetas']
def study_v1_bag_export(data_dir,bag_file):
    joy_spec_dict = joy_bag_export('/joy','joy',['axes_x','axes_y','axes_z'],['buttons_0','buttons_1'])
    joint_states_spec_dict = joint_states_bag_export('/my_gen3/joint_states','joint_states',JOINT_NAMES)
    export_bag(data_dir,bag_file,[joy_spec_dict,joint_states_spec_dict])


if __name__ == '__main__':
    import os,rospkg
    data_dir = os.path.join(rospkg.RosPack().get_path('assist_pref_grasp_demo'),'trial_data')
    study_v1_bag_export(data_dir,os.path.join(data_dir,'trial_data.bag'))
