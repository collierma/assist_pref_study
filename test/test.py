#!/usr/bin/env python3

import os
import rospy,rospkg
import signal

#from quartet_py import initialize

from assist_pref_grasp_demo.loggers.rosbag_recorder import RosbagRecorder

#TEST_ENV_FILE = os.path.join(rospkg.RosPack().get_path('assist_pref_grasp_demo'),'env_data/env_configs/pilot_v1.yaml')
#TEST_ENV_DIR = os.path.join(rospkg.RosPack().get_path('assist_pref_grasp_demo'),'env_data/models')

TEST_LOG_DIR = os.path.join(rospkg.RosPack().get_path('assist_pref_grasp_demo'),'trial_data')
TEST_TOPICS = ['/my_gen3/joint_states','/joy']

if __name__ == "__main__":
    rospy.init_node('test_assist_pref_grasp_demo',anonymous=True)
    
    #robot,env = initialize('gen3',env_data_dir=TEST_ENV_DIR,env_config_filename=TEST_ENV_FILE,sim=True,enable_rviz=True)

    bag_record = RosbagRecorder(TEST_LOG_DIR,TEST_TOPICS)
    bag_record.start()


    rospy.spin()
    bag_record.stop()

