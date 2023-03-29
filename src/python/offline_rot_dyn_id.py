#!/usr/bin/env python3
import argparse

import rospkg

from utilities.offline_rot_dyn_cal import OfflineRotDynCal
from utilities.offline_rot_dyn_cal import str2bool

import h5py
from horizon.utils import mat_storer

from termcolor import colored
import os
import shutil
from datetime import datetime
from datetime import date

file_name = os.path.splitext(os.path.basename(__file__))[0]
file_name.replace(".py", "")

unique_id = date.today().strftime("%d-%m-%Y") + "-" + \
            datetime.now().strftime("%H_%M_%S")

pretakeoff_opt_name = "pretakeoff"
apex_opt_name = "apex"
full_opt_name = "full"

sol_mat_basename = "awesome_jump"


def main(args):
    
    a = True

if __name__ == '__main__':

    parser = argparse.ArgumentParser(
        description='Script to perform rotor dynamics calibration with offline-data')

    parser.add_argument('--data_basepath', '-dpath', type=str, default=rospkg.RosPack().get_path(
        "moving_horizon_jnt_calib") + "/description/urdf" + "/" + "awesome_leg_test_rig" + ".urdf")
    
    parser.add_argument('--gnagna', '-gnagna', type=str2bool, \
                        help='', default=True)

    args = parser.parse_args()

    rospackage = rospkg.RosPack()  # Only for taking the path to the leg package
#    exec_path = rospackage.get_path("awesome_leg") + "/src/horizon_code""

    os.mkdir(args.results_dir)

    main(args)