#!/bin/sh

rosbag record /xbotcore/aux /xbotcore/command /xbotcore/fault /xbotcore/joint_states /xbotcore/statistics /traj_replayer/calib_traj_status /mhe/jnt_calib_status
