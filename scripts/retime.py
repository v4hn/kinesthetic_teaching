#!/usr/bin/env python

import math
import sys
import os

import pandas

import rospy
import moveit_commander
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory, RobotState

def main():
    if len(sys.argv) < 2:
        print('Usage: {} [csv_file]'.format(sys.argv[0]))
        return

    rospy.init_node('trajectory_player', anonymous=True)
    group_name = rospy.get_param('~group', 'right_arm')
    group = moveit_commander.MoveGroupCommander(group_name)

    csv_file = sys.argv[1]
    df = pandas.read_csv(open(csv_file, 'r'))
    trajectory = JointTrajectory()
    trajectory.joint_names = [v[:-len('_pos')] for v in df.columns.values if v.endswith('_pos')]

    for _, row in df.iterrows():
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration.from_sec(float(row["time_from_start"]))
        point.positions = row.filter(regex='_pos$').to_list()
        point.velocities = row.filter(regex='_vel$').to_list()
        point.accelerations = row.filter(regex='_acc_nd$').to_list()
        trajectory.points.append(point)

    trajectory= group.retime_trajectory(RobotState(is_diff= True), RobotTrajectory(joint_trajectory= trajectory), algorithm= "time_optimal_trajectory_generation").joint_trajectory

    df2= pandas.DataFrame().reindex_like(df).dropna()
    for j in trajectory.joint_names:
      df2[f'{j}_acc']= []

    for point in trajectory.points:
        df2.loc[len(df2)] = [point.time_from_start.to_sec(), *point.positions, *point.velocities, *point.accelerations]
     
    # Write the updated data to a new csv file with the suffix "_approx_vel_acc"
    output_filename = os.path.splitext(csv_file)[0] + '_retimed.csv'
    df2.to_csv(output_filename, index=False)

    

if __name__ == '__main__':
    main()
