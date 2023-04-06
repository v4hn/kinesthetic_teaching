#!/usr/bin/env python

import math
import rospy
import moveit_commander
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory
import pandas
import sys

def main():
    if len(sys.argv) < 2:
        print('Usage: {} [csv_file]'.format(sys.argv[0]))
        return

    rospy.init_node('trajectory_player', anonymous=True)
    robot = moveit_commander.RobotCommander()
    group_name = rospy.get_param('~group', 'right_arm')
    group = moveit_commander.MoveGroupCommander(group_name)

    csv_file = sys.argv[1]
    df = pandas.read_csv(open(csv_file, 'r'))
    trajectory = JointTrajectory()
    trajectory.joint_names = [v[:-len('_pos')] for v in df.columns.values if v.endswith('_pos')]

    for _, row in df[df.index%2==0].iterrows():
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration.from_sec(float(row["time_from_start"]))
        point.positions = row.filter(regex='_pos$').to_list()
        point.velocities = row.filter(regex='_vel$').to_list()
        point.accelerations = row.filter(regex='_acc$').to_list()
        trajectory.points.append(point)

    rospy.sleep(1)

    start_pose = trajectory.points[0]
    current_pos = group.get_current_joint_values()
    dist = math.sqrt(sum([(current_pos[i] - start_pose.positions[i])**2 for i in range(len(current_pos))]))
    if dist > 0.1:
        print('Moving to starting position...')
        group.go(start_pose.positions)
        print('Starting position reached.')

    rospy.sleep(1)

    print('Executing trajectory...')

    group.execute(RobotTrajectory(joint_trajectory= trajectory))

    print('Trajectory execution completed.')

if __name__ == '__main__':
    main()
