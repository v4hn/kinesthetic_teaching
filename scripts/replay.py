#!/usr/bin/env python

import math
import rospy
import moveit_commander
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import csv
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
    with open(csv_file, 'r') as file:
        reader = csv.reader(file)
        cols = next(reader)
        time_col = 0
        pos_cols = range(1, (len(cols)-1)//2 + 1)
        vel_cols = range((len(cols)-1)//2+1, len(cols)-1)

        trajectory = JointTrajectory()
        trajectory.joint_names = [col[0:-len("_pos")] for col in cols[1:int(len(cols)/2)+1]]

        for row in reader:
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration.from_sec(float(row[time_col]))
            point.positions = [float(row[col]) for col in pos_cols]
            point.velocities = [float(row[col]) for col in vel_cols]
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

    group.execute(trajectory)

    print('Trajectory execution completed.')

if __name__ == '__main__':
    main()
