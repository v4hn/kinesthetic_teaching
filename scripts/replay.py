#!/usr/bin/env python

import rospy
import moveit_commander
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory

from kinesthetic_teaching.recording import Recorder

from multiprocessing import Lock
import pandas as pd
import pickle
import math
import sys


def trajectory_from_csv(filename):
    df = pd.read_csv(open(filename, "r"))
    trajectory = JointTrajectory()
    trajectory.joint_names = [
        v[: -len("_pos")] for v in df.columns.values if v.endswith("_pos")
    ]

    # build trajectory from csv
    for _, row in df[df.index % 2 == 0].iterrows():
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration.from_sec(float(row["time_from_start"]))
        point.positions = row.filter(regex="_pos$").to_list()
        point.velocities = row.filter(regex="_vel$").to_list()
        point.accelerations = row.filter(regex="_acc$").to_list()
        trajectory.points.append(point)

    return trajectory


class Replay:
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.group_name = rospy.get_param("~group", "right_arm")
        self.group = moveit_commander.MoveGroupCommander(self.group_name)

        self.recorder = Recorder(joint_names=self.group.get_active_joints())

    @property
    def trace(self):
        return self.recorder.recording
    
    def execute(self, trajectory):
        start_pose = trajectory.points[0]
        current_pos = self.group.get_current_joint_values()
        dist = math.sqrt(
            sum(
                [
                    (current_pos[i] - start_pose.positions[i]) ** 2
                    for i in range(len(current_pos))
                ]
            )
        )
        if dist > 0.1:
            print("Moving to starting position...")
            self.group.go(start_pose.positions)
            print("Starting position reached.")

        rospy.sleep(0.5)

        print("Executing trajectory...")

        self.recorder.start()
        self.group.execute(RobotTrajectory(joint_trajectory=trajectory))
        self.recorder.stop()


def main():
    if len(sys.argv) < 2:
        print("Usage: {} [csv_file]".format(sys.argv[0]))
        return
    csv_file = sys.argv[1]

    repetitions = int(sys.argv[2]) if len(sys.argv) > 2 else 1

    rospy.init_node("trajectory_player", anonymous=True)

    trajectory = trajectory_from_csv(csv_file)

    replay = Replay()

    rospy.sleep(1)

    # todo iterate execution, store recorded trajectories as pickle
    traces = []
    for i in range(repetitions):
        replay.execute(trajectory)
        traces.append(replay.trace)

    pickle.dump(traces, open(f"{csv_file[0:-4]}.traces.pkl", "wb"))
    print("Trajectory execution completed.")


if __name__ == "__main__":
    main()
