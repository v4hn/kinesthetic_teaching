#!/usr/bin/env python

import rospy
import moveit_commander
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
import datetime
import csv
from multiprocessing import Lock


class TrajectoryRecorder:

    def __init__(self):
        rospy.init_node('trajectory_recorder', anonymous=True)
        self.group_name = rospy.get_param('~group', 'right_arm')
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.joint_names = self.group.get_active_joints()
        self.joint_positions = dict.fromkeys(self.joint_names, 0.0)
        self.joint_velocities = dict.fromkeys(self.joint_names, 0.0)

        self.rate = rospy.Rate(50)
        self.threshold_stopped_time = rospy.Duration(1.5)
        self.motion_threshold = rospy.get_param('~motion_threshold', 0.01)

        self.mutex = Lock()
        self.recorded_trajectory = JointTrajectory()
        self.recording = False
        self.last_time_moving = rospy.Time()
        self.last_update = rospy.Time()

        self.subscriber = rospy.Subscriber('/joint_states', JointState, self.joint_states_callback, queue_size=5, tcp_nodelay=True)

    def moving(self):
        return any([abs(v) > self.motion_threshold for v in self.joint_velocities.values()])

    def joint_states_callback(self, msg):
        now = msg.header.stamp
        with self.mutex():
            for name, position, velocity in zip(msg.name, msg.position, msg.velocity):
                self.joint_positions[name] = position
                self.joint_velocities[name] = velocity
            self.last_update = now

        if self.moving():
            self.last_time_moving = now
            if not self.recording:
                rospy.loginfo('Recording trajectory...')
                self.recorded_trajectory.joint_names = self.joint_names
                self.recorded_trajectory.header.stamp = now
                self.recording = True
                self.last_time_moving = now

        if self.recording:
            if now - self.last_time_moving >= self.threshold_stopped_time:
                with self.mutex():
                    self.recording = False
                    self.save()

    def record(self):
        while not rospy.is_shutdown():
            if self.recording:
                with self.mutex():
                    msg = JointTrajectoryPoint()
                    msg.positions = [self.joint_positions[joint_name] for joint_name in self.joint_names]
                    msg.velocities = [self.joint_velocities[joint_name] for joint_name in self.joint_names]
                    msg.time_from_start = rospy.Time.now() - self.recorded_trajectory.header.stamp
                    self.recorded_trajectory.points.append(msg)

            self.rate.sleep()

    def save(self):
        filename = 'trajectory_' + datetime.datetime.now().strftime('%Y%m%d-%H-%M-%S') + '.csv'
        with open(filename, 'w') as file:
            writer = csv.writer(file)
            writer.writerow(['time_from_start']+[name + '_pos' for name in self.recorded_trajectory.joint_names] + [name + '_vel' for name in self.recorded_trajectory.joint_names])
            for point in self.recorded_trajectory.points:
                row = [point.time_from_start.to_sec()] + point.positions + point.velocities
                writer.writerow(row)
        self.recorded_trajectory = JointTrajectory()
        rospy.loginfo(f'Trajectory recorded as {filename}.')


if __name__ == '__main__':
    recorder = TrajectoryRecorder()
    try:
        recorder.record()
    except rospy.ROSInterruptException:
        pass