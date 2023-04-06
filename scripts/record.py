#!/usr/bin/env python

import rospy
import moveit_commander
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header, String as StringMsg
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
        self.motion_threshold = rospy.get_param('~motion_threshold', 0.2)

        self.mutex = Lock()
        self.recorded_trajectory = JointTrajectory()
        self.recording = False
        self.last_time_moving = rospy.Time()
        self.last_update = rospy.Time()

        # TODO: optionally trigger teach/mannequin mode and disable again on shutdown

        self.state_pub = rospy.Publisher('say', StringMsg, queue_size= 5, tcp_nodelay=True, latch= True)

        self.subscriber = rospy.Subscriber('joint_states', JointState, self.joint_states_callback, queue_size=5, tcp_nodelay=True)

    def moving(self):
        moving_joints = {j:v for j,v in self.joint_velocities.items() if abs(v) > self.motion_threshold}
        return moving_joints

    def joint_states_callback(self, msg):
        now = msg.header.stamp
        with self.mutex:
            for name, position, velocity in [tup for tup in zip(msg.name, msg.position, msg.velocity) if tup[0] in self.joint_names]:
                self.joint_positions[name] = position
                self.joint_velocities[name] = velocity
            self.last_update = now

            moving_joints= self.moving()
            if len(moving_joints) > 0:
                self.last_time_moving = now
                if not self.recording:
                    rospy.loginfo(f'Detected motion: {moving_joints}. Recording trajectory...')
                    self.state_pub.publish("I'm moving")
                    self.recorded_trajectory.joint_names = self.joint_names
                    self.recording = True
                    self.last_time_moving = now

            if self.recording and (now - self.last_time_moving) >= self.threshold_stopped_time:
                    self.state_pub.publish("stop")
                    self.recorded_trajectory.points = self.recorded_trajectory.points[0:-(int(self.threshold_stopped_time/self.rate.sleep_dur)-2)]
                    self.recording = False
                    self.save()

    def record(self):
        rospy.loginfo('Recorder is ready to record')
        self.state_pub.publish("I'm here.")
        while not rospy.is_shutdown():
            if self.recording:
                with self.mutex:
                    now = rospy.Time.now() 
                    if len(self.recorded_trajectory.points) == 0:
                        self.recorded_trajectory.header.stamp = now
                    msg = JointTrajectoryPoint()
                    msg.positions = [self.joint_positions[joint_name] for joint_name in self.joint_names]
                    msg.velocities = [self.joint_velocities[joint_name] for joint_name in self.joint_names]
                    msg.time_from_start = now - self.recorded_trajectory.header.stamp
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
