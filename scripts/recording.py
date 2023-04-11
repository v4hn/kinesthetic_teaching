import rospy
from sensor_msgs.msg import JointState

import pandas as pd

from multiprocessing import Lock


class Recorder:
    def __init__(self, joint_names=None):
        self.mutex = Lock()
        self.record = False
        self.trajectories = {}
        self.recording_start = None

        self.joints = joint_names

        self.subscriber = rospy.Subscriber(
            "joint_states",
            JointState,
            self._joint_states_callback,
            queue_size=5,
            tcp_nodelay=True,
        )

    def _joint_states_callback(self, js: JointState):
        if not self.record:
            return
        with self.mutex:
            if self.joints is None:
                joints = dict(zip(js.name, js.position))
            else:
                joints = {n: p for n, p in zip(js.name, js.position) if n in self.joints}
            if not self.record or len(joints) == 0:
                return

            if js.header.stamp < self.recording_start:
                return

            first_joint = next(iter(joints.keys()))
            if first_joint not in self.trajectories:
                self.trajectories[first_joint] = pd.DataFrame(
                    columns=["time_from_start", *joints.keys()]
                )
            d = self.trajectories[first_joint]
            self.trajectories[first_joint] = pd.concat(
                [
                    d,
                    pd.DataFrame(
                        [[
                            js.header.stamp.to_sec() - self.recording_start.to_sec(),
                            *joints.values(),
                        ]],
                        columns=d.columns,
                    ),
                ],
                ignore_index=True,
            )

    def start(self):
        with self.mutex:
            self.trajectories = {}
            self.recording_start = rospy.Time.now()
            self.record = True

    def stop(self):
        with self.mutex:
            self.record = False
            self.recording_start = None
        # TODO: validate all active joints received values or warn

    @property
    def recording(self):
        return list(self.trajectories.values())
