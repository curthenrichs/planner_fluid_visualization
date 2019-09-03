#!/usr/bin/env python

'''
MIT License

Copyright (c) 2019 Curt Henrichs

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''

'''
Path Reader RIK Node

Open loop real-time trajectory path travel given path file generated by Path
Saver node using relaxed-ik. This node specifically will run through each target
pose within path using interpolation for real-time steps on each update of the
joint solver.

Publishers:
    - /relaxed_ik/ee_pose_goals := EEPoseGoals
        "Real-time pose goal for trajectory as relaxed-ik target"
    - /rik/joint_state := JointState
        "Next joint state for robot in simulation"

Subscribers:
    - /tf := tfMessage
        "Standard transform tree"
    - /relaxed_ik/joint_angle_solutions := JointAngles
        "Joint state results from relaxed-ik for robot next state"

Parameters:
    - ~filepath
        "Path Pose JSON file to execute"
    - ~repeat (false)
        "Loop over path, if not provided then (default)"
    - ~joint_names
        "Name of robot joints in same order as returned by relaxed-ik"
    - ~publish_rate (30)
        "Rate in Hz to publish updates to relaxed-ik"
    - ~hold_count (5)
        "Number of real-time updates to hold target pose position"
    - ~step_size (0.005)
        "Interpolation step size between poses in trajectory"
    - ~base_frame (base_link)
        "Reference frame for pose information"
    - ~ee_frame (ee_link)
        "Target pose frame"
    - ~pose_switch_delay (1)
        "Open loop delay to let relaxed-ik to catch up to target poses"
'''

import tf
import json
import math
import rospy

from sensor_msgs.msg import JointState
from relaxed_ik.msg import EEPoseGoals, JointAngles
from geometry_msgs.msg import Pose, Vector3, Quaternion


DEFAULT_PUBLISH_RATE = 30
DEFAULT_HOLD_COUNT = 5
DEFAULT_STEP_SIZE = 0.005
DEFAULT_BASE_FRAME = 'base_link'
DEFAULT_EE_FRAME = 'ee_link'
DEFAULT_POSE_SWITCH_DELAY = 1


class PathReaderRelaxedIk:

    def __init__(self, filepath, repeat, joint_names, publish_rate, hold_count,
                 step_size, base_frame, ee_frame, pose_switch_delay):
        self._repeat = repeat
        self._joint_names = joint_names
        self._publish_rate = publish_rate
        self._hold_count = hold_count
        self._step_size = step_size
        self._base_frame = base_frame
        self._ee_frame = ee_frame
        self._pose_switch_delay = pose_switch_delay

        file = open(filepath,'r')
        self._data = json.load(file)
        file.close()

        self._listener = tf.TransformListener()

        self._ee_goals_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals',EEPoseGoals,queue_size=5)
        self._joint_state_pub = rospy.Publisher('/rik/joint_state',JointState,queue_size=5)
        self._joint_angle_sub = rospy.Subscriber('/relaxed_ik/joint_angle_solutions',JointAngles, self._ja_cb)

    def spin(self):

        rospy.sleep(10)
        raw_input("Ready to start movement? (press enter)")

        (pos, rot) = self._listener.lookupTransform(self._base_frame, self._ee_frame, rospy.Time(0))
        current_pose = {'position':pos, 'orientation': rot}

        rate = rospy.Rate(self._publish_rate)
        while not rospy.is_shutdown():

            # run path
            count = 1
            for p in self._data['path']:

                print 'Pose #{}'.format(count)
                count += 1

                target_pose = p

                linear_path = self._generate_linear_path(current_pose,target_pose)
                for lp in linear_path:
                    self._ee_goals_pub.publish(EEPoseGoals(ee_poses=[self._format_pose(lp)]))
                    rate.sleep()

                current_pose = target_pose
                rospy.sleep(self._pose_switch_delay)

            # if done
            if not self._repeat:
                break

    def _format_pose(self, dct):
        return Pose(
            position=Vector3(
                x=dct['position'][0],
                y=dct['position'][1],
                z=dct['position'][2]),
            orientation=Quaternion(
                x=dct['orientation'][0],
                y=dct['orientation'][1],
                z=dct['orientation'][2],
                w=dct['orientation'][3]))

    def _generate_linear_path(self, current, target):

        # define position step vector
        dif = [target['position'][i] - current['position'][i] for i in range(0,3)]
        lengthDifVector = math.sqrt(sum([dif[i]**2 for i in range(0,3)]))
        if lengthDifVector <= 0:
            return [target]
        stepVector = [self._step_size / lengthDifVector * dif[i] for i in range(0,3)]

        # generate position array
        positions = []
        lengthNext = 0
        lengthPath = math.sqrt(sum([dif[i]**2 for i in range(0,3)]))
        prev = current['position']
        while True:
            next = [prev[i] + stepVector[i] for i in range(0,3)]
            lengthNext += self._step_size

            if lengthNext >= lengthPath:
                positions.append(target['position'])
                break
            else:
                positions.append(next)
            prev = next

        # generate orientation array
        steps = len(positions)
        orientations = []
        if steps == 1:
            orientations.append(target['orientation'])
        elif steps:
            for i in range(0,steps):
                orientations.append(tf.transformations.quaternion_slerp(
                    current['orientation'],
                    target['orientation'],
                    i/(steps-1.0)))

        # format pose array
        path = [{'position': positions[i], 'orientation': orientations[i]} for i in range(0,len(positions))]

        # append hold poses
        for h in range(0,self._hold_count):
            path.append(target)

        return path

    def _ja_cb(self, msg):
        data = JointState()
        data.name = self._joint_names
        data.position = msg.angles.data
        data.velocity = [0] * len(data.position)
        data.effort = [0] * len(data.position)
        self._joint_state_pub.publish(data)


if __name__ == "__main__":
    rospy.init_node('path_reader_rik')

    filepath = rospy.get_param('~filepath')
    repeat = rospy.get_param('~repeat',False)
    joint_names = rospy.get_param('~joint_names')
    publish_rate = rospy.get_param('~publish_rate',DEFAULT_PUBLISH_RATE)
    hold_count = rospy.get_param('~hold_count',DEFAULT_HOLD_COUNT)
    step_size = rospy.get_param('~step_size',DEFAULT_STEP_SIZE)
    base_frame = rospy.get_param('~base_frame',DEFAULT_BASE_FRAME)
    ee_frame = rospy.get_param('~ee_frame',DEFAULT_EE_FRAME)
    pose_switch_delay = rospy.get_param('~pose_switch_delay',DEFAULT_POSE_SWITCH_DELAY)

    node = PathReaderRelaxedIk(filepath, repeat, joint_names, publish_rate, hold_count,
                               step_size, base_frame, ee_frame, pose_switch_delay)
    node.spin()
