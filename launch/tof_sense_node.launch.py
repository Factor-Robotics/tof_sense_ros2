# Copyright 2021 Factor Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.event_handlers.on_shutdown import OnShutdown
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_node_name
from lifecycle_msgs.msg import Transition

tof_sense_params = [
    {"device_name": '/dev/ttyUSB0'},
    {"baud_rate": 115200},
    {"num_sensors": 2},
    {"field_of_view": 0.4712389},
    {"min_range": 0.01},
    {"max_range": 5.00},
]

def generate_launch_description():
    tof_sense_driver = LifecycleNode(
        package='tof_sense_ros2',
        executable='tof_sense_node',
        name='tof_sense_node',
        output='screen',
        parameters=tof_sense_params,
    )

    configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=tof_sense_driver,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(tof_sense_driver),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        )
    )

    activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=tof_sense_driver,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(tof_sense_driver),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        )
    )

    shutdown_event_handler = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_node_name('tof_sense_node'),
                        transition_id=Transition.TRANSITION_ACTIVE_SHUTDOWN,
                    )
                )
            ]
        )
    )

    nodes = [
        tof_sense_driver,
        configure_event_handler,
        activate_event_handler,
        shutdown_event_handler,
    ]

    return LaunchDescription(nodes)
