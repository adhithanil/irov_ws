# Brings up the sediment collector lifecycle node and drives it through
# configure -> activate automatically, so the action is ready to accept goals.

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.events import matches_action
import lifecycle_msgs.msg


def generate_launch_description():
    collector = LifecycleNode(
        package='rov_sediment',
        executable='sediment_collector_node',
        name='sediment_collector',
        namespace='',
        output='screen',
    )

    # configure as soon as the node starts
    to_configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(collector),
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
    ))

    # activate once it reports 'inactive' (i.e. configure finished)
    to_activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=collector,
        goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(collector),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        ))],
    ))

    return LaunchDescription([collector, to_activate, to_configure])
