#!/usr/bin/env python3

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor

from pymoveit2 import MoveIt2, MoveIt2State

def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_pose_goal")

    # Declare parameters for position and orientation
    node.declare_parameter("position", [0.4, 0.0, 1.71])  # Considering the table height of 1.21m
    node.declare_parameter("quat_xyzw", [0.0, 0.0, 0.0, 1.0])
    node.declare_parameter("synchronous", True)
    node.declare_parameter("cancel_after_secs", 0.0)
    node.declare_parameter("planner_id", "RRTConnectkConfigDefault")
    node.declare_parameter("cartesian", False)
    node.declare_parameter("cartesian_max_step", 0.0025)
    node.declare_parameter("cartesian_fraction_threshold", 0.0)
    node.declare_parameter("cartesian_jump_threshold", 0.0)
    node.declare_parameter("cartesian_avoid_collisions", False)

    callback_group = ReentrantCallbackGroup()

    # Set up MoveIt2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=[
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ],
        base_link_name="base_link",
        end_effector_name="tool0",
        group_name="manipulator",
        callback_group=callback_group,
    )
    moveit2.planner_id = (
        node.get_parameter("planner_id").get_parameter_value().string_value
    )

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # Wait for MoveIt services to be available
    node.get_logger().info("Waiting for MoveIt services...")
    moveit2.wait_for_service(timeout=Duration(seconds=10.0))

    moveit2.max_velocity = 0.5
    moveit2.max_acceleration = 0.5

    position = node.get_parameter("position").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    synchronous = node.get_parameter("synchronous").get_parameter_value().bool_value
    cancel_after_secs = (
        node.get_parameter("cancel_after_secs").get_parameter_value().double_value
    )
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value
    cartesian_max_step = (
        node.get_parameter("cartesian_max_step").get_parameter_value().double_value
    )
    cartesian_fraction_threshold = (
        node.get_parameter("cartesian_fraction_threshold")
        .get_parameter_value()
        .double_value
    )
    cartesian_jump_threshold = (
        node.get_parameter("cartesian_jump_threshold")
        .get_parameter_value()
        .double_value
    )
    cartesian_avoid_collisions = (
        node.get_parameter("cartesian_avoid_collisions")
        .get_parameter_value()
        .bool_value
    )

    moveit2.cartesian_avoid_collisions = cartesian_avoid_collisions
    moveit2.cartesian_jump_threshold = cartesian_jump_threshold

    # Move to initial joint configuration
    initial_joint_positions = [0.0, -1.57, 0.0, -1.57, 0.0, 1.57]  # Define your initial joint positions here
    node.get_logger().info(f"Moving to initial joint positions: {initial_joint_positions}")
    moveit2.move_to_joint_positions(initial_joint_positions)
    moveit2.wait_until_executed()

    # Move to pose
    node.get_logger().info(
        f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
    )
    moveit2.move_to_pose(
        position=position,
        quat_xyzw=quat_xyzw,
        cartesian=cartesian,
        cartesian_max_step=cartesian_max_step,
        cartesian_fraction_threshold=cartesian_fraction_threshold,
    )
    if synchronous:
        moveit2.wait_until_executed()
    else:
        rate = node.create_rate(10)
        while moveit2.query_state() != MoveIt2State.EXECUTING:
            rate.sleep()

        future = moveit2.get_execution_future()

        if cancel_after_secs > 0.0:
            sleep_time = node.create_rate(cancel_after_secs)
            sleep_time.sleep()
            moveit2.cancel_execution()

        while not future.done():
            rate.sleep()

        print("Result status: " + str(future.result().status))
        print("Result error code: " + str(future.result().result.error_code))

    rclpy.shutdown()
    executor_thread.join()
    exit(0)

if __name__ == "__main__":
    main()
