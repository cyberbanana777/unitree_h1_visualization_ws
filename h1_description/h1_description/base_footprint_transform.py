# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

'''
АННОТАЦИЯ
ROS2-узел для трансляции преобразований координат с коррекцией высоты на
основе данных IMU. Подписывается на IMU-данные, вычисляет смещение по Z
из опорных фреймов ног и публикует преобразование между base_footprint и
pelvis. Использует TF2 для получения трансформаций и компенсирует наклон
корпуса.

ANNOTATION
ROS2 node for broadcasting coordinate transforms with height correction
based on IMU data. Subscribes to IMU data, computes Z-offset from leg
reference frames, and publishes transform between `base_footprint`
and `pelvis`. Uses TF2 for transform lookups and compensates for body tilt.
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import (
    TransformBroadcaster, Buffer, TransformListener,
    LookupException, ConnectivityException, ExtrapolationException
)
from tf_transformations import quaternion_multiply, euler_from_quaternion
import math as m
from typing import List, Optional


class ImuTfBroadcaster(Node):
    """
    ROS2 node that broadcasts IMU-based transform with Z-offset compensation.
    
    This node subscribes to IMU data, computes Z-offset from reference frames,
    and broadcasts a transform between parent and child frames with corrected
    orientation and height.
    """
    
    # Constants for configuration
    DEFAULT_FREQUENCY = 30.0
    STATIC_OFFSET = 0.07
    ERROR_THRESHOLD = 5
    IMU_TOPIC = '/sensors/imu/unitree_h1'
    
    def __init__(self):
        super().__init__('imu_tf_broadcaster')
        
        # Initialize TF components
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Declare parameters with descriptive names
        self._declare_parameters()
        
        # Get parameter values
        frequency = self.get_parameter('frequency').value
        
        # Frame configuration
        self.parent_frame = 'base_footprint'
        self.child_frame = 'pelvis'
        self.reference_frames = {
            'a': 'pelvis',
            'b_left': 'left_ankle_link', 
            'b_right': 'right_ankle_link',
            'imu': 'imu_link'
        }
        
        # State variables
        self.z_offset = 0.0
        self.transform_imu = Quaternion()
        self.imu_msg: Optional[Imu] = Imu()
        self.error_counter = 0
        
        # Initialize subscriptions and timers
        self._setup_subscriptions_and_timers(frequency)
        
        self.get_logger().info('IMU TF broadcaster with Z offset started')

    def _declare_parameters(self) -> None:
        """Declare all node parameters."""
        self.declare_parameter('frequency', self.DEFAULT_FREQUENCY)
        self.declare_parameter('imu_topic', self.IMU_TOPIC)

    def _setup_subscriptions_and_timers(self, frequency: float) -> None:
        """Set up ROS subscriptions and timers."""
        # IMU data subscription
        self.imu_subscription = self.create_subscription(
            Imu,
            self.get_parameter('imu_topic').value,
            self._imu_callback,
            10
        )
        
        # Single timer for both updates (more efficient)
        self.update_timer = self.create_timer(1.0 / frequency, self._update_and_broadcast)

    def _update_z_offset(self) -> bool:
        """
        Update Z-offset from TF transformations.
        
        Returns:
            bool: True if update was successful, False otherwise
        """
        try:
            # Get transformations for both ankles
            transform_left = self._get_transform(
                self.reference_frames['a'], 
                self.reference_frames['b_left']
            )
            transform_right = self._get_transform(
                self.reference_frames['a'],
                self.reference_frames['b_right']  
            )
            transform_imu = self._get_transform(
                self.reference_frames['a'],
                self.reference_frames['imu']
            )
            
            # Store IMU transform for later use
            self.transform_imu = transform_imu.transform.rotation
            
            # Get base_footprint to pelvis transform for orientation
            transform_base = self._get_transform('base_footprint', 'pelvis')
            transform_quat = transform_base.transform.rotation
            
            # Calculate roll and pitch for height compensation
            roll, pitch, _ = euler_from_quaternion(self._quaternion_to_list(transform_quat))
            
            # Compute mean height with orientation compensation
            mean_z = -(transform_left.transform.translation.z + 
                      transform_right.transform.translation.z) / 2.0
                      
            self.z_offset = mean_z * m.cos(roll) * m.cos(pitch) + self.STATIC_OFFSET
            
            self.get_logger().debug(
                f'Z offset updated: {self.z_offset:.3f}', 
                throttle_duration_sec=5.0
            )
            return True
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.error_counter += 1
            if self.error_counter >= self.ERROR_THRESHOLD:
                self.get_logger().warning(
                    f'TF transform error: {str(e)}', 
                    throttle_duration_sec=5.0
                )
            return False

    def _get_transform(self, source_frame: str, target_frame: str):
        """Helper method to get transform between frames."""
        return self.tf_buffer.lookup_transform(
            source_frame,
            target_frame, 
            rclpy.time.Time()
        )

    def _imu_callback(self, msg: Imu) -> None:
        """Callback for IMU data subscription."""
        self.imu_msg = msg

    def _update_and_broadcast(self) -> None:
        """Main update loop: update Z-offset and broadcast transform."""
        # Update Z-offset from TF data
        self._update_z_offset()
        self._broadcast_transform()

    def _broadcast_transform(self) -> None:
        """Broadcast the computed transform."""
        transform = TransformStamped()
        
        # Set header
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        
        # Set translation (Z-offset only)
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0  
        transform.transform.translation.z = self.z_offset
        
        # Compute and set orientation
        transform.transform.rotation = self._compute_orientation()
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(transform)

    def _compute_orientation(self) -> Quaternion:
        """Compute the final orientation by combining IMU and transform data."""
        imu_quat = self._quaternion_to_list(self.imu_msg.orientation)
        tf_quat = self._quaternion_to_list(self.transform_imu)
        
        # Combine orientations
        combined_quat = quaternion_multiply(imu_quat, tf_quat)
        return self._list_to_quaternion(combined_quat)

    @staticmethod
    def _quaternion_to_list(quat: Quaternion) -> List[float]:
        """Convert Quaternion to list [x, y, z, w]."""
        return [quat.x, quat.y, quat.z, quat.w]

    @staticmethod  
    def _list_to_quaternion(quat_list: List[float]) -> Quaternion:
        """Convert list [x, y, z, w] to Quaternion."""
        quat = Quaternion()
        quat.x = quat_list[0]
        quat.y = quat_list[1] 
        quat.z = quat_list[2]
        quat.w = quat_list[3]
        return quat


def main(args=None):
    rclpy.init(args=args)
    node = ImuTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()