#!/usr/bin/env python3
"""
Morph Sequence Orchestrator
----------------------------
Runs the full test sequence:
  1. Wait for drone to stabilize in X-config
  2. Hover for 30s → collect data
  3. Morph to O → hover 30s
  4. Morph to T → hover 30s
  5. Morph to H → hover 30s
  6. Signal data collection complete

Publishes config change commands to /morphing_drone/set_config
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# Test sequence: (config_name, hover_duration_seconds)
TEST_SEQUENCE = [
    ('X', 30.0),
    ('O', 30.0),
    ('T', 30.0),
    ('H', 30.0),
]

MORPH_TRANSITION_TIME = 5.0   # Extra time for morph + settle
INITIAL_STABILIZE_TIME = 10.0  # Wait for takeoff and initial hover


class MorphSequence(Node):

    def __init__(self):
        super().__init__('morph_sequence')

        self.config_pub = self.create_publisher(
            String, '/morphing_drone/set_config', 10
        )
        self.status_pub = self.create_publisher(
            String, '/morphing_drone/sequence_status', 10
        )

        self.get_logger().info('Morph sequence node started')
        self.get_logger().info(
            f'Test plan: {len(TEST_SEQUENCE)} configurations, '
            f'{sum(d for _, d in TEST_SEQUENCE)}s total hover time'
        )

        # Run the sequence in a timer callback to not block
        self.sequence_idx = -1  # -1 = initial stabilization
        self.phase_start_time = None
        self.phase = 'init'  # 'init', 'hover', 'morphing', 'done'

        self.timer = self.create_timer(0.5, self._sequence_tick)

    def _publish_status(self, status_text):
        msg = String()
        msg.data = status_text
        self.status_pub.publish(msg)

    def _set_config(self, config_name):
        msg = String()
        msg.data = config_name
        self.config_pub.publish(msg)

    def _sequence_tick(self):
        t = self.get_clock().now().nanoseconds * 1e-9

        if self.phase == 'init':
            if self.phase_start_time is None:
                self.phase_start_time = t
                self._set_config('X')
                self.get_logger().info(
                    f'Initializing — waiting {INITIAL_STABILIZE_TIME}s '
                    f'for takeoff and hover stabilization...'
                )
                self._publish_status('INITIALIZING')

            elapsed = t - self.phase_start_time
            if elapsed >= INITIAL_STABILIZE_TIME:
                self.sequence_idx = 0
                self.phase = 'hover'
                self.phase_start_time = t
                config, duration = TEST_SEQUENCE[0]
                self._set_config(config)
                self.get_logger().info(
                    f'[1/{len(TEST_SEQUENCE)}] Hovering in {config}-config '
                    f'for {duration}s — collecting data...'
                )
                self._publish_status(f'HOVER_{config}')

        elif self.phase == 'hover':
            elapsed = t - self.phase_start_time
            _, duration = TEST_SEQUENCE[self.sequence_idx]

            if elapsed >= duration:
                # Move to next config
                self.sequence_idx += 1
                if self.sequence_idx >= len(TEST_SEQUENCE):
                    self.phase = 'done'
                    self.get_logger().info(
                        '=' * 50 + '\n'
                        'ALL CONFIGURATIONS TESTED SUCCESSFULLY\n'
                        'Run: ros2 run morphing_drone analyze_flight_data\n'
                        '=' * 50
                    )
                    self._publish_status('COMPLETE')
                    self.timer.cancel()
                    return

                config, duration = TEST_SEQUENCE[self.sequence_idx]
                self.phase = 'morphing'
                self.phase_start_time = t
                self._set_config(config)
                self.get_logger().info(
                    f'Morphing to {config}-config...'
                )
                self._publish_status(f'MORPHING_TO_{config}')

        elif self.phase == 'morphing':
            elapsed = t - self.phase_start_time
            if elapsed >= MORPH_TRANSITION_TIME:
                config, duration = TEST_SEQUENCE[self.sequence_idx]
                self.phase = 'hover'
                self.phase_start_time = t
                self.get_logger().info(
                    f'[{self.sequence_idx + 1}/{len(TEST_SEQUENCE)}] '
                    f'Hovering in {config}-config for {duration}s — '
                    f'collecting data...'
                )
                self._publish_status(f'HOVER_{config}')


def main(args=None):
    rclpy.init(args=args)
    node = MorphSequence()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Sequence interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
