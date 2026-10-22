#!/usr/bin/env python3
"""
Morph Sequence Orchestrator
----------------------------
Two-phase test:

  PHASE 1 — GROUND MORPHING (no flight)
    X → O → T → H → X (10s each shape, drone stays on ground)

  PHASE 2 — FLIGHT MORPHING
    Take off to 5m, then:
    X → O → T → H (30s hover each, collecting flight data)

Publishes config change commands to /morphing_drone/set_config
and flight enable flag to /morphing_drone/flight_enabled
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


GROUND_SEQUENCE = [
    ('X', 10.0),
    ('O', 10.0),
    ('Y', 10.0),
    ('T', 10.0),
    ('H', 10.0),
    ('X', 10.0),
]

FLIGHT_SEQUENCE = [
    ('X', 10.0),
    ('O', 10.0),
    ('Y', 10.0),
    ('T', 10.0),
    ('H', 10.0),
    ('X', 10.0),
]

MORPH_TRANSITION_TIME = 5.0
TAKEOFF_STABILIZE_TIME = 10.0


class MorphSequence(Node):

    def __init__(self):
        super().__init__('morph_sequence')

        self.config_pub = self.create_publisher(String, '/morphing_drone/set_config', 10)
        self.status_pub = self.create_publisher(String, '/morphing_drone/sequence_status', 10)
        self.flight_pub = self.create_publisher(Bool, '/morphing_drone/flight_enabled', 10)

        self.phase = 'ground_init'
        self.sequence_idx = 0
        self.phase_start_time = None

        total_ground = sum(d for _, d in GROUND_SEQUENCE)
        total_flight = sum(d for _, d in FLIGHT_SEQUENCE)
        self.get_logger().info('Morph sequence node started')
        self.get_logger().info(
            f'Phase 1: Ground morphing ({len(GROUND_SEQUENCE)} shapes, {total_ground:.0f}s)'
        )
        self.get_logger().info(
            f'Phase 2: Flight morphing ({len(FLIGHT_SEQUENCE)} shapes, {total_flight:.0f}s)'
        )

        self.timer = self.create_timer(0.5, self._tick)

    def _pub_status(self, text):
        msg = String(); msg.data = text; self.status_pub.publish(msg)

    def _set_config(self, name):
        msg = String(); msg.data = name; self.config_pub.publish(msg)

    def _set_flight(self, enabled):
        msg = Bool(); msg.data = enabled; self.flight_pub.publish(msg)

    def _tick(self):
        t = self.get_clock().now().nanoseconds * 1e-9

        # ═══ PHASE 1: GROUND MORPHING ═══
        if self.phase == 'ground_init':
            self.phase_start_time = t
            self.sequence_idx = 0
            self._set_flight(False)
            config = GROUND_SEQUENCE[0][0]
            self._set_config(config)
            self.get_logger().info('═══ PHASE 1: GROUND MORPHING ═══')
            self.get_logger().info(f'  [1/{len(GROUND_SEQUENCE)}] {config}-config for {GROUND_SEQUENCE[0][1]:.0f}s (grounded)')
            self._pub_status(f'GROUND_{config}')
            self.phase = 'ground_hold'

        elif self.phase == 'ground_hold':
            elapsed = t - self.phase_start_time
            _, duration = GROUND_SEQUENCE[self.sequence_idx]
            self._set_flight(False)

            if elapsed >= duration:
                self.sequence_idx += 1
                if self.sequence_idx >= len(GROUND_SEQUENCE):
                    self.get_logger().info('═══ Ground morphing complete ═══')
                    self.phase = 'takeoff_init'
                    self.phase_start_time = t
                    return

                config, dur = GROUND_SEQUENCE[self.sequence_idx]
                self.phase = 'ground_morphing'
                self.phase_start_time = t
                self._set_config(config)
                self.get_logger().info(f'  Morphing to {config}-config...')
                self._pub_status(f'GROUND_MORPH_{config}')

        elif self.phase == 'ground_morphing':
            elapsed = t - self.phase_start_time
            self._set_flight(False)

            if elapsed >= MORPH_TRANSITION_TIME:
                config, dur = GROUND_SEQUENCE[self.sequence_idx]
                self.phase = 'ground_hold'
                self.phase_start_time = t
                self.get_logger().info(
                    f'  [{self.sequence_idx+1}/{len(GROUND_SEQUENCE)}] '
                    f'{config}-config for {dur:.0f}s (grounded)'
                )
                self._pub_status(f'GROUND_{config}')

        # ═══ TAKEOFF ═══
        elif self.phase == 'takeoff_init':
            self._set_config('X')
            self._set_flight(True)
            self.get_logger().info('═══ PHASE 2: TAKING OFF ═══')
            self.get_logger().info(f'  Climbing to 5m, stabilizing for {TAKEOFF_STABILIZE_TIME:.0f}s...')
            self._pub_status('TAKEOFF')
            self.phase = 'takeoff_wait'
            self.phase_start_time = t

        elif self.phase == 'takeoff_wait':
            self._set_flight(True)
            elapsed = t - self.phase_start_time
            if elapsed >= TAKEOFF_STABILIZE_TIME:
                self.get_logger().info('═══ PHASE 2: FLIGHT MORPHING ═══')
                self.sequence_idx = 0
                config, dur = FLIGHT_SEQUENCE[0]
                self._set_config(config)
                self.phase = 'flight_hold'
                self.phase_start_time = t
                self.get_logger().info(
                    f'  [1/{len(FLIGHT_SEQUENCE)}] Hovering in {config}-config '
                    f'for {dur:.0f}s — collecting data...'
                )
                self._pub_status(f'FLIGHT_{config}')

        # ═══ PHASE 2: FLIGHT MORPHING ═══
        elif self.phase == 'flight_hold':
            self._set_flight(True)
            elapsed = t - self.phase_start_time
            _, duration = FLIGHT_SEQUENCE[self.sequence_idx]

            if elapsed >= duration:
                self.sequence_idx += 1
                if self.sequence_idx >= len(FLIGHT_SEQUENCE):
                    self.get_logger().info(
                        '\n' + '=' * 50 + '\n'
                        'ALL CONFIGURATIONS TESTED SUCCESSFULLY\n'
                        'Run: ros2 run morphing_drone analyze_flight_data\n'
                        '=' * 50
                    )
                    self._pub_status('COMPLETE')
                    self.timer.cancel()
                    return

                config, dur = FLIGHT_SEQUENCE[self.sequence_idx]
                self.phase = 'flight_morphing'
                self.phase_start_time = t
                self._set_config(config)
                self.get_logger().info(f'  Morphing to {config}-config...')
                self._pub_status(f'FLIGHT_MORPH_{config}')

        elif self.phase == 'flight_morphing':
            self._set_flight(True)
            elapsed = t - self.phase_start_time
            if elapsed >= MORPH_TRANSITION_TIME:
                config, dur = FLIGHT_SEQUENCE[self.sequence_idx]
                self.phase = 'flight_hold'
                self.phase_start_time = t
                self.get_logger().info(
                    f'  [{self.sequence_idx+1}/{len(FLIGHT_SEQUENCE)}] '
                    f'Hovering in {config}-config for {dur:.0f}s — collecting data...'
                )
                self._pub_status(f'FLIGHT_{config}')


def main(args=None):
    rclpy.init(args=args)
    node = MorphSequence()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Sequence interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
