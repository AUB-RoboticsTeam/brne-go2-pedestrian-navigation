"""Tests for rotation-aware camera ego-motion compensation."""

import math

import numpy as np

from pedestrian_tracker.pedestrian_state_node import (
    extrapolate_planar_camera_pose,
    quaternion_multiply,
)


def rotate_vector(vector, quaternion):
    """Rotate a vector by an [x, y, z, w] quaternion."""
    q = np.asarray(quaternion[:3], dtype=float)
    vector = np.asarray(vector, dtype=float)
    intermediate = 2.0 * np.cross(q, vector)
    return vector + quaternion[3] * intermediate + np.cross(q, intermediate)


def test_camera_lever_arm_rotates_with_base_yaw():
    """The offset camera must orbit base_link during an in-place turn."""
    position, quaternion = extrapolate_planar_camera_pose(
        camera_position=[0.34, 0.0, 0.08],
        camera_quaternion=[0.0, 0.0, 0.0, 1.0],
        base_position=[0.0, 0.0, 0.0],
        base_quaternion=[0.0, 0.0, 0.0, 1.0],
        body_linear_velocity=[0.0, 0.0, 0.0],
        yaw_rate=1.0,
        dt=math.pi / 2.0,
    )

    np.testing.assert_allclose(position, [0.0, 0.34, 0.08], atol=1e-12)
    np.testing.assert_allclose(
        quaternion,
        [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)],
        atol=1e-12,
    )


def test_stationary_world_point_is_invariant_after_rotation_compensation():
    """A static world point remains fixed after compensated reconstruction."""
    initial_position = np.array([0.34, 0.0, 0.08])
    initial_quaternion = np.array([0.0, 0.0, 0.0, 1.0])
    position, quaternion = extrapolate_planar_camera_pose(
        camera_position=initial_position,
        camera_quaternion=initial_quaternion,
        base_position=[0.0, 0.0, 0.0],
        base_quaternion=[0.0, 0.0, 0.0, 1.0],
        body_linear_velocity=[0.0, 0.0, 0.0],
        yaw_rate=0.8,
        dt=0.5,
    )

    world_point = np.array([2.0, 0.3, 0.0])
    inverse_quaternion = np.array([
        -quaternion[0],
        -quaternion[1],
        -quaternion[2],
        quaternion[3],
    ])
    camera_measurement = rotate_vector(
        world_point - position,
        inverse_quaternion,
    )
    reconstructed = position + rotate_vector(camera_measurement, quaternion)

    np.testing.assert_allclose(reconstructed, world_point, atol=1e-12)


def test_quaternion_product_preserves_camera_pitch_during_yaw():
    """Yaw compensation composes with rather than replaces camera pitch."""
    pitch = -math.radians(30.0)
    camera_pitch = np.array([
        0.0,
        math.sin(pitch / 2.0),
        0.0,
        math.cos(pitch / 2.0),
    ])
    yaw = math.radians(20.0)
    yaw_delta = np.array([
        0.0,
        0.0,
        math.sin(yaw / 2.0),
        math.cos(yaw / 2.0),
    ])

    expected = quaternion_multiply(yaw_delta, camera_pitch)
    _, actual = extrapolate_planar_camera_pose(
        camera_position=[0.34, 0.0, 0.08],
        camera_quaternion=camera_pitch,
        base_position=[0.0, 0.0, 0.0],
        base_quaternion=[0.0, 0.0, 0.0, 1.0],
        body_linear_velocity=[0.0, 0.0, 0.0],
        yaw_rate=yaw,
        dt=1.0,
    )

    np.testing.assert_allclose(actual, expected, atol=1e-12)
