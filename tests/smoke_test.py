#!/usr/bin/env python3
"""Start FAST-LIO in mapping and localization; verify graph/services/PCD load.

No LiDAR, bag recording, synthetic sensor data, or reference-map writes.
Checks TF/QoS initialization and the runtime parameter guard in both modes.
This is a startup test, not a mapping-accuracy test.
"""
import os
import json
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import DescribeParameters, GetParameters, SetParameters

ROOT = Path(__file__).resolve().parents[1]


def parameter_request(node, service_type, suffix, request):
    name = '/laser_mapping/' + suffix
    client = node.create_client(service_type, name)
    try:
        assert client.wait_for_service(timeout_sec=5), f'{name} unavailable'
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        assert future.done() and future.result() is not None, f'{name} timed out'
        return future.result()
    finally:
        node.destroy_client(client)


def check_runtime_parameter_guard(node):
    # All writes target this smoke test's isolated, hardware-free child node.
    tf_publishers = node.get_publishers_info_by_topic('/tf')
    assert any(info.node_name == 'laser_mapping' and info.topic_type == 'tf2_msgs/msg/TFMessage'
               for info in tf_publishers), 'FAST-LIO TF publisher unavailable'
    tf_qos = 'qos_overrides./tf.publisher.durability'
    names = [tf_qos, 'tracking.min_ratio', 'record.note', 'record.sample_every_n', 'static_map.enabled',
             'static_map.confirmation_enabled', 'static_map.clearing_enabled', 'static_map.clear_observation_interval']
    original = parameter_request(node, GetParameters, 'get_parameters',
                                 GetParameters.Request(names=names)).values
    assert len(original) == len(names), original
    assert original[0].type == ParameterType.PARAMETER_STRING and original[0].string_value, original[0]
    assert original[1].type == ParameterType.PARAMETER_DOUBLE, original[1]
    assert original[2].type == ParameterType.PARAMETER_STRING, original[2]
    assert original[3].type == ParameterType.PARAMETER_INTEGER, original[3]
    assert original[4].type == ParameterType.PARAMETER_BOOL, original[4]
    assert all(value.type == ParameterType.PARAMETER_BOOL for value in original[5:7]), original
    assert original[7].type == ParameterType.PARAMETER_DOUBLE, original[7]
    descriptors = parameter_request(node, DescribeParameters, 'describe_parameters',
                                    DescribeParameters.Request(names=[tf_qos])).descriptors
    assert len(descriptors) == 1 and descriptors[0].read_only, 'TF QoS must remain read-only'

    forbidden = [
        Parameter(name=names[1], value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=original[1].double_value + 0.01)),
        Parameter(name=tf_qos, value=ParameterValue(
            type=ParameterType.PARAMETER_STRING,
            string_value='volatile' if original[0].string_value == 'transient_local' else 'transient_local')),
        Parameter(name=names[3], value=ParameterValue(
            type=ParameterType.PARAMETER_INTEGER, integer_value=0)),
        Parameter(name=names[4], value=ParameterValue(
            type=ParameterType.PARAMETER_BOOL, bool_value=not original[4].bool_value)),
    ]
    forbidden += [Parameter(name=names[i], value=ParameterValue(
        type=ParameterType.PARAMETER_BOOL, bool_value=not original[i].bool_value)) for i in (5, 6)]
    forbidden.append(Parameter(name=names[7], value=ParameterValue(
        type=ParameterType.PARAMETER_DOUBLE, double_value=.05)))
    results = parameter_request(node, SetParameters, 'set_parameters',
                                SetParameters.Request(parameters=forbidden)).results
    assert len(results) == len(forbidden) and all(not result.successful for result in results), results
    assert 'startup-only' in results[0].reason, results[0]
    assert 'positive integer' in results[2].reason, results[2]
    assert all('startup-only' in result.reason for result in results[3:]), results
    unchanged = parameter_request(node, GetParameters, 'get_parameters',
                                  GetParameters.Request(names=names)).values
    assert list(unchanged) == list(original), 'Rejected writes changed parameters'

    allowed = [
        Parameter(name=names[2], value=ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value='smoke_parameter_guard')),
        Parameter(name=names[3], value=ParameterValue(
            type=ParameterType.PARAMETER_INTEGER, integer_value=original[3].integer_value + 1)),
    ]
    try:
        results = parameter_request(node, SetParameters, 'set_parameters',
                                    SetParameters.Request(parameters=allowed)).results
        assert len(results) == 2 and all(result.successful for result in results), results
        updated = parameter_request(node, GetParameters, 'get_parameters',
                                    GetParameters.Request(names=names[2:4])).values
        assert len(updated) == 2 and updated[0].string_value == 'smoke_parameter_guard', updated
        assert updated[1].integer_value == original[3].integer_value + 1, updated
    finally:
        restore = [Parameter(name=name, value=value) for name, value in zip(names[2:4], original[2:4])]
        results = parameter_request(node, SetParameters, 'set_parameters',
                                    SetParameters.Request(parameters=restore)).results
        assert len(results) == 2 and all(result.successful for result in results), results


def stop_process(process):
    forced = False
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        forced = True
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
    # A launch parent can exit while a deadlocked child remains in its session.
    # Clean only this test's isolated process group, even if the parent exited.
    try:
        os.killpg(process.pid, signal.SIGKILL)
        forced = True
    except ProcessLookupError:
        pass
    return forced


def check_driver(node):
    log_dir = ROOT / 'log' / 'smoke'
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / 'driver_loopback.log'
    with log_path.open('w') as output:
        process = subprocess.Popen(
            ['ros2', 'launch', 'livox_ros_driver2', 'msg_MID360_launch.py',
             f'user_config_path:={ROOT / "tests" / "MID360_loopback.json"}'],
            cwd=ROOT, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
                if process.poll() is not None:
                    raise RuntimeError(f'Driver launch exited; see {log_path}')
                text = log_path.read_text()
                if 'Init lds lidar fail!' in text:
                    raise RuntimeError(f'Driver loopback SDK initialization failed; see {log_path}')
                if ('Init lds lidar success!' in text
                        and 'livox_lidar_publisher' in node.get_node_names()):
                    break
            else:
                raise RuntimeError(f'Driver loopback startup timeout; see {log_path}')
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.2)
            assert process.poll() is None, 'Driver exited after SDK initialization'
            print('[PASS] driver: shared SDK loaded, loopback-only initialization and ROS node', flush=True)
        finally:
            forced = stop_process(process)
    assert not forced, f'Driver required forced cleanup; see {log_path}'
    assert 'failed to terminate' not in log_path.read_text(), f'Driver did not stop cleanly; see {log_path}'


def check_mode(node, mode, lidar_config=None):
    log_dir = ROOT / 'log' / 'smoke'
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f'{mode}.log'
    reference = []
    subscription = None
    status_subscription = odometry_subscription = None
    statuses, odometry = [], []
    tracking = []
    tracking_subscription = node.create_subscription(String, '/tracking/status', tracking.append,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    if mode == 'localization':
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscription = node.create_subscription(PointCloud2, '/reference_map', reference.append, qos)
        status_subscription = node.create_subscription(String, '/localization/status', statuses.append, qos)
        odometry_subscription = node.create_subscription(Odometry, '/Odometry', odometry.append, 10)
    with log_path.open('w') as output:
        arguments = ['map_name:=smoke_test']
        if mode == 'localization':
            arguments = [f'map_path:={ROOT / "pcd_map" / "test.pcd"}']
        process = subprocess.Popen(
            ['bash', str(ROOT / 'scripts/run.sh'), mode,
             f'with_driver:={"true" if lidar_config else "false"}',
             'rviz:=false', 'record_bag:=false', *arguments,
             *([f'lidar_config:={lidar_config}'] if lidar_config else [])],
            cwd=ROOT, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
                if process.poll() is not None:
                    raise RuntimeError(f'{mode} launch exited; see {log_path}')
                services = dict(node.get_service_names_and_types())
                topics = dict(node.get_topic_names_and_types())
                if ('/map_save' in services and '/start_path_record' in services
                        and '/stop_path_record' in services and '/Odometry' in topics
                        and '/tf' in topics and 'Node init finished.' in log_path.read_text()
                        and '/tracking/status' in topics and '/map_save/status' in topics and tracking
                        and (mode != 'localization' or (reference and statuses))
                        and (not lidar_config or 'Init lds lidar success!' in log_path.read_text())):
                    break
            else:
                raise RuntimeError(f'{mode} graph/reference-map timeout; see {log_path}')
            # Assert that FAST-LIO is actually listening for the correct sensor types.
            lidar = node.get_subscriptions_info_by_topic('/livox/lidar')
            imu = node.get_subscriptions_info_by_topic('/livox/imu')
            assert any(info.topic_type == 'livox_ros_driver2/msg/CustomMsg' for info in lidar), lidar
            assert any(info.topic_type == 'sensor_msgs/msg/Imu' for info in imu), imu
            check_runtime_parameter_guard(node)
            archive = json.loads(tracking[-1].data)['static_map']
            assert archive['enabled'] == (mode == 'mapping'), archive
            assert archive['confirmation_enabled'] == archive['clearing_enabled'] == (mode == 'mapping'), archive
            assert 'archive_clear_gate' in json.loads(tracking[-1].data), tracking[-1]
            assert archive['confirmed'] == 0 and archive['candidates'] == 0, archive
            if mode == 'mapping':
                client = node.create_client(Trigger, '/map_save')
                try:
                    assert client.wait_for_service(timeout_sec=5)
                    future = client.call_async(Trigger.Request())
                    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                    assert future.done() and not future.result().success, 'Empty mapping save must fail'
                    assert 'No mapping points' in future.result().message, future.result()
                finally:
                    node.destroy_client(client)
            if mode == 'localization':
                assert reference[0].width * reference[0].height > 0, 'Empty reference PCD'
                assert json.loads(statuses[-1].data)['state'] == 'waiting_for_imu', statuses[-1]
                assert not odometry, 'Uninitialized localization published odometry'
                record = node.create_client(Trigger, '/start_path_record')
                try:
                    assert record.wait_for_service(timeout_sec=5)
                    future = record.call_async(Trigger.Request())
                    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                    assert future.done() and not future.result().success, 'Uninitialized localization allowed recording'
                    assert 'not ready' in future.result().message, future.result()
                finally:
                    node.destroy_client(record)
                client = node.create_client(Trigger, '/map_save')
                try:
                    assert client.wait_for_service(timeout_sec=5), 'map_save unavailable'
                    future = client.call_async(Trigger.Request())
                    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                    assert future.done(), 'map_save call timeout'
                    result = future.result()
                    assert not result.success and 'localization' in result.message, result
                finally:
                    node.destroy_client(client)
            # Check that the node has not crashed immediately after registration.
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.2)
            assert process.poll() is None, f'{mode} exited after startup'
            if mode == 'localization':
                assert not odometry, 'Waiting startup gate emitted odometry'
            print(f'[PASS] {mode}: topics, subscriptions, services, TF/QoS and runtime parameter guard'
                  + ('; explicit relative local JSON initialized loopback SDK' if lidar_config else '')
                  + ('; reference PCD loaded and map_save is read-only' if reference else ''), flush=True)
        finally:
            forced = stop_process(process)
            node.destroy_subscription(tracking_subscription)
            if subscription is not None:
                node.destroy_subscription(subscription)
            if status_subscription is not None:
                node.destroy_subscription(status_subscription)
            if odometry_subscription is not None:
                node.destroy_subscription(odometry_subscription)
    assert not forced, f'{mode} required forced cleanup; see {log_path}'
    assert 'process has finished cleanly' in log_path.read_text(), f'{mode} did not stop cleanly on SIGINT; see {log_path}'
    # Let DDS remove the first process's endpoints before starting the next one.
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if '/map_save' not in dict(node.get_service_names_and_types()):
            break


def main():
    rclpy.init()
    node = rclpy.create_node('fastlio_workspace_smoke_test')
    try:
        check_driver(node)
        local_dir = ROOT / 'config/local'
        local_dir.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='smoke-', dir=local_dir) as directory:
            config = Path(directory) / 'MID360.smoke.local.json'
            subprocess.run(['bash', str(ROOT / 'scripts/init_local_config.sh'),
                            '--output', str(config), '--host-ip', '127.0.0.1',
                            '--lidar-ip', '127.0.0.2'], check=True, capture_output=True, text=True)
            check_mode(node, 'mapping', lidar_config=str(config.relative_to(ROOT)))
        check_mode(node, 'localization')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print('[PASS] Hardware-free smoke tests complete (no sensor accuracy claim).')


if __name__ == '__main__':
    main()
