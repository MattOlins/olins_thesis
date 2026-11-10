"""Validate experiment measurements and guaranteed owned-process cleanup."""

import csv
import subprocess
import sys
from types import SimpleNamespace

import pytest

try:
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from nav_msgs.msg import Odometry
    import rclpy
    from thesis_test.comparison_runner import best_effort_stop, EpisodeObserver
    from thesis_test.comparison_runner import stop_process, write_csv
except ModuleNotFoundError:
    pytest.skip('ROS Python modules unavailable', allow_module_level=True)


def test_owned_process_is_stopped():
    process = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'],
                               start_new_session=True, stderr=subprocess.DEVNULL)
    stop_process(process)
    assert process.poll() is not None


def test_context_shutdown_during_stop_does_not_abort_cleanup(monkeypatch):
    def invalid_context(message):
        raise RuntimeError('publisher context became invalid')

    observer = SimpleNamespace(stop_pub=SimpleNamespace(publish=invalid_context))
    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    best_effort_stop(observer)


def test_csv_preserves_union_of_diagnostic_fields(tmp_path):
    path = tmp_path/'navigation.csv'
    write_csv(path, [{'state': 'TRACKING'}, {'state': 'FAILED', 'reason': 'stuck'}])
    rows = list(csv.DictReader(path.open()))
    assert rows[0]['reason'] == ''
    assert rows[1]['reason'] == 'stuck'


def test_distance_clearance_and_stuck_metrics_are_independent():
    rclpy.init()
    observer = EpisodeObserver()
    try:
        message = Odometry()
        observer.odom_callback(message)
        # Pre-navigation settling must not count as navigation distance.
        message.pose.pose.position.x = 1.0
        observer.odom_callback(message)
        assert observer.path_length == 0
        status = DiagnosticStatus(name='navigation')
        status.values = [KeyValue(key=k, value=v) for k, v in {
            'state': 'TRACKING', 'reason': 'ok', 'observed_clearance': '.45',
            'turn_reversals': '2',
        }.items()]
        diagnostics = DiagnosticArray(status=[status])
        observer.state_callback(diagnostics)
        message.pose.pose.position.y = 1.0
        observer.odom_callback(message)
        assert observer.path_length == pytest.approx(1.0)
        assert observer.minimum_clearance == pytest.approx(.45)
        assert observer.turn_reversals == 2
        status.values = [KeyValue(key=k, value=v) for k, v in {
            'state': 'REPLANNING', 'reason': 'stuck', 'observed_clearance': 'nan',
        }.items()]
        observer.state_callback(diagnostics)
        observer.state_callback(diagnostics)
        assert observer.stuck_events == 1
        assert observer.minimum_clearance == pytest.approx(.45)
    finally:
        observer.destroy_node()
        rclpy.shutdown()
