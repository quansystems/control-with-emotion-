# SPDX-License-Identifier: Apache-2.0
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from nav2_simple_commander.robot_navigator import TaskResult

from robot_node import waypoint_navigator as route


class FakeNavigator:
    def __init__(self, results, reject=False, pending=False):
        self.results = iter(results)
        self.reject = reject
        self.pending = pending
        self.sent = []
        self.canceled = False

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))

    def goToPose(self, pose):
        self.sent.append(pose)
        return not self.reject

    def isTaskComplete(self):
        return not self.pending

    def getResult(self):
        return next(self.results)

    def cancelTask(self):
        self.canceled = True

    def info(self, message):
        pass

    error = info


GOALS = [(1.2, -0.3, 0.0), (0.0, -1.3, 1.57), (-0.8, 1.5, -1.57)]


def test_success_sends_goals_in_order():
    navigator = FakeNavigator([TaskResult.SUCCEEDED] * 3)
    assert route.navigate_goals(navigator, GOALS, 180.0)
    assert [(p.pose.position.x, p.pose.position.y) for p in navigator.sent] == [
        (x, y) for x, y, yaw in GOALS]
    for pose in navigator.sent:
        assert pose.header.frame_id == 'map'
        assert pose.pose.position.z == 0.0
        q = pose.pose.orientation
        assert q.z ** 2 + q.w ** 2 == pytest.approx(1.0)


@pytest.mark.parametrize('result', [TaskResult.FAILED, TaskResult.CANCELED, TaskResult.UNKNOWN])
def test_failure_at_second_goal_stops_route(result):
    navigator = FakeNavigator([TaskResult.SUCCEEDED, result])
    assert not route.navigate_goals(navigator, GOALS, 180.0)
    assert len(navigator.sent) == 2


def test_rejection_stops_route():
    navigator = FakeNavigator([], reject=True)
    assert not route.navigate_goals(navigator, GOALS, 180.0)
    assert len(navigator.sent) == 1


def test_timeout_cancels_without_sending_next_goal(monkeypatch):
    navigator = FakeNavigator([], pending=True)
    times = iter([0.0, 181.0])
    monkeypatch.setattr(route.time, 'monotonic', lambda: next(times))
    assert not route.navigate_goals(navigator, GOALS, 180.0)
    assert navigator.canceled
    assert len(navigator.sent) == 1


@pytest.mark.parametrize('content', ['goals: []', 'goals: [{x: 1, y: 2}]',
                                     'goals: [{x: .nan, y: 2, yaw: 0}]'])
def test_invalid_goals_rejected_before_navigation(tmp_path, content):
    filename = tmp_path / 'goals.yaml'
    filename.write_text(content)
    with pytest.raises(ValueError):
        route.load_goals(filename)


def test_default_route_reads_current_catalog_in_id_order(sample_catalog_file):
    from robot_node.semantic_map import RegionCatalog
    catalog = RegionCatalog(sample_catalog_file)
    assert route.load_region_goals(sample_catalog_file) == [
        tuple(catalog.by_id[r]['goal'][key] for key in ('x', 'y', 'yaw'))
        for r in sorted(catalog.by_id)]
