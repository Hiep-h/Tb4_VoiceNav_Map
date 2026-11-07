import pytest
import rclpy
from action_msgs.msg import GoalStatus

from tb4_voicenav.voice_nav_node import VoiceNavNode


class FakeFuture:
    def __init__(self, result, auto_resolve=True):
        self._result = result
        self._auto_resolve = auto_resolve

    def result(self):
        return self._result

    def add_done_callback(self, callback):
        self._callback = callback
        if self._auto_resolve:
            callback(self)

    def resolve_now(self, result=None):
        if result is not None:
            self._result = result
        self._callback(self)


class FakeResult:
    def __init__(self, status):
        self.status = status


class FakeGoalHandle:
    def __init__(self, accepted=True, status=GoalStatus.STATUS_SUCCEEDED):
        self.accepted = accepted
        self.status = status

    def get_result_async(self):
        return FakeFuture(FakeResult(self.status))


class FakeActionClient:
    def __init__(self, accept=True, status=GoalStatus.STATUS_SUCCEEDED, resolve=True):
        self.accept = accept
        self.status = status
        self.resolve = resolve
        self.sent_goals = []
        self.send_futures = []

    def send_goal_async(self, goal, feedback_callback=None):
        self.sent_goals.append(goal)
        future = FakeFuture(
            FakeGoalHandle(accepted=self.accept, status=self.status),
            auto_resolve=self.resolve)
        self.send_futures.append(future)
        return future


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = VoiceNavNode()
    n.action_client = FakeActionClient()
    statuses = []
    n.publish_status = lambda text: statuses.append(text)
    n._statuses = statuses
    yield n
    n.destroy_node()


def test_single_destination_sends_one_goal(node):
    node.handle_command('đi đến phòng khách')
    assert len(node.action_client.sent_goals) == 1
    goal = node.action_client.sent_goals[0]
    assert goal.pose.header.frame_id == 'map'
    assert goal.pose.pose.position.x == pytest.approx(0.7)
    assert goal.pose.pose.position.y == pytest.approx(-4.93)


def test_from_to_sends_goals_in_order(node):
    node.handle_command('đi từ phòng khách đến phòng bếp')
    assert len(node.action_client.sent_goals) == 2
    xs = [g.pose.pose.position.x for g in node.action_client.sent_goals]
    assert xs == [pytest.approx(0.7), pytest.approx(6.1)]


def test_unknown_region_does_not_move_robot(node):
    node.handle_command('đi đến hành lang')
    assert node.action_client.sent_goals == []
    assert 'Khu vực không tồn tại' in node._statuses[-1]


def test_not_a_command_does_not_move_robot(node):
    node.handle_command('hôm nay trời đẹp quá')
    assert node.action_client.sent_goals == []
    assert 'Không hiểu lệnh' in node._statuses[-1]


def test_short_noise_text_is_ignored(node):
    node.handle_command('đi')
    assert node.action_client.sent_goals == []
    assert node._statuses == []


def test_noise_text_while_navigating_does_not_override_status(node):
    node.action_client = FakeActionClient(resolve=False)
    node.handle_command('đi đến phòng khách')
    assert node.navigating
    statuses_before = list(node._statuses)
    node.handle_command('hôm nay trời đẹp quá')
    assert node._statuses == statuses_before


def test_three_point_route_sends_goals_in_order(node):
    node.handle_command('đi từ phòng ngủ qua phòng khách rồi đến phòng bếp')
    assert len(node.action_client.sent_goals) == 3
    xs = [g.pose.pose.position.x for g in node.action_client.sent_goals]
    assert xs == [pytest.approx(-8.85), pytest.approx(0.7), pytest.approx(6.1)]


def test_unknown_region_in_chain_does_not_move_robot(node):
    node.handle_command('đi từ phòng khách qua hành lang rồi đến phòng bếp')
    assert node.action_client.sent_goals == []
    assert 'Khu vực không tồn tại' in node._statuses[-1]


def test_goal_rejected_reports_status_and_stops_queue(node):
    node.action_client = FakeActionClient(accept=False)
    node.handle_command('đi từ phòng khách đến phòng bếp')
    assert len(node.action_client.sent_goals) == 1
    assert 'Không đến được' in node._statuses[-1]


def test_goal_aborted_reports_status_and_stops_queue(node):
    node.action_client = FakeActionClient(status=GoalStatus.STATUS_ABORTED)
    node.handle_command('đi từ phòng khách đến phòng bếp')
    assert len(node.action_client.sent_goals) == 1
    assert 'Không đến được' in node._statuses[-1]


def test_new_command_preempts_old_goal_without_reporting_stale_failure(node):
    node.action_client = FakeActionClient(resolve=False)
    node.handle_command('đi đến phòng khách')
    first_future = node.action_client.send_futures[0]
    first_target_name = node.current_target.name

    node.action_client = FakeActionClient(resolve=False)
    node.handle_command('đi đến phòng bếp')
    second_target_name = node.current_target.name
    assert second_target_name != first_target_name
    statuses_after_new_command = list(node._statuses)

    first_future.resolve_now(FakeGoalHandle(accepted=False))
    assert node._statuses == statuses_after_new_command
    assert not any('Không đến được' in s for s in node._statuses)
    assert node.current_target.name == second_target_name
    assert node.navigating is True
