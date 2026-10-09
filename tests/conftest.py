import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from episode_hardware.clock import FakeClock  # noqa: E402
from episode_hardware.joint_mapping import VERTICAL_MOTOR_DEG  # noqa: E402
from episode_hardware.mock_adapter import MockHardwareAdapter, SimulatedArm  # noqa: E402
from episode_hardware.robot_controller import ControllerConfig, SafeRobotController  # noqa: E402
from episode_sdk.mock_server import MockEpisodeServer  # noqa: E402
from episode_sdk.models import ProtocolMode  # noqa: E402


@pytest.fixture
def clock():
    return FakeClock()


@pytest.fixture
def arm(clock):
    return SimulatedArm(clock=clock, initial_motor_deg=VERTICAL_MOTOR_DEG, homing_duration_s=1.0)


@pytest.fixture
def controller(arm, clock):
    c = SafeRobotController(MockHardwareAdapter(arm), ControllerConfig(), clock=clock)
    c.configure()
    c.poll_once()
    return c


def run(controller, clock, steps=200, dt=0.05, until=None):
    for _ in range(steps):
        clock.advance(dt)
        controller.poll_once()
        if until is not None and until():
            return True
    return until is None


@pytest.fixture
def ready_controller(controller, clock):
    controller.home()
    assert run(controller, clock, until=lambda: controller.state.value == "READY")
    return controller


@pytest.fixture
def legacy_server():
    srv = MockEpisodeServer(mode=ProtocolMode.LEGACY).start()
    yield srv
    srv.stop()


@pytest.fixture
def enhanced_server():
    srv = MockEpisodeServer(mode=ProtocolMode.ENHANCED).start()
    yield srv
    srv.stop()
