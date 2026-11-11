"""Series boundaries that previously produced unsafe or missing matches."""
import importlib.util
from pathlib import Path
import sys

SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0,str(SCRIPTS))
from match_hardware_drivers import driver_scope, sdk_scope


def test_controller_generations_are_not_interchangeable():
    assert driver_scope('dobot','cr5','DobotV3Driver.py')[0]
    assert not driver_scope('dobot','cr5','DobotV4Driver.py')[0]
    assert not sdk_scope('dobot','cr5')[0]
    assert driver_scope('dobot','cr5a','DobotV4Driver.py')[0]
    assert not driver_scope('dobot','cr5a','DobotV3Driver.py')[0]
    assert driver_scope('elite','ec66','EliteRobotsDriverEC.py')[0]
    assert not driver_scope('elite','ec66','EliteRobotsDriverCS.py')[0]
    assert not sdk_scope('elite','ec66')[0]


def test_tm_prefix_and_omron_non_tm():
    assert driver_scope('techmanrobot','techmantm5700','TMDriver.py')[0]
    assert driver_scope('omron','tm5700','TMDriver.py')[0]
    assert not driver_scope('omron','ecobra600','TMDriver.py')[0]
    assert not driver_scope('omron','ld60','TMDriver.py')[0]


def test_kuka_controller_families():
    assert driver_scope('kuka','kr6r900sixx','kukabridge.exe')[0]
    assert not driver_scope('kuka','kr6r900sixx','apikukaiiwa.py')[0]
    assert driver_scope('kuka','lbr iiwa7r800','apikukaiiwa.py')[0]
    assert not driver_scope('kuka','lbr iiwa7r800','kukabridge.exe')[0]
