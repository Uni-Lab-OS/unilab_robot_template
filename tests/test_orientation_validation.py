import numpy as np
from scipy.spatial.transform import Rotation
from unilab_robot_runtime.orientation_validation import native_local_tolerances

def test_old_moveit_local_box_stays_inside_requested_reference_box():
    rng=np.random.default_rng(17)
    for _ in range(40):
        quat=Rotation.random(random_state=rng).as_quat();bounds=np.array([.001,.002,6.283])
        local=np.array(native_local_tolerances(quat,bounds))
        assert np.all(abs(Rotation.from_quat(quat).as_matrix())@local<=bounds+1e-12)

def test_world_z_freedom_maps_to_correct_local_axis():
    quat=Rotation.from_euler("x",np.pi/2).as_quat()
    local=native_local_tolerances(quat,(.001,.001,6.283))
    assert local[1]>6. and local[0]<=.001 and local[2]<=.001
