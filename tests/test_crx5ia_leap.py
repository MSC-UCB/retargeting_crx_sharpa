"""CRX kinematics use independent source-chain equations, not generated FK goldens."""

from pathlib import Path
import hashlib
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import yaml

from retargeting.config import load_robot_config
from retargeting.core.kinematics import RobotAdaptor, RobotPinocchio


BUNDLE = Path("assets/robots/crx5ia_leap_paxini")
PROFILE = "vector_wrist_joint_crx5ia_leap_paxini"


@pytest.fixture
def crx():
    config = load_robot_config("configs/robots/crx5ia_leap_paxini.yaml")
    model = RobotPinocchio(config.robot_file_path, "urdf")
    return config, model, RobotAdaptor(model, list(config.actuated_joints))


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    result = np.eye(4)
    result[:3, 3] = xyz
    result[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    return result


def reference_frames(q):
    # Independent serial-chain calculation transcribed from FANUC's pinned macro.
    # J3/J5 use -Y; J4/J6 use -X. Do not apply teach-pendant sign guesses.
    offsets = [(0, 0, .185), (0, 0, 0), (0, 0, .410),
               (0, 0, 0), (.430, 0, 0), (0, -.130, 0)]
    axes = [(0, 0, 1), (0, 1, 0), (0, -1, 0),
            (-1, 0, 0), (0, -1, 0), (-1, 0, 0)]
    pose = np.eye(4)
    frames = {"base_link": pose.copy(), "wbase": transform((0, 0, .185))}
    for i, (offset, axis, angle) in enumerate(zip(offsets, axes, q), 1):
        rotation = np.eye(4)
        rotation[:3, :3] = Rotation.from_rotvec(np.array(axis) * angle).as_matrix()
        pose = pose @ transform(offset) @ rotation
        frames[f"J{i}_link"] = pose.copy()
    frames["flange"] = pose @ transform((.145, 0, 0))
    frames["ee_mount"] = frames["flange"]
    frames["fanuc_flange"] = frames["flange"] @ transform(rpy=(np.pi, -np.pi/2, 0))
    frames["palm_lower"] = frames["flange"] @ transform(
        (.01, -.03, -.065), (-np.pi, 1.56, 0)
    )
    frames["wrist"] = frames["palm_lower"] @ transform((-.16, -.04, -.01), (0, np.pi/2, 0))
    return frames


def test_source_fk_and_joint_contract(crx):
    config, model, adaptor = crx
    assert model.dof == 22
    assert list(config.actuated_joints) == [f"J{i}" for i in range(1, 7)] + [f"joint_{i}" for i in range(16)]
    np.testing.assert_allclose(np.rad2deg(config.initial_qpos[:6]), [-90, -30, 210, 0, -60, 0])
    limits = model.joint_limits[adaptor.actuated_joints_model_idx][:6]
    np.testing.assert_allclose(np.rad2deg(limits),
                               [[-200, 200], [-179.9, 179.9], [-68, 248],
                                [-190, 190], [-179.9, 179.9], [-225, 225]])
    rng = np.random.default_rng(710)
    cases = [np.zeros(6), np.array(config.initial_qpos[:6]),
             np.deg2rad([-90, 0, 180, 0, 90, 0]),
             np.deg2rad([0, 0, 0, 0, -90, 0]),
             *rng.uniform(limits[:, 0], limits[:, 1], size=(20, 6))]
    for arm in cases:
        q = np.array(config.initial_qpos)
        q[:6] = arm
        for name, expected in reference_frames(arm).items():
            actual = model.get_frame_pose(name, adaptor.forward_qpos(q)).copy()
            np.testing.assert_allclose(actual, expected, atol=1e-10)


def test_wrist_jacobian_matches_finite_difference(crx):
    config, model, adaptor = crx
    q = np.array(config.initial_qpos)
    q[:6] += [.2, -.3, -.4, .25, -.2, .1]
    jacobian = adaptor.backward_jacobian(model.get_frame_space_jacobian("wrist", adaptor.forward_qpos(q)))
    eps = 1e-6
    for j in range(22):
        delta = np.eye(22)[j] * eps
        plus = model.get_frame_pose("wrist", adaptor.forward_qpos(q + delta)).copy()
        minus = model.get_frame_pose("wrist", adaptor.forward_qpos(q - delta)).copy()
        linear = (plus[:3, 3] - minus[:3, 3]) / (2 * eps)
        angular = Rotation.from_matrix(plus[:3, :3] @ minus[:3, :3].T).as_rotvec() / (2 * eps)
        np.testing.assert_allclose(jacobian[:, j], np.r_[linear, angular], atol=1e-8)


def test_nonidentity_base_placement_applies_once(crx, tmp_path):
    config, _, adaptor = crx
    root = ET.parse(config.robot_file_path)
    origin = root.find("joint[@name='base_joint']/origin")
    origin.set("xyz", "0.2 -0.4 0.1")
    origin.set("rpy", "0.1 -0.2 0.6")
    path = tmp_path / "placed.urdf"
    root.write(path)
    model = RobotPinocchio(str(path), "urdf")
    q = np.array(config.initial_qpos)
    placement = transform((.2, -.4, .1), (.1, -.2, .6))
    for name, expected in reference_frames(q[:6]).items():
        np.testing.assert_allclose(
            model.get_frame_pose(name, adaptor.forward_qpos(q)),
            placement @ expected, atol=1e-10,
        )


def test_twenty_reachable_targets_with_exact_vector_references():
    from retargeting_apps.composition import build_execution_flow
    from retargeting_apps.main import compose_hydra_base_config

    # Numerical solver capability test: exact robot vectors, no human pinch
    # rescaling/regularization, tighter tolerance than the default live profile.
    flow = build_execution_flow(compose_hydra_base_config([
        "app=teleop_exe", f"retargeting_profiles={PROFILE}",
        "solver.params.ftol_abs=1e-10",
    ]))
    retargeter = flow.retargeter
    adaptor = retargeter.robot_adaptor
    model = adaptor.robot_model
    rng = np.random.default_rng(20260907)
    for _ in range(20):
        q = retargeter.qpos_init.copy()
        q[:6] += rng.uniform(-.3, .3, 6)
        model.compute_forward_kinematics(adaptor.forward_qpos(q))
        target = model.get_frame_pose("wrist").copy()
        vectors = np.array([
            model.get_frame_pose(t)[:3, 3] - model.get_frame_pose(o)[:3, 3]
            for o, t in retargeter.profile_config.target.link_pairs
        ])
        jacobian = adaptor.backward_jacobian(model.get_frame_space_jacobian("wrist", adaptor.forward_qpos(q)))
        assert np.linalg.svd(jacobian[:, :6], compute_uv=False)[-1] > .01
        seed = q.copy()
        seed[:6] += rng.uniform(-.03, .03, 6)
        solved = retargeter.optimizer.retarget({
            "links_vec": vectors,
            "wrist_quat": np.roll(Rotation.from_matrix(target[:3, :3]).as_quat(), 1),
            "qpos_doa": q,
            "qpos_doa_last": seed,
            "weights": {"links_vec": np.full(len(vectors), 10.), "wrist_rot": 1.,
                        "joint_pos": np.zeros(22), "joint_vel": np.zeros(22)},
        })
        assert solved.shape == (22,) and np.isfinite(solved).all()
        bounds = retargeter.optimizer.joint_limits
        assert np.all(solved >= bounds[:, 0] - 1e-6)
        assert np.all(solved <= bounds[:, 1] + 1e-6)
        actual = model.get_frame_pose("wrist", adaptor.forward_qpos(solved)).copy()
        assert np.linalg.norm(actual[:3, 3] - target[:3, 3]) < .001
        assert Rotation.from_matrix(actual[:3, :3] @ target[:3, :3].T).magnitude() < np.deg2rad(1)


def test_assets_resolve_and_preserve_source_scales():
    manifest = yaml.safe_load((BUNDLE / "manifest.yaml").read_text())
    urdf = BUNDLE / manifest["entrypoints"]["urdf"]
    assert hashlib.sha256(urdf.read_bytes()).hexdigest() == manifest["generated_urdf_sha256"]
    for source, expected_hash in manifest["source"]["sha256"].items():
        if "/meshes/crx5ia/" in source:
            copied = BUNDLE / "meshes/crx5ia" / source.split("/meshes/crx5ia/")[1]
        else:
            filename = "FANUC-README.md" if source == "README.md" else Path(source).name
            copied = BUNDLE / "provenance" / filename
        assert hashlib.sha256(copied.read_bytes()).hexdigest() == expected_hash
    root = ET.parse(urdf).getroot()
    for mesh in root.findall(".//mesh"):
        name = mesh.get("filename")
        assert "package://" not in name and not Path(name).is_absolute()
        path = (urdf.parent / name).resolve()
        assert path.is_relative_to(Path("assets").resolve())
        assert path.is_file()
    for mesh in root.findall(".//collision/geometry/mesh"):
        if "crx5ia" in mesh.get("filename"):
            assert mesh.get("scale") == "0.001 0.001 0.001"
    for dae in (BUNDLE / "meshes/crx5ia/visual").glob("*.dae"):
        document = ET.parse(dae)
        namespace = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
        assert document.find("c:asset/c:unit", namespace).get("meter") == "1"
        # No external textures are required by this pinned CRX mesh set.
        assert not document.findall(".//c:library_images/c:image", namespace)
        for vertices in document.findall(".//c:mesh/c:vertices", namespace):
            position_id = vertices.find("c:input[@semantic='POSITION']", namespace).get("source")[1:]
            array = document.find(f".//c:source[@id='{position_id}']/c:float_array", namespace)
            points = np.fromstring(array.text, sep=" ").reshape(-1, 3)
            assert np.isfinite(points).all()
            assert 0 < np.max(np.ptp(points, axis=0)) < 2


@pytest.mark.parametrize("robot_name", ["crx5ia_leap_paxini", "crx5ia_leap_paxini_left"])
def test_optional_viewer_mesh_loading_without_starting_viewer(robot_name):
    yourdfpy = pytest.importorskip("yourdfpy", reason="Optional Viser URDF loader is not installed")
    pytest.importorskip("collada", reason="Optional DAE loader pycollada is not installed")
    model = yourdfpy.URDF.load(
        str(Path("assets/robots") / robot_name / "urdf" / f"{robot_name}.urdf"),
        load_meshes=True, load_collision_meshes=True, build_collision_scene_graph=True,
    )
    assert len(model.actuated_joint_names) == 22
    assert len(model.scene.geometry) >= 7
    assert len(model.collision_scene.geometry) >= 7
    assert np.isfinite(model.scene.bounds).all()


def test_crx_leap_uses_existing_execution_flow_and_reset():
    from retargeting_apps.composition import build_execution_flow
    from retargeting_apps.main import compose_hydra_base_config
    from teleoperation.inputs.avp import AvpOfflineInput
    from teleoperation.types import ExecutionStatus, SensorHandSample

    config = compose_hydra_base_config([
        "app=teleop_exe", f"retargeting_profiles={PROFILE}",
        "teleoperation_modes=offline_kinematic", "evaluate=true",
    ])
    flow = build_execution_flow(config)
    source = AvpOfflineInput("tests/fixtures/avp_short_replay.npz", start=0, end=2)
    source.open()
    try:
        first = source.read()
        result = flow.step(first)
        assert result.status is ExecutionStatus.EXECUTED
        assert result.command_qpos.shape == (22,)
        assert np.isfinite(result.command_qpos).all()
        np.testing.assert_allclose(result.command_qpos, result.actual_qpos)
        held = flow.step(SensorHandSample(None, None, raw=None))
        assert held.status is ExecutionStatus.HELD
        np.testing.assert_allclose(held.command_qpos, result.command_qpos)
        flow.reset()
        repeated = flow.step(first)
        np.testing.assert_allclose(repeated.command_qpos, result.command_qpos, atol=1e-8)
    finally:
        source.close()
