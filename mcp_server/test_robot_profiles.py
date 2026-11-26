#!/usr/bin/env python3
"""Robot profiles (ELLAMA_ROBOT). No hardware needed:

    ELLAMA_SERIAL_PORT=/dev/none mcp_server/.venv/bin/python mcp_server/test_robot_profiles.py
"""
import importlib
import json
import os
import sys
from pathlib import Path

os.environ.setdefault("ELLAMA_SERIAL_PORT", "/dev/none_for_tests")
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # for bridge/
import bridge  # noqa: E402


def test_default_profile_is_config_json():
    if "ELLAMA_ROBOT" in os.environ:
        del os.environ["ELLAMA_ROBOT"]
    importlib.reload(bridge)
    assert bridge.ROBOT_NAME == "default"
    assert bridge.resolve_config_path() == bridge.CONFIG_PATH
    assert bridge.resolve_config_path().name == "config.json"


def test_missing_profile_is_a_loud_error():
    os.environ["ELLAMA_ROBOT"] = "no-such-robot"
    try:
        # Even importing with a bad profile fails: a typo must never boot the
        # server onto silently wrong calibration.
        try:
            importlib.reload(bridge)
        except RuntimeError as e:
            assert "no-such-robot" in str(e), e
        else:
            raise AssertionError("missing profile did not raise")
    finally:
        del os.environ["ELLAMA_ROBOT"]
        importlib.reload(bridge)


def test_named_profile_selects_its_file():
    # A real file in the real robots/ dir (removed in finally): this exercises
    # the whole chain -- import, resolve, load -- exactly as the daemon runs it.
    prof = bridge.CONFIG_PATH.parent / "robots" / "testbot.json"
    assert not prof.exists(), prof
    prof.write_text(json.dumps({
        "wheel_diameter_in": 10.0, "ticks_per_rev": 4096, "gear_ratio": 4.5,
        "track_width_in": 21.5, "encoder_sign": [1, 1],
        "required_sensors": ["enc"],
    }))
    os.environ["ELLAMA_ROBOT"] = "testbot"
    try:
        importlib.reload(bridge)
        assert bridge.resolve_config_path() == prof
        cfg = bridge.load_config()
        assert cfg["track_width_in"] == 21.5, cfg
        assert cfg["required_sensors"] == ["enc"], cfg
    finally:
        del os.environ["ELLAMA_ROBOT"]
        prof.unlink()
        importlib.reload(bridge)
    assert bridge.ROBOT_NAME == "default"


# ---- runtime switching (the dashboard's robot picker) ----------------------

def _tmp_profiles(tmpdir):
    """Point bridge at a scratch robots/ dir with two robots; returns restore()."""
    saved = bridge.PROFILES_DIR
    bridge.PROFILES_DIR = Path(tmpdir)
    base = {"wheel_diameter_in": 10.0, "ticks_per_rev": 4096, "gear_ratio": 4.5,
            "track_width_in": 19.0}
    (Path(tmpdir) / "alpha.json").write_text(json.dumps(
        dict(base, encoder_sign=[-1, -1], drive_swap_sides=True)))
    (Path(tmpdir) / "beta.json").write_text(json.dumps(
        dict(base, encoder_sign=[1, 1], drive_swap_sides=False, required_sensors=["enc"])))

    def restore():
        bridge.PROFILES_DIR = saved
        bridge.set_robot("default")
    return restore


def test_set_robot_switches_name_and_encoder_sign():
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        restore = _tmp_profiles(d)
        try:
            assert bridge.list_profiles() == ["alpha", "beta"]
            bridge.set_robot("beta")
            assert bridge.ROBOT_NAME == "beta" and bridge.ENC_SIGN == (1, 1)
            assert bridge.parse_line(b"ENC 10 20 5")[1] == (10, 20, 5)
            bridge.set_robot("alpha")
            assert bridge.ENC_SIGN == (-1, -1)
            assert bridge.parse_line(b"ENC 10 20 5")[1] == (-10, -20, 5)
            try:
                bridge.set_robot("gamma")
            except RuntimeError as e:
                assert "gamma" in str(e)
            else:
                raise AssertionError("unknown robot accepted")
            assert bridge.ROBOT_NAME == "alpha" and bridge.ENC_SIGN == (-1, -1)
        finally:
            restore()


def test_save_profile_validates_and_refuses_overwrite():
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        restore = _tmp_profiles(d)
        try:
            for bad in ("Lisa", "default", "", "../x", "a b"):
                try:
                    bridge.save_profile(bad, {})
                except ValueError:
                    pass
                else:
                    raise AssertionError("accepted bad name %r" % bad)
            try:
                bridge.save_profile("alpha", {})
            except FileExistsError:
                pass
            else:
                raise AssertionError("overwrote without asking")
            bridge.save_profile("alpha", {"x": 1}, overwrite=True)
            assert json.loads((Path(d) / "alpha.json").read_text()) == {"x": 1}
        finally:
            restore()


def _fake_ctx(link=None, lease_held_by=None):
    import types
    lease = {"held": lease_held_by is not None, "holder": lease_held_by,
             "move_in_progress": False}
    return types.SimpleNamespace(link=link, LEASE=types.SimpleNamespace(snapshot=lambda: lease))


def test_dashboard_select_and_save():
    import tempfile
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import daemon_http as dh
    with tempfile.TemporaryDirectory() as d:
        restore = _tmp_profiles(d)
        try:
            # refused while someone else holds the wheels
            code, out = dh.robot_select(_fake_ctx(lease_held_by="claude"), "beta")
            assert code == 409 and bridge.ROBOT_NAME == "default", out
            code, out = dh.robot_select(_fake_ctx(), "beta")
            assert code == 200 and out["active"] == "beta", out
            code, out = dh.robot_select(_fake_ctx(), "unknown")
            assert code == 200 and out["active"] == "default", out
            code, out = dh.robot_select(_fake_ctx(), "nope")
            assert code == 400 and bridge.ROBOT_NAME == "default", out
            # save from the direction check: copies the active robot's sizes
            bridge.set_robot("beta")
            code, out = dh.robot_save(_fake_ctx(), "gamma", [1, -1], True)
            assert code == 200 and out["active"] == "gamma", out
            g = json.loads((Path(d) / "gamma.json").read_text())
            assert g["encoder_sign"] == [1, -1] and g["drive_swap_sides"] is True, g
            assert g["required_sensors"] == ["enc"], g     # carried from beta
            code, out = dh.robot_save(_fake_ctx(), "gamma", [1, 1], False)
            assert code == 409 and out.get("exists"), out
            code, out = dh.robot_save(_fake_ctx(), "gamma", [1, 1], False, overwrite=True)
            assert code == 200, out
            for bad_sign in ([1], [2, 1], "11", [True, 1]):
                code, out = dh.robot_save(_fake_ctx(), "delta", bad_sign, False)
                assert code == 400, (bad_sign, out)
            code, out = dh.robot_save(_fake_ctx(), "delta", [1, 1], "no")
            assert code == 400, out
        finally:
            restore()


def test_link_switch_resets_odometry_and_bumps_packet_generation():
    import tempfile
    import threading
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import robot_link as rl

    class Vet:
        def log(self, *a, **kw):
            pass

    with tempfile.TemporaryDirectory() as d:
        restore = _tmp_profiles(d)
        saved_policy = list(rl.POLICY.required)
        try:
            bridge.set_robot("alpha")
            link = rl.RobotLink.__new__(rl.RobotLink)
            link.cfg, link.track_width_calibrated = link._load_profile()
            link.lock = threading.Lock()
            link.enc = bridge.EncoderState(link.cfg)
            link.recorder = type("R", (), {"vet": Vet()})()
            link.enc.update(1000, 1000, 10)
            link.enc.update(5000, 5000, 20)
            assert link.enc.dist_l != 0
            gen = link._profile_gen
            link.switch_robot("beta")
            assert link._profile_gen == gen + 1
            assert link.enc.dist_l == 0 and link.enc.first == [None, None]
            assert link.cfg["drive_swap_sides"] is False
            assert link.cfg["gear_ratio"] == rl.TRUE_GEAR_RATIO
            if rl._required_env is None:
                assert rl.POLICY.required == ["enc"], rl.POLICY.required
                link.switch_robot("alpha")      # no required_sensors -> back to default
                assert sorted(rl.POLICY.required) == sorted(rl.sensorlib.SensorPolicy.DEFAULT)
            try:
                link.switch_robot("gamma")
            except RuntimeError:
                pass
            else:
                raise AssertionError("unknown robot accepted")
            assert bridge.ROBOT_NAME in ("alpha", "beta")
        finally:
            rl.POLICY.set(saved_policy, by="test restore")
            restore()


def main():
    fns = [v for k, v in sorted(globals().items())
           if k.startswith("test_") and callable(v)]
    failed = 0
    for fn in fns:
        try:
            fn()
            print("PASS", fn.__name__)
        except AssertionError as e:
            failed += 1
            print("FAIL", fn.__name__, "-", e)
    print("%d failed, %d passed" % (failed, len(fns) - failed))
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
