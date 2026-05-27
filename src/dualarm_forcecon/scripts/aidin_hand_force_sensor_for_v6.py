import omni.graph.core as og

# ============================================================
# Isaac Sim 5.1 / 6.0 compatibility import
# ============================================================
def _enable_isaac_extensions():
    try:
        import omni.kit.app
        ext_mgr = omni.kit.app.get_app().get_extension_manager()

        for ext_name in [
            "isaacsim.core.prims",
            "isaacsim.core.api",
        ]:
            try:
                ext_mgr.set_extension_enabled_immediate(ext_name, True)
            except Exception:
                pass
    except Exception:
        pass


_enable_isaac_extensions()

try:
    # Isaac Sim 6.0 recommended current wrapper path for this old one-robot usage
    from isaacsim.core.prims import SingleArticulation as Articulation
    ARTICULATION_API_NAME = "isaacsim.core.prims.SingleArticulation"
except Exception:
    try:
        # Isaac Sim 5.x / 6.x legacy-compatible path
        from isaacsim.core.api.articulations import Articulation
        ARTICULATION_API_NAME = "isaacsim.core.api.articulations.Articulation"
    except Exception:
        # Isaac Sim 5.1 old path
        from omni.isaac.core.articulations import Articulation
        ARTICULATION_API_NAME = "omni.isaac.core.articulations.Articulation"


# ============================================================
# User settings
# ============================================================
ROBOT_PRIM_PATH = "/World/aidin_rby1"
VERBOSE = True

# Publish order:
# left thumb/index/middle/ring/baby,
# right thumb/index/middle/ring/baby
# each finger = Fx, Fy, Fz
FINGER_JOINT_CANDIDATES = [
    ["left_thumb_joint_tactile", "left_thumb_joint4"],
    ["left_index_joint_tactile", "left_index_joint4"],
    ["left_middle_joint_tactile", "left_middle_joint4"],
    ["left_ring_joint_tactile", "left_ring_joint4"],
    ["left_baby_joint_tactile", "left_baby_joint4"],
    ["right_thumb_joint_tactile", "right_thumb_joint4"],
    ["right_index_joint_tactile", "right_index_joint4"],
    ["right_middle_joint_tactile", "right_middle_joint4"],
    ["right_ring_joint_tactile", "right_ring_joint4"],
    ["right_baby_joint_tactile", "right_baby_joint4"],
]

FORCE_SIGN = [1.0, 1.0, 1.0]


# ============================================================
# Internal helpers
# ============================================================
def _reset_state(db: og.Database):
    db.state.robot = None
    db.state.row_indices = []
    db.state.resolved_joint_names = []
    db.state.initialized = False
    db.state.import_api_name = ARTICULATION_API_NAME


def _log(db: og.Database, msg: str):
    try:
        db.log_info(msg)
    except Exception:
        print(msg)


def _set_output_data(db: og.Database, data):
    wrote = False

    try:
        db.outputs.data = data
        wrote = True
    except Exception:
        pass

    try:
        db.outputs.Data = data
        wrote = True
    except Exception:
        pass

    if not wrote and VERBOSE:
        _log(db, "[HAND_FT] No output named 'data' or 'Data'. Add a float[] output attribute.")


def _set_zero_outputs(db: og.Database):
    _set_output_data(db, [0.0] * 30)


def _to_numpy_like(x):
    if x is None:
        return None

    try:
        if hasattr(x, "detach"):
            return x.detach().cpu().numpy()
    except Exception:
        pass

    try:
        if hasattr(x, "cpu") and hasattr(x, "numpy"):
            return x.cpu().numpy()
    except Exception:
        pass

    return x


def _make_articulation(prim_path: str):
    # SingleArticulation accepts prim_path=...
    try:
        return Articulation(prim_path=prim_path, name="aidin_hand_ft_robot")
    except TypeError:
        pass

    # Old Articulation accepts positional prim path
    try:
        return Articulation(prim_path)
    except TypeError:
        pass

    # Some view-style wrappers accept prim_paths_expr=...
    return Articulation(prim_paths_expr=prim_path, name="aidin_hand_ft_robot")


def _resolve_joint_name(target_name: str, joint_indices):
    if target_name in joint_indices:
        return target_name

    suffix_matches = []
    for name in joint_indices.keys():
        name_str = str(name)

        if name_str == target_name:
            suffix_matches.append(name)
        elif name_str.endswith(target_name):
            suffix_matches.append(name)
        elif name_str.split("/")[-1] == target_name:
            suffix_matches.append(name)

    if len(suffix_matches) == 1:
        return suffix_matches[0]

    return None


def _get_metadata(robot):
    articulation_view = getattr(robot, "_articulation_view", None)
    if articulation_view is not None:
        metadata = getattr(articulation_view, "_metadata", None)
        if metadata is not None:
            return metadata

    metadata = getattr(robot, "_metadata", None)
    if metadata is not None:
        return metadata

    return None


def _try_initialize(db: og.Database) -> bool:
    if getattr(db.state, "initialized", False):
        return True

    try:
        if db.state.robot is None:
            db.state.robot = _make_articulation(ROBOT_PRIM_PATH)

        try:
            db.state.robot.initialize()
        except Exception:
            pass

        handles_ready = False
        try:
            handles_ready = bool(db.state.robot.handles_initialized)
        except Exception:
            try:
                handles_ready = bool(db.state.robot.is_physics_handle_valid())
            except Exception:
                handles_ready = False

        if not handles_ready:
            return False

        metadata = _get_metadata(db.state.robot)
        if metadata is None:
            _log(db, "[HAND_FT] Failed to access articulation metadata.")
            return False

        joint_indices = metadata.joint_indices
        joint_names = list(metadata.joint_names)

        row_indices = []
        resolved_names = []
        missing = []

        for candidates in FINGER_JOINT_CANDIDATES:
            resolved_name = None

            for target_name in candidates:
                resolved_name = _resolve_joint_name(target_name, joint_indices)
                if resolved_name is not None:
                    break

            if resolved_name is None:
                missing.append(candidates)
                continue

            # get_measured_joint_forces() row:
            # row 0 = base incoming joint
            # row joint_index + 1 = corresponding joint reaction force/torque
            row_indices.append(int(joint_indices[resolved_name]) + 1)
            resolved_names.append(str(resolved_name))

        if missing:
            _log(db, f"[HAND_FT] Missing finger force joints: {missing}")
            _log(db, f"[HAND_FT] Available joints: {joint_names}")
            return False

        db.state.row_indices = row_indices
        db.state.resolved_joint_names = resolved_names
        db.state.initialized = True

        if VERBOSE:
            _log(db, f"[HAND_FT] Articulation API: {ARTICULATION_API_NAME}")
            _log(db, f"[HAND_FT] Robot prim path: {ROBOT_PRIM_PATH}")

            for candidates, resolved_name, row in zip(
                FINGER_JOINT_CANDIDATES,
                resolved_names,
                row_indices,
            ):
                _log(db, f"[HAND_FT] {candidates} -> {resolved_name}, force row {row}")

        return True

    except Exception as e:
        _log(db, f"[HAND_FT] Initialization failed: {e}")
        return False


# ============================================================
# OmniGraph callbacks
# ============================================================
def setup(db: og.Database):
    _reset_state(db)
    _set_zero_outputs(db)
    _log(db, "[HAND_FT] Script node setup called.")


def compute(db: og.Database):
    if not _try_initialize(db):
        _set_zero_outputs(db)
        return True

    try:
        forces = db.state.robot.get_measured_joint_forces()
        forces = _to_numpy_like(forces)

        if forces is None:
            _set_zero_outputs(db)
            return True

        # If a view wrapper returns shape (num_envs, num_joints + 1, 6),
        # use the first articulation.
        try:
            if len(forces.shape) == 3:
                forces = forces[0]
        except Exception:
            pass

        out = []

        for row in db.state.row_indices:
            wrench = forces[row]

            try:
                wrench = wrench.tolist()
            except Exception:
                pass

            fx = float(wrench[0]) * FORCE_SIGN[0]
            fy = float(wrench[1]) * FORCE_SIGN[1]
            fz = float(wrench[2]) * FORCE_SIGN[2]

            out.extend([fx, fy, fz])

        _set_output_data(db, out)
        return True

    except Exception as e:
        _log(db, f"[HAND_FT] Read failed: {e}")
        db.state.initialized = False
        _set_zero_outputs(db)
        return True


def cleanup(db: og.Database):
    _reset_state(db)
    _log(db, "[HAND_FT] Script node cleaned up.")
