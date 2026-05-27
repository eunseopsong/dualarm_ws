import omni.graph.core as og
from omni.isaac.core.articulations import Articulation

# ============================================================
# User settings
# ============================================================
ROBOT_PRIM_PATH = "/World/aidin_rby1"  # Isaac Sim stage articulation prim path
VERBOSE = True

# Publish order for std_msgs/msg/Float32MultiArray.data:
#   left thumb/index/middle/ring/baby, then right thumb/index/middle/ring/baby
#   each finger = Fx, Fy, Fz
# Joint candidates per finger.
# - *_joint_tactile exists in the AIDIN hand xacro as a fingertip fixed joint.
# - Some Isaac USD imports do not keep fixed joints in articulation metadata.
#   In that case, fall back to *_joint4, which is visible in /isaac_joint_states.
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

# Optional per-axis sign calibration. Change after checking force direction.
FORCE_SIGN = [1.0, 1.0, 1.0]


# ============================================================
# Internal helpers
# ============================================================
def _reset_state(db: og.Database):
    db.state.robot = None
    db.state.row_indices = []
    db.state.resolved_joint_names = []
    db.state.initialized = False


def _log(db: og.Database, msg: str):
    try:
        db.log_info(msg)
    except Exception:
        print(msg)


def _set_output_data(db: og.Database, data):
    # Recommended Script Node output attribute name: data
    # If you created the output as "Data" in the UI, this also supports that.
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


def _resolve_joint_name(target_name: str, joint_indices):
    """
    USD/Isaac usually exposes URDF joint names directly. Some imported assets
    can expose scoped names, so fall back to suffix matching.
    """
    if target_name in joint_indices:
        return target_name

    suffix_matches = [
        name for name in joint_indices.keys()
        if str(name).endswith(target_name)
    ]
    if len(suffix_matches) == 1:
        return suffix_matches[0]

    return None


def _try_initialize(db: og.Database) -> bool:
    """
    After physics handles are ready, find each tactile fixed joint's internal
    joint index. get_measured_joint_forces() row index is joint_index + 1.
    """
    if getattr(db.state, "initialized", False):
        return True

    try:
        if db.state.robot is None:
            db.state.robot = Articulation(ROBOT_PRIM_PATH)

        try:
            db.state.robot.initialize()
        except Exception:
            pass

        if not db.state.robot.handles_initialized:
            return False

        metadata = db.state.robot._articulation_view._metadata
        joint_indices = metadata.joint_indices
        joint_names = metadata.joint_names

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

            row_indices.append(joint_indices[resolved_name] + 1)
            resolved_names.append(resolved_name)

        if missing:
            _log(db, f"[HAND_FT] Missing finger force joints: {missing}")
            _log(db, f"[HAND_FT] Available joints: {list(joint_names)}")
            return False

        db.state.row_indices = row_indices
        db.state.resolved_joint_names = resolved_names
        db.state.initialized = True

        if VERBOSE:
            _log(db, f"[HAND_FT] Robot prim path: {ROBOT_PRIM_PATH}")
            for candidates, resolved_name, row in zip(FINGER_JOINT_CANDIDATES, resolved_names, row_indices):
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

        if forces is None:
            _set_zero_outputs(db)
            return True

        out = []
        for row in db.state.row_indices:
            fx, fy, fz, _tx, _ty, _tz = forces[row]
            out.extend([
                float(fx) * FORCE_SIGN[0],
                float(fy) * FORCE_SIGN[1],
                float(fz) * FORCE_SIGN[2],
            ])

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
