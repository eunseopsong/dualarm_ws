#!/usr/bin/env python3
from pathlib import Path
import xml.etree.ElementTree as ET

SRC = Path("/home/eunseop/dualarm_ws/src/dualarm_kinematics/urdf/rby1a_official_raw.urdf")
DST = Path("/home/eunseop/dualarm_ws/src/dualarm_kinematics/urdf/rby1a_kdl.urdf")

tree = ET.parse(SRC)
root = tree.getroot()

# 1) Remove all <collision> blocks.
# Reason: official URDF uses <capsule>, which standard ROS urdfdom does not parse.
for link in root.findall("link"):
    for collision in list(link.findall("collision")):
        link.remove(collision)

# 2) Ensure every revolute/continuous/prismatic joint limit has effort and velocity.
# Some parsers fail if these attributes are missing.
for joint in root.findall("joint"):
    jtype = joint.attrib.get("type", "")
    if jtype in ("revolute", "prismatic"):
        limit = joint.find("limit")
        if limit is None:
            limit = ET.SubElement(joint, "limit")
        if "lower" not in limit.attrib:
            limit.set("lower", "-3.141592653589793")
        if "upper" not in limit.attrib:
            limit.set("upper", "3.141592653589793")
        if "effort" not in limit.attrib:
            limit.set("effort", "1000.0")
        if "velocity" not in limit.attrib:
            limit.set("velocity", "10.0")

    elif jtype == "continuous":
        limit = joint.find("limit")
        if limit is None:
            limit = ET.SubElement(joint, "limit")
        if "effort" not in limit.attrib:
            limit.set("effort", "1000.0")
        if "velocity" not in limit.attrib:
            limit.set("velocity", "10.0")

ET.indent(tree, space="  ")
tree.write(DST, encoding="utf-8", xml_declaration=True)

print(f"[OK] Wrote cleaned URDF: {DST}")
