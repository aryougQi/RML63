#!/usr/bin/env python3
"""Package the unmodified official link geometry and linkage as a Xacro macro.

Source and revision are recorded in upstream/SOURCE.txt. The fixed branch is
optional for calibration with an unchanging aperture, not live joint feedback.
"""
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET
from xml.dom import minidom

ROOT = Path(__file__).resolve().parents[1]
NS = "http://www.ros.org/wiki/xacro"
ET.register_namespace("xacro", NS)


def clean(element):
    element.text = None if element.text is None or not element.text.strip() else element.text
    element.tail = None
    for child in element:
        clean(child)


def main():
    original = ET.parse(ROOT / "upstream/EG2-4C2.urdf").getroot()
    root = ET.Element("robot", {"name": "eg2_4c2"})
    macro = ET.SubElement(root, "{" + NS + "}macro", {
        "name": "eg2_4c2", "params": "parent xyz rpy fixed:=false angle:=0"})
    for source in original:
        if source.tag == "transmission":
            # RealMan's existing gripper protocol remains responsible for hardware control.
            continue
        child = deepcopy(source)
        for mesh in child.iter("mesh"):
            mesh.set("filename", mesh.attrib["filename"].replace(
                "package://EG2-4C2/meshes/",
                "package://rml63_tools_description/meshes/eg2_4c2/"))
        if child.tag != "joint":
            macro.append(child)
            continue
        articulated = ET.SubElement(macro, "{" + NS + "}unless", {"value": "${fixed}"})
        articulated.append(deepcopy(child))  # Keep original axis, limits, origin and mimic intact.
        frozen = ET.SubElement(macro, "{" + NS + "}if", {"value": "${fixed}"})
        child.set("type", "fixed")
        axis = child.find("axis").attrib["xyz"].split()
        mimic = child.find("mimic")
        multiplier = float(mimic.attrib.get("multiplier", "1")) if mimic is not None else 1.0
        # The six joints in this pinned official model all rotate about +/- Y.
        assert axis[0] == axis[2] == "0" and abs(float(axis[1])) == 1
        sign = float(axis[1]) * multiplier
        assert child.find("origin").attrib["rpy"] == "0 0 0"
        child.find("origin").set("rpy", "0 ${%s * float(angle)} 0" % sign)
        for item in list(child):
            if item.tag in ("axis", "limit", "mimic", "dynamics"):
                child.remove(item)
        frozen.append(child)
    mount = ET.SubElement(macro, "joint", {"name": "eg2_mount_joint", "type": "fixed"})
    ET.SubElement(mount, "parent", {"link": "${parent}"})
    ET.SubElement(mount, "child", {"link": "4C2_baselink"})
    ET.SubElement(mount, "origin", {"xyz": "${xyz}", "rpy": "${rpy}"})
    clean(root)
    output = minidom.parseString(ET.tostring(root)).toprettyxml(indent="  ")
    (ROOT / "urdf/eg2_4c2.xacro").write_text(output)


if __name__ == "__main__":
    main()
