#!/usr/bin/env python3
"""Create only a Gazebo adapter; the frozen runtime URDF stays byte-identical."""
import sys
import xml.etree.ElementTree as E
from pathlib import Path
import rospkg
root=E.parse(Path(rospkg.RosPack().get_path('rml63_tools_description'))/'urdf/fixed_runtime.urdf').getroot()
# Planning retains the complete original collision model. ODE uses no robot
# contact surfaces in this scene-placement simulation (no grip physics claim).
for parent in root.iter():
    for child in list(parent):
        if child.tag in ('collision','dynamics'): parent.remove(child)
        elif child.tag=='selfCollide': child.text='false'
for i,t in enumerate(root.findall('transmission')): t.set('name','sim_transmission_%d'%i)
E.SubElement(root,'link',name='world')
j=E.SubElement(root,'joint',name='world_anchor',type='fixed')
E.SubElement(j,'parent',link='world'); E.SubElement(j,'child',link='dummy')
E.SubElement(j,'origin',xyz='0 0 0.5',rpy='0 0 -3.14')
for plugin in root.iter('plugin'):
    if plugin.get('filename')=='libgazebo_ros_control.so':
        E.SubElement(plugin,'robotParam').text='/sim_robot_description'
print(E.tostring(root,encoding='unicode'))
