"""Convert URDF to SDF and isolate robot visuals from idealized 2D LiDAR rays."""
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


def prepare_model(urdf_path, sdf_path):
    result = subprocess.run(['gz', 'sdf', '-p', str(urdf_path)], check=True,
                            capture_output=True, text=True)
    root = ET.fromstring(result.stdout)
    # Cameras see all bits. Lidar sees bit 1; robot-only visuals occupy bit 2.
    for visual in root.findall('model/link/visual'):
        flags = visual.find('visibility_flags')
        if flags is None:
            flags = ET.SubElement(visual, 'visibility_flags')
        flags.text = '2'
    ET.indent(root)
    Path(sdf_path).write_text(ET.tostring(root, encoding='unicode'))
