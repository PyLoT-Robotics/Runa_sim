"""Prepare the fork's field for Harmonic without modifying the dependency."""
from pathlib import Path
import xml.etree.ElementTree as ET
import xacro


def prepare_world(source, models, output, full_field=False):
    root = ET.fromstring(xacro.process_file(str(source)).toxml())
    world = root.find('world')
    if world is None:
        raise ValueError('World file has no world element')
    omitted = []
    missing = []
    for node in list(world.findall('include')):
        uri = node.findtext('uri', '')
        if uri.startswith('model://'):
            relative = uri[len('model://'):]
            if not any((Path(directory) / relative).is_dir() for directory in models):
                if full_field:
                    missing.append(uri)
                else:
                    omitted.append(uri)
                    world.remove(node)
    if missing:
        raise FileNotFoundError('Full-field assets missing: ' + ', '.join(missing) +
                                '. Import full-field.repos or use full_field:=false.')
    for plugin in world.findall('plugin'):
        if plugin.get('name') == 'gz::sim::systems::Sensors' and plugin.find('render_engine') is None:
            ET.SubElement(plugin, 'render_engine').text = 'ogre2'
    # The physical arena is retained; no substitute plane or field is generated.
    ET.indent(root)
    Path(output).write_text(ET.tostring(root, encoding='unicode'))
    return world.get('name'), omitted
