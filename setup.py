from glob import glob
from setuptools import setup
setup(name='runa_sim', version='0.1.0', packages=['runa_sim'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/runa_sim']),
                  ('share/runa_sim', ['package.xml'])] +
                 [('share/runa_sim/' + d, glob(d + '/*')) for d in ['launch', 'config', 'urdf']],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='PyLoT Robotics', maintainer_email='163533636+PyLoT-Robotics@users.noreply.github.com',
      description='Runa simulation for ROS 2 Jazzy and Gazebo Harmonic', license='Apache-2.0',
      entry_points={'console_scripts': [
          'compatibility = runa_sim.compatibility:main',
          'rgbd_bridge = runa_sim.rgbd_bridge:main',
          'smoke_test = runa_sim.smoke_test:main']})
