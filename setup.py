from distutils.core import setup

setup(name='robotic_arm',
      description='Python interface for fischertechnik 6-axis robot arm',
      author='David Holtz',
      license='MIT',
      package_dir={'robotic_arm': 'robotic_arm'},
      package_data={'': ['LICENSE']}
      )