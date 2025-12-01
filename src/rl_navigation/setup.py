from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rl_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'),
         glob('share/models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gabriel',
    maintainer_email='gabearod2@gmail.com',
    description='Package to deploy RL navigation.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go2_pose_command = rl_navigation.go2_pose_command:main',
            'go2_rl_nav_actions_onnx = rl_navigation.go2_rl_nav_actions_onnx:main',
            'go2_rl_nav_actions_rough_onnx = rl_navigation.go2_rl_nav_actions_onnx_real_rough:main',
            'go2_rl_flat_actions_onnx = rl_navigation.go2_rl_nav_actions_onnx_real_flat:main',
            
            'real_nav_rough_onnx = rl_navigation.go2_rl_nav_actions_onnx_real_nav:main',
            'go2_rl_nav_actions_jit = rl_navigation.go2_rl_nav_actions_jit:main',
        
            'go2_rl_nav_actions_rough_onnx_9000 = rl_navigation.go2_rl_nav_actions_onnx_real_rough_9000:main',
            'go2_infer_local = rl_navigation.go2_infer_local:main',
            'go2_infer_local_Hz = rl_navigation.go2_infer_local_Hz:main',
            'go2_infer_local_Hz_walker = rl_navigation.go2_infer_local_Hz_walker:main',
            'go2_infer_local_1 = rl_navigation.go2_infer_local_1:main',
            'go2_infer_local_Hz_1 = rl_navigation.go2_infer_local_Hz_1:main'
        ],
    },
)
