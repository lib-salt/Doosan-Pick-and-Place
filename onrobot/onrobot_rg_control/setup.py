from setuptools import find_packages, setup
from glob import glob

# fetch values from package.xml

package_name = 'onrobot_rg_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/nodes', [f for f in glob('nodes/*') if not f.endswith('DualChanger')]),
        ('share/' + package_name + '/msg', glob('msg/*')),
        ('share/' + package_name + '/srv', glob('srv/*')),
        ('share/' + package_name + '/src/onrobot_rg_control', glob('/src/onrobot_rg_control/*')),
    ],
    package_data={'': ['*.pyi', '*.marker']},
    maintainer='Takuya Kiyokawa',
    maintainer_email='kiyokawa@hlab.sys.es.osaka-u.ac.jp',
    description='onrobot_rg_control',
    license='MIT',
    install_requires=['rclpy', 'td_srvs', 'onrobot_rg_modbus_tcp'],
    entry_points={
        'console_scripts': [
            'onrobot_rg_control = onrobot_rg_control.onrobot_rg_control:main',
            'OnRobotRGSimpleController = onrobot_rg_control.OnRobotRGSimpleController:main',
        ],
    },
)
