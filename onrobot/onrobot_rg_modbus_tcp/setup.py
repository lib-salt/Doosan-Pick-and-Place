from setuptools import setup
from setuptools import find_packages


package_name ='onrobot_rg_modbus_tcp'

# fetch values from package.xml
setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    package_data={
        'onrobot_rg_modbus_tcp': [
            'comModbusTcp.py'
        ],
    },
    data_files=[
        ('share/' + package_name, ['package.xml'])
    ],
    # python_requires>='3.6',
    install_requires=['rclcpp', 'rcutils', 'pymodbus==2.5.3'],
    maintainer='Takuya Kiyokawa',
    maintainer_email='kiyokawa@hlab.sys.es.osaka-u.ac.jp',
    author='Takuya Kiyokawa',
    description='A stack to communicate with OnRobot RG grippers using the Modbus/TCP protocol',
    license='MIT',
    entry_points={
        'console_scripts': [
            'comModbusTcp = onrobot_rg_modbus_tcp.comModbusTcp:main'
        ],
    },
)


