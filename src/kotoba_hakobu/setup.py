from setuptools import find_packages, setup

package_name = 'kotoba_hakobu'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matsuy71',
    maintainer_email='246271979+matsuy71@users.noreply.github.com',
    description='TODO:Listen. Confirm. Carry. Report. A language-driven pick-and-place robot on CRANE+ V2.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'try_joint_goal = kotoba_hakobu.try_joint_goal:main',
            'try_pose_goal = kotoba_hakobu.try_pose_goal:main',
            'check_reach = kotoba_hakobu.check_reach:main',
            'try_level_move = kotoba_hakobu.try_level_move:main',
            'try_grasp = kotoba_hakobu.try_grasp:main',
            'try_serve = kotoba_hakobu.try_serve:main',
            'bowl_locator = kotoba_hakobu.bowl_locator:main',
            'serve_server = kotoba_hakobu.serve_server:main',
            'teishu_console = kotoba_hakobu.teishu_console:main',
            'measure_color = kotoba_hakobu.measure_color:main',
            'compare_locators = kotoba_hakobu.camera_locator:main',
            'eval_understanding = kotoba_hakobu.eval_understanding:main',
        ],
    },
)
