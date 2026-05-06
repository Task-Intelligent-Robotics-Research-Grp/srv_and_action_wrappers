from setuptools import setup, find_packages
from glob import glob

package_name = "srv_and_action_wrappers"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=['test']),
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Toshio Ueshiba",
    maintainer_email="t.ueshiba@aist.go.jp",
    description="Package with basic routines for moving robots with MoveIt",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        'console_scripts': [
            'test_simple_action_client = ' + package_name + '.test_simple_action_client:main',
            'test_action_server = ' + package_name + '.test_action_server:main',
        ],
    },
)
