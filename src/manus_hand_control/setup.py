from setuptools import find_packages, setup

package_name = "manus_hand_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/aidin_hand_control.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="eunseop",
    maintainer_email="eunseop@example.com",
    description="MANUS glove to AIDIN hand command bridge for dualarm_forcecon.",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "aidin_hand_control = manus_hand_control.aidin_hand_control:main",
        ],
    },
)
