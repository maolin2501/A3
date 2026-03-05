from setuptools import setup, find_packages

setup(
    name="el_a3_sdk",
    version="0.4.0",
    description="EL-A3 7-DOF Robotic Arm Python SDK (Direct CAN + ROS Control dual mode, multi-arm)",
    author="EL-A3 Team",
    packages=find_packages(),
    python_requires=">=3.8",
    install_requires=[
        "numpy",
        "pyyaml",
    ],
    extras_require={
        "dynamics": ["pin"],       # pinocchio Python bindings
        "ros": [],                 # rclpy 及 ROS2 消息包由 ROS2 环境提供
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: POSIX :: Linux",
    ],
)
