from setuptools import setup, find_packages

setup(
    name="el_a3_sdk",
    version="1.0.0",
    description="EL-A3 7-DOF Robotic Arm Pure Python SDK (Direct CAN, multi-arm)",
    author="EL-A3 Team",
    packages=find_packages(),
    python_requires=">=3.8",
    install_requires=[
        "numpy",
        "pyyaml",
    ],
    extras_require={
        "dynamics": ["pin"],
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: POSIX :: Linux",
    ],
)
