from setuptools import find_packages, setup

setup(
    name="phys_sim",
    version="0.1",
    packages=find_packages(),
    install_requires=[
        "pybullet",
    ],
    python_requires=">=3.10,<3.12",
    author="6 Figures",
    description="A package to simulate the ur3e robot arm physics for security usages",
    package_data={
        "simulator": [
            "simulator/urdfs/*",
        ],
    },
)
