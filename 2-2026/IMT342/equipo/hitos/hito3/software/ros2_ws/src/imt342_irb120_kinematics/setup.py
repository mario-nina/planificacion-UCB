from setuptools import setup

name = "imt342_irb120_kinematics"
setup(
    name=name,
    tests_require=["pytest"],
    version="0.1.0",
    packages=[name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + name]),
        ("share/" + name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    maintainer="Equipo IMT-342",
    maintainer_email="imt342@example.invalid",
    description="IRB 120 URDF-based kinematics for IMT-342",
    license="UNLICENSED",
)
