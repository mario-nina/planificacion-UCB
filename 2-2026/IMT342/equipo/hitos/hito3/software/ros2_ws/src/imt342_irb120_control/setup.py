from setuptools import setup

package = "imt342_irb120_control"
setup(
    name=package,
    tests_require=["pytest"],
    version="0.1.0",
    packages=[package],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package]),
        ("share/" + package, ["package.xml"]),
    ],
    description="RWS/RAPID backends for the IRB 120",
    maintainer="Equipo IMT-342",
    maintainer_email="imt342@example.invalid",
    license="UNLICENSED",
)
