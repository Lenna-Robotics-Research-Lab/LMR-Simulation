from glob import glob

from setuptools import setup

package_name = "lmr2_stereo_depth"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
        ("share/" + package_name + "/textures", glob("textures/*.png")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Erfan Riazati",
    maintainer_email="erf.riazati@gmail.com",
    description="Stereo depth + point cloud for the simulated LMR2 stereo cameras.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "stereo_depth_node = lmr2_stereo_depth.stereo_depth_node:main",
            "make_sim_calibration = lmr2_stereo_depth.make_sim_calibration:main",
        ],
    },
)
