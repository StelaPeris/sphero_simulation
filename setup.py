import os
from glob import glob
from setuptools import setup

package_name = "sphero_simulation"


def tree(src, dst_root):
    """Sve datoteke iz src (rekurzivno) -> share/<paket>/<src>/..., zadrzava podfoldere (npr. meshes/)."""
    out = []
    for d, _, files in os.walk(src):
        if files:
            out.append((os.path.join(dst_root, d), [os.path.join(d, f) for f in files]))
    return out


share = os.path.join("share", package_name)
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name, package_name + ".control"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (share, ["package.xml"]),
        (os.path.join(share, "launch"), glob("launch/*.launch.py")),
        (os.path.join(share, "config"), glob("config/*.yaml")),
    ] + tree("models", share),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Stela",
    maintainer_email="perisstela@gmail.com",
    description="MuJoCo simulacija sfernog robota s njihalom (ROS 2)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "sim_node = sphero_simulation.sim_node:main",
            "sim_bridge = sphero_simulation.sim_bridge:main",
            "sim_pose_viz = sphero_simulation.sim_pose_viz:main",
            'follow_target = sphero_simulation.follow_target:main',
            'moving_point = sphero_simulation.moving_point:main',
        ],
    },
)