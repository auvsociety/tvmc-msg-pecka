from setuptools import setup, find_packages

package_name = "rose_tvmc_msg"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Rose",
    maintainer_email="cs21b1041@iiitdm.ac.in",
    description="TVMC messages and interfaces in ROS 2",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "interface = scripts.others.interface:main",
        ],
    },
)

