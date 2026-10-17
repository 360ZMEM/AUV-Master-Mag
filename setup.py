"""Install the magnetic tracking library for ROS entry points outside this tree."""

from setuptools import find_packages, setup


setup(
    name="auv-mag-tracking",
    version="0.2.0",
    description="AUV magnetic cable search and inspection algorithms",
    package_dir={"": "src"},
    packages=find_packages("src"),
    python_requires=">=3.10",
    install_requires=["numpy", "scipy"],
)
