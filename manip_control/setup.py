from setuptools import setup, find_packages

setup(
    name="manip_control",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "numpy",
        "roboticstoolbox-python",
        "spatialmath-python",
        "spatialgeometry",
    ],
    python_requires=">=3.8",
    author="Tomek Żebrowski",
    author_email="zebromek@gmail.com",
    description="A RTB-based robot manipulation control package",
    keywords="robotics, rtb, manipulation",
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Science/Research",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
    ],
) 