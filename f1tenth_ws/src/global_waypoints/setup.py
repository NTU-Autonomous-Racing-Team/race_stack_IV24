from setuptools import setup, find_packages

setup(
    name="Global Waypoints",
    version="0.1.0",
    author="Jingxiang Mo",
    author_email="jingxiangmo@gmail.com",
    description="Generating and optimizing track",
    packages=find_packages(),
    python_requires='>=3.6',
    install_requires=[
        "numpy",
        "pandas",
        "Pillow",
        "matplotlib",
        "scipy",
        "scikit-image",
        "PyYAML",
    ],
)
