import setuptools
import os

# https://stackoverflow.com/questions/26900328/install-dependencies-from-setup-py
theLibFolder = os.path.dirname(os.path.realpath(__file__))
requirementPath = theLibFolder + '/requirements.txt'
install_requires = []  # Here we'll get: ["gunicorn", "docutils>=0.3", "lxml==0.5a7"]
if os.path.isfile(requirementPath):
    with open(requirementPath) as f:
        install_requires = f.read().splitlines()

setuptools.setup(
    name="FH0A",
    version="0.0.2",
    description="Python Controller FH0A",
    # https://stackoverflow.com/questions/51286928/what-is-where-argument-for-in-setuptools-find-packages
    # DO NOT pack mock/test (like js) into output
    packages=setuptools.find_packages(where='src'),
    # special the root
    package_dir={
        '': 'src',
    },
    classifiers=[
    ],
    # install_requires=install_requires,
    install_requires=[
        'pyserial>=3.5',
    ],
    author='Jeremie',
    author_email='lucheng989898@protonmail.com',
    python_requires='>=3.8',
)

# pip install mypy
# mypy src/FH0A/__init__.py
# stubgen src/FH0A/

# <del> pip install wheel </del>
# <del> python setup.py bdist_wheel </del>

# https://blog.ganssle.io/articles/2021/10/setup-py-deprecated.html
# pip install build
# python -m build

