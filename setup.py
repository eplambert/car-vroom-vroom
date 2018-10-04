# -*- coding: utf-8 -*-

from setuptools import setup, find_packages


with open('README.md') as f:
    readme = f.read()


setup(
    name='Car-Vroom-Vroom',
    version='0.0.1',
    description='Self Driving Car Scripts',
    long_description=readme,
    author='Eric Lambert',
    author_email='eplamber@mail.usf.edu',
    url='https://github.com/eplambert/car-vroom-vroom',
    license=license,
    packages=find_packages(exclude=('tests', 'docs'))
)