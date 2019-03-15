from setuptools import setup, find_packages
import sys, os.path

# Don't import gym module here, since deps may not be installed
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'gym-gazebo2'))

setup(name='gym-gazebo2',
      version='0.0.2',
      packages=[package for package in find_packages()
                if package.startswith('gym_gazebo2')],
      description='Gym-gazebo-2: A toolkit for developing and comparing your reinforcement learning agents using Gazebo and ROS 2.',
      url='https://github.com/AcutronicRobotics/gym-gazebo2',
      author='Acutronic Robotics',
)
