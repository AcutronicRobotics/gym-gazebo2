from setuptools import setup, find_packages
import sys, os.path

# Don't import gym module here, since deps may not be installed
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'gym-gazebo-ros2'))

setup(name='gym-gazebo-ros2',
      version='0.0.1',
      packages=[package for package in find_packages()
                if package.startswith('gym_gazebo2')],
      install_requires=['gym>=0.2.3'],
      description='Gym-gazebo-2: A toolkit for developing and comparing your reinforcement learning agents using Gazebo and ROS 2.',
      url='https://github.com/erlerobot/gym-gazebo-ros2',
      author='Acutronic Robotics',
)
