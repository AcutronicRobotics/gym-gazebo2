from gym.envs.registration import register

# Gazebo
# ----------------------------------------
# MARA
register(
    id='MARATop3DOFROS2-v0',
    entry_point='gym_gazebo.envs.MARA:GazeboMARATop3DOFv0EnvROS2',
)

register(
    id='MARAOrientCollisionROS2-v0',
    entry_point='gym_gazebo.envs.MARA:GazeboMARATopOrientCollisionv0EnvROS2',
)

register(
    id='MARAOrientVisionCollisionROS2-v0',
    entry_point='gym_gazebo.envs.MARA:GazeboMARATopOrientVisionCollisionv0EnvROS2',
)

register(
    id='MARAOrientVisionROS2-v0',
    entry_point='gym_gazebo.envs.MARA:GazeboMARATopOrientVisionv0EnvROS2',
)