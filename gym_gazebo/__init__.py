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