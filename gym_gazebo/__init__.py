from gym.envs.registration import register

# Gazebo
# ----------------------------------------
# MARA
register(
    id='MARA-v0',
    entry_point='gym_gazebo.envs.MARA:GazeboMARAEnv',
)

register(
    id='MARAOrientCollision-v0',
    entry_point='gym_gazebo.envs.MARA:GazeboMARAOrientCollisionEnv',
)

register(
    id='MARAOrientVisionCollision-v0',
    entry_point='gym_gazebo.envs.MARA:GazeboMARAOrientVisionCollisionEnv',
)

register(
    id='MARAOrientVision-v0',
    entry_point='gym_gazebo.envs.MARA:GazeboMARAOrientVisionEnv',
)