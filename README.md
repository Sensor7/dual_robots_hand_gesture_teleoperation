# dual_panda_demo
dual_panda_demo ros2


# prepare
1. install
```
wget http://fishros.com/install -O fishros && . fishros
install ros-humble-desktop
install rosdepc

git clone https://github.com/wawatt/dual_panda_demo.git
cd src
sudo apt update && rosdepc install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```


# build and run
```
colcon build --cmake-args  -DCMAKE_BUILD_TYPE=Release
. install/setup.bash

ros2 launch dual_panda_description demo.launch.py
ros2 launch dual_panda_moveit_config demo_my.launch.py
ros2 launch dual_panda_demo my_move_group.launch.py

```

# xxx
- ros2 launch moveit_setup_assistant setup_assistant.launch.py

# issues
- Q: <br />
  [ERROR] [1646681894.535920515, 851.070000000]: IKConstraintSampler received dirty robot state, but valid transforms are required. Failing.

  A: <br />
  clone MoveIt2 2.5.5(humble), change func:sample to MoveIt 2.10
  src/moveit_core/constraint_samplers/src/union_constraint_sampler.cpp 
  ```
  bool UnionConstraintSampler::sample(moveit::core::RobotState& state, const moveit::core::RobotState& reference_state,
                                      unsigned int max_attempts)
  {
    state = reference_state;
    for (ConstraintSamplerPtr& sampler : samplers_)
    {
      // ConstraintSampler::sample returns states with dirty link transforms (because it only writes values)
      // but requires a state with clean link transforms as input. This means that we need to clean the link
      // transforms between calls to ConstraintSampler::sample.
      state.updateLinkTransforms();
      if (!sampler->sample(state, max_attempts))
        return false;
    }
    return true;
  }
  ```