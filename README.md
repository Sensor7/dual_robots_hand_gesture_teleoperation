# dual_panda_demo
dual_panda_demo ros2


# prepare
1. install ros2 humble

2. install moveit2 and ros2 control

``` bash
sudo apt install ros-humble-moveit*
sudo apt install ros-humble-tf-transformations
```

3. install mediapipe, torch and torch vision
```bash
pip install mediapipe
pip install torch==2.5.1 torchvision==0.20.1 torchaudio==2.5.1 --index-url https://download.pytorch.org/whl/cu121
pip install transformers
pip install --upgrade transforms3d

```

**if you don't have cuda gpu, remember to deactivate the DL monocular camera depth estimation module, otherwise it will run very slow**

4. 



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