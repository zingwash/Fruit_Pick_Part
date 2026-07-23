# RM65-B-V model source

The URDF and STL files in this directory are unmodified model assets downloaded
from RealMan Robotics' official public `rm_models` repository on 2026-07-22:

- Documentation: https://develop.realman-robotics.com/en/robot4th/download/model/
- Repository: https://github.com/RealManRobot/rm_models/tree/main/RM65/urdf/RM65-B-V

Only `base_link` and `link_1` through `link_6` are loaded by TeachPendant.
The camera/end-tool meshes included by the upstream RM65-B-V package are not
copied or displayed, in accordance with the current simulation scope.

The model is used only for visual trajectory preview. It does not certify the
real robot path, controller interpolation, stopping distance, or collision
safety.
