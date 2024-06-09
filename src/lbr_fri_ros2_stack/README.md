# lbr_fri_ros2_stack
[![Build status](https://github.com/lbr-stack/lbr_fri_ros2_stack/actions/workflows/build.yml/badge.svg?branch=humble)](https://github.com/lbr-stack/lbr_fri_ros2_stack/actions)
[![License](https://img.shields.io/github/license/lbr-stack/lbr_fri_ros2_stack)](https://github.com/lbr-stack/lbr_fri_ros2_stack/tree/humble?tab=Apache-2.0-1-ov-file#readme) 
[![Documentation Status](https://readthedocs.org/projects/lbr-stack/badge/?version=latest)](https://lbr-stack.readthedocs.io/en/latest/?badge=latest)
[![JOSS](https://joss.theoj.org/papers/c43c82bed833c02503dd47f2637192ef/status.svg)](https://joss.theoj.org/papers/c43c82bed833c02503dd47f2637192ef) 
[![Code Style: Black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)

ROS 2 packages for the KUKA LBR, including communication to the real robot via the Fast Robot Interface ([FRI](https://github.com/lbr-stack/fri)) and [Gazebo](http://gazebosim.org/) simulation support. Included are the `iiwa7`, `iiwa14`, `med7`, and `med14`.

<body>
    <table>
        <tr>
            <th align="left" width="25%">LBR IIWA 7 R800</th>
            <th align="left" width="25%">LBR IIWA 14 R820</th>
            <th align="left" width="25%">LBR Med 7 R800</th>
            <th align="left" width="25%">LBR Med 14 R820</th>
        </tr>
        <tr>
            <td align="center"><img src="https://raw.githubusercontent.com/lbr-stack/lbr_fri_ros2_stack/humble/lbr_fri_ros2_stack/doc/img/foxglove/iiwa7_r800.png" alt="LBR IIWA 7 R800"></td>
            <td align="center"><img src="https://raw.githubusercontent.com/lbr-stack/lbr_fri_ros2_stack/humble/lbr_fri_ros2_stack/doc/img/foxglove/iiwa14_r820.png" alt="LBR IIWA 14 R820"></td>
            <td align="center"><img src="https://raw.githubusercontent.com/lbr-stack/lbr_fri_ros2_stack/humble/lbr_fri_ros2_stack/doc/img/foxglove/med7_r800.png" alt="LBR Med 7 R800"></td>
            <td align="center"><img src="https://raw.githubusercontent.com/lbr-stack/lbr_fri_ros2_stack/humble/lbr_fri_ros2_stack/doc/img/foxglove/med14_r820.png" alt="LBR Med 14 R820"></td>
        </tr>
    </table>
</body>

## Documentation
Full documentation available on [Read the Docs](https://lbr-stack.readthedocs.io/en/latest).

## Quick Start
1. Install ROS 2 development tools

    ```shell
    sudo apt install ros-dev-tools
    ```

2. Create a workspace, clone, and install dependencies

    ```shell
    source /opt/ros/humble/setup.bash
    export FRI_CLIENT_VERSION=1.15
    mkdir -p lbr-stack/src && cd lbr-stack
    vcs import src --input https://raw.githubusercontent.com/lbr-stack/lbr_fri_ros2_stack/humble/lbr_fri_ros2_stack/repos-fri-${FRI_CLIENT_VERSION}.yaml
    rosdep install --from-paths src -i -r -y
    ```

> [!NOTE]
> FRI client is cloned from [fri](https://github.com/lbr-stack/fri) and must be available as branch, refer [README](https://github.com/lbr-stack/fri?tab=readme-ov-file#contributing).

3. Build

    ```shell
    colcon build --symlink-install
    ```

4. Launch the simulation via

    ```shell
    source install/setup.bash
    ros2 launch lbr_bringup bringup.launch.py \
        model:=iiwa7 # [iiwa7, iiwa14, med7, med14] \
        sim:=true # [true, false] \
        rviz:=true # [true, false] \
        moveit:=true # [true, false]
    ```

> [!TIP]
> List all arguments for the launch file via `ros2 launch lbr_bringup bringup.launch.py -s`

Now, run the [demos](https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_demos/doc/lbr_demos.html). To get started with the real robot, checkout the [Hardware Setup](https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_fri_ros2_stack/doc/hardware_setup.html).

## Citation
If you enjoyed using this repository for your work, we would really appreciate ❤️ if you could leave a ⭐ and / or cite it, as it helps us to continue offering support.

```
@misc{huber2023lbrstack,
      title={LBR-Stack: ROS 2 and Python Integration of KUKA FRI for Med and IIWA Robots}, 
      author={Martin Huber and Christopher E. Mower and Sebastien Ourselin and Tom Vercauteren and Christos Bergeles},
      year={2023},
      eprint={2311.12709},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```

## Acknowledgements
### Open Source Contributors
We would like to acknowledge all contributors 🚀

**lbr_fri_ros2_stack**

[![lbr_fri_ros2_stack contributors](https://contrib.rocks/image?repo=lbr-stack/lbr_fri_ros2_stack&max=20)](https://github.com/lbr-stack/lbr_fri_ros2_stack/graphs/contributors)

**fri**

[![fri contributors](https://contrib.rocks/image?repo=lbr-stack/fri&max=20)](https://github.com/lbr-stack/fri/graphs/contributors)

### Organizations and Grants
We would further like to acknowledge following supporters:

| Logo | Notes |
|:--:|:---|
| <img src="https://medicalengineering.org.uk/wp-content/themes/aalto-child/_assets/images/medicalengineering-logo.svg" alt="wellcome" width="150" align="left">  | This work was supported by core and project funding from the Wellcome/EPSRC [WT203148/Z/16/Z; NS/A000049/1; WT101957; NS/A000027/1]. |
| <img src="https://upload.wikimedia.org/wikipedia/commons/thumb/b/b7/Flag_of_Europe.svg/1920px-Flag_of_Europe.svg.png" alt="eu_flag" width="150" align="left"> | This project has received funding from the European Union's Horizon 2020 research and innovation programme under grant agreement No 101016985 (FAROS project). |
| <img src="https://rvim.online/author/avatar_hu8970a6942005977dc117387facf47a75_62303_270x270_fill_lanczos_center_2.png" alt="RViMLab" width="150" align="left"> | Built at [RViMLab](https://rvim.online/). |
| <img src="https://avatars.githubusercontent.com/u/75276868?s=200&v=4" alt="King's College London" width="150" align="left"> | Built at [CAI4CAI](https://cai4cai.ml/). |
| <img src="https://upload.wikimedia.org/wikipedia/commons/1/14/King%27s_College_London_logo.svg" alt="King's College London" width="150" align="left"> | Built at [King's College London](https://www.kcl.ac.uk/). |
