# ament_python Packaging

Never hand-write a package skeleton. Generate it, then extend it:

```bash
ros2 pkg create --build-type ament_python --license Apache-2.0 \
  --dependencies rclpy std_msgs std_srvs --node-name my_node my_pkg
```

This creates the files `ros2 run` silently depends on. Do not delete them:

| File | Why it matters |
| :--- | :--- |
| `resource/my_pkg` (empty marker) + its `data_files` entry | Without it `ros2 pkg` and `ros2 launch` cannot find the package. |
| `setup.cfg` (`script_dir` / `install_scripts` = `$base/lib/my_pkg`) | Without it executables land in `bin/` and `ros2 run` reports "No executable found". |
| `('share/' + package_name, ['package.xml'])` in `data_files` | Required for dependency resolution at runtime. |
| `<export><build_type>ament_python</build_type></export>` | Without it colcon treats the package as plain CMake. |

## Extend `setup.py` for launch and config files

```python
import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'my_pkg'

setup(
    ...
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    entry_points={
        'console_scripts': ['my_node = my_pkg.my_node:main'],
    },
)
```

## `package.xml`

- One `<exec_depend>` per runtime import (`rclpy`, `std_msgs`, `tf2_ros`, `launch_ros`, ...).
- Add `<test_depend>launch_testing_ros</test_depend>` and `<test_depend>python3-pytest</test_depend>` when shipping launch tests.
- Replace the `TODO` license/description or `ament_copyright` and `colcon test` fail.

## Custom `.msg` / `.srv` / `.action`

An `ament_python` package cannot generate interfaces. Create a sibling `ament_cmake` package:

```bash
ros2 pkg create --build-type ament_cmake --license Apache-2.0 my_pkg_interfaces
```

Then add `rosidl_default_generators` (`<buildtool_depend>`), `rosidl_default_runtime` (`<exec_depend>`) and the `rosidl_interface_packages` member group to its `package.xml`, call `rosidl_generate_interfaces(...)` in `CMakeLists.txt`, and `<depend>my_pkg_interfaces</depend>` from the Python package.

## Build, test, run

```bash
colcon build --symlink-install --packages-select my_pkg
source install/setup.bash
colcon test --packages-select my_pkg && colcon test-result --verbose
ros2 run my_pkg my_node
```

`ament_flake8` enforces a 99-column limit and `ament_pep257` enforces docstring style; both run in the generated `test/` files.

`--symlink-install` links Python sources, but **new** launch/config files, entry points and `setup.py` changes still need a rebuild.
