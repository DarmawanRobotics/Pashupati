# Dev container

ROS 2 Humble with every dependency of `src/` (rosdep), Livox-SDK2 and the zsibot SDK, plus a
desktop over VNC for RViz. The repo is bind-mounted at the same path as on the robot
(`/home/robot/dev/Pashupati`), so configs and launch defaults behave the same. `build/`,
`install/` and `log/` live in Docker volumes (fast on macOS, kept across restarts).

```bash
script/dev.sh up        # first run builds the image (~15 min), then starts the container
script/dev.sh build     # colcon build (symlink-install, Release, compile_commands.json)
script/dev.sh test      # colcon test + results
script/dev.sh shell     # shell with ROS and the workspace sourced
script/dev.sh vnc       # macOS Screen Sharing; or http://localhost:6080/vnc.html
script/dev.sh down      # stop (volumes kept), script/dev.sh clean to wipe build/install/log
```

VNC password defaults to `pashupati` (`VNC_PASSWORD=... script/dev.sh up` to change); ports are
bound to localhost only. Inside the desktop, right-click for a terminal and run `rviz2`.

Rebuild the image (`script/dev.sh up` does it automatically) after a `package.xml` gains a
dependency or a vendor SDK changes; code changes only need `script/dev.sh build`.

The genisom submodule must contain `script/setup.sh` (it installs the zsibot SDK into the image).
Docker Desktop on macOS cannot do DDS multicast to the robot, so use the container for building,
tests and offline tools; view the live robot over VNC on the robot or a Linux laptop on its network.
