# amr-butler-dev — Gazebo Harmonic development container

Container for the **new** Gazebo Harmonic (gz-sim 8) stack: `amr_butler_description`,
`amr_worlds`, `amr_concierge`. It is separate from `amr-ros-dev` because the
non-official `ros-humble-ros-gzharmonic` binaries (from packages.osrfoundation.org)
conflict with the Gazebo Classic ROS packages used by `amr_description` / `amr_vision`.

Both containers share the same `/workspace` volume (the repo root), so the colcon
workspace is common. This service runs with **`ROS_DOMAIN_ID=42`** so its DDS traffic
is isolated from the Classic container (domain 0) despite host networking — set both
containers to the same domain ID only if you deliberately want them to talk.

## Build & run

On the host, once per session (X11 access for GUI apps):

```bash
xhost +local:root
```

Then:

```bash
cd docker
docker compose build amr-butler-dev
docker compose up -d amr-butler-dev
docker exec -it amr-butler-dev bash
```

Both `amr-ros-dev` and `amr-butler-dev` can be up at the same time.

## Smoke test

Inside the container (`docker exec -it amr-butler-dev bash`):

**1. Headless sim** — should print gz-sim 8.x startup logs and keep running
(Ctrl-C to stop). `shapes.sdf` ships with gz-harmonic and is found automatically:

```bash
gz sim -s -r shapes.sdf
```

**2. GUI sim** — a window with a few primitive shapes should appear:

```bash
gz sim shapes.sdf
```

> **VM / no-GPU users:** the default `ogre2` engine with `LIBGL_ALWAYS_SOFTWARE=1`
> (already set in the compose service) is the verified-working configuration —
> tested in VirtualBox without 3D acceleration, shadows render correctly. Only if
> that fails, fall back to the older engine:
>
> ```bash
> gz sim --render-engine ogre shapes.sdf
> ```

**3. ros_gz packages present** — should list `ros_gz_bridge`, `ros_gz_sim`,
`ros_gz_image`, `ros_gz_interfaces`, ...:

```bash
ros2 pkg list | grep ros_gz
```

**4. Bridge test (`/clock`)** — with the headless sim from step 1 running
(the `-r` flag matters: the clock only advances while the sim runs), open a
second shell in the container and start the bridge:

```bash
ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
```

Then in a third shell, confirm ROS sees simulated time:

```bash
ros2 topic echo /clock --once
```

## Troubleshooting

**"Authorization required, but no authorization protocol specified"** (GUI apps fail
to open a window): the container is not allowed to use the host's X server. Run this
on the **host**, once per session (it does not persist across reboots/logins):

```bash
xhost +local:root
```

**Rendering in a VM / without a GPU:** the verified-working configuration is the
**default `ogre2` engine + `LIBGL_ALWAYS_SOFTWARE=1`** (already set in the compose
service) — confirmed in VirtualBox with no 3D acceleration, including correct
shadows. `gz sim --render-engine ogre` also works but is a fallback only; don't
reach for it unless the default fails.

**Never set `QT_QUICK_BACKEND=software`.** It segfaults the Gazebo Harmonic GUI
(crash in MinimalScene / QOpenGLContext). It is not needed: Qt renders fine through
the Mesa software pipeline that `LIBGL_ALWAYS_SOFTWARE=1` selects.

**Sensor rendering shares the software pipeline.** Camera and GPU-LiDAR sensors
render through the same Mesa software rasterizer as the GUI, so on VMs keep camera
resolutions and sensor update rates modest (e.g. 640×480 @ 10–15 Hz rather than
full-HD @ 30 Hz) or the sim's real-time factor will collapse.

## Rules reminder

- Never install `gazebo_ros_pkgs` / Gazebo Classic packages in this image, and never
  add `ros_gz` dependencies to `amr_description` / `amr_vision` (see CLAUDE.md).
