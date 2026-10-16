# Isaac Sim: From Zero to a Robot That Works — course assets

Every script here is one the course actually runs. They are written to
be executed inside the Isaac Sim container:

    ./python.sh /workspace/stages/<script>.py

with sim-workspace mounted at /workspace. Section 2 shows the container
command; section 9.8 covers the ROS 2 configuration in detail.

## What is NOT here

The debugging probes used while building the course are excluded — they
are dead ends, not lessons.

## Files by section

### Section 2 — Install and Prove the GPU

- `headless_capture.py`
- `headless_render.py`

### Section 3 — USD

- `usd_cameras.py`
- `usd_formats.py`
- `usd_lighting.py`
- `usd_materials.py`
- `usd_prims.py`
- `usd_references.py`
- `usd_units.py`
- `usd_xforms.py`

### Section 4 — PhysX

- `physx_approx.py`
- `physx_bodies.py`
- `physx_friction.py`
- `physx_inertia.py`
- `physx_joints.py`
- `physx_lid.py`
- `physx_mass.py`
- `physx_readback.py`
- `physx_timestep.py`
- `physx_tip.py`

### Section 5 — Build the Robot

- `carter_gui.py`
- `carter_scene.py`
- `carter_walkthrough.py`
- `course_robot.py`
- `robot_articulation.py`
- `robot_build.py`
- `robot_drive.py`
- `robot_jointstate.py`
- `robot_stability.py`
- `robot_urdf.py`
- `robot_vibrate.py`

### Section 6 — Sensors

- `make_cal_stage.py`
- `sensor_annotators.py`
- `sensor_camera.py`
- `sensor_contact.py`
- `sensor_depth.py`
- `sensor_imu.py`
- `sensor_intrinsics.py`
- `sensor_lidar.py`
- `sensor_rates.py`

### Section 7 — The Scene

- `frame_persp.py`
- `scene_environment.py`
- `scene_layers.py`
- `scene_lighting.py`
- `scene_materials.py`
- `scene_perf.py`
- `scene_simready.py`
- `scene_sky.py`
- `scene_vram_limit.py`

### Section 8 — Python and OmniGraph

- `py_articulation.py`
- `py_diffdrive.py`
- `py_extension.py`
- `py_omnigraph.py`
- `py_sensor_in_loop.py`
- `py_sim_loop.py`
- `py_standalone_vs_ext.py`
- `py_velocity_controller.py`
- `py_wont_step.py`

### Section 9 — ROS 2 Integration

- `ros_9_1_what_it_is.py`
- `ros_9_2_the_mismatch.py`
- `ros_9_3_joints_tf.py`
- `ros_9_3_tf_odom.py`
- `ros_9_4_camera_live.py`
- `ros_9_4_sensors.py`
- `ros_9_5_cmdvel.py`
- `ros_9_6_rate.py`
- `ros_9_7_rviz.py`
- `ros_9_8_container.py`
- `ros_9_9_nav2_gap.py`

### Section 10 — Replicator

- `rep_domain_random.py`
- `rep_graph_model.py`
- `rep_probe.py`
- `rep_probe2.py`
- `rep_randomise.py`
- `rep_real_dataset.py`
- `rep_sim2real_gap.py`
- `rep_train_detector.py`
- `rep_why_and_when.py`
- `rep_writers.py`

### Section 11 — Capstone and RL

- `capstone_assemble.py`
- `capstone_break_it.py`
- `capstone_dwell.py`
- `capstone_loop_ablate.py`
- `capstone_report.py`
- `capstone_v2.py`
- `rl_first_policy.py`
- `rl_onramp_cost.py`

### Supporting scripts

- `capstone.py`
- `frame.py`
- `pure_dds.py`
- `read_schemas.py`
- `render_carter.py`

## Extras

- `fastdds_noshm.xml` — the DDS profile from section 9. The C++ bridge
  reads `DEFAULT_FASTRTPS_PROFILES.xml` from its working directory, not
  from the environment variable, which is why 9.8 installs it at
  container start-up.
- `isaac_lib.sh` — the container bring-up used throughout, including the
  ROS 2 settings that took section 9 four bugs to establish.

## The one habit worth keeping

Every number in this course was measured on one machine and reported as
it came out, including the times it contradicted what I expected. Run
these on your hardware and you will get different numbers. That is the
point — measure yours.
