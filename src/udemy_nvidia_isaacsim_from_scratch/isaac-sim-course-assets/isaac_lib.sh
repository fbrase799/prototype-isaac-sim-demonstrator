#!/bin/bash
# Bringing Isaac Sim up on the recording display and proving it is actually
# rendering before a take starts.
#
# Source order matters:
#   SOFTWARE_GL=0
#   source scripts/capture/lib.sh
#   source scripts/capture/gui_lib.sh     # gui_pick / gui_fullscreen / gui_lit
#   source scripts/capture/gpu_lib.sh     # Vulkan ICD pin, gpu_busy, ISAAC_*_FLAGS
#   source scripts/capture/isaac_lib.sh
#
# ---------------------------------------------------------------------------
# Isaac Sim breaks three assumptions that Course 2's gz_gui_up relied on.
# ---------------------------------------------------------------------------
#
# 1. STARTUP TIME. Gazebo under software GL painted in ~40 s and gz_gui_up
#    budgeted 70x2 s. Isaac Sim is ~25 s warm and ~51 s cold (measured; see the
#    cache note below). A 140 s budget silently fails on a cold cache, so the budget
#    here is 300 s and it fails LOUDLY rather than letting the script drive a
#    take that nothing is recording.
#
# 2. THE SPLASH SCREEN. This is the important one. Kit shows a fully-lit
#    splash/loading window long before the viewport exists, so `gui_lit 40`
#    PASSES on a loading screen. A brightness check will therefore green-light a
#    capture of a progress bar -- which is exactly the failure class that shipped
#    six Course 2 lectures containing no simulator. Brightness cannot distinguish
#    a splash from a viewport. GPU UTILISATION CAN: a splash is bright but uses
#    ~0% GPU, a live RTX viewport does not. So readiness gates on Kit's own log
#    plus gpu_busy, and only then on pixels.
#
# 3. WINDOW NAMING. Kit's title carries the stage name and changes as the stage
#    loads, so a fixed --name regex misses it. Match loosely on "Isaac Sim" and
#    let gui_pick choose by AREA (it already does), which also discards the
#    splash window because the viewport is larger.
# ---------------------------------------------------------------------------

ISAAC_CONTAINER="${ISAAC_CONTAINER:-isaac-sim}"

# Pinned by DIGEST, not by tag. A floating tag makes the course rot the moment
# NVIDIA repushes, and a student following along gets different behaviour from
# what was recorded. Verified present locally: 21.1 GB, image id 8fb4b3a4df6f.
ISAAC_IMAGE="${ISAAC_IMAGE:-nvcr.io/nvidia/isaac-sim@sha256:68735a60b6c15c85e0dd0098570c6d2cc79e928f2d068ce2790aa43284ac165d}"
ISAAC_IMAGE_TAG="${ISAAC_IMAGE_TAG:-nvcr.io/nvidia/isaac-sim:6.0.0}"   # human-readable equivalent

# The Vulkan ICD path INSIDE the container is NOT the host path.
#
# On the host the NVIDIA ICD is /usr/share/vulkan/icd.d/nvidia_icd.json, but the
# nvidia container runtime injects it at /etc/vulkan/icd.d/nvidia_icd.json.
# Passing the host path in means VK_ICD_FILENAMES points at a file that does not
# exist, the pin silently does nothing, and Isaac Sim is free to pick whatever it
# enumerates first. Verified inside this image:
#   /etc/vulkan/icd.d/nvidia_icd.json
#   /etc/vulkan/implicit_layer.d/nvidia_layers.json
ISAAC_ROS2_LIB="${ISAAC_ROS2_LIB:-/isaac-sim/exts/isaacsim.ros2.core/jazzy/lib}"
ISAAC_ICD_IN_CONTAINER="${ISAAC_ICD_IN_CONTAINER:-/etc/vulkan/icd.d/nvidia_icd.json}"

# Which Kit experience to launch. THIS IS A MEMORY DECISION, NOT A TASTE ONE.
#
# Do NOT use /isaac-sim/isaac-sim.sh. Line 22 of that script hardcodes
#   exec kit/kit apps/isaacsim.exp.full.kit
# and `full` is the wrong default here. Note the widely-repeated "full costs
# 12-13 GB RSS" figure is WRONG: measured on this machine, full peaks at 6.16 GB
# and base at 5.73, and full runs fine. See lecture 2.6.
#
# `base` is used because it is FASTER, not because full does not fit: 24.8 s to
# app ready against full's 49.5 s. Exit 137 is still real, but it depends on how
# much memory is FREE at launch, not on which experience you picked, and it reads
# like a rendering or Vulkan problem when it is RAM.
ISAAC_KIT_APP="${ISAAC_KIT_APP:-/isaac-sim/apps/isaacsim.exp.base.kit}"
ISAAC_WS="${ISAAC_WS:-$HOME/cortex-udemy-isaac-sim/sim-workspace}"
ISAAC_LOGDIR="${ISAAC_LOGDIR:-$HOME/.nvidia-omniverse/logs/Kit}"

# In this image HOME=/isaac-sim, NOT /root, so Kit writes its logs to
# /isaac-sim/.nvidia-omniverse/logs -- and mounting the host log dir at
# /root/.nvidia-omniverse/logs (the obvious guess) mounts a path Kit never writes.
#
# That failure is nasty because it is SILENT and it FAILS OPEN INTO A STALE PASS:
# isaac_stage_ready globs the host dir for the newest kit_*.log, finds a log from a
# previous WEEK, and greps that. Here it found a 2026-07-29 log and reported "no
# stage opened" for a session that was in fact fully up ("app ready" at 60.9 s,
# window present, GPU at 35%). The inverse is worse: if that old log happens to
# contain "Stage opened", the gate PASSES on evidence from a different run.
ISAAC_LOGDIR_IN_CONTAINER="${ISAAC_LOGDIR_IN_CONTAINER:-/isaac-sim/.nvidia-omniverse/logs}"

# ---------------------------------------------------------------------------
# isaac_container_up
#   Start (or reuse) the Isaac Sim container with GPU + X access.
#
#   NVIDIA_DRIVER_CAPABILITIES must include `graphics`. The common
#   `compute,utility` value is enough for CUDA but gives NO rendering, and the
#   failure looks like a Vulkan device that cannot be found.
#
#   The cache mounts are load-bearing, not housekeeping. MEASURED on this machine
#   with base.kit, wiping the host cache from inside a container first (the files
#   are container-owned, so a host `rm -rf` only half-works and silently leaves a
#   warm cache behind, which is how you measure a "cold" launch that is not cold):
#       truly cold   50.985 s to app ready,  host cache 0 -> 238 MB
#       warm         24.771 s / 24.495 s
#   So cold is about TWICE warm, roughly 26 s of one-time cost. The widely
#   repeated "4-6 minutes cold vs 60-90 s warm" is not what this machine does.
#   In practice only the `ov` mount takes writes (DerivedDataCache + shaders);
#   the other four stay empty for this app but cost nothing to keep.
#
#   THEY MUST POINT AT $HOME, AND HOME IS /isaac-sim, NOT /root. This is the same
#   trap as the log path. Mounted at /root/.cache/... the five host directories
#   stayed EMPTY across every capture in sections 1 and 2 while Isaac wrote its
#   real caches to /isaac-sim/.cache/ov (301 MB DerivedDataCache, 152 MB shaders),
#   inside the container, thrown away on every `docker rm`. Nothing failed loudly:
#   launches just paid a partial cold cost forever. Verify with
#     du -sh ~/.cache/ov/ov     # must be non-trivial after one launch
# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# isaac_fix_log_perms
#   The container writes as uid 1234; these host dirs are ours (uid 1000). Open
#   them up so Kit's log and the shader caches can actually be written. Without
#   this the log is silently dropped and readiness can never be established.
#   Deliberately scoped to the two Omniverse dirs, and only the levels Kit 6.0
#   uses: an old root-owned Isaac-Sim/4.5 subtree here is not ours to chmod and
#   is not used, so its failures are ignored.
# ---------------------------------------------------------------------------
isaac_fix_log_perms() {
  mkdir -p "$ISAAC_LOGDIR" 2>/dev/null || true
  chmod -R a+rwX "$HOME/.nvidia-omniverse/logs" 2>/dev/null || true
  chmod -R a+rwX "$OV_CACHE" 2>/dev/null || true
  # The render OUTPUT dir has exactly the same uid problem as the log dir: the
  # container writes as uid 1234, the host dir is ours. Replicator renders
  # perfectly and then fails on every single write with "Permission denied",
  # which looks like a hung render because no frames ever appear.
  mkdir -p "$ISAAC_WS/out" 2>/dev/null || true
  chmod -R a+rwX "$ISAAC_WS/out" 2>/dev/null || true
  return 0
}

isaac_container_up() {
  if docker ps --format '{{.Names}}' | grep -qx "$ISAAC_CONTAINER"; then
    return 0
  fi
  docker rm -f "$ISAAC_CONTAINER" >/dev/null 2>&1 || true
  mkdir -p "$OV_CACHE"/{kit,ov,pip,glcache,computecache} "$ISAAC_WS"
  isaac_fix_log_perms
  # --entrypoint is REQUIRED, not stylistic. This image's ENTRYPOINT is
  # /isaac-sim/runheadless.sh, which starts the STREAMING Kit app. A trailing
  # `tail -f /dev/null` does not replace an entrypoint, it becomes its ARGUMENT:
  # the container launches full streaming Kit anyway, eats 12 GB, and is OOM-killed
  # a few seconds in -- before the `docker exec` in isaac_up ever gets to run. The
  # symptom is a container that "will not stay up" plus a Kit log that says
  # "Isaac-Sim Streaming" even though nothing asked for streaming. Overriding the
  # entrypoint is what makes this an idle container we can exec into.
  # Do NOT add --user here. The image runs as uid 1234 (isaac-sim) and the host dirs
  # bind-mounted below are 1000:1000 mode 755, so Kit cannot write its own log: it
  # announces "Logging to file: .../kit_<stamp>.log" and that file NEVER APPEARS.
  # Kit does not treat this as fatal and renders perfectly, so the only visible
  # effect is that isaac_stage_ready has no log to read and the launch "times out"
  # while a healthy viewport sits on :99.
  #
  # The tempting fix, --user "$(id -u):$(id -g)", trades one silent failure for a
  # loud one: /isaac-sim/kit/kit is not executable by uid 1000, so the launch dies
  # with `nohup: failed to run command '/isaac-sim/kit/kit': Permission denied` and
  # no Kit starts at all. Measured both ways.
  #
  # So the container keeps its native uid and the HOST side is made group-writable
  # for it instead (isaac_fix_log_perms, called below).
  # ROS 2 BRIDGE, SOLVED 2026-08-15. THE REAL BUG WAS world.step(render=False).
  #
  # OnPlaybackTick fires off the TIMELINE, and world.step(render=False)
  # advances physics WITHOUT advancing the timeline: measured sim_time frozen
  # at 0.0167 across 90 steps. With render=True it advances normally
  # (0.0167 -> 0.3500 -> 0.6833 -> 1.0167). So the graph ticked exactly once,
  # published one message at t=0, and every subscriber that attached later saw
  # nothing. That looked identical to a transport failure and sent me through
  # shared-memory, UDP-only profiles, loopback pinning and RMW swaps -- none of
  # which were the cause. Same lesson as Section 8.2: THE TIMELINE DECIDES.
  #
  # With render=True: in-container subscriber gets a continuous stream, and so
  # does a HOST subscriber on stock DDS config. No transport overrides needed.
  # Publishing to a host subscriber VERIFIED:
  #   ros2 topic echo /clock  ->  clock: sec: 0 nanosec: 0
  #
  # THREE separate things had to be right, and two of them I broke myself:
  #   1. LD_LIBRARY_PATH -> Isaac's own jazzy tree (fixes the segfault).
  #   2. ROS_DISTRO must be UNSET. extension.py only sets
  #      internal_lib_fallback=True when it is absent; exporting it tells the
  #      bridge a user-sourced ROS 2 exists and it stops using its bundled libs.
  #      Correct log line is "Using backup internal ROS2 jazzy distro".
  #   3. DO NOT force FASTDDS_BUILTIN_TRANSPORTS/profile files here. The host
  #      subscriber uses stock config; overriding only the container created a
  #      transport MISMATCH, which looks exactly like "the bridge does not
  #      publish". Leave both sides on defaults.
  #
  # ROS 2 BRIDGE, earlier note from 2026-08-14. The 2026-08-11 probe concluded the bridge
  # "does not instantiate in this container" and Section 9 was deferred on that
  # basis. That conclusion was WRONG, and so was my first fix.
  #
  # WHAT IS ACTUALLY TRUE: isaacsim.ros2.core ships its OWN complete ROS 2
  # trees -- exts/isaacsim.ros2.core/{humble,jazzy}/lib, 344 libraries
  # including librcl, librclcpp and librmw_fastrtps_cpp, plus backends
  # libisaacsim.ros2.core.{humble,jazzy}.so. No host ROS 2 is needed at all.
  # The bridge only needs LD_LIBRARY_PATH pointed at the matching tree.
  #
  # Without it, the 34 node types still REGISTER (the .ogn descriptions are
  # metadata and load regardless) and instantiation then SEGFAULTS -- which is
  # why the earlier probe saw registration succeed and creation fail.
  #
  # MEASURED, and this is the part worth keeping: mounting the HOST /opt/ros
  # made it WORSE, not better. Two ROS 2 stacks resolving the same symbols
  # segfaulted Kit on the ROS2Context create call. With the host mount removed
  # and LD_LIBRARY_PATH set to Isaac's own jazzy lib, ROS2Context instantiates:
  # Node("/World/ros_probe/Context").
  docker run --name "$ISAAC_CONTAINER" -d \
    --entrypoint /bin/bash \
    --runtime=nvidia --gpus all \
    -e "ACCEPT_EULA=Y" -e "PRIVACY_CONSENT=Y" \
    -e DISPLAY="${CAP_DISPLAY:-:99}" \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e VK_ICD_FILENAMES="$ISAAC_ICD_IN_CONTAINER" \
    -e VK_DRIVER_FILES="$ISAAC_ICD_IN_CONTAINER" \
    -e QT_X11_NO_MITSHM=1 \
    -e RMW_IMPLEMENTATION="${ISAAC_RMW:-rmw_fastrtps_cpp}" \
    -e LD_LIBRARY_PATH="$ISAAC_ROS2_LIB" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$OV_CACHE/kit:/isaac-sim/kit/cache:rw" \
    -v "$OV_CACHE/ov:/isaac-sim/.cache/ov:rw" \
    -v "$OV_CACHE/pip:/isaac-sim/.cache/pip:rw" \
    -v "$OV_CACHE/glcache:/isaac-sim/.cache/nvidia/GLCache:rw" \
    -v "$OV_CACHE/computecache:/isaac-sim/.nv/ComputeCache:rw" \
    -v "$HOME/.nvidia-omniverse/logs:$ISAAC_LOGDIR_IN_CONTAINER:rw" \
    -v "$ISAAC_WS:/workspace:rw" \
    --network host --ipc=host \
    "$ISAAC_IMAGE" \
    -c 'tail -f /dev/null' >/dev/null 2>&1 || {
      echo "isaac_container_up: docker run failed" >&2; return 1; }
  sleep 4
  docker ps --format '{{.Names}}' | grep -qx "$ISAAC_CONTAINER" || return 1
  # THE BRIDGE READS ITS DDS PROFILE FROM CWD, NOT FROM THE ENV. Install it on
  # every bring-up: hand-copying it is what made results irreproducible.
  docker exec "$ISAAC_CONTAINER" bash -c \
    'cp /workspace/fastdds_noshm.xml /isaac-sim/DEFAULT_FASTRTPS_PROFILES.xml' \
    >/dev/null 2>&1 || true
  isaac_fix_container_cache
}

# ---------------------------------------------------------------------------
# isaac_fix_container_cache
#   /isaac-sim/.cache is ROOT-OWNED in the image while Kit runs as uid 1234, so
#   any extension that wants a cache dir of its own cannot create one. This is
#   the same uid trap as the log dir, one level deeper, and it has to run INSIDE
#   the container after start: isaac_fix_log_perms runs on the host before
#   `docker run` and cannot touch this path.
#
#   It presents as a UI bug, not a permissions one, which is why it survived two
#   whole sections. Measured on this image:
#     PermissionError: [Errno 13] Permission denied: '/isaac-sim/.cache/warp'
#   kills omni.replicator.core at startup, and THE PROPERTY PANEL THEN RENDERS
#   EMPTY FOR EVERY SELECTED PRIM. Selection works, the Stage row highlights,
#   the viewport draws its selection outline, and Property shows only a Search
#   box. With this chown in place the same click fills Property with Prim Path
#   and a Transform section (measured: crop std 4.3 -> 23.8).
#
#   It is also why lecture 2.9 concluded Replicator's writer path is a dead end
#   on this image. Some of that was this. Re-check before repeating the claim.
# ---------------------------------------------------------------------------
isaac_fix_container_cache() {
  docker exec -u 0 "$ISAAC_CONTAINER" bash -c \
    'mkdir -p /isaac-sim/.cache/warp /isaac-sim/.cache/matplotlib &&
     chown -R 1234:1234 /isaac-sim/.cache' >/dev/null 2>&1 || {
      echo "isaac_fix_container_cache: chown failed; Property will render EMPTY" >&2
      return 1; }
  return 0
}

# ---------------------------------------------------------------------------
# isaac_property_populates
#   Gate for the cache-permission bug above, on the PIXELS, because that bug is
#   invisible in the log once the chown is in place and the panel is the thing
#   the lecture points at. Call it after selecting a prim and before a take that
#   narrates Property.
#
#   Measured on an empty panel vs a populated one, same crop:
#     empty      std  4.3   (Search box on flat background)
#     populated  std 23.8   (Prim Path, Transform rows, value fields)
#   12 sits clear of both.
# ---------------------------------------------------------------------------
isaac_property_populates() {
  local want="${1:-12}" shot=/tmp/_isaac_prop.png
  DISPLAY="${CAP_DISPLAY:-:99}" import -window root "$shot" 2>/dev/null || return 1
  python3 - "$shot" "$want" <<'PY'
import sys
import numpy as np
from PIL import Image
shot, want = sys.argv[1], float(sys.argv[2])
a = np.asarray(Image.open(shot).convert("L").crop((1340, 540, 1920, 1080)), dtype=float)
std = a.std()
print(f"property panel std {std:.1f} (want >= {want})")
if std < want:
    print("  Property is EMPTY with a prim selected. /isaac-sim/.cache is likely",
          "root-owned again: see isaac_fix_container_cache.", file=sys.stderr)
sys.exit(0 if std >= want else 1)
PY
}

# ---------------------------------------------------------------------------
# isaac_stage_ready
#   The honest readiness test. Asks Kit and the GPU, not the pixels.
# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# isaac_stage_has_content
#   Did the requested stage actually LOAD? Kit reports "Failed to open stage"
#   in the status bar and then runs happily with an empty one, so a take can
#   record correct-looking panels containing nothing. Checks the log rather than
#   the pixels, because an empty Stage tree and a loading one look identical.
# ---------------------------------------------------------------------------
isaac_stage_has_content() {
  local newest
  newest=$(find "$ISAAC_LOGDIR" -name 'kit_*.log' -mmin -10 -printf '%T@ %p\n' 2>/dev/null \
           | sort -rn | head -1 | cut -d' ' -f2-)
  [ -n "$newest" ] || return 1
  if grep -qi "Failed to open stage" "$newest" 2>/dev/null; then
    echo "isaac_stage_has_content: Kit FAILED to open the requested stage." >&2
    echo "  A .py scene script must be passed to --exec directly, not through" >&2
    echo "  open_stage.py, which expects USD. The app is up with an EMPTY stage." >&2
    return 1
  fi
  return 0
}

isaac_stage_ready() {
  # (a) Kit says it is up, IN THIS RUN.
  #
  # Two corrections over the first version, both learned by watching it fail on a
  # session that was actually fine:
  #
  #   1. The log must be FRESH. Globbing for "newest" silently accepted a log from
  #      a previous week (see ISAAC_LOGDIR_IN_CONTAINER). -mmin -10 makes stale
  #      evidence fail closed instead of deciding the run.
  #   2. "Stage opened" is the wrong string for an empty stage. base.kit launched
  #      with no --exec open_stage never opens one, so the gate could never pass:
  #      measured, it logs `app ready` at 60.9 s and renders happily. Accept either,
  #      because a stage-loading launch and a bare launch are both legitimate.
  local newest
  newest=$(find "$ISAAC_LOGDIR" -name 'kit_*.log' -mmin -10 -printf '%T@ %p\n' 2>/dev/null \
           | sort -rn | head -1 | cut -d' ' -f2-)
  [ -n "$newest" ] || return 1
  grep -qiE "Stage opened|stage loaded|Opened stage|app ready" "$newest" 2>/dev/null || return 1
  # (b) and the GPU is genuinely rendering, not sitting on a bright splash.
  gpu_busy 5 || return 1
  return 0
}

# ---------------------------------------------------------------------------
# isaac_up [budget-x2s] [stage-usd]
#   Launch Isaac Sim on the capture display and block until it is really up.
# ---------------------------------------------------------------------------
isaac_up() {
  local budget="${1:-150}" stage="${2:-}" i w
  # ORDER MATTERS: the display comes FIRST. vulkaninfo opens an XCB surface, so
  # against a DISPLAY that does not exist yet it fails with
  #   AppCreateXcbSurface failed to establish connection
  # and enumerates ZERO devices -- and gpu_assert_nvidia then reports "no NVIDIA
  # Vulkan device enumerated", which points at the GPU instead of at the missing
  # display. Found after a reboot on 2026-08-11; it never fired before because
  # :99 happened to survive from an earlier session. Same enumeration is correct
  # (1 device, NVIDIA) the moment Xvfb is up.
  ensure_display
  gpu_assert_nvidia || return 1
  isaac_container_up || return 1

  # One Kit client, always. Repeated captures otherwise stack clients and
  # gui_pick may choose a window belonging to a dead process (Course 2 hit this
  # with four stacked Gazebo windows and recorded a stale frame).
  docker exec "$ISAAC_CONTAINER" pkill -9 -f 'isaac-sim|kit' >/dev/null 2>&1 || true
  sleep 3

  # A .usd is a stage to OPEN; a .py is a scene script to RUN. Wrapping a .py in
  # open_stage.py makes Kit try to parse Python as USD and log
  #   Failed to open stage /workspace/stages/foo.py
  # while coming up perfectly well with an EMPTY stage, so a take records real
  # panels with nothing in them and the failure is a one-line status bar message.
  local execflag=""
  case "$stage" in
    "")      execflag="" ;;
    *.py)    execflag="--exec \"$stage\"" ;;
    *)       execflag="--exec \"open_stage.py $stage\"" ;;
  esac

  # Launch Kit directly against ISAAC_KIT_APP rather than through isaac-sim.sh,
  # which would force the full (OOM-killed) experience. See ISAAC_KIT_APP above.
  # shellcheck disable=SC2086
  # ISAAC_ENV lets a caller parameterise the stage script, e.g.
  #   ISAAC_ENV="CAM_SHOT=tight" isaac_up 250 /workspace/stages/foo.py
  # 3.4 needs two framings of one stage (a 64 m object and a 0.6 m one cannot
  # share a shot), and re-authoring the stage per shot would mean the two takes
  # were of different scenes.
  local envflags=""
  local kv
  for kv in ${ISAAC_ENV:-}; do
    envflags="$envflags -e $kv"
  done
  # shellcheck disable=SC2086
  docker exec -d -e DISPLAY="${CAP_DISPLAY:-:99}" $envflags "$ISAAC_CONTAINER" \
    bash -lc "nohup /isaac-sim/kit/kit $ISAAC_KIT_APP $ISAAC_GPU_FLAGS $ISAAC_RES_FLAGS $ISAAC_RTX_FLAGS $execflag > /tmp/isaac.log 2>&1 &"

  for i in $(seq 1 "$budget"); do
    sleep 2
    # Is the container still alive? An OOM kill (exit 137) otherwise burns the
    # whole 300 s budget and then reports "no rendered viewport", which points at
    # the GPU instead of at memory. Fail immediately and say which it was.
    if ! docker ps --format '{{.Names}}' | grep -qx "$ISAAC_CONTAINER"; then
      local code
      code=$(docker inspect -f '{{.State.ExitCode}}' "$ISAAC_CONTAINER" 2>/dev/null)
      echo "isaac_up: container died after $((i*2)) s (exit ${code:-?})" >&2
      if [ "$code" = "137" ]; then
        echo "  exit 137 = OOM-killed. This is RAM, not rendering." >&2
        echo "  This is about FREE memory, not which .kit you chose:" >&2
        echo "  measured here, full peaks 6.16 GB and base 5.73 GB." >&2
        echo "  available: $(free -m | awk 'NR==2{print $7}') MiB" >&2
      fi
      docker logs --tail 15 "$ISAAC_CONTAINER" 2>&1 | sed 's/^/  /' >&2
      return 1
    fi
    if DISPLAY="${CAP_DISPLAY:-:99}" xdotool search --name "Isaac Sim" >/dev/null 2>&1; then
      w=$(gui_pick "Isaac Sim" 2>/dev/null) || continue
      gui_fullscreen "Isaac Sim" >/dev/null 2>&1
      if isaac_stage_ready; then
        echo "isaac up (${i}x2 s, VRAM $(gpu_vram_used) MiB)"
        return 0
      fi
    fi
  done
  echo "isaac_up: no rendered viewport in $((budget*2)) s" >&2
  echo "  container log tail:" >&2
  docker exec "$ISAAC_CONTAINER" tail -20 /tmp/isaac.log 2>/dev/null >&2
  return 1
}

# ---------------------------------------------------------------------------
# isaac_teach_env
#   Rule 6a: type the environment on camera when it is load-bearing. For this
#   course the Vulkan pin IS load-bearing -- omit it and the learner's Isaac Sim
#   silently selects the iGPU. So it gets typed, not narrated.
# ---------------------------------------------------------------------------
isaac_teach_env() {
  type_cmd "export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json" 30
  type_cmd "export DISPLAY=${CAP_DISPLAY:-:99}" 30
  type_cmd "vulkaninfo --summary | grep deviceName" 30
  sleep 2
}

# ---------------------------------------------------------------------------
# isaac_vram_watch <logfile> [interval]
#   Background sampler. Isaac Sim can hit a VRAM ceiling DURING a long take and
#   drop to a black viewport while the window still exists, which no start-time
#   check can catch. Cheap insurance that turns a silent 4-minute loss into a
#   named failure. Returns the sampler pid; kill it after the take.
# ---------------------------------------------------------------------------
isaac_vram_watch() {
  local log="${1:?}" iv="${2:-10}"
  ( while :; do
      printf '%s %s MiB util=%s%%\n' "$(date +%H:%M:%S)" \
        "$(gpu_vram_used)" \
        "$(nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits 2>/dev/null | head -1)" \
        >> "$log"
      sleep "$iv"
    done ) >/dev/null 2>&1 &
  echo $!
}

# ---------------------------------------------------------------------------
# isaac_down
# ---------------------------------------------------------------------------
isaac_down() {
  docker exec "$ISAAC_CONTAINER" pkill -9 -f 'isaac-sim|kit' >/dev/null 2>&1 || true
  docker rm -f "$ISAAC_CONTAINER" >/dev/null 2>&1 || true
}
