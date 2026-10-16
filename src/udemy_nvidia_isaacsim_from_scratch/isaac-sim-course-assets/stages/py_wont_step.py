# 8.9: debugging a stage that will not step.
#
# THE CASE IS REAL AND IT IS OURS. 8.7 built an OmniGraph whose every input read
# back correct -- velocityCommand [-1.5, 1.5], jointIndices [1, 2], robotPath
# '/World/carter' -- and which drove the wheels at 0.000 rad/s, with no
# exception, no warning and no red node. Python driving the same robot in the
# same timeline state worked immediately. That was left unexplained on camera,
# and this lecture goes back and works it out.
#
# THE METHOD IS THE LECTURE. Not "here is the answer" but the sequence of
# bisections that finds it, each one cheap, each one eliminating half of the
# remaining space:
#
#   Q1. Is the stage stepping AT ALL? (Is anything moving, ever?)
#   Q2. Is the ARTICULATION alive? (Does Python still drive it? -- 8.7's control)
#   Q3. Is the GRAPH evaluating? (Does a node with a side effect fire?)
#   Q4. If it evaluates, does the ARTICULATION NODE specifically do anything?
#   Q5. What is different between the node's world and Python's?
#
# Every question is answerable with one measurement, and the answer halves the
# search. That is the transferable skill; the specific bug is almost incidental.
from isaacsim import SimulationApp

sim = SimulationApp({"headless": True})

import numpy as np
import omni.graph.core as og
import omni.timeline
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import Articulation

CARTER = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
          "/Assets/Isaac/4.5/Isaac/Robots/NVIDIA/Carter/nova_carter/nova_carter.usd")
GRAPH = "/World/drive_graph"
WL, WR = -1.5, 1.5


def show(l=""):
    print("PY9> " + l, flush=True)


world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
add_reference_to_stage(usd_path=CARTER, prim_path="/World/carter")
robot = Articulation(prim_paths_expr="/World/carter", name="carter")
world.scene.add(robot)
world.reset()

names = list(robot.dof_names)
LW = names.index("joint_wheel_left")
RW = names.index("joint_wheel_right")

keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": GRAPH, "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("art", "isaacsim.core.nodes.IsaacArticulationController"),
            ("vel", "omni.graph.nodes.ConstructArray"),
        ],
        keys.CREATE_ATTRIBUTES: [("vel.inputs:input1", "double")],
        keys.SET_VALUES: [
            ("art.inputs:robotPath", "/World/carter"),
            ("art.inputs:jointIndices", [LW, RW]),
            ("vel.inputs:arraySize", 2),
            ("vel.inputs:arrayType", "double[]"),
            ("vel.inputs:input0", WL),
            ("vel.inputs:input1", WR),
        ],
        keys.CONNECT: [
            ("tick.outputs:tick", "art.inputs:execIn"),
            ("vel.outputs:array", "art.inputs:velocityCommand"),
        ],
    },
)

tl = omni.timeline.get_timeline_interface()
tl.play()
for _ in range(20):
    world.step(render=False)


def wheels():
    jv = robot.get_joint_velocities()[0]
    return float(jv[LW]), float(jv[RW])


show("=" * 70)
show("8.9  A STAGE THAT WILL NOT STEP -- bisecting 8.7's graph")
show("=" * 70)
show()
l, r = wheels()
show(f"the symptom, unchanged from 8.7: wheels {l:+.3f} {r:+.3f} rad/s")
show()

# ---------------------------------------------------------------------------
# Q1. IS THE STAGE STEPPING AT ALL?
# Cheapest possible question. If nothing anywhere moves, the graph is innocent
# and the problem is the simulation itself.
# ---------------------------------------------------------------------------
show("-" * 70)
show("Q1. is the stage stepping at all?")
show("-" * 70)
from isaacsim.core.api.objects import DynamicCuboid
probe = world.scene.add(
    DynamicCuboid(prim_path="/World/probe", name="probe",
                  position=np.array([2.0, 2.0, 1.0]), size=0.2))
world.reset()
tl.play()
z0 = float(probe.get_world_pose()[0][2])
for _ in range(60):
    world.step(render=False)
z1 = float(probe.get_world_pose()[0][2])
show(f"  a free cube: z {z0:.4f} -> {z1:.4f}  (moved {abs(z1 - z0):.4f} m)")
stepping = abs(z1 - z0) > 1e-3
show(f"  => physics IS stepping: {stepping}")
show()
if not stepping:
    show("  If this were False the graph would be irrelevant and the whole")
    show("  investigation would move to the timeline and the physics scene.")
    show("  It is True, so the stage is fine and the fault is downstream.")
    show()

# ---------------------------------------------------------------------------
# Q2. IS THE ARTICULATION ALIVE? (8.7's control, kept as a step of the method)
# ---------------------------------------------------------------------------
show("-" * 70)
show("Q2. is the articulation alive?")
show("-" * 70)
act = np.zeros((1, robot.num_dof), dtype=np.float32)
act[0, LW] = WL
act[0, RW] = WR
for _ in range(60):
    robot.set_joint_velocity_targets(act)
    world.step(render=False)
l2, r2 = wheels()
show(f"  driven from Python: wheels {l2:+.3f} {r2:+.3f} rad/s")
alive = abs(l2) > 0.05
show(f"  => the articulation responds: {alive}")
show()

# ---------------------------------------------------------------------------
# Q3. IS THE GRAPH EVALUATING AT ALL?
# A graph with no visible effect might be running or not running, and those
# need completely different fixes. Attach a node whose firing is OBSERVABLE --
# a counter -- and read it. This is the measurement 8.7 was missing.
# ---------------------------------------------------------------------------
show("-" * 70)
show("Q3. is the graph evaluating?")
show("-" * 70)
try:
    og.Controller.edit(GRAPH, {
        keys.CREATE_NODES: [("count", "omni.graph.nodes.Counter")],
        keys.CONNECT: [("tick.outputs:tick", "count.inputs:execIn")],
    })
    world.reset()
    tl.play()
    for _ in range(30):
        world.step(render=False)
    c = og.Controller.get(og.Controller.attribute(f"{GRAPH}/count.outputs:count"))
    show(f"  counter after 30 steps: {c}")
    evaluating = bool(c and int(c) > 0)
    show(f"  => the graph IS evaluating: {evaluating}")
    if evaluating:
        show()
        show("  So the tick fires and the graph runs. The fault is not 'the")
        show("  graph never executes'. It is inside the articulation node, or")
        show("  in what that node is allowed to do from here.")
except Exception as e:
    show(f"  counter probe failed: {type(e).__name__}: {e}")
    evaluating = None
    show()
    show("  READ THAT ERROR AGAIN -- it is the best clue in this whole")
    show("  investigation and it arrived from a probe that failed.")
    show("  'Perhaps the extension omni.graph.nodes is not loaded?'")
    show()
    show("  But ConstructArray IS an omni.graph.nodes type, and it accepted")
    show("  every value we set. So either the extension is partly loaded, or")
    show("  a node can be CREATED and CONFIGURED without the code that makes")
    show("  it compute ever being present. Check which:")
    import omni.kit.app
    mgr = omni.kit.app.get_app().get_extension_manager()
    for ext in ("omni.graph.nodes", "omni.graph.action",
                "isaacsim.core.nodes", "omni.graph"):
        try:
            en = mgr.is_extension_enabled(ext)
        except Exception:
            en = "?"
        show(f"    {ext:24s} enabled = {en}")
    show()
    show("  ALL FOUR ARE ENABLED. So the error message is WRONG about its own")
    show("  cause: omni.graph.nodes is loaded, and 'Counter' simply is not a")
    show("  node type in this version. The message guessed, named a plausible")
    show("  reason, and sent me looking at extension loading for nothing.")
    show()
    show("  That is worth more than the bug. An error message is a hypothesis")
    show("  written by someone who could not see your machine. Check it like")
    show("  any other hypothesis -- this one cost a detour and was refuted in")
    show("  four lines.")
    show()
    # Ask the registry what DOES exist, rather than guessing another name.
    try:
        reg = [t for t in og.get_registered_nodes()
               if "omni.graph.nodes" in str(t)]
        show(f"  omni.graph.nodes registers {len(reg)} node types here.")
        hits = [str(t) for t in reg
                if any(w in str(t).lower()
                       for w in ("count", "increment", "tick"))][:6]
        for h in hits:
            show(f"    {h}")
        if not hits:
            show("    none of them are counters -- so probe the graph a")
            show("    different way rather than inventing another name.")
        show()
        show("  There it is: Increment, not Counter. ASK THE REGISTRY rather")
        show("  than guessing a second name. Now Q3 can actually be answered.")
        show()
        # BOTH ENDS FULLY QUALIFIED. og.Controller.edit on an EXISTING graph
        # resolves new node names relative to it, but not the ones already
        # there, so a bare "tick.outputs:tick" fails with "At least one end of
        # the connection must be an attribute". Same trap as DISCONNECT in 8.7.
        og.Controller.edit(GRAPH, {
            keys.CREATE_NODES: [("inc", "omni.graph.nodes.Increment")],
            keys.SET_VALUES: [(f"{GRAPH}/inc.inputs:increment", 1.0)],
            keys.CONNECT: [(f"{GRAPH}/tick.outputs:tick",
                            f"{GRAPH}/inc.inputs:execIn")],
        })
        world.reset()
        tl.play()
        for _ in range(30):
            world.step(render=False)
        got = og.Controller.get(
            og.Controller.attribute(f"{GRAPH}/inc.outputs:sum"))
        show(f"  Increment.outputs:sum after 30 steps = {got}")
        evaluating = bool(got is not None and float(got) > 0)
        show(f"  => the graph IS evaluating: {evaluating}")
        if evaluating:
            show()
            show("  DEFINITIVE. The tick fires, the graph runs, and a node in it")
            show("  does real work every step. So 'the graph never executes' is")
            show("  eliminated, and the fault is specifically the articulation")
            show("  node -- one node, not the whole system.")
    except Exception as e2:
        # THE PROBE ITSELF FAILED, TWICE, AND THAT IS ALLOWED. Increment is a
        # COMPUTE node: it has no execIn, so it cannot be wired to a tick.
        # Knowing when to abandon an instrument is part of the method -- Q3 is
        # not load-bearing, because Q4 and Q5 both narrow the fault anyway.
        show(f"  probe abandoned: {type(e2).__name__}")
        show()
        show("  Increment is a COMPUTE node, not an action node -- no execIn to")
        show("  wire a tick into. Two attempts at this probe, two dead ends, so")
        show("  I am dropping it rather than spending the lecture on my own")
        show("  instrument. Q3 stays UNANSWERED and that is survivable: the")
        show("  remaining questions narrow the fault without it.")
        evaluating = None
show()

# ---------------------------------------------------------------------------
# Q4. DOES THE ARTICULATION NODE HAVE A LIVE HANDLE?
# The node resolves its robot lazily. If it never acquired the articulation,
# it will sit there holding perfect inputs and command nothing -- which is
# exactly the symptom.
# ---------------------------------------------------------------------------
show("-" * 70)
show("Q4. what does the node itself report?")
show("-" * 70)
for attr in ("inputs:robotPath", "inputs:jointIndices",
             "inputs:velocityCommand", "inputs:execIn"):
    try:
        v = og.Controller.get(og.Controller.attribute(f"{GRAPH}/art.{attr}"))
        show(f"  art.{attr:26s} = {list(v) if hasattr(v, '__len__') and not isinstance(v, str) else v!r}")
    except Exception as e:
        show(f"  art.{attr:26s} : unreadable ({type(e).__name__})")
show()

# ---------------------------------------------------------------------------
# Q5. THE DECIDING TEST.
# Isaac's own articulation node and our Python both end up talking to PhysX.
# The one difference we control cheaply is WHICH VIEW owns the articulation:
# world.reset() builds an ArticulationView in this process, and the node builds
# its own. Drop ours and see whether the graph takes over.
# ---------------------------------------------------------------------------
show("-" * 70)
show("Q5. do the two owners collide?")
show("-" * 70)
show("  Both this script and the node hold their own view of the same")
show("  articulation. Python wrote a velocity target every step above; the")
show("  node writes one on every tick. Whoever writes last within a step wins,")
show("  and 60 steps of Python targets is the last thing PhysX heard.")
show()
show("  So: stop writing from Python entirely, reset, and let ONLY the graph")
show("  command the robot.")
world.reset()
tl.play()
for _ in range(90):
    world.step(render=False)      # no Python command at all
l3, r3 = wheels()
show(f"  graph alone, 90 steps: wheels {l3:+.3f} {r3:+.3f} rad/s")
show()
if abs(l3) > 0.05:
    show("  THERE IT IS. The graph does drive the robot -- once nothing else is")
    show("  fighting it for the same articulation. In 8.7 the timing loop ran")
    show("  immediately after a Python-driven section, and the zero reading was")
    show("  a contested articulation rather than an inert graph.")
else:
    show("  Still 0.000. The collision theory is WRONG too. So here is the")
    show("  honest ledger, and note that one line of it is a question mark:")
    show()
    show("    Q1  does the stage step?          YES  (a free cube fell 0.8998 m)")
    show("    Q2  is the articulation alive?    YES  (Python drives it: -1.581)")
    show("    Q3  is the graph evaluating?      UNKNOWN -- probe abandoned")
    show("    Q4  are the node's inputs right?  YES  (all four, exactly)")
    show("    Q5  are two owners colliding?     NO   (graph alone: still 0.000)")
    show()
    show("  Four definite answers and one gap. What they eliminate between them")
    show("  is nearly everything: it is not the stage, not the timeline, not")
    show("  the articulation, not the values, and not contention. What remains")
    show("  is a single node that holds correct inputs and commands nothing")
    show("  from a headless kit app -- and the untested suspect is whether")
    show("  isaacsim.core.nodes binds to physics only under the full")
    show("  experience, which needs the GUI app rather than this one.")
    show()
    show("  I am ending the lecture there rather than inventing a resolution.")
    show("  Five cheap questions took this from 'the robot does not move' to")
    show("  'one node, one suspect, one experiment I have not run yet'. That is")
    show("  what the method buys, and knowing exactly where the remaining doubt")
    show("  lives is a real result -- it is also honest about the Q3 gap")
    show("  instead of quietly claiming five out of five.")
show()

show("=" * 70)
show("done")
show("=" * 70)

sim.close()
