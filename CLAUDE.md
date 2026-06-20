# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

T-REG is a Unity project that uses [Unity ML-Agents](https://github.com/Unity-Technologies/ml-agents) (reinforcement learning) to train a ragdoll T-Rex to walk/hop toward a moving target. See the [project overview doc](https://hackmd.io/Q8IJhiZXRCK4nl1kpiCvUA) linked from README.md for background and motivation.

It is a Unity Editor project (Unity **6000.4.10f1**, see `ProjectSettings/ProjectVersion.txt`) — there is no separate web/server build. Most "development" happens by editing C# MonoBehaviour/Agent scripts and the `DinoScene.unity` scene in the Unity Editor, plus running Python-based RL training against that scene.

## Setup and common commands

There is no npm/make-style build here; everything goes through Unity Editor + the `mlagents-learn` CLI.

**Python/ML-Agents training environment** (one-time setup):
```bash
git clone --depth=1 --branch release_20 https://github.com/Unity-Technologies/ml-agents.git
conda create -n treg python=3.10.8
conda activate treg
pip install torch -f https://download.pytorch.org/whl/torch_stable.html
pip install -e ./ml-agents-envs
pip install -e ./ml-agents
pip install protobuf==3.20
```

**Run training** (from the repo root, with the `treg` conda env active and the `DinoScene` open and ready to press Play in the Unity Editor):
```bash
mlagents-learn config/ppo/TReg.yaml
```
This reads the PPO hyperparameters/training config from [config/ppo/TReg.yaml](config/ppo/TReg.yaml) (behavior name `TRexWalk`). Training results land under `results/` (gitignored) as ONNX models; promote a model into the project by copying it into `Assets/TrainedModels/` and assigning it to the Agent's Behavior Parameters in the scene.

**No automated test suite or linter is configured for the C# scripts.** The vendored `com.unity.ml-agents` package brings its own editor tests (referenced in `Packages/manifest.json` `testables`), but there is nothing project-specific to run from the command line — verification is done by pressing Play in the Unity Editor and/or running training.

## Architecture

### Two copies of the same scripts/prefabs — read this before editing

The gameplay/RL scripts and key prefabs exist in **two places that are kept manually in sync,
by design**:
- `Assets/Scripts/*.cs`, `Assets/Prefabs/*.prefab` — the live project, used by `DinoScene.unity`.
- `com.nbarlow.t-reg/` — a **real, self-contained, importable Unity package** (see below) meant to
  be dropped into a *different* project. It is **not** referenced anywhere in this project's own
  `Packages/manifest.json`, so Unity never loads it here — it's an inert folder as far as this
  project's Editor is concerned, which is exactly what keeps the GUID duplication below safe.

Files that exist in both places (`DinoBehaviourScript.cs`, `JointDriveController.cs`,
`GroundContact.cs`, `TargetContact.cs`, `OrientationCubeController.cs`, `DirectionIndicator.cs`,
`FindAllRigidBodies.cs`, `DinoCamController.cs`, `T-rex.prefab`, `PlatformDynamicTarget.prefab`,
`OrientationCube.prefab`, `DirectionIndicator.prefab`, `Platform.prefab`, `WalkerRagdoll.prefab`,
`trex.fbx`) were copied with their `.meta` files intact, so **they share identical Unity GUIDs
across both locations** (e.g. `DinoBehaviourScript.cs` is GUID `d967c2322f3234158a37020f26fed065`
in both places). This is normally a fragile thing to do in a Unity project — Unity expects GUIDs
to be unique — but it's safe here specifically *because* the package is never loaded alongside
`Assets/` in this same project. **If `com.nbarlow.t-reg` is ever added to this project's own
`Packages/manifest.json`, expect duplicate-GUID errors** — don't do that; the package's purpose is
to be copied out, not consumed in-place. When editing any of the shared files above, decide
whether the change needs to be mirrored into `com.nbarlow.t-reg` too, or is project-specific
(scene wiring, training-only tweaks) and should stay only in `Assets/`.

### Shared ragdoll helper scripts live in `Assets/Scripts/`, under the `TReg` namespace

`DinoBehaviourScript.cs` depends on five helper types that used to live in the vendored
`Assets/ML-Agents/Examples/SharedAssets/Scripts/` folder under `Unity.MLAgentsExamples` — they
were relocated into `Assets/Scripts/` and renamespaced to `TReg` so the T-Rex's own code no
longer reaches into vendored example content:
- `JointDriveController.cs` (also declares the `BodyPart` class) — drives ragdoll joint targets/strength.
- `GroundContact.cs` / `TargetContact.cs` — collision flags consumed by `BodyPart`.
- `OrientationCubeController.cs` — stabilized reference frame for observations.
- `DirectionIndicator.cs` — visual indicator toward target.

These five are still plain Unity scripts with no asmdef boundary, so `DinoBehaviourScript.cs`
just needs `using TReg;` (plus `using BodyPart = TReg.BodyPart;`) to see them.

### Vendored ML-Agents examples

`Assets/ML-Agents/Examples/` still carries the upstream ML-Agents sample content not used by
this project (3DBall, Basic, FoodCollector, Pyramids, WallJump, GridWorld, Hallway, RollerBall,
RollerRex, Soccer, Sorter, CubeEatsBall, Startup, and `SharedAssets`, which several of those still
depend on). Walker, Crawler, Worm, and the old `FirstTRexScene` reference scene were removed
since they exercised the five scripts above and nothing else in the live scene needed them — a
handful of GUID-stable assets they actually contributed were rescued first (see below), so don't
assume "vendored example" means "safe to delete" without checking what else references its GUIDs
(`grep -r "guid: <guid>"` across `Assets/Prefabs` and `Assets/Scenes`) — Unity links scripts and
nested prefabs by GUID, not by path, so a deleted-but-still-referenced asset shows up as a missing
reference rather than a compile error.

### The T-Rex agent (`DinoBehaviourScript`)

`Assets/Scripts/DinoBehaviourScript.cs` (mirrored in the package, see above) is the ML-Agents `Agent` subclass driving the simulation:
- **Body**: a ragdoll of `Transform` body parts (feet, hips, thighs, shins, tail segments, spine, neck, jaw, shoulders, arms) wired up via `JointDriveController.SetupBodyPart()` in `Initialize()`. `FindAllRigidBodies.cs` walks the hierarchy to collect every `Rigidbody` for resetting velocities each episode.
- **Actions** (`OnActionReceived`): 5 continuous actions control left/right thigh rotation, tail rotation, and lower-spine rotation (x/z) by setting joint target rotations/strength.
- **Reward shaping**: positive reward for closing distance to `target` and for torso ("but") staying upright (dot product with world up); reaching the target (`distance < 9`) gives `+1` and advances the target to the next of 8 waypoints (`MoveTargetToNextPosition`); straying too far (`distance > 150`) gives `-0.5` and ends the episode.
- **Heuristic mode** (manual play, no RL model): `Q/W` and `O/P` rotate the left/right thighs, `A/S` rotates the tail, `D/F` and `G/H` rotate the lower spine on two axes; `T` toggles the main rigidbody's `isKinematic` (suspend the T-Rex mid-air).
- **Observations** (`CollectObservations`/`CollectObservationBodyPart`): per-body-part ground contact, velocity/angular velocity and position (all relative to the `OrientationCubeController`'s stabilized frame), local rotation and current joint strength, plus target position relative to that frame.
- `DinoCamController.cs` (renamed from `CameraController.cs`) simply follows the agent's average body-part position with a fixed offset, computed from `DinoBehaviourScript.GetAvgPosition()`.

### Trained models

`Assets/TrainedModels/*.onnx` holds exported, ready-to-use models. The live T-Rex's Behavior
Parameters has `TrexHop_050724.onnx` assigned with `m_BehaviorType: 2` (InferenceOnly) and
`m_BehaviorName: TRexWalk` (matching `config/ppo/TReg.yaml`'s behavior name) — so pressing Play in
`DinoScene` runs inference immediately, no Python trainer needed. `TrexHop_040724.onnx` is the
other/earlier checkpoint, not currently assigned anywhere. `com.nbarlow.t-reg/Runtime/Models/`
ships both, same assignment, as part of the package.

### Scenes

`Assets/Scenes/DinoScene.unity` is the main/only working scene (it's also currently modified in the working tree per `git status` — check `git diff` before assuming a clean baseline). Scenes under `Assets/ML-Agents/Examples/**` are upstream sample scenes, not used by this project's own simulation.

**The live T-Rex in `DinoScene.unity` is unpacked, not a Prefab Instance.** At some point it was
disconnected from its prefab and re-rigged with a different bone hierarchy (`PelvisBone`,
`Sternum`, `Skeleton`, `RightKneeTarget`, etc. — same `trex.fbx` mesh, different transform
names/structure). `Assets/Prefabs/T-rex.prefab` is a prefab created *from* that live hierarchy (so
it's currently in sync) — but if the scene's T-Rex is ever edited directly again, the prefab will
drift stale exactly like its predecessor (`trex-new.prefab`, since removed) did. There is no
"Apply" button to keep them in sync since the scene instance isn't a linked Prefab Instance
anymore; re-syncing means dragging the GameObject into `Assets/Prefabs/` again.

`Assets/Prefabs/OrientationCube.prefab`, `DirectionIndicator.prefab`, and `Platform.prefab` are
nested (as Prefab Instances) inside `T-rex.prefab`/`PlatformDynamicTarget.prefab`/`DinoScene.unity`
and are actually live — they were rescued out of the deleted `Walker` example folder rather than
left dangling. `Assets/Prefabs/WalkerRagdoll.prefab` and `Assets/TrainedModels/Walker.nn` were
rescued the same way but are only reachable through a **disabled**, inert nested object inside
`PlatformDynamicTarget.prefab` (inherited from how Unity's own sample "dynamic target platform"
prefab was authored) — harmless, but expect a couple of "Missing (Mono Script)"/missing-model
warnings on that one disabled object (its `WalkerAgent` and `ModelOverrider` scripts, and one of
its model references, don't exist/resolve — same in the package's copy, deliberately not fixed,
see below).

### The `com.nbarlow.t-reg` package

A self-contained Unity package (currently inert in *this* project — see above) with everything
needed to drop the T-Rex into a different project and have it run inference immediately:
`Runtime/*.cs` (the agent + ragdoll helper scripts, `T-REG.asmdef`), `Runtime/Models/*.onnx`/`.nn`
(trained weights), `Models/trex.fbx` (the mesh), `Prefabs/` (the agent, target, floor, and their
nested dependencies, mirroring `Assets/Prefabs/` plus a few materials/meshes pulled in
transitively from `Assets/ML-Agents/Examples/SharedAssets` and `WallJump`), and
`Samples~/TRexDemo/TRexDemo.unity` (a copy of `DinoScene.unity`, importable via the Package
Manager's "Samples" tab — `package.json`'s `samples` entry points at it).

Every file was copied preserving its GUID rather than re-created, so nested Prefab Instance
references resolve with zero manual rewiring — verified by tracing every `guid:` reference inside
the package back to either another file in the package or a declared dependency
(`com.unity.ml-agents`, `com.unity.ugui`), and by spinning up a throwaway Unity project, adding
`com.nbarlow.t-reg` as a local `file:` package, and confirming `nbarlow.t-reg.dll` compiles with
zero errors. The same handful of dangling references already accepted in the main project's
`PlatformDynamicTarget.prefab` (the disabled WalkerRagdoll vestige) carry over unchanged —
deliberate, not a packaging bug. `ModelOverrider.cs` was *not* included (it's only used by that
same disabled object and would've pulled in an undeclared `Unity.Barracuda` dependency for code
that never runs).
