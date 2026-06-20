# com.nbarlow.t-reg

A ragdoll T-Rex agent (Unity ML-Agents) that walks/hops toward a moving target, with a trained
model wired up for inference out of the box.

## Contents

- `Prefabs/T-rex.prefab` — the agent, ready to drop into a scene. Behavior Parameters is set to
  `InferenceOnly` with `Runtime/Models/TrexHop_050724.onnx` assigned, so it runs immediately
  without a Python trainer connection. `Runtime/Models/TrexHop_040724.onnx` ships alongside it as
  an alternative checkpoint you can swap in via the Behavior Parameters component.
- `Prefabs/PlatformDynamicTarget.prefab` — the moving target the agent walks toward. Assign it to
  the agent's `target` field.
- `Prefabs/Platform.prefab` — the floor/ground the agent stands on (tagged `ground`).
- `Runtime/*.cs` — the agent script (`DinoBehaviourScript`) and its supporting ragdoll/orientation
  scripts (`TReg` namespace), plus the camera follow script (`DinoCamController`).

## Quick start

Import the **T-Rex Demo** sample from this package's entry in the Package Manager window, open
`TRexDemo.unity`, and press Play.

To build your own scene: add `Prefabs/Platform.prefab` (or any GameObject tagged `ground`),
`Prefabs/T-rex.prefab`, and `Prefabs/PlatformDynamicTarget.prefab`; assign the target instance to
the T-Rex's `target` field; add a camera with `DinoCamController` pointed at the T-Rex.
