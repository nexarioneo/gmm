# GMM — Godot Motion Matching

**GMM is an open-source Motion Matching plugin for Godot 4.7+, written in C++ as a GDExtension.**

This **Godot 4.7 Motion Matching** implementation brings data-driven character animation to your games: instead of hand-authoring state machines, transitions, and blend trees, you feed the **GDExtension C++ animation** core a database of animation frames. Every update, it searches that database for the frame whose **features** (trajectory, joint positions, joint velocities) best match what the character is being asked to do — producing fluid, responsive, natural-looking locomotion straight from raw animation data.

This Godot motion matching plugin is a Godot-API port of the concept and architecture from **O3DE's MotionMatching Gem** (built on EMotionFX), redesigned for the engine and built with modern **C++17**.

## 🎯 Why Motion Matching?

- **No transition spaghetti** — no state machines, no blend trees, no hand-authored transitions.
- **Every pose is a candidate** — the runtime search considers your entire animation database on every search tick.
- **Data-driven** — add more animations to the `motions` array, and the system rebuilds its database around them.

---

## ✨ Features

Everything below runs inside a single Godot 4.7 GDExtension — no engine fork and no custom Godot build required:

- **Feature-based matching** against a sampled pose database, accelerated with a **KD-tree**
- **Extensible feature system** via `Feature` resources:
  - `FeatureTrajectory` — predicted trajectory: future positions + facing directions over a time horizon
  - `FeaturePosition` — joint positions for selected bones
  - `FeatureVelocity` — joint linear velocities
  - `FeatureAngularVelocity` — joint angular velocities
- **Automatic default feature schema** (`FeatureSchema`) — root trajectory; pelvis position/velocity; feet positions/velocities — or define your own blend of features
- **Two trajectory modes**:
  - *Target-driven* — follow a gameplay-driven target position/facing direction
  - *Automatic (Demo)* — self-moving demo path for quick testing
- **Data normalization** — Standard Scaler and Min–max Scaler, with optional feature clipping
- **Motion mirroring** — mirror the motion set to double your animation coverage
- **Motion events** — Animation *method tracks* (call names) and *audio tracks* become events; receive them via the `motion_event_fired(tag)` signal (`audio:<stream>` tags for audio tracks)
- **Motion extraction** — root motion delta extraction, with blending between the previous and next motion when switching
- **Tunable search** — search frequency (Hz), database sample rate, KD-tree depth and leaf size
- **Debug & profiling** — editor **Monitor dock** (update timing histograms, database stats, per-feature search costs), feature schema debug draw, query pose and query velocity visualization
- **Serialization** — CSV export (`csv_serializers`) of the feature matrix, frame database, and serialized data, plus versioned binary (de)serialization of query vectors
- **Two ways to integrate** — a self-contained `MotionMatching` node that drives a `Skeleton3D` directly, or `MotionMatchingNode` (an `AnimationNode`) for use inside an `AnimationTree`

---

## 📦 Repository layout

```
gmm/
├── addons/gmm/                        # The Godot addon — copy this into your project
│   ├── motion_matching.gdextension    # GDExtension configuration
│   └── bin/android/                   # Prebuilt binaries (currently: Android arm64 template_debug)
├── src/                               # GDExtension C++ sources
│   ├── core/
│   │   ├── math/                      # FeatureMatrix, QueryVector, scalers (standard / min–max), transformers
│   │   └── types/                     # Skeleton, SkeletonNode, Transform, Velocity
│   ├── actor/                         # ActorInstance, ActorManager, AnimGraph*, MotionSystem
│   ├── animation/                     # Motion, MotionData, MotionInstance(Pool), MotionSet, MotionEvent*
│   ├── pose/                          # Pose, PoseData, PoseDataJointVelocities, TransformData, TransformSpace
│   ├── features/                      # Feature + Position/Velocity/AngularVelocity/Trajectory, FeatureSchema
│   ├── database/                      # Frame, FrameDatabase, KdTree, MotionMatchingData
│   ├── trajectory/                    # TrajectoryQuery, TrajectoryHistory
│   ├── runtime/                       # MotionMatchingInstance, EventData, CSV serializers
│   ├── nodes/                         # MotionMatching node + MotionMatchingNode (AnimationNode)
│   ├── debug/                         # Debug renderer (lines/balls/cylinders/overlay), Monitor dock, compat
│   └── system/                        # Class registration, project settings
├── godot-cpp/                         # Godot C++ bindings (git submodule — do not modify)
└── SConstruct                         # SCons build script
```

---

## ✅ Requirements

- **Godot 4.7** or newer (`compatibility_minimum = "4.7"`)
- To build from source:
  - [SCons](https://scons.org/) 4.x
  - A C++17-capable compiler (Clang/GCC/MSVC)
  - Python 3
  - `godot-cpp` — already bundled as a git submodule in this repository

---

## 🚀 Installation (using the prebuilt binary)

1. Copy the `addons/gmm/` folder into your Godot project's `addons/` directory.
2. Reload the project in Godot (**Project → Reload Current Project**).
3. Add a `MotionMatching` node to your scene (see *Quick start* below).

> **Editor note:** the prebuilt binary is a runtime `template_debug` build. To use the
> in-editor tools (like the Monitor dock) you need to build an `editor` target for your
> desktop platform and add the matching entry to the `.gdextension` file.

---

## ⚠️ About the prebuilt binaries — and an apology 🗿

This **Godot 4.7 motion matching** GDExtension currently ships **exactly one prebuilt binary**:

| Platform | Architecture | Target | File |
| --- | --- | --- | --- |
| Android | arm64 | `template_debug` | `addons/gmm/bin/android/libmotion_matching.android.template_debug.arm64.so` |

**Why only one?** This entire GDExtension was developed **on an Android phone**. GMM can be
compiled for **any platform Godot supports** — Windows, Linux, macOS, Android (all ABIs), iOS,
Web, and any combination of `editor` / `template_debug` / `template_release` — but a phone is
simply not a build server, and my phone is definitely not strong enough to compile binaries for
every platform out there. 🗿

So, sincerely: **sorry I can't provide all platform binaries — please compile it yourself
for the platform you need.** Assembly instructions are below; it's one command once the
submodule is in place. 🙏

### Adding your built library to the `.gdextension`

After building, register your platform in `addons/gmm/motion_matching.gdextension`:

```ini
[libraries]

; prebuilt: Android arm64 debug (template_debug)
android.debug.arm64 = "bin/android/libmotion_matching.android.template_debug.arm64.so"

; examples — uncomment / adapt after you build your own:
; linux.editor.x86_64 = "bin/linux/libmotion_matching.linux.editor.x86_64.so"
; linux.debug.x86_64  = "bin/linux/libmotion_matching.linux.template_debug.x86_64.so"
; windows.debug.x86_64 = "bin/windows/libmotion_matching.windows.template_debug.x86_64.dll"
```

The naming convention is `<platform>.<feature>.<arch>`, where `feature` is `editor`,
`debug` (`template_debug`), or `release` (`template_release`).

---

## 🔨 Building from source

```bash
# 1. Clone (or update) with the godot-cpp submodule
git clone --recurse-submodules <this-repo-url>
cd gmm
# if you already cloned without --recurse-submodules:
git submodule update --init --recursive

# 2. Build for your platform. Examples:

# Android arm64, debug template (the configuration of the prebuilt binary)
scons platform=android arch=arm64 target=template_debug -j4

# Linux x86_64, editor + debug template
scons platform=linux arch=x86_64 target=editor -j4
scons platform=linux arch=x86_64 target=template_debug -j4

# Windows x86_64, release template
scons platform=windows arch=x86_64 target=template_release -j4
```

Output is written automatically to `addons/gmm/bin/<platform>/`.

### Build options

| Option | Default | Description |
| --- | --- | --- |
| `platform=` | host platform | Target platform: `linux`, `windows`, `macos`, `android`, `ios`, `web`, … |
| `arch=` | host arch | Target architecture: `x86_64`, `arm64`, `x86_32`, `arm32`, … |
| `target=` | `template_debug` | Build target: `editor`, `template_debug`, or `template_release` |
| `api_version=` | `4.7` | Godot GDExtension API version |
| `motion_matching_monitor=` | `1` | Set `0` to compile out the runtime monitor (`GMM_MONITOR_ENABLED`) |
| `-jN` | — | Parallel build jobs (e.g. `-j4`) |

**Notes**

- The first build also compiles `godot-cpp`, which takes a while. Later builds are incremental and fast.
- For Android you need the Android NDK. This project was built with **NDK r30**; make sure `ANDROID_NDK_ROOT` (or `ANDROID_HOME`) points at your NDK.
- Android builds link with `-static-libstdc++` automatically (handled by the `SConstruct`).

---

## 🎮 Quick start

1. Import your character as a **`Skeleton3D`** (with animations either in an `AnimationPlayer` or as `Animation` resources).
2. Add a **`MotionMatching`** node and point **`skeleton_path`** at the skeleton.
3. Fill the **`motions`** array with your locomotion `Animation` resources (idle, walk, run, strafes…).
4. Leave **`feature_schema`** empty to auto-generate the default schema, or assign your own `FeatureSchema` resource to customize which bones/features are matched.
5. Drive the target every frame from your gameplay code:

```gdscript
extends Node3D

@onready var motion_matching: MotionMatching = $MotionMatching
@onready var player: CharacterBody3D = $Player

var input_direction := Vector3.ZERO  # your input, world space

func _process(_delta: float) -> void:
    motion_matching.target_position = player.global_position + input_direction
    motion_matching.target_facing_dir = input_direction
    motion_matching.use_facing_dir = input_direction != Vector3.ZERO
```

6. React to motion events — any *method track* call name (or `audio:<file>` for audio tracks) in your animations is reported:

```gdscript
func _ready() -> void:
    $MotionMatching.motion_event_fired.connect(_on_motion_event)

func _on_motion_event(tag: String) -> void:
    # tag is e.g. "FootstepL" or "audio:footstep.wav"
    print("Motion event: ", tag)
```

> Prefer an `AnimationTree` workflow? Use **`MotionMatchingNode`** (an `AnimationNode`)
> instead. It exposes the same matching core with `target_pos`, `target_facing_dir` and
> `use_facing_dir` inputs for `AnimationNodeBlendTree`.

---

## ⚙️ Node properties (`MotionMatching` / `MotionMatchingNode`)

### Matching

| Property | Default | Description |
| --- | --- | --- |
| `search_frequency` | `5.0` | How many times per second a new best-matching frame is searched (Hz). Higher = more responsive, more CPU. |
| `sample_rate` | `30` | Database sample rate in frames per second. |
| `trajectory_query_mode` | `Target-driven` | `Target-driven` follows `target_position`; `Automatic (Demo)` generates a demo path. |
| `path_radius` | `1.0` | Demo path radius (Automatic mode only). |
| `path_speed` | `1.0` | Demo path speed (Automatic mode only). |

### Data normalization

| Property | Default | Description |
| --- | --- | --- |
| `normalize_data` | `false` | Normalize features before matching. |
| `feature_scaler_type` | `Standard Scaler` | `Standard Scaler` (z-score) or `Min-max Scaler`. |
| `feature_min` / `feature_max` | `0.0` / `1.0` | Output range for the min–max scaler. |
| `clip_features` | `false` | Clamp normalized features into the output range. |

### Acceleration structure & data

| Property | Default | Description |
| --- | --- | --- |
| `max_kd_tree_depth` | `15` | Maximum depth of the KD-tree used for searching. |
| `min_frames_per_kd_tree_node` | `1000` | A KD-tree node splits only if it holds more frames than this. |
| `mirror` | `false` | Also build mirrored versions of all motions. |
| `feature_schema` | *(auto)* | Custom `FeatureSchema` resource; a default schema is generated when empty. |
| `motions` | `[]` | `Array[Animation]` — the animations to build the motion database from. |

### Target & debug

| Property | Default | Description |
| --- | --- | --- |
| `skeleton_path` | — | `Skeleton3D` that the node drives. |
| `target_position` | — | World-space target position (Target-driven mode). |
| `target_facing_dir` | `(0, 0, -1)` | Desired facing direction. |
| `use_facing_dir` | `false` | Whether facing direction should be used in the query. |
| `visualize_feature_schema` | `false` | Debug-draw the active feature schema. |

### Signals

| Signal | Description |
| --- | --- |
| `motion_event_fired(tag: String)` | Emitted when a motion event is reached. Tags come from Animation method tracks (the call name) and audio tracks (`audio:<stream name>`). |

---

## 🛠 Debug tools & project settings

GMM adds the following **project settings** (created automatically on first use):

| Setting | Default | Description |
| --- | --- | --- |
| `motion_matching/debug_draw` | `false` | Master switch for runtime debug drawing. |
| `motion_matching/debug_draw_query_pose` | `false` | Draw the query (searched) pose. |
| `motion_matching/debug_draw_query_velocities` | `false` | Draw query velocities as arrows. |
| `motion_matching/debug_draw_velocity_scale` | `0.1` | Velocity arrow scale for debug drawing. |
| `motion_matching/use_kd_tree` | `true` | Use the KD-tree for searching (fallback: linear search). |
| `motion_matching/multi_threaded_initialization` | `true` | Build the frame database using multiple threads. |

**Monitor dock** — with an `editor`-target build loaded, GMM exposes a Monitor dock in the
editor with database statistics, update timing histograms, and per-feature search costs. The
monitor code is compiled in by default; build with `motion_matching_monitor=0` to strip it.

---

## 🤖 Made with AI assistance

Transparency time: **this project was built with the help of AI.** An AI coding agent
assisted with the complete restructuring of the codebase (a full "rombak total" into the
domain-driven `src/` layout you see today — file architecture, naming conventions,
includes, and documentation), alongside the human maintainer. It's a human + AI
collaboration, so if you find anything cursed in here, blame both of us. 🤝🤖

---

## 📄 License

This project is licensed under the **MIT License** — see [LICENSE](LICENSE).
Copyright © 2026 **Nexarioneo**.

---

## 🙏 Credits & acknowledgements

- **[O3DE — Open 3D Engine](https://o3de.org)** — the `MotionMatching` Gem and **EMotionFX**
  (Apache-2.0) are the origin of this implementation's architecture, algorithms, and class
  design. This project is a Godot-API port and continuation of those ideas.
- **[godot-cpp](https://github.com/godotengine/godot-cpp)** — official C++ bindings for Godot.
- **[Godot Engine](https://godotengine.org)** — the engine, MIT licensed.
