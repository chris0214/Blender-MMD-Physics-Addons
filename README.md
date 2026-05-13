# MikuMikuPhysics 2.0

> **本插件使用 GPT-5.5 辅助编写、调试与文档整理。**

MikuMikuPhysics 是一个 Blender 4.2+ 插件，用于直接模拟由 mmd_tools 导入的 PMX/MMD 模型刚体物理。插件通过外部 `pmx_bullet.dll` 使用 Bullet 2.82 r2704，不依赖 Blender 内置 Bullet，目标是尽量接近 MMD 与 PMXEditor TransformView 的物理行为。

- 作者/维护者：克里斯提亚娜
- 插件类型：Blender Add-on / Extension
- 许可证：GPL-3.0-or-later
- 当前插件包名：`MikuMikuPhysics`

## 主要功能

- 直接读取 mmd_tools 导入后的 PMX 模型根对象、骨架、刚体、关节、碰撞组与骨骼绑定。
- 使用外部 Bullet 2.82 native DLL 进行物理解算。
- 支持不依赖 Blender 时间线播放的实时物理预览。
- 支持时间线模式，方便按帧模拟和动作检查。
- 支持将物理结果烘焙为 Blender 骨骼关键帧。
- 支持多模型预览：独立世界、影子碰撞、完全共享世界。
- 支持彩色半透明刚体/关节调试可视化。
- 支持性能统计，显示 Bullet、读回、骨骼写回和 depsgraph 刷新耗时。
- 支持高速拖动保护，减少拖动模型时的滞后和穿模。
- 支持针对头发、裙摆、尾巴、链条等结构的名称规则参数。

## 安装

推荐把 `MikuMikuPhysics` 文件夹作为 Blender 插件安装。目录结构应类似：

```text
MikuMikuPhysics/
  __init__.py
  blender_manifest.toml
  config.py
  native/
  operators/
  panels/
  physics/
```

安装方式：

1. 打开 Blender 4.2 或更新版本。
2. 进入 `Edit > Preferences > Add-ons`。
3. 点击 `Install...`，选择打包好的 zip，或把 `MikuMikuPhysics` 文件夹放入 Blender 插件目录。
4. 启用 `MikuMikuPhysics`。
5. 在 3D View 右侧 Sidebar 中打开 `MMP` 面板。

传统插件目录示例：

```text
%APPDATA%\Blender Foundation\Blender\4.2\scripts\addons\MikuMikuPhysics
```

## 使用前准备

本插件不负责导入 PMX 模型。请先安装并启用 mmd_tools，然后：

1. 使用 mmd_tools 导入 PMX 模型。
2. 确认模型在 Outliner 中包含 `armature`、`rigidbodies`、`joints` 等对象。
3. 如需导入动作，可以先用 mmd_tools 导入 VMD，也可以使用本插件面板中的 `Import VMD Motion` 调用 mmd_tools。

注意：

- 开始模拟时，插件会临时停用 Blender 内置 rigid body world，避免双重物理。
- 插件会临时静音部分 mmd_tools 物理约束，停止或重置后会恢复。
- 如果刚体/关节没有由 mmd_tools 正确创建，插件会提示 `Selected model has no mmd_tools rigid bodies`。

## 快速开始

### 实时预览

1. 选择 PMX 模型根对象，或选择模型下任意对象。
2. 点击 `Use Active Model`。
3. 点击 `Scan Model` 检查刚体、关节、碰撞数据。
4. 选择质量预设：
   - `Preview`：低配视口预览，速度优先。
   - `Default`：默认平衡档，适合多数实时预览。
   - `Mid`：更高质量，接近 MMD 行为。
   - `Hard`：更强约束和更多子步，适合复杂链条。
5. 点击 `Start` 开始实时模拟。
6. 点击 `Stop` 停止模拟。
7. 点击 `Reset` 恢复开始模拟前捕获的状态。

实时模式使用 Blender Timer，不依赖时间线播放。时间线停止时，物理仍可继续运行。

### 时间线模式

勾选 `Timeline Mode` 后，插件按 Blender 帧变化进行模拟。适合：

- 按帧检查物理结果。
- 与 VMD 动作一起检查指定帧效果。
- 烘焙前做确定性测试。

实时预览和时间线模式不要同时混用。切换模式前建议先停止模拟并重置。

### 多模型

`Multi Model Mode` 控制 `Start All` 的行为：

- `Independent Worlds`：每个模型一个独立 Bullet 世界，最稳定，模型之间不碰撞。
- `Root Isolated`：独立世界兼容模式。
- `Shadow Collision`：推荐的多模型碰撞模式。每个模型保留自己的 Bullet 世界，其他模型作为运动学影子碰撞体插入，避免远距离拖动互相干涉。
- `Full Shared World`：实验模式。所有模型进入同一个动态 Bullet 世界，物理耦合更强，也更容易相互带动。

当前建议：

- 多角色但不需要碰撞时，使用 `Independent Worlds`。
- 需要角色之间碰撞时，优先使用 `Shadow Collision`。
- `Full Shared World` 主要用于调试和对比。

### 烘焙

1. 设置 `Start` 和 `End`。
2. 设置 `Preroll`，让物理在写关键帧前先稳定。
3. 根据需要勾选 `Restore After Bake`。
4. 点击 `Bake`。

烘焙会把动态物理骨骼结果写入 Blender 骨骼关键帧。烘焙完成后，可以关闭实时物理并直接播放 Blender 动画。

## 参数建议

### Bullet 设置

- `Fixed Step`：常用 `60 Hz`、`90 Hz`、`120 Hz`。数值越高越精细，也越慢。
- `Max Substeps`：每次 Timer tick 最多补多少个物理步。提高可减少掉步，但会降低视口帧率。
- `Solver Iterations`：约束求解迭代次数。提高可增强稳定性。
- `Gravity / Gravity Scale`：重力方向和倍率。
- `Time Scale`：物理时间倍率。

### 高速拖动保护

`Realtime Performance` 中的高速拖动选项用于减少拖动模型时的滞后和穿模：

- `Fast Drag Protection`：总开关。
- `Static Body Compensation`：静态/运动学刚体拖动时拆分成更细物理段。
- `Dynamic Bone Compensation`：动态骨刚体拖动时提高自适应分段。
- `Max Drag Segments`：一次快速拖动最多拆成多少段。默认 32，可按性能提高到 48。
- `Extreme Drag Resync`：拖动跳变过大时做一次物理重同步。
- `Resync Threshold`：触发极限重同步的模型空间位移阈值。
- `Clear Velocity On Resync`：重同步时清速度。更稳定，但惯性更弱。

推荐默认：

```text
Fast Drag Protection: On
Static Body Compensation: On
Dynamic Bone Compensation: On
Max Drag Segments: 32
Extreme Drag Resync: On
Resync Threshold: 0.5
Clear Velocity On Resync: Off
```

如果快速拖动仍明显穿模，可以把 `Max Drag Segments` 调到 48，或把 `Resync Threshold` 降到 0.25。

### 性能优化

如果视口帧率很低：

- 质量预设切到 `Preview`。
- 关闭 `Update Rigid Objects`。
- 保持 `Skip Unchanged Bones` 开启。
- 关闭调试可视化。
- 降低 `Solver Iterations` 与 `Max Substeps`。
- 多模型时优先使用 `Independent Worlds` 或 `Shadow Collision`。

性能面板中常见瓶颈：

- `Native Bullet` 高：物理解算本身重，降低子步、迭代或刚体数量。
- `Apply Bones` 高：骨骼写回重，保持跳过未变骨骼开启。
- `Depsgraph` 高：Blender 主线程刷新重，降低视口显示复杂度。
- `Object Writes` 高：关闭 `Update Rigid Objects`。

## 技术设计

插件分为三层：

1. Blender UI / 控制层
   - `config.py`
   - `operators/AddonOperators.py`
   - `panels/AddonPanels.py`

2. Python 同步层
   - `physics/pmx_data_reader.py`
   - `physics/physics_world.py`
   - `physics/physics_sync.py`
   - `physics/bake.py`
   - `physics/shadow_physics_world.py`

3. Native Bullet 物理层
   - `physics/bullet_native.py`
   - `native/pmx_bullet_api.h`
   - `native/pmx_bullet_api.cpp`
   - `native/pmx_bullet.dll`

数据流：

```text
mmd_tools PMX model
  -> pmx_data_reader.read_model()
  -> PmxModelData / RigidBodyData / JointData
  -> ctypes bridge
  -> native/pmx_bullet.dll (Bullet 2.82 r2704)
  -> body transform readback
  -> Pose Bone / rigid object writeback
  -> Blender viewport or baked keyframes
```

核心设计：

- 独立物理引擎：不依赖 Blender 内置 Bullet。
- 模型本地坐标：物理空间以 PMX 模型根对象为基准，模型在场景中移动/旋转不会破坏解算。
- 骨骼-物理双向同步：静态刚体从骨骼获取位置，动态/动态骨刚体从物理结果回写骨骼。
- Temporal Kinematic 初始化：动态刚体初始化时先以 kinematic 状态对齐骨骼，再恢复 dynamic，减少初始爆炸。
- Timer 实时预览：实时模式独立于 Blender 时间线播放。
- 影子碰撞：多模型碰撞时参考 Babylon-MMD 的 shadow body 思路，减少远距离动态岛互相拖拽。

## 已知限制

- Blender Python 与 depsgraph 刷新存在主线程开销，实时手感无法完全等同 MMD 本体。
- 高速拖动时，离散 Bullet 碰撞仍可能穿模；可通过高速拖动保护缓解。
- 多模型烘焙和完全物理耦合仍属于后续计划。
- `Full Shared World` 是实验模式，不建议作为默认生产流程。

## 参考与借物说明

本项目实现过程中参考了以下项目或软件的行为、设计和公开代码结构。感谢这些项目的作者与维护者。

- **mmd_tools**：参考其 PMX 模型数据组织、根对象查找、刚体/关节归属、骨骼同步方式。本插件不直接包含 mmd_tools 源码。
  - GitHub：<https://github.com/MMD-Blender/blender_mmd_tools>
  - 原始仓库：<https://github.com/powroupi/blender_mmd_tools>
- **Babylon-MMD**：参考其多物理世界、`worldId`、rigid body bundle、shadow body、Immediate/Buffered 更新链路等设计。作者：**noname0310**。
  - 作者 GitHub：<https://github.com/noname0310>
  - 项目 GitHub：<https://github.com/noname0310/babylon-mmd>
- **Saba**：参考其 MMD/PMX 运行时和 Bullet 物理处理思路。
  - 作者 GitHub：<https://github.com/benikabocha>
  - 项目 GitHub：<https://github.com/benikabocha/saba>
- **MMD / MikuMikuDance**：作为目标物理行为参考。
- **PMXEditor / TransformView**：作为模型编辑器内实时物理解算行为参考。
- **MMD Bridge**：参考其 MMD 本体物理与烘焙流程。
- **NexGiMa**：参考其 PMX/MMD 物理表现和参数风格。
- **MikuMikuDayo**：参考其 MMD 模型与物理运行逻辑。
- **Bullet Physics**：本插件 native 层使用 Bullet 2.82 r2704 进行物理解算。

如开源发布，请同时保留本 README、`LICENSE`、`COPYING`，并遵守各参考项目自身许可证。本文中的“参考”表示行为和架构层面的兼容性研究，不表示这些项目为本插件背书。

## 更新日志

### 2.0.0

- 插件包名改为 `MikuMikuPhysics`，Blender N 面板标签缩写为 `MMP`。
- 使用外部 Bullet 2.82 r2704 DLL 进行 PMX/MMD 刚体与关节模拟。
- 支持实时 Timer 物理预览、时间线模式、单步、停止、强制停止与重置。
- 支持 mmd_tools PMX 刚体/关节读取、VMD 导入入口和物理烘焙。
- 支持多模型模拟：独立世界、影子碰撞、共享世界等模式。
- 增加高速拖动保护、极限重同步、静态/动态骨补偿等交互稳定化选项。
- 增加彩色半透明物理调试可视化与性能统计面板。
- 整理 README、GPL 许可证、参考项目与借物说明，作为首次公开版本。

## 后续计划

- 交互即时响应模式：拖动模型或骨骼时，尽量同帧执行物理同步，降低滞后感。
- 更完善的多模型碰撞烘焙。
- 更细粒度的性能模式和视口降级策略。
- 更完善的物理结果对比工具。
