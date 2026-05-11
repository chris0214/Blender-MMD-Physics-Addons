# PMX Physics 1.1.0

PMX Physics 是一个 Blender 4.2+ 插件，用于在 Blender 中直接模拟 mmd_tools 导入的 PMX 模型刚体物理。插件通过外部 `pmx_bullet.dll` 使用 Bullet 2.82 r2704，而不是 Blender 内置刚体系统，目标是尽量接近 MMD / PMXEditor TransformView 的物理行为。

作者 / 维护者：克里斯提亚娜  
许可证：GPL-3.0-or-later  
插件类型：Blender Add-on / Extension  

## 1.1.0 更新

- 基于 `pmx_physics-1.0.0` 稳定版整理发布。
- 修复实时模拟时拖动 mmd_tools 模型根空物体没有物理反馈的问题。
- 运行中移动或旋转模型根对象时，动态刚体会保留上一刻世界空间位置，再由 Bullet 继续解算，因此头发、裙摆、尾巴等物理部件会产生拖拽惯性。
- 该版本仍使用 Bullet 2.82 r2704 native DLL，DLL API 版本为 6。

## 1. 功能概览

- 直接读取 mmd_tools 导入后的 PMX 模型根对象、刚体、关节、碰撞组和骨骼绑定。
- 使用独立 Bullet 2.82 native DLL 进行物理计算。
- 支持不依赖 Blender 时间线播放的实时物理预览。
- 支持时间线模式，便于按帧模拟。
- 支持将物理结果烘焙为 Blender 骨骼关键帧。
- 支持彩色半透明刚体/关节调试可视化。
- 支持性能统计，显示 Bullet、读回、骨骼写回和 depsgraph 刷新耗时。
- 支持针对头发、裙摆、尾巴、链条等结构的 mmd_tools 兼容规则。

## 2. 安装方法

推荐使用发布包：

```text
pmx_physics-1.1.0.zip
```

安装步骤：

1. 打开 Blender 4.2 或更新版本。
2. 进入 `Edit > Preferences > Add-ons`。
3. 点击 `Install...`。
4. 选择 `pmx_physics-1.1.0.zip`。
5. 启用 `PMX Physics`。
6. 在 3D View 右侧 Sidebar 中打开 `PMX Physics` 面板。

如果使用 Blender 4.2 的传统插件目录，也可以将插件文件夹放到：

```text
%APPDATA%\Blender Foundation\Blender\4.2\scripts\addons\pmx_physics
```

## 3. 使用前准备

本插件不负责导入 PMX 模型。请先安装并启用 mmd_tools，然后：

1. 使用 mmd_tools 导入 PMX 模型。
2. 确认模型在 Outliner 中包含：
   - armature
   - rigidbodies
   - joints
3. 如需导入动作，可以先用 mmd_tools 导入 VMD，也可以使用本插件面板中的 `Import VMD Motion` 按钮调用 mmd_tools 导入。

注意：

- 本插件会在开始模拟时临时停用 Blender 内置 rigid body world，避免双重物理。
- 本插件会静音 mmd_tools 的部分物理约束，停止或重置后会恢复。
- 如果刚体/关节没有由 mmd_tools 正确创建，插件会提示 `Selected model has no mmd_tools rigid bodies`。

## 4. 快速开始

### 4.1 实时预览

1. 选择 PMX 模型根对象，或选择模型下任意对象。
2. 在 `PMX Physics` 面板点击 `Use Active Model`。
3. 点击 `Scan Model` 检查刚体、关节、非碰撞对数量。
4. 选择质量预设：
   - `Default`：低子步，较轻量，适合快速预览。
   - `Mid`：更接近 MMD 的平衡预设。
   - `Hard`：更高子步和预热，适合较复杂模型。
5. 点击 `Start` 开始实时模拟。
6. 点击 `Stop` 停止模拟。
7. 点击 `Reset` 恢复开始模拟前捕获的初始状态。

实时模式使用 Blender Timer，不依赖时间线播放。即使时间线停止，物理也可以继续运行。

### 4.2 时间线模式

勾选 `Timeline Mode` 后，插件会按 Blender 帧变化进行模拟。

适合：

- 需要按帧检查物理结果。
- 准备烘焙前做确定性测试。
- 与 VMD 动作一起检查指定帧效果。

实时预览和时间线模式不要同时使用。切换模式前建议先停止模拟并重置。

### 4.3 烘焙物理

1. 设置 `Start` 和 `End`。
2. 设置 `Preroll`，用于开始写关键帧前让物理先稳定。
3. 根据需要勾选 `Restore After Bake`。
4. 点击 `Bake`。

烘焙会把动态物理骨骼的结果写入 Blender 骨骼关键帧。烘焙完成后，可以关闭实时物理并直接播放 Blender 动画。

### 4.4 烘焙对比

`Compare Bake` 会把当前已有关键帧与一次新的 Bullet 模拟结果做对比，输出位移和旋转误差统计。

用途：

- 检查烘焙是否成功。
- 对比不同质量参数对结果的影响。
- 定位某个模型是否存在明显的物理偏差。

## 5. 面板参数说明

### 5.1 Bullet 设置

| 参数 | 说明 |
| --- | --- |
| `Bullet DLL` | 外部 Bullet 2.82 DLL 路径。默认使用插件内置 `native/pmx_bullet.dll`。 |
| `Fixed Step` | 物理步长预设，常用 `60 Hz`、`90 Hz`、`120 Hz`。 |
| `Max Substeps` | 每次 Timer tick 最多补多少个物理步。 |
| `Solver Iterations` | Bullet 约束求解迭代次数。更高更稳定，但更慢。 |
| `Gravity` | Blender 模型本地坐标下的重力方向。 |
| `Gravity Scale` | 重力倍率。 |
| `Time Scale` | 物理时间倍率。 |

### 5.2 质量预设

| 预设 | 特点 |
| --- | --- |
| `Default` | 默认 ERP/CFM，较轻量，适合快速检查。 |
| `Mid` | 120Hz 固定步长，使用 Bullet 默认 stop ERP/CFM，适合多数模型。 |
| `Hard` | 90Hz、高子步和预热，适合复杂长链结构。 |

点击预设右侧的勾选按钮后，当前预设会写入面板参数。

### 5.3 关节质量

| 参数 | 说明 |
| --- | --- |
| `Use Frame Offset` | 使用 Bullet Generic6Dof frame offset 行为。 |
| `Joint Stop ERP` | 关节角度/位移限制的 ERP 覆盖值。负数表示使用 Bullet 默认值。 |
| `Joint Stop CFM` | 关节角度/位移限制的 CFM 覆盖值。负数表示使用 Bullet 默认值。 |
| `Locked Joint ERP` | 锁定位移轴使用的 ERP。 |
| `Locked Joint CFM` | 锁定位移轴使用的 CFM。 |
| `Locked Joint Pullback` | 对锁定位移关节做小幅回拉修正，默认开启。 |
| `Resting Body Stabilization` | 接近静止的动态刚体进入稳定状态，默认关闭。 |
| `Parent Chain Mode Fix` | 修正动态/动态骨父子链模式，默认开启。 |

### 5.4 mmd_tools 兼容规则

`Name Rules` 默认关闭。启用 `MMD Compatible` 后，插件会按刚体/关节名称匹配头发、裙摆、尾巴、软部件和配件，并应用更柔和的阻尼、ERP/CFM 或弹簧阻尼。

可调参数：

- `Rule Strength`
- `Soft Joint ERP`
- `Soft Joint CFM`
- `Locked ERP`
- `Locked CFM`
- `Spring Damping`
- `Linear Damping Scale`
- `Angular Damping Scale`
- 各类关键词

建议：

- 普通模型先保持关闭。
- 如果尾巴、裙摆、链条有轻微高频抖动，再开启 `MMD Compatible` 并小幅调节。

### 5.5 实时性能

`Realtime Performance` 区域用于提高视口流畅度。

| 参数 | 说明 |
| --- | --- |
| `Update Rigid Objects` | 实时预览时是否把物理结果写回 mmd_tools 刚体对象。默认关闭。 |
| `Skip Unchanged Bones` | 骨骼变化小于阈值时跳过写入，默认开启。 |
| `Write Position Threshold` | 跳过写入的位置阈值。 |
| `Write Rotation Threshold` | 跳过写入的旋转阈值。 |

建议：

- 正常预览模型时保持 `Update Rigid Objects` 关闭，可以减少数百个 Object transform 写入。
- 需要观察刚体调试形状跟随时，再打开 `Update Rigid Objects`。
- 如果模型静止后仍有很轻微的显示抖动，可以略微提高写入阈值。

### 5.6 性能统计

勾选 `Performance` 后会显示：

- 刚体数量、关节数量、非碰撞对数量
- 当前 tick 的 step 数和 smoothing 数
- `Timer Tick`
- `World Step`
- `Collect Pose`
- `Native Bullet`
- `Readback`
- `Apply Bones`
- `Depsgraph`
- `Writes Bones / Objects`

判断瓶颈：

- `Native Bullet` 高：物理求解本身重，降低 `Max Substeps`、`Solver Iterations` 或刚体数量。
- `Apply Bones` 高：骨骼写回重，保持 `Skip Unchanged Bones` 开启。
- `Depsgraph` 高：Blender 主线程刷新重，隐藏复杂网格或降低视口显示负担。
- `Object Writes` 高：关闭 `Update Rigid Objects`。

## 6. 调试可视化

`Physics Debug` 可以给刚体/关节应用彩色半透明材质。

模式：

- `By Body Mode`：按静态、动态、动态骨模式着色。
- `By Collision Group`：按 PMX 碰撞组着色。

按钮：

- `Apply Debug Visuals`：应用调试显示。
- `Clear Debug Visuals`：恢复原显示状态。

调试可视化主要用于检查：

- 刚体是否导入完整。
- 碰撞组是否合理。
- 尾巴、裙摆、头发是否存在异常交叉。
- 哪些刚体是静态、动态或动态骨。

## 7. 技术架构

插件分为三层：

1. **Blender UI / 控制层**
   - `config.py`
   - `operators/AddonOperators.py`
   - `panels/AddonPanels.py`

2. **Python 同步层**
   - `physics/pmx_data_reader.py`
   - `physics/physics_world.py`
   - `physics/physics_sync.py`
   - `physics/bake.py`

3. **Native Bullet 物理层**
   - `physics/bullet_native.py`
   - `native/pmx_bullet_api.h`
   - `native/pmx_bullet_api.cpp`
   - `native/pmx_bullet.dll`

数据流：

```text
mmd_tools PMX model
  -> pmx_data_reader.read_model()
  -> PmxModelData / RigidBodyData / JointData
  -> bullet_native ctypes
  -> native/pmx_bullet.dll (Bullet 2.82 r2704)
  -> body transform readback
  -> physics_world writes Pose Bones / rigid objects
  -> Blender viewport or baked keyframes
```

## 8. 坐标系统

PMX / MMD 使用左手坐标系，Y 轴向上。Blender 使用右手坐标系，Z 轴向上。

本插件使用模型本地坐标作为物理空间：

- 刚体矩阵以模型根对象为基准。
- 模型在 Blender 场景中移动或旋转时，不影响物理解算。
- 写回时再乘以模型根对象 `matrix_world`。

原因：

1. 支持模型在场景中自由移动。
2. 避免场景坐标影响物理。
3. 更接近 MMD / PMXEditor TransformView 的局部模型行为。

## 9. 刚体和骨骼同步

PMX 刚体模式：

| 模式 | 行为 |
| --- | --- |
| `MODE_STATIC` | 从骨骼读取位置，作为运动学刚体驱动物理。 |
| `MODE_DYNAMIC` | 从物理读取位置和旋转，驱动骨骼。 |
| `MODE_DYNAMIC_BONE` | 从物理读取旋转，保留骨骼当前位置。 |

同步策略：

- 开始模拟时读取当前 Pose。
- 静态刚体每步从骨骼获得 transform。
- 动态刚体从 Bullet 读回 transform。
- Python 层根据刚体绑定关系写回 Pose Bone。
- 实时预览可跳过微小变化写入以提高视口性能。

## 10. Temporal Kinematic 初始化

MMD 模型在当前帧可能已经套用了 VMD 动作。如果直接把动态刚体放入物理世界，长发、裙摆、尾巴可能因为初始距离过大而炸开。

插件使用：

- Temporal Kinematic Init
- Startup Sync Steps
- Prewarm

流程：

1. 动态刚体先临时以 kinematic 方式放到骨骼位置。
2. 从 rest pose 平滑同步到当前动画 pose。
3. 再恢复为 dynamic。
4. 预热并清理初始速度。

相关参数：

- `Startup Sync Steps`
- `Prewarm Steps`

## 11. 碰撞和关节

插件读取 PMX / mmd_tools 的：

- `collision_group_number`
- `collision_group_mask`
- 刚体形状、大小、质量、阻尼
- 关节线性限制
- 关节角度限制
- 线性弹簧
- 角度弹簧

native 层使用 Bullet 6DoF Spring constraint。非碰撞对由 PMX mask 和关节关系共同构建。

如果出现自碰撞、局部炸开或高频抖动，优先检查：

1. 调试可视化中的碰撞组。
2. 模型自身 PMX 刚体是否重叠严重。
3. `Name Rules` 是否需要开启。
4. `Max Substeps` 和 `Solver Iterations` 是否过低。
5. 是否导入 VMD 后直接在大幅姿态处开始模拟。

## 12. 性能设计

实时预览中的主要成本通常不是 Bullet 本身，而是 Blender 主线程：

- 写回 Pose Bone
- 写回刚体 Object transform
- 刷新 depsgraph
- 视口绘制复杂网格

1.4 版加入了几项实时优化：

- Timer 未达到固定步长时不刷新 depsgraph。
- 实时预览默认不写回所有刚体对象。
- 骨骼目标变化低于阈值时跳过写入。
- 初始化时缓存物理骨骼和写回顺序。
- 性能面板显示骨骼/对象写入数量。

多线程限制：

- Bullet 求解可以在 native 层或 worker 中并行化。
- Blender 的 `bpy.data`、Pose Bone 写入和 depsgraph 刷新必须留在主线程。
- GPU / CUDA 对当前瓶颈帮助有限，因为主要耗时在 Blender 主线程数据写回和视图刷新。

## 13. 文件结构

| 路径 | 作用 |
| --- | --- |
| `__init__.py` | 插件入口和 `bl_info` 元数据 |
| `blender_manifest.toml` | Blender Extension manifest |
| `LICENSE` | GPL v3 正文 |
| `COPYING` | GPL-3.0-or-later 简短声明 |
| `config.py` | Scene 设置、质量预设、性能统计属性 |
| `localization.py` | 中文本地化 |
| `operators/AddonOperators.py` | UI 操作器 |
| `panels/AddonPanels.py` | View3D Sidebar 面板 |
| `physics/types.py` | PMX 刚体、关节、模型数据结构 |
| `physics/transforms.py` | 坐标转换 |
| `physics/pmx_data_reader.py` | mmd_tools 数据读取 |
| `physics/bullet_native.py` | ctypes native API 封装 |
| `physics/physics_world.py` | 物理世界、同步、写回、性能统计 |
| `physics/physics_sync.py` | Timer / timeline 控制 |
| `physics/bake.py` | 烘焙和烘焙对比 |
| `physics/debug_visual.py` | 调试可视化 |
| `native/pmx_bullet_api.h` | native C API |
| `native/pmx_bullet_api.cpp` | Bullet 2.82 运行时实现 |
| `native/pmx_bullet.dll` | 已编译 native DLL |

## 14. 常见问题

### 14.1 找不到刚体

提示：

```text
Selected model has no mmd_tools rigid bodies
```

处理：

1. 确认模型是通过 mmd_tools 导入的 PMX。
2. 确认 Outliner 中存在 `rigidbodies`。
3. 点击 mmd_tools 的刚体/物理生成按钮后再扫描。
4. 选择模型根对象或模型下任意对象，再点击 `Use Active Model`。

### 14.2 导入 VMD 后一开始模拟就碎掉

处理：

1. 停止模拟。
2. 点击 `Reset`。
3. 确认 `Startup Sync Steps` 大于 0，建议 30。
4. 增加 `Prewarm Steps`。
5. 不要在动作变化特别大的中间帧直接开始模拟，先从较稳定帧开始测试。

### 14.3 尾巴、头发或裙摆抖动

处理顺序：

1. 使用 `Mid` 或 `Hard` 预设。
2. 开启 `Locked Joint Pullback`。
3. 适当提高 `Max Substeps`。
4. 开启 `Name Rules > MMD Compatible`。
5. 检查调试可视化中的碰撞组。

### 14.4 视口很慢

处理顺序：

1. 关闭 `Update Rigid Objects`。
2. 开启 `Skip Unchanged Bones`。
3. 关闭调试可视化。
4. 隐藏高面数网格或切换到 Solid 模式。
5. 降低 `Max Substeps`。
6. 查看 `Performance` 面板确认瓶颈。

### 14.5 Reset 后没有恢复

插件在开始模拟时会捕获初始快照。若需要回到最初导入状态，建议：

1. 停止模拟。
2. 点击 `Reset`。
3. 如果之前已经修改了 Pose，请先回到期望的初始 Pose，再重新点击 `Start`。

## 15. Native Runtime

插件默认加载：

```text
native/pmx_bullet.dll
```

该 DLL 的目标 Bullet 版本：

```text
Bullet 2.82 r2704
```

Python 侧会检查 native API 版本，避免 DLL 和脚本不匹配。

如需重新编译 native runtime，需要准备 CMake、MinGW 和 Bullet 2.82 r2704 开发文件。普通用户不需要重新编译。

## 16. 发布信息

当前版本：

```text
PMX Physics 1.1.0
```

Blender 要求：

```text
Blender 4.2+
```

依赖：

```text
mmd_tools
pmx_bullet.dll
```

许可证：

```text
GPL-3.0-or-later
```
