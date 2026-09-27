# Modeling_and_Interaction_of_Parametric_CCurves
这是一个用来展示各种形式的参数曲线的创建与变换的应用程序
# 参数曲线建模与交互工具（ParametricCurveLab）

一个基于 **Unity 2022.3** 构建的 Windows 桌面程序：在 3D 场景中通过**控制点直接拖拽**来建模各类参数曲线，并支持曲线类型之间的**精确转换**。数学核心与 Unity 场景完全解耦（纯 C#，可独立测试）。

---

## 1. 运行与构建

### 直接运行
双击 `run.bat`，或运行 `ParametricCurveLab\Build\ParametricCurveLab.exe`。

### 从源码重新构建
```bat
build.bat
```
脚本调用 Unity 批处理模式（`-batchmode -executeMethod BuildScript.BuildWindows`）把 `ParametricCurveLab` 工程构建为 `ParametricCurveLab\Build\ParametricCurveLab.exe`（Windows x64）。

**前置条件**：安装 Unity 2022.3.62f3c1 并激活免费 Personal 许可证（Unity Hub → 登录 → 激活）。

### 数学核心单元测试（不依赖 Unity）
```bat
cd MathTest
dotnet run -c Release
```
44 项断言全部通过（`==== ALL TESTS PASSED ====`），覆盖求值/切向/形状保持/节点插入/升阶/退化输入。

---

## 2. 功能总览

### 曲线类型（7 种，可随时新建、共存、切换）
| 类型 | 说明 | 参数域 t |
|---|---|---|
| **Bézier** | 任意阶（次数 2–4），de Casteljau 求值 | [0, 1] |
| **三次 Hermite** | 分段插值，每个型值点带**可拖拽切向** | [0, n−1] |
| **Catmull-Rom** | 均匀参数化插值样条，过所有型值点 | [0, n−1] |
| **准均匀 B 样条** | 端点钳制，曲线过首末控制点 | [0, 1] |
| **开放均匀 B 样条** | 节点等距，不过端点 | [0, 1] |
| **闭合 B 样条** | 周期闭合（C⁰/C¹ 连续） | [0, 1] |
| **NURBS** | 有理 B 样条，控制点带**权重**（w 越大曲线越被拉近） | [0, 1] |

### 曲线转换（8 个按钮，全部作用于“当前曲线”）
| 按钮 | 算法 | 精度 |
|---|---|---|
| Hermite → 分段 Bézier | 每段三次：B₀=P₀, B₁=P₀+T₀/3, B₂=P₁−T₁/3, B₃=P₁ | 精确（形状不变） |
| 分段 Bézier → Hermite | 切向 T = n·(B₁−B₀) | 精确 |
| Catmull-Rom → 分段 Bézier | 每段：B₁=P₀+(P₂−P₀)/6, B₂=P₁−(P₃−P₁)/6 | 精确 |
| Bézier → B 样条 | 重数 p 的钳制节点向量重建 | 精确 |
| B 样条 → Bézier 段 | 节点插入至满重数（Boehm），逐段提取 | 精确 |
| Bézier 升阶 | 升阶公式 Qᵢ | 形状不变 |
| B 样条插入节点 | Boehm 算法，在当前参数 t 处插入 | 形状不变 |
| NURBS → B 样条 | 丢弃权重 | — |

此外“**用当前曲线点构建**”可把任意曲线的控制点（含切向/权重）复制为当前选中的目标类型——这是最直接的跨类型转换入口。

### 交互设计（自设计控制点交互方案）
| 操作 | 行为 |
|---|---|
| 左键点击空白 | 在光标处新增控制点（自动挂到当前曲线） |
| 左键拖拽控制点 | 移动控制点（分段 Bézier 连接点自动联动两侧） |
| 左键拖拽紫色柄 | 编辑 Hermite 型值点切向 |
| 右键点击控制点 | 删除该点（B 样条自动重建节点向量） |
| 右键拖拽 / 中键拖拽 / 滚轮 | 旋转 / 平移 / 缩放视角（轨道相机） |
| t 滑块 | 参数求值：白色标记显示曲线上点与切向 |
| 权重滑块 | 编辑选中 NURBS 控制点的权重 |
| Ctrl+Z / Delete / Tab | 撤销 / 删除选中点 / 循环切换曲线 |

### 信息面板
实时显示：曲线类型、次数、控制点数、节点向量（B 样条/NURBS）、选中点位置/切向/权重、当前 t 处的曲线上点与切向。

---

## 3. 数学原理摘要

### 3.1 de Casteljau（Bézier 求值）
对控制点列 d₀…dₙ 逐层线性插值：
```
dᵢ⁽ʳ⁾ = (1−t)·dᵢ⁽ʳ⁻¹⁾ + t·dᵢ₊₁⁽ʳ⁻¹⁾， r = 1…n， i = 0…n−r
C(t) = d₀⁽ⁿ⁾
```
切线：C′(t) = n·(d₁⁽ⁿ⁻¹⁾ − d₀⁽ⁿ⁻¹⁾)。

**升阶**（次数 n→n+1，形状不变）：
```
Qᵢ = (i/(n+1))·Pᵢ₋₁ + (1 − i/(n+1))·Pᵢ
```

### 3.2 Hermite 与 Catmull-Rom（分段三次）
Hermite 段 [P₀,T₀,P₁,T₁]：
```
C(u) = (2u³−3u²+1)P₀ + (u³−2u²+u)T₀ + (−2u³+3u²)P₁ + (u³−u²)T₁
```
Catmull-Rom 段（过 P₁,P₂，均匀参数化）：
```
C(u) = ½[(2P₁) + (−P₀+P₂)u + (2P₀−5P₁+4P₂−P₃)u² + (−P₀+3P₁−3P₂+P₃)u³]
```

### 3.3 de Boor（B 样条求值）
给定节点向量 U 与次数 p，span k 内：
```
dᵢ⁽ʳ⁾ = (1−αᵢ)dᵢ₋₁⁽ʳ⁻¹⁾ + αᵢ dᵢ⁽ʳ⁻¹⁾,  αᵢ = (t−Uᵢ)/(Uᵢ₊ₚ₋ᵣ₊₁−Uᵢ)
```
用户参数统一映射到内部域 [U_p, U_n]（闭合样条周期延拓控制点）。

### 3.4 NURBS 有理求值
C(t) = Σ Nᵢ(t)·wᵢ·Pᵢ / Σ Nᵢ(t)·wᵢ，基函数按 Piegl–Tiller 递归计算；节点插入在**齐次坐标 (wP, w)** 下混合后投影回点，保证形状不变。

### 3.5 节点插入（Boehm）
在 t 处插入新节点：仅影响 [k−p+1, k] 区间内的控制点
```
Qᵢ = (1−αᵢ)Pᵢ₋₁ + αᵢPᵢ， αᵢ = (t−Uᵢ)/(Uᵢ₊ₚ−Uᵢ)
```
重数 m < p 的内部节点连续插入 p−m 次后，该节点处曲线与 Bézier 段拼接点等价——这正是“B 样条 → 分段 Bézier”的原理。

### 3.6 关键等价关系
- **Bézier ↔ B 样条**：单段 p 次 Bézier 等价于“首尾各 p+1 重节点”的 p 次 B 样条；多段拼接时内部节点重数 = 次数（保持 C⁰）。
- **Bézier 升阶不变量**：升阶前后曲线逐点相等（测试断言 |C₁(t)−C₂(t)| < 1e-9）。
- **NURBS 全 1 权重 ≡ B 样条**（测试断言）。

---

## 4. 工程结构

```
new-chat/
├── build.bat / run.bat          # 一键构建 / 启动
├── ParametricCurveLab/          # Unity 工程
│   ├── Assets/
│   │   ├── Editor/BuildScript.cs          # 批处理构建入口
│   │   ├── Scenes/Main.unity              # 场景（挂 App）
│   │   └── Scripts/
│   │       ├── CurveMath/CurveMath.cs     # 数学核心（纯 C#，~1000 行）
│   │       └── App/                       # 应用层（Unity 依赖隔离在顶层）
│   │           ├── App.cs                 # 入口/动作/撤销/示例
│   │           ├── CurveModel.cs          # 数据模型
│   │           ├── CurveView3D.cs         # 3D 渲染（曲线/控制点/切线/标记）
│   │           ├── CurveInteraction.cs    # 控制点交互
│   │           ├── CurveUI.cs             # 界面逻辑
│   │           ├── UIFactory.cs           # uGUI 程序化构建
│   │           └── SceneBuilder.cs        # 场景/相机/网格构建
│   └── Packages/ ProjectSettings/
└── MathTest/                     # 数学核心单元测试（dotnet run）
```

**分层原则**：`CurveMath.cs` 不引用任何 UnityEngine 类型，可在任意 .NET 环境编译测试；Unity 场景只负责交互与可视化。
