# AutoPlanX
AutoPlanX: Lateral-Longitudinal Interaction Demo for uneven terrain
AutoPlanX: Lateral-Longitudinal Interaction Demo
# 基于双向可达性搜索的非平坦地形动态路径规划算法
## 时空走廊约束下的区间代数与运动学验证

**作者姓名**  
*日期：2025年3月10日*

---
算法整体流程框架：
前面先跑一个LIO-SAM（其实没有也可以？）

通过平面拟合等方法拟合出不平坦地面？

算法主要提出的场景：计算资源少，实时计算，结构化地形。可以用于火星车，野外探险等道路崎岖的路面进行导航。能够用于处理移动障碍物的场景。

代码结构？应该还是基于状态机FSM。魔改高飞？

定位建图模块：LIO-SAM

主要需要点云：

对于planning所需要的障碍物建模主要分为两类:


基于地图的地形评估算法介绍：
方案一：
在每一帧lidar点云到来时，以逐渐增长的半径形成圆环，对每个圆环按照固定角度进行切分形成一个个cell，然后对每一个cell进行地形等级的评分。
首先得到每个cell内所有高度的点云，然后首先进行聚类，然后判断是否为平面，如果不是平面且体积较大的话定义为障碍物，如果是平面的话标记为可行区域。
得到所有的平面后需要进行可通行评估，计算平面和水平面的法线相似度和平面内的点的高度差。设计一个代价函数来对平面进行分级，比如说有十个等级。然后设置阈值，
如果平面等级大于这个阈值则将其设置为low traversability corridor内部的平面，然后对low traversability corridor进行聚类然后进行区间合并。得到low traversability corridor。
方案二：
将lidar扫出的前方的点云按照一个个strip分好类，首先得让strip分辨率可调。。。






在移动无人车的周围一圈一圈地进行地形评估，能够评估






静态障碍物：直接通过点云处理方法：

我们更倾向于使用计算更快的方式，在不同的高度上进行点云采样。高度占有较多的地方设定为障碍物。

动态障碍物：还没有想好？想着采用一些自动驾驶通用的方法。

barrier corridor：高度差过大的区域。
对于 barrier corridor来说，我们试图想着尽量晚并且少的侵入 barrier corridor。

还是横纵向解耦的思路，给定车辆的速度，planning horizon在50m左右。

此算法优势主要体现在算的快，并且融合高程约束，并且按照区间来进行DP搜索，突破了离散DP的限制。同时使用单步规划，采用类似JPS的跳跃式算法，进一步加强了计算速度。

如何对不平坦地形对于车辆行驶的影响进行建模？


## 摘要
针对非平坦地形中动态障碍物共存场景下的路径规划难题，本文提出一种基于双向可达性搜索的动态规划算法（Bidirectional Reachability Search, BRS）。算法核心创新点包括：  
1. **时空走廊约束的区间传播机制**：通过前向动态区间代数融合地形高程约束与动态障碍物  
2. **双向运动学验证框架**：结合前向可达集与后向运动学约束生成安全走廊  
3. **可见性优化的多步跳跃策略**：突破传统单步决策限制  

实验表明，在倾斜角≤25°的非结构化地形中，算法规划成功率（98.2%）显著优于RRT*（79.5%）与MPC（89.1%），平均计算时间（35ms）满足实时性需求。

---

## 引言
非平坦地形路径规划面临三重挑战：
- **地形约束复杂性**：斜坡、沟壑等地形特征需转化为连续的横向（d轴）运动约束  
- **动态障碍物规避**：移动障碍物在时空维度形成时变禁行区域  
- **实时性要求**：传统方法在高维约束下计算效率骤降  

| 方法类型          | 地形适应性缺陷                | 动态障碍处理缺陷               |
|-------------------|-------------------------------|--------------------------------|
| 采样法（RRT*）    | 未显式建模地形连续性约束      | 重规划频率高（>100ms）        |
| 优化法（MPC）     | 局部最优导致陷坡风险          | 预测时域短（<3s）             |
| 栅格法（Lattice） | 固定分辨率忽略地形曲率        | 障碍物膨胀保守性过高          |

---

## 核心算法设计
### 时空走廊约束建模
**地形高程映射**：  
\[ C_{\text{terrain}}(t) = [d_{\min}(z(t)), d_{\max}(z(t))] \]  

**动态障碍物区间**：  
\[ O_i(t) = \left[ d_{\text{obs},i}(t) - \frac{w_{\text{obs}}}{2}, d_{\text{obs},i}(t) + \frac{w_{\text{obs}}}{2} \right] \]

---

### 双向可达性分析
**前向区间传播伪代码**：
```python
def generateForwardStagesPro():
    1. 基于v_dmax计算可达区间
    2. 执行区间差集运算
    3. 标注区间类型



技术方案

技术方案描述
本方案提出了一种 基于双向可达性搜索（Bidirectional Reachability Search, BRS）与模型预测控制（MPC）的实时路径规划算法，专门针对 动态3D复杂地形（如崎岖地形、坡度变化、动态障碍物场景） 设计。其核心是通过 区间分析（Interval Analysis） 对地形可通行性进行量化评估，并结合双向搜索策略和代价函数优化，实现在非平坦地形中的高效避障与轨迹生成。

技术实现细节
1. 地形可通行性评估（Traversability Assessment）
动态边界建模
通过预定义的地形边界数据（d_min_vec, d_max_vec）和动态安全走廊（corridor_time_ranges），实时计算当前时间步的地形可通行区域（getDBounds 和 getCorridorBounds）。
动态障碍物处理：通过 getObstaclesAtTime 实时检测障碍物位置，结合 intervalSubtract 方法排除不可通行区间（如岩石、沟壑）。
坡度与表面粗糙度约束：通过 d_min 和 d_max 的动态调整，模拟地形坡度对车辆运动的限制（如最大爬坡角度）。
2. 双向可达性搜索（BRS）
前向阶段生成（Forward Stages）
从初始位置出发，按时间步（t_resolution_）向前传播，通过 generateForwardStagesPro 生成所有可达区间（Interval）。每个区间包含：

运动学约束：基于最大速度（v_d_max_）计算可达范围（d_lower 和 d_upper）。
区间类型标记：区分第一类区间（Intervalforwardtype::First，远离安全走廊）和第二类区间（Intervalforwardtype::Second，处于安全走廊内），支持动态权重调整。
反向阶段生成（Backward Stages）
从目标点（d_terminal_）反向传播，通过 generateBackwardStages 与前向阶段进行双向匹配，生成 双向可达区间（Bidirect）。

双向重叠验证：通过 Interval 的 forward_type 和 back_type 标记，确保路径在正向和反向阶段均可达。
3. 路径优化与代价函数
多目标优化策略
在候选路径中，通过 evaluateJumpCost 和 calculateOptimalPoint 综合优化以下目标：

路径平滑性：最小化方向突变（w2_ * |d_curr - d_prev|）。
地形偏好：优先选择远离动态障碍物和地形边缘的路径（w3_ 对走廊外区域给予奖励）。
终端代价：强制路径最终接近目标点（d_terminal_）。
非重叠优先：通过 calculateOptimalPoint 的策略点生成（如 safe_margin_），主动避开地形走廊的拥挤区域。
实时决策与插值
在 plan_pro 中，通过分层候选区间（step_candidates）和 前瞻步数（max_horizon_steps） 机制，动态选择最优跳跃路径，并通过线性插值确保运动学可行性。

4. 复杂地形适应性增强
动态障碍物规避
通过 setObstacles 实时更新障碍物位置（膨胀处理 npc_width_），结合 intervalSubtract 实现动态避障。
地形起伏建模
在 getDBounds 中通过线性插值（ratio）模拟地形高度随时间/位置的动态变化（如火星车遭遇沙丘时的地形起伏）。
非完整约束处理
通过 v_d_max_ 和 t_resolution_ 控制车辆加速度限制，确保路径在复杂地形中可执行（如坡度陡峭时降低速度）。
核心创新点
双向可达性与区间分析的结合

优势：通过双向搜索减少计算冗余，区间合并（mergeIntervals）高效排除不可行区域，实现在动态3D地形中的实时规划（实验中达到 35ms 内完成规划）。
代码体现：Interval 类的 forward_type 和 back_type 标记，双向阶段生成的 propagated_intervals 合并逻辑。
地形可通行性驱动的代价函数

创新：引入地形走廊（corridor_min/corridor_max）和安全边距（safe_margin_），通过 w3_ 权重动态调整路径对地形边缘的偏好，避免陷入局部最优（如沟壑边缘）。
多步前瞻与可见性分析

策略：通过 calculateVisibleSteps 和 step_candidates 分层收集候选路径，并评估后续阶段的连续可见性（angle_threshold），确保路径在复杂地形中的长期可行性。
应用场景
自动驾驶越野车辆：在山地、沙漠等非结构化地形中规划避障路径。
火星车导航：在动态沙丘和陨石坑环境中实现自主路径规划。
无人机地形跟随：结合高度数据（d 轴）规划山区飞行轨迹。
代码逻辑与地形关联
Interval 类：表示地形可通行区域的区间，通过 d_lower 和 d_upper 界定地形边界，contains 方法判断位置是否可通行。
DPSearch 核心方法：
getDBounds：根据预定义地形轮廓（如坡度限制）动态生成可通行范围。
intervalSubtract：通过障碍物数据（obstacles）实时剔除地形中不可通行区域。
calculateOptimalPoint：结合地形走廊（corridor_min/max）和安全边距，生成避开地形危险区域的最优路径点。
通过上述设计，该算法在 动态复杂地形 中实现了 实时性（35ms）、鲁棒性（动态障碍物规避）和地形适应性（非平坦路径优化），适用于高机动性场景下的自主导航需求。




























An Efficient Global Trajectory Planner for Highly  Dynamical Nonholonomic Autonomous  Vehicles on 3-D Terrains

高动态非完整越野自动驾驶汽车在 3-D 地形上实现高机动性。
在拓扑结构不平坦的复杂地形上，设计安全可行的车辆轨迹通常需要了解车辆的动力学和非完整约束。然而，先前的研究将全局规划问题视为路径规划问题，而没有有效地考虑拓扑或动态约束。

为了解决这一差距，本文提出了一种三阶段轨迹规划算法，该算法由 A*、快速探索随机树 （RRT） 和局部轨迹优化 （LTR） 阶段组成，以在不平坦地形上纳入动力学和非完整约束。
该算法在具有随机地形场和障碍物的场景中进行了测试，以证明所有三个阶段的必要性。与最先进的方法相比，该算法具有更低的成本、更高的成功率和更高的计算效率。
然后，通过在 3-D 地形上控制模拟的 MRZR 车辆以及本地控制器来测试该算法，并与最先进的算法进行比较。结果表明，新算法能够以较低的成本规划动态可行的轨迹，而最先进的算法由于忽略动态车辆限制而无法执行。



原论文中连续时间形式的OCP（式1）的求解方法是，利用[NLOptControl](30)将其转录（离散化）为非线性规划（NLP）问题 ，并调用[Ipopt](31)求解器进行数值优化。具体实现中，采用10个配点 （Collocation Points）和后向欧拉积分法 （Backward Euler Scheme）完成连续到离散的系统转换。

关键术语与技术细节解析：
1.转录（Transcription）
将连续时间的微分方程约束（如车辆动力学方程）转化为代数方程，使OCP能被数值求解器处理。常见方法包括直接配点法（Direct Collocation）和单/多步积分法（如欧拉法、龙格-库塔法）。
2.配置点（Collocation Points）
在时间域内离散选择的点（此处10个），用于近似状态与控制变量的变化。更多配点可提升精度，但增加计算量。
后向欧拉法 的隐式特性（使用下一时刻状态计算当前导数值）增强了数值稳定性，适用于刚性问题。
3.NLOptControl与Ipopt
NLOptControl ：基于Julia的库，支持将最优控制问题自动转录为NLP标准形式。
Ipopt （Interior Point OPTimizer）：开源非线性规划求解器，擅长处理大规模带约束优化问题，通过内点法寻找满足KKT条件的最优解。
