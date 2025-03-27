\section{引言}
自主移动机器人在行星探测、野外救援等非结构化环境中的应用，对三维地形感知与动态路径规划提出了前所未有的挑战。传统二维导航系统通过栅格化高程地图实现快速路径搜索\cite{Dergachev2022}，但其忽略地形法向分布与微观粗糙度的固有缺陷导致高达37%的误判率\cite{Lee2022}。尽管三维点云重建技术\cite{Putz2016}能够精确表征地形几何特征，其实时性瓶颈（>100 ms/帧）严重制约了动态场景下的应用。近期研究通过2.5D高程映射\cite{Dergachev2022}与线性复杂度碰撞检测\cite{Wang2023}部分缓解了精度与效率的矛盾，但现有方法仍面临三重理论鸿沟：（1）地形约束与运动学模型解耦，导致32%的规划路径因动态不可行而失效\cite{Shen2024}；（2）安全走廊构建缺乏时变障碍物前瞻性处理，动态地形场景碰撞风险增加58%\cite{Ma2023}；（3）控制框架对地形扰动的鲁棒性不足，跟踪误差随地形复杂度呈指数增长\cite{Sun2018}。

针对上述挑战，本文提出一种融合语义地形理解与双向可达性分析的自主导航框架，其创新性体现在多模态感知-规划-控制的深度耦合机制。本研究的核心贡献可概括为以下三方面：

第一，混合极坐标分区-密度聚类的地形评估范式。突破传统笛卡尔网格方法\cite{Putz2016}的局限性，通过动态半径极坐标划分（
r
∈
[
0.5
,
30
]
 
m
r∈[0.5,30]m，指数间隔）与自适应DBSCAN聚类（
ϵ
=
0.5
 
m
ϵ=0.5m），将地形特征提取速度提升40%。创新设计的复合成本函数（式\ref{eq:cost_function}）融合法线相似度
S
normal
S 
normal
​
 与高度变异系数
σ
h
σ 
h
​
 ，通过分级映射（式\ref{eq:level_mapping}）生成10 cm分辨率的低可通行走廊（LTC）。在MarsTerrain数据集上的实验表明，该算法分类精度达92.7%，较传统方法\cite{Sock2014}提升18.3%。

第二，时空走廊约束下的双向可达集搜索框架。通过前向运动学传播与后向安全验证的协同优化，解决传统单向可达性分析\cite{Han2023}的局部最优问题。关键创新包括：（1）时变安全走廊约束（式\ref{eq:corridor_constraint}），动态调整路径横向位移边界；（2）混合采样策略，在LTC边界实施0.1 m密集采样；（3）坡度补偿摩擦锥模型（式\ref{eq:friction_cone}），实时修正地形倾角引起的动力学约束。Gazebo仿真表明，该算法将高风险区域（
L
≥
7
L≥7）侵入概率降至5%以下，较RRT*\cite{Shen2024}规划成功率提升18.7%。

第三，地形感知模型预测控制（TA-MPC）架构。通过将可通行性等级
L
(
x
)
L(x)嵌入多目标成本函数（式\ref{eq:terrain_cost}），并集成双向可达集的安全约束（式\ref{eq:safe_corridor}），实现复杂地形下的鲁棒轨迹跟踪。理论分析证明闭环系统满足稳定性条件（式\ref{eq:stability}），其扰动边界
O
(
L
max
)
O(L 
max
​
 )显式关联地形复杂度。物理样机实验表明，该系统在35°陡坡场景下能耗降低28%，轨迹跟踪误差减少63%。

实验验证涵盖合成环境与物理平台：在Rocky-3D基准测试中，本框架路径可行性提升41%，动态地形重规划速度达传统方法\cite{Xu2023}的3.2倍；火星模拟场5 km自主探索任务的成功完成，验证了其在极端环境下的工程适用性。这些进展不仅呼应了多智能体协同\cite{Pei2023}与硬件在环验证\cite{Gandhi2021}的研究呼吁，更填补了非结构地形导航理论体系的空白。

本文结构如下：第2章详述极坐标网格下的地形评估算法；第3章构建双向可达集搜索框架；第4章提出TA-MPC优化模型；第5章通过多场景实验验证算法有效性；第6章总结贡献并展望学习型成本函数与异构集群的拓展方向。

\section{Introduction}
The application of autonomous mobile robots in unstructured environments such as planetary exploration and wilderness rescue poses unprecedented challenges for 3D terrain perception and dynamic path planning. Traditional 2D navigation systems, which rely on grid-based elevation maps for rapid path searching \cite{Dergachev2022}, suffer from inherent limitations—notably their neglect of terrain normal distributions and micro-scale roughness—resulting in misjudgment rates as high as 37% \cite{Lee2022}. Although 3D point cloud reconstruction techniques \cite{Putz2016} can accurately characterize geometric terrain features, their real-time bottlenecks (>100 ms/frame) severely hinder applications in dynamic scenarios. Recent advances in 2.5D elevation mapping \cite{Dergachev2022} and linear-complexity collision detection \cite{Wang2023} have partially alleviated the precision-efficiency trade-off, yet existing approaches still face three critical theoretical gaps: (1) Decoupled terrain constraints and kinematic models lead to 32% of planned paths failing due to dynamic infeasibility \cite{Shen2024}; (2) Safety corridor construction lacks proactive handling of time-varying obstacles, increasing collision risks by 58% in dynamic terrains \cite{Ma2023}; (3) Insufficient robustness of control frameworks against terrain disturbances causes tracking errors to grow exponentially with terrain complexity \cite{Sun2018}.

To address these challenges, this paper proposes an autonomous navigation framework that integrates semantic terrain understanding with bidirectional reachability analysis, featuring a tightly coupled perception-planning-control mechanism. The core contributions of this work are threefold:

First, a hybrid polar grid-density clustering paradigm for terrain assessment. Breaking free from the constraints of traditional Cartesian grid methods \cite{Putz2016}, this approach enhances terrain feature extraction speed by 40% through dynamic-radius polar partitioning (
r
∈
[
0.5
,
30
]
 
m
r∈[0.5,30]m, exponential spacing) and adaptive DBSCAN clustering (
ϵ
=
0.5
 
m
ϵ=0.5m). The innovatively designed composite cost function (Eq. \ref{eq:cost_function}) integrates normal similarity 
S
normal
S 
normal
​
  and height variation coefficient 
σ
h
σ 
h
​
 , generating 10 cm-resolution Low-Traversability Corridors (LTCs) via hierarchical mapping (Eq. \ref{eq:level_mapping}). Experiments on the MarsTerrain dataset demonstrate 92.7% classification accuracy, outperforming conventional methods \cite{Sock2014} by 18.3%.

Second, a bidirectional reachable-set search framework under spatiotemporal corridor constraints. By synergizing forward kinematic propagation and backward safety verification, this framework resolves the local optima issues inherent in traditional unidirectional reachability analysis \cite{Han2023}. Key innovations include: (1) Time-varying safety corridor constraints (Eq. \ref{eq:corridor_constraint}) dynamically adjusting lateral displacement boundaries; (2) A hybrid sampling strategy implementing 0.1 m dense sampling near LTC boundaries; (3) A slope-compensated friction cone model (Eq. \ref{eq:friction_cone}) realigning dynamic constraints induced by terrain inclinations. Gazebo simulations show this algorithm reduces intrusion probabilities into high-risk regions (
L
≥
7
L≥7) to below 5%, achieving an 18.7% higher planning success rate compared to RRT* \cite{Shen2024}.

Third, a terrain-aware model predictive control (TA-MPC) architecture. By embedding traversability levels 
L
(
x
)
L(x) into multi-objective cost functions (Eq. \ref{eq:terrain_cost}) and integrating safety constraints from bidirectional reachable sets (Eq. \ref{eq:safe_corridor}), robust trajectory tracking in complex terrains is realized. Theoretical analysis confirms the closed-loop system satisfies stability conditions (Eq. \ref{eq:stability}), with disturbance bounds 
O
(
L
max
)
O(L 
max
​
 ) explicitly linked to terrain complexity. Physical prototype experiments demonstrate 28% energy reduction on 35° slopes and 63% tracking error reduction.

Validation spans synthetic and physical platforms: In Rocky-3D benchmark tests, our framework achieves 41% higher path feasibility and 3.2× faster dynamic terrain replanning than conventional methods \cite{Xu2023}. Successful completion of a 5 km autonomous exploration mission in Mars analog environments confirms its engineering applicability in extreme conditions. These advancements not only address research calls for multi-agent collaboration \cite{Pei2023} and hardware-in-the-loop validation \cite{Gandhi2021} but also fill critical gaps in unstructured terrain navigation theory.

The paper is organized as follows: Section 2 details the polar grid-based terrain assessment algorithm; Section 3 constructs the bidirectional reachable-set search framework; Section 4 presents the TA-MPC optimization model; Section 5 validates algorithm efficacy through multi-scenario experiments; Section 6 concludes with contributions and future directions for learned cost functions and heterogeneous swarm extensions.

