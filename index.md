## 目标在障碍物之间的行走策略: RRT-based Random Walk
问题：一个机器人需要依赖于某种策略，在障碍物分布较为密集且地图未知的场景中，对一个在二维空间内自由移动的目标
实现在 时间/距离 方面较为高效的导航。机器人所能获得的信息除了自身状态数据外，只有激光雷达测距数据，以及导航
目标当前及过去时刻的坐标点。  
而为了让问题场景更接近真实，我们可以把导航目标设想为一个 人/机器人/小动物 等智能体，导航策略则需要驱动我的机器
人对该 人/机器人/小动物 进行拦截。因此，这就需要对导航目标的移动方式有所要求：  
- 首先，目标的移动应该具有较强的随机性，而不是不断以某种模式进行机械地运动；
- 其次，目标的移动应该符合 人/机器人/小动物 等智能体的移动方式，所以随机性应体现在行走的目的上，而非目标在每
  一时刻的行走动作均是随机的；
- 最后，目标的移动路径需要符合运动学规律，应该足够平滑，有一定曲率，而不是僵硬直线段。且在移动的过程中应当能
  自由地避障。  

现在有很多行人数据集，类似ETH/...（补充）。这些数据集常常被用于基于深度学习的测试行人轨迹预测算法的预测精度。
但是这些行人的路径数据是作为监督学习的训练和测试，且都限定于某些特定场景，因此难以移植到 Gazebo 的自定义环境
中。  
（看看上面这一段说得是否正确）  
RRT (Rapidly-exploring Random Tree)作为一种路径规划算法，通过随机采样的方式，能够在指定区域内快速地找
到一条连接起、终点并避开障碍的路径。其算法伪代码如下所示：  
pseudocode  
下面这张 gif 展示了 RRT 算法的寻路过程：  
gif  
（RRT中的 "Tree" 由多个节点组成，每个节点（可看作一个结构体）包含的信息有该点坐标及其父节点，即此节点由哪一
个节点扩展而来）  
RRT 作为路径规划算法存在一个问题，由于采样的随机性，其搜索所得的最终路径也引入了随机性，并不能保证找到一条最优
路径。而作为随机行走算法时，路径的随机性却正是 RRT 相比于 A* 算法等最优路径规划算法的优越性所在。因此，
RRT-based Random Walk 可以以这种方式生成：
1. 设定一个路径的相对长度 path_length。从起点开始扩展随机树，当随机树中有一个节点距起点的相对距离大于等于
path_length 时停止扩展。
2. 以上一条子随机路径的终点为起点，再以相同的方式扩展随机树。
3. 重复上述过程 N 次，得到随机路径。

为了让随机树在各个方向扩展的概率更均匀，我将采样的范围设置为（3 x env_length, 3 x env_width）而不是
（env_length, env_width）。此外还需要对 RRT 随机路径进行平滑，这里采用的是 bezier smoothing 的
方法，得到最终的随机路径。整个过程如下图所示：  
gif  
## 无地图导航策略: Hierarchical RL with Trajectory Predictor
原论文的策略是基于 DDPG（或 Spiking-DDPG），对静止目标实现无地图导航。当然，此策略同样可应用于对运动目
标的导航，但由于该策略在每一时刻总以运动目标当前的坐标值作为导航目标，机器人对运动目标的追踪就总是滞后的。因
此无论在时间和距离上的导航效率方面还是导航成功率方面，该策略都效果欠佳，尤其在运动目标的移动速度等于甚至大于
机器人速度的情况下。因此，可以分析目标过去的移动轨迹，对其未来轨迹进行长距离预测，根据预测的坐标值实现有效、高
效的导航。然而这仍然面临着一些问题，主要体现在 2 个方面：
1. 轨迹预测的精度。当前学术界已经有了不少轨迹预测的算法，【论文xxx】将它们分为 physics-based、
   pattern-based、planning-based 三类。很多的轨迹预测算法对移动随机性较强的目标（如行人等）也已能进行
   高精度的长距离预测【论文1】【论文2】【论文3】。然而这些算法都不可避免地需要依赖移动目标所在的场景信息，这
   是很容易理解的，因为针对移动随机性较强的目标，仅依靠其自身过去的轨迹信息达到高精度长距离预测几乎是不可能达到的。
2. 路径规划的最优性。若我们忽略第一个问题，假设能够得到目标未来轨迹的最优预测，可如何根据预测轨迹，规划机器人的导航路线也是一个
   问题。由于机器人只能通过激光雷达获取环境的局部信息，而没有全局地图，环境对于机器人来说是动态变化的，机器人
   便无法规划出最优的拦截路径。  

针对上述问题，我们提出分层强化学习的导航策略，能够在地图未知、仅获取目标过去移动轨迹的条件下，以近似最优的拦截路
径对目标实现高效导航。同时，通过鲁棒的导航策略和迭代式预测来弥补长距离轨迹预测的精度缺失问题。总的来说，导航策
略采用分层强化学习的机制，上层的 meta-controller 首先分析目标的过去的坐标值，预测得到其未来轨迹，然后从预测
轨迹中选出导航目标点；下层的 controller 则给出机器人此刻应采取的速度，向该导航目标点靠近，同时避开障碍物。整
个过程不断迭代进行，直到机器人成功拦截移动目标。
### Iterative Kalman Filter Trajectory Predictor
#### Brief Intro to Kalman Filter
Kalman Filter 本身是一种解决数据融合问题的算法，用于融合预测数据和观测数据。对于某一系统的状态数据
$X_{0},X_{1},...,X_{n}$，我们能根据系统的物理特性，建立预测模型，得到在 $k$ 时刻的系统状态的预测值：
$$X_{k}^{-}=AX_{k-1}+B\mu_{k-1}+\omega_{k-1},  P(\omega)\sim N(0, Q) $$
预测模型必然存在误差，用服从高斯分布的 $\omega_{k-1}$ 表示，Q 为协方差矩阵。  
除 $X_{k}$ 的预测值外，我们还可通过传感器在 $k$ 时刻对真实值进行测量，得到测量值 $Z_{k}$，$Z_{k}$ 经
过观测模型即可得到对系统状态 $X_{k}$ 的观测值：
$$Z_{k}=HX_{k}+\nu_{k},  P(\nu)\sim N(0, R)$$
观测模型也必然存在误差，因此也用服从高斯分布的 $\nu$ 表示，R 为协方差矩阵。  
有了 $X_{k}$ 的预测值和测量值，我们就可以对 $X_{k}$ 的真实值给出最优估计，使得最终的估计值比预测值误差和
测量值误差都小，且在理论上达到最小，Kalman Filter 的融合公式如下：  
$$\hat X_{k}=\hat X_{k}^{-}+K_{k}(Z_{k}-H\hat X_{k}^{-})$$
由这个公式，更形式化地讲 Kalman Filter 的目标，就是找到一个 $K_{k}$ 的值，使得 $e_{k}=X_{k}-\hat X_{k}$ 的
方差最小（协方差矩阵的迹最小）。
$$P(e_{k})\sim N(0, P_{k})$$
$$P_{k}=E(e_{k}e_{k}^{T})$$
因为预测模型和观测模型都是线性模型，这个优化问题可以简单粗暴地通过求导解决，得到：
$$K_{k}=(P_{k}^{-}H^{T})/(HP_{k}^{-}H^{T}+R)$$
因此，Kalman Filter 在每一步的完整过程就是两个部分，五个公式：  
图  
#### Kalman Filter for Trajectory Prediction  
状态变量：
$$X_{k}=[x, v_{x}, y, v_{y}, a_{x}, a_{y}]^{T}$$
预测模型——恒加速度模型：
$$X_{k+1}=$$

$${\left\lbrack /matrix{2 & 3 \cr 4 & 5} \right\rbrack} 
\begin{bmatrix}
 1 & dt & 0 & 0 & 0.5dt^{2} & 0 \\\\
 0 & 1 & 0 & 0 & dt & 0 \\\\
 0 & 0 & 1 & dt & 0 & 0.5dt^{2} \\\\
 0 & 0 & 0 & 1 & 0 & dt \\\\
 0 & 0 & 0 & 0 & 1 & 0 \\\\
 0 & 0 & 0 & 0 & 0 & 1 
\end{bmatrix}
$$]

预测模型误差协方差：
$$Q=\left[
\begin{matrix}
 0.1 & 0 & 0 & 0 & 0 & 0 \\\\
 0 & 0.1 & 0 & 0 & 0 & 0 \\\\
 0 & 0 & 0.1 & 0 & 0 & 0 \\\\
 0 & 0 & 0 & 0.1 & 0 & 0 \\\\
 0 & 0 & 0 & 0 & 0.1 & 0 \\\\
 0 & 0 & 0 & 0 & 0 & 0.1 \\\\
\end{matrix}
\right]$$
观测矩阵：
$$H=\left[
\begin{matrix}
 1 & 0 & 0 & 0 & 0 & 0 \\\\
 0 & 0 & 1 & 0 & 0 & 0 \\\\
\end{matrix}
\right]$$
观测模型误差协方差：
$$R=\left[
\begin{matrix}
 0.0001 & 0 \\\\
 0 & 0.0001 \\\\
\end{matrix}
\right]$$
轨迹预测观测步长：$ /tau=30 $
轨迹预测长度：$ L=150 $
gif    
### Hierarchical Framework
#### Meta-controller
- 状态:
$$s=[G_{dir}, G_{dis}, v, \omega, L]$$
- 动作：
$$a=p_{goal}$$
- 奖励：
$$R=\begin{cases}
  R_{goal}, & g_{dis}<G_{th} \\\\
  R_{obs}, & O_{dis}<O_{th} \\\\
  R_{step}, & text{otherwise}
\end{cases}
$$

#### Controller
- 状态:
$$s=[g_{dir}, g_{dis}, v, \omega, L]$$
- 动作：
$$a=[v_{left}, v_{right}]$$
- 奖励：
$$R=\begin{cases}
  R_{goal}, & g_{dis}<G_{th} \\\\
  R_{obs}, & O_{dis}<O_{th} \\\\
  A*[g_dis(t-1)-g_dis(t)], & text{otherwise}
\end{cases}
$$
  
#### 训练方式
- 阶段一：在 env(1,2,3,4) 以定点训练下层的 controller，上层不训练
- 阶段二：固定下层 controller 的策略，在 env5 用移动目标训练上层 meta-controller

图片（训练环境）  
#### 实验效果
九宫格图片（DDPG, greedy meta-controller, trained meta-controller）
三者的导航成功率，路径长度，时间，均速
#### 下一步的思考
1. 上下层的协同训练不可行，但是否需要对比协同训练的分层 RL 算法（HIRO, HAC）
2. 是否改进上层网络对预测轨迹的特征提取能力，而不是直接展平扔给神经网络
3. 是否再改进预测算法（如 UKF 等）
4. 应从哪个方向改进算法
5. 看看类似任务的工作————**明早**

### Jekyll Themes

Your Pages site will use the layout and styles from the Jekyll theme you have selected in your [repository settings](https://github.com/Toyoid/toyoid.github.io/settings/pages). The name of this theme is saved in the Jekyll `_config.yml` configuration file.

### Support or Contact

Having trouble with Pages? Check out our [documentation](https://docs.github.com/categories/github-pages-basics/) or [contact support](https://support.github.com/contact) and we’ll help you sort it out.
