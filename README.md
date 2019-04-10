# VINS-Mono-Learning

 VINS-Mono代码解读

Forked from VINS-Mono: https://github.com/HKUST-Aerial-Robotics/VINS-Mono

VINS-Mono A Robust and Versatile Monocular Visual-Inertial State Estimator

详细内容可参考微博
[VINS-Mono论文学习与代码解读——目录与参考](https://blog.csdn.net/qq_41839222/article/details/85793998)


---

# 目录

 [VINS-Mono论文翻译](https://blog.csdn.net/qq_41839222/article/details/85683373)

[VINS-Mono代码解读——启动文件launch与参数配置文件yaml介绍](https://blog.csdn.net/qq_41839222/article/details/86564879)


 [VINS-Mono代码解读——各种数据结构 sensor_msgs measurements](https://blog.csdn.net/qq_41839222/article/details/86030962)

 [VINS-Mono代码解读——视觉跟踪 feature_trackers](https://blog.csdn.net/qq_41839222/article/details/85797156)

 VINS-Mono理论学习——IMU预积分 Pre-integration

 [VINS-Mono代码解读——状态估计器流程 estimator](https://blog.csdn.net/qq_41839222/article/details/86293038)

 [VINS-Mono代码解读——视觉惯性联合初始化 initialStructure() sfm.construct() visualInitialAlign()](https://blog.csdn.net/qq_41839222/article/details/88942414)

 VINS-Mono理论学习——视觉惯性联合初始化与外参估计

 VINS-Mono代码解读——基于滑动窗口的非线性优化 solveOdometry() optimization() slideWindow()
 
 [VINS-Mono代码解读——回环检测与重定位 pose graph  loop closing](https://blog.csdn.net/qq_41839222/article/details/87878550)
 
 VINS-Mono代码解读——四自由度位姿图优化 pose_graph  optimize4DoF
 
 [TUM VIO数据集介绍与尝试](https://blog.csdn.net/qq_41839222/article/details/86180964)
 
  [Realsense D435i如何拿到IMU数据并顺利运行VINS-Mono](https://blog.csdn.net/qq_41839222/article/details/86552367)

---


## VINS介绍：
VINS是一种具有鲁棒性和通用性的单目视觉惯性状态估计器。
该算法主要有以下几个模块：
 1. 预处理
图像特征光流跟踪
IMU数据预积分
 2. 初始化
纯视觉Sfm
Sfm与IMU积分的松耦合
 3. 基于滑动窗口的非线性优化实现紧耦合
 4. 回环检测与重定位
 5. 四自由度位姿图优化

![在这里插入图片描述](https://img-blog.csdnimg.cn/20190104194533165.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzQxODM5MjIy,size_16,color_FFFFFF,t_70)

----

## rqt_graph
node only
![在这里插入图片描述](https://img-blog.csdnimg.cn/20190108100859579.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzQxODM5MjIy,size_16,color_FFFFFF,t_70)


node all![在这里插入图片描述](https://img-blog.csdnimg.cn/20190108102632561.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzQxODM5MjIy,size_16,color_FFFFFF,t_70)

---

## 代码的文件目录
1、ar_demo：一个ar应用demo
2、benchmark_publisher：接收并发布数据集的基准值
3、camera_model
&emsp; &emsp;calib：相机参数标定
&emsp; &emsp;camera_models：各种相机模型类
&emsp; &emsp;chessboard：检测棋盘格
&emsp; &emsp;gpl
&emsp; &emsp;sparse_graph
&emsp; &emsp;intrinsic_calib.cc：相机标定模块main函数
4、config：系统配置文件存放处
5、feature_trackers：
&emsp; &emsp;feature_tracker_node.cpp	ROS 节点函数，回调函数
&emsp; &emsp;feature_tracker.cpp	图像特征光流跟踪
6、pose_graph：
&emsp; &emsp;keyframe.cpp	关键帧选取、描述子计算与匹配
&emsp; &emsp;pose_graph.cpp 位姿图的建立与图优化
&emsp; &emsp;pose_graph_node.cpp	ROS 节点函数，回调函数，主线程
7、support_files：帮助文档、Bow字典、Brief模板文件
8、vins_estimator
&emsp; &emsp;factor：实现IMU、camera等残差模型
&emsp; &emsp;initial：系统初始化，外参标定，SFM
&emsp; &emsp;utility：相机可视化，四元数等数据转换
&emsp;&emsp; estimator.cpp：紧耦合的VIO状态估计器实现
&emsp;&emsp; estimator_node.cpp：ROS 节点函数，回调函数，主线程
&emsp; &emsp;feature_manager.cpp：特征点管理，三角化，关键帧等
&emsp; &emsp;parameters.cpp：读取参数

--------------------- 

参考笔记：

VINS论文推导及代码解析 by 崔华坤

[VINS技术路线与代码详解](https://blog.csdn.net/wangshuailpp/article/details/78461171) by 五行缺帅wangshuailpp

[VINS-Mono 学习笔记](https://zhuanlan.zhihu.com/p/36161028) by 童哲航

[VINS-Mono 代码解读](https://blog.csdn.net/u012871872/article/details/78128087?locationNum=8&fps=1) by  Rain-XIA

[VINS-mono详细解读](https://www.cnblogs.com/ilekoaiq/p/8836970.html)  by 极品巧克力

VINS代码注释：

https://github.com/castiel520/VINS-Mono	by [castiel520](https://github.com/castiel520)

https://github.com/QingSimon/VINS-Mono-code-annotation	by [QingSimon](https://github.com/QingSimon)

