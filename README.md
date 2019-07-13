# VINS-Mono-Learning

 VINS-Mono代码注释，仅供学习 

详细内容可参考微博
[VINS-Mono论文学习与代码解读——目录与参考](https://blog.csdn.net/qq_41839222/article/details/85793998)

Forked from VINS-Mono: https://github.com/HKUST-Aerial-Robotics/VINS-Mono  
29 Dec 2017: New features: Add map merge, pose graph reuse, online temporal calibration function, and support rolling shutter camera. 

---

# 目录

 [VINS-Mono论文翻译](https://blog.csdn.net/qq_41839222/article/details/85683373)

[VINS-Mono代码解读——启动文件launch与参数配置文件yaml介绍](https://blog.csdn.net/qq_41839222/article/details/86564879)


 [VINS-Mono代码解读——各种数据结构 sensor_msgs measurements](https://blog.csdn.net/qq_41839222/article/details/86030962)

 [VINS-Mono代码解读——视觉跟踪](https://blog.csdn.net/qq_41839222/article/details/85797156)

 [VINS-Mono理论学习——IMU预积分](https://blog.csdn.net/qq_41839222/article/details/86290941)

 [VINS-Mono代码解读——状态估计器流程](https://blog.csdn.net/qq_41839222/article/details/86293038)

 [VINS-Mono代码解读——视觉惯性联合初始化流程](https://blog.csdn.net/qq_41839222/article/details/88942414)

 [VINS-Mono理论学习——视觉惯性对齐与外参标定](https://blog.csdn.net/qq_41839222/article/details/89106128)

 [VINS-Mono理论学习——后端非线性优化](https://blog.csdn.net/qq_41839222/article/details/93593844)

 VINS-Mono理论学习——边缘化

 VINS-Mono代码解读——滑动窗口的非线性优化流程
 
 [VINS-Mono代码解读——回环检测与重定位](https://blog.csdn.net/qq_41839222/article/details/87878550)
 
 VINS-Mono代码解读——四自由度位姿图优化
 
 [TUM VIO数据集介绍与尝试](https://blog.csdn.net/qq_41839222/article/details/86180964)
 
  [Realsense D435i如何拿到IMU数据并顺利运行VINS-Mono](https://blog.csdn.net/qq_41839222/article/details/86552367)

---


## VINS介绍：

VINS是一种具有鲁棒性和通用性的单目视觉惯性状态估计器。  
该算法主要有以下几个模块：  
 1. 预处理  
&emsp; &emsp;1）图像特征光流跟踪  
&emsp; &emsp;2）IMU数据预积分  
 2. 初始化  
&emsp; &emsp;1）纯视觉Sfm  
&emsp; &emsp;2）Sfm与IMU预积分的松耦合  
 3. 基于滑动窗口的非线性优化  
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
2、benchmark\_publisher：接收并发布数据集的基准值  
3、camera\_model  
&emsp; &emsp;calib：相机参数标定  
&emsp; &emsp;camera\_models：各种相机模型类  
&emsp; &emsp;chessboard：检测棋盘格  
&emsp; &emsp;gpl  
&emsp; &emsp;sparse\_graph  
&emsp; &emsp;intrinsic\_calib.cc：相机标定模块main函数  
4、config：系统配置文件存放处  
5、feature\_trackers：  
&emsp; &emsp;feature\_tracker\_node.cpp	ROS 节点函数，回调函数  
&emsp; &emsp;feature\_tracker.cpp	图像特征光流跟踪
6、pose\_graph：  
&emsp; &emsp;keyframe.cpp	关键帧选取、描述子计算与匹配   
&emsp; &emsp;pose\_graph.cpp 位姿图的建立与图优化  
&emsp; &emsp;pose\_graph\_node.cpp	ROS 节点函数，回调函数，主线程  
7、support\_files：帮助文档、Bow字典、Brief模板文件  
8、vins\_estimator   
&emsp; &emsp;factor：实现IMU、camera等残差模型  
&emsp; &emsp;initial：系统初始化，外参标定，SFM  
&emsp; &emsp;utility：相机可视化，四元数等数据转换  
&emsp;&emsp; estimator.cpp：紧耦合的VIO状态估计器实现  
&emsp;&emsp; estimator\_node.cpp：ROS 节点函数，回调函数， 主线程  
&emsp; &emsp;feature\_manager.cpp：特征点管理，三角化，关键帧等  
&emsp; &emsp;parameters.cpp：读取参数  

--------------------- 

参考笔记：

VINS论文推导及代码解析 by 崔华坤

[VINS技术路线与代码详解](https://blog.csdn.net/wangshuailpp/article/details/78461171) by 五行缺帅wangshuailpp

[VINS-Mono 学习笔记](https://zhuanlan.zhihu.com/p/36161028) by 童哲航

[VINS-Mono 代码解读](https://blog.csdn.net/u012871872/article/details/78128087?locationNum=8&fps=1) by  Rain-XIA

[VINS-mono详细解读](https://www.cnblogs.com/ilekoaiq/p/8836970.html)  by 极品巧克力

参考VINS代码注释：

https://github.com/castiel520/VINS-Mono	by [castiel520](https://github.com/castiel520)

https://github.com/QingSimon/VINS-Mono-code-annotation	by [QingSimon](https://github.com/QingSimon)

----
