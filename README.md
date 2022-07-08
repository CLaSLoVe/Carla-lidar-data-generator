---
author:
- 金典
title: 基于CARLA模拟器的lidar数据采集项目
---

# CARLA设置

## 服务端设置

### 构建服务端

CARLA模拟器端口2000。traffic
manager端口默认8000，但是由于被占用，设置为8005。

### 添加actor

利用generate_traffic.py生成车辆60（种类随机）以及行人40。

## 客户端设置

### 自车设置

自车为Tesla的model3。

### 传感器设置

设置三个lidar，高度分别为0.4,0.8,1.6。均安装在车中心靠前（车头）半个车子长度的位置。

# Bounding box获取

已知每个actor对应bbox的八个角在世界坐标系下坐标：
$$\bm{a}^w_i = (x^w_i, y^w_i, z^w_i), i=0,\cdots,7$$

已知世界坐标系到lidar坐标系的转换矩阵$\mathbf{S}$，可求得传感器坐标系下的坐标$\bm{a}^s$：
$$[\bm{a}^s, \bm{1}] = \mathbf{S}\cdot [\bm{a}^w, \bm{1}]$$

接下来可以求得在actor坐标系下三个方向的单位向量： $$\begin{split}
        \bm{e}_x &= (\bm{o}_x-\bm{o})/|\bm{o}_x-\bm{o}|\\
        \bm{e}_y &= (\bm{o}_y-\bm{o})/|\bm{o}_y-\bm{o}|\\
        \bm{e}_z &= (\bm{o}_z-\bm{o})/|\bm{o}_z-\bm{o}|
    \end{split}$$ 其中， $$\begin{split}
        \bm{o}  &= (\bm{a}^s_0+\bm{a}^s_6)/2\\
        \bm{o}_x&= (\bm{a}^s_0+\bm{a}^s_2)/2\\
        \bm{o}_y&= (\bm{a}^s_0+\bm{a}^s_5)/2\\
        \bm{o}_z&= (\bm{a}^s_5+\bm{a}^s_2)/2
    \end{split}$$

## 计算box中点云数量

考虑点云坐标向量矩阵$\mathbf{X}_w$:
$$\mathbf{X}_w = [\bm{x},\bm{y},\bm{z}]$$

计算box中心到点云的向量矩阵：

$$\mathbf{X}_b = \mathbf{X}_w-\bm{o}$$

可以计算在actor坐标系下，点云中各点坐标$\mathbf{X}_a$：
$$\mathbf{X}_a=\mathbf{X}_b\cdot 
    [\bm{e}_x,\bm{e}_y,\bm{e}_z]^\text{T}=
    \begin{bmatrix}
        p_{x1}&p_{y1}&p_{z1}\\
        \vdots &\vdots&\vdots\\
        p_{xn}&p_{yn}&p_{zn}
    \end{bmatrix}$$

逐项取绝对值： $$|\mathbf{X}_a|=
\begin{bmatrix}
    |p_{x1}|&|p_{y1}|&|p_{z1}|\\
    \vdots &\vdots&\vdots\\
    |p_{xn}|&|p_{yn}|&|p_{zn}|
\end{bmatrix}=
    [\bm{p}_x,\bm{p}_y,\bm{p}_z]$$

统计数量：
$$c = \text{cnt}((\bm{p}_x<=l)~\wedge ~(\bm{p}_y<=w)~\wedge~(\bm{p}_z<=h))$$

## 计算bounding box有效性

对于不同距离，设定不同阈值$\xi$：

$$\xi = \left\{\begin{array}{ll}
    \xi_1&L\in[0,d_1)\\
    \xi_2&L\in[d_1,d_2)\\
    \xi_3&L\in[d_2,\infty)
    \end{array}\right.$$

其中$\xi_1,\xi_2,\xi_2$以及$d_1,d_2,d_3$为设定参数。

# 数据集导出

## 目录格式

ROOT/\
GPS/\
RGB/\
/\
lidar/\
label/\
calib/\
pics/\
posit/\

## 模拟器参数

### 环境

其中$\xi_1,\xi_2,\xi_2$以及$d_1,d_2,d_3$为设定参数。

   Vehicle         Pedestrain  
  --------- ----- ------------ -----
   参数名    值      参数名     值
   $\xi_1$   10     $\xi_1$      5
   $\xi_2$    8     $\xi_2$      4
   $\xi_3$    5     $\xi_3$      3
    $d_1$    50      $d_1$      50
    $d_2$    100     $d_2$      100
    $d_3$    150     $d_3$      150

  : 设定参数

### lidar

         参数名           值
  -------------------- --------
         range           150
   points_per_second    576000
   rotation_frequency     10

  : 设定参数

## 数据内容

### GPS

可由外参calib得到传感器的位姿。使用时忽略该文件。

### RGB

120米高度俯视照片。如图[1](#fig:rgb){reference-type="ref"
reference="fig:rgb"}所示。

![俯视图](fig/rgb.jpg){#fig:rgb width=".6\\linewidth"}

### label

label/文件夹中文件每一条如下所示：

     参数名                                值
  ------------ ----------------------------------------------------------
      type      当$l\ge 4$时，为Truck。另外还有Pedestrain， Car和Cyclist
   truncated                             缺省1
    occluded                             缺省1
     alpha           $\arctan({\bm{o}[1]/\bm{o}[0]})$，$[-\pi,\pi)$
      bbox                            缺省1,1,1,1
   dimensions                          $2h,2w,2l$
    location              $\bm{o}$，前、右、上、顺时针为正方向
   rotation_y    两车yaw角之差$\theta=\theta_e-\theta_a$，$[-\pi,\pi)$
     score                               缺省1

  : 参数介绍

如图[2](#fig:co){reference-type="ref" reference="fig:co"}所示。

![左手系，图示方向为各个参数正方向](fig/co.pdf){#fig:co
width=".25\\linewidth"}

### lidar

每个lidar每一帧的点云二进制文件。

### pics

每帧lidar点云的俯视图。有效的bbox为绿色，无效的为黄色，如图[3](#fig:lidar){reference-type="ref"
reference="fig:lidar"}所示。

![lidar点云的俯视图](fig/lidar.png){#fig:lidar width=".5\\linewidth"}

### calib

记录了每个lidar每一帧的外参矩阵。

### posit

记录了actor的各类信息，用于计算点云有效性。使用时忽略该文件。

# 附录 {#附录 .unnumbered}

## A 代码托管链接 {#a-代码托管链接 .unnumbered}

<https://github.com/CLaSLoVe/Carla-lidar-data-generator>

## B 代码功能 {#b-代码功能 .unnumbered}

按照表[1](#tab:code){reference-type="ref"
reference="tab:code"}的顺序运行代码。其中validate_bb_thread.py需要设置数据集路径作为运行参数。

::: {#tab:code}
             名称                        功能
  --------------------------- --------------------------
        server_setup.py               设置服务器
      generate_traffic.py           生成行人、车辆
   lidar_data_obtain_dir4.py           采集数据
     validate_bb_thread.py     多进程实现bbox有效性计算

  : 代码介绍
:::

[]{#tab:code label="tab:code"}
