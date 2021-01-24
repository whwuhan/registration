# Registration

## 1.简介

这是一个采用点云配准，将扫描点云与模型(Mesh)生成点云，进行自动配准，同时筛选出扫描点云相较于模型点云偏差较大位置的程序。

编译环境：

* VS2017
* PCL1.9.1

## 2.使用

如果不需要修改源码，可直接修改`x64\Release`下的`registration.pcproperties`参数文件，命令行调用同目录下的Registration_20201114.exe。

### 2.1 参数设置

程序将读取`x64\Release`下的`registration.pcproperties`文件，打开之后如下:

![](https://github.com/whwuhan/Registration/blob/master/img/QQ%E6%88%AA%E5%9B%BE20210124183556.png)

* dis_threshold：最近点的偏差阈值，超过此阈值的点会被筛选出，现在已经设置为0.005
* iter：是配准的迭代此时，次数愈多时间花费愈多，但是效果越好，暂时设为100
* input_format：是输入的点云文件格式，都使用ply
* input_file_scan：就是扫描的原始点云所在的位置，目前直接放在程序所在目录`x64\Release`，所以直接填写点云文件名`scan_point_cloud_2.ply`
* input_file_origin：就是使用mesh生成的点云文件，一般生成的点云越密集，最后出来的效果越好,目前直接放在程序所在目录`x64\Release`，所以直接填写点云文件名`origin_point_cloud_1000000.ply`
* output_format：输出点云的格式，都使用ply
* output_file_icp：是扫描点云icp配准的结果，目前放在`x64\Release\data`下
* output_file_outlier：偏差超过阈值`dis_threshold`的点，目前放在`x64\Release\data`下
* output_file_icp_color：点云染色后的结果，偏差在0\~output_color_grade_1之间的点会被染成由蓝到绿，偏差在output_color_grade_1\~output_color_grade_2之间的点会被染成由绿到黄，偏差在output_color_grade_1\~output_color_grade_3之间的点会被染成由黄到红，偏差超过output_color_grade_3的点会被染成红色，目前放在`x64\Release\data`下
* output_file_total_bias：统计所有点的信息，包含了如下信息，目前放在`x64\Release\data`下
  * vertex amount：所有点的总个数
  * exc_vertex amount：超过阈值点的总个数
  * total_ave_bias：所有点的平均误差
  * index_1：当前点在扫描点云中的索引
  * index_2：当前点对应的最近点，在模型点云中的索引
  * point_1：当前点的坐标
  * point_2：当前点在模型点云中最近点的索引
  * bias：误差
* output_file_exc_bias：统计超过阈值点的信息，包含如下信息，目前放在`x64\Release\data`下
  * vertex amount：所有点的总个数
  * exc_vertex amount：超过阈值点的总个数
  * exc_ave_bias：超过阈值点的平均误差
  * index_1：当前超过阈值点在扫描点云中的索引
  * index_2：当前超过阈值点对应的最近点，在模型点云中的索引
  * point_1：当前超过阈值点的坐标
  * point_2：当前超过阈值点在模型点云中最近点的索引
  * bias：误差
* output_file_trans_mat：旋转矩阵信息，包含如下信息，目前放在`x64\Release\data`下
  * has_converged：算法是否收敛，1是收敛，0是不收敛
  * fitness_score：误差评价，越低越好
  * trans_mat：旋转平移矩阵
* output_color_grade_1：颜色区分第一级(0\~output_color_grade_1)
* output_color_grade_2：颜色区分第二级(output_color_grade_1\~output_color_grade_2)
* output_color_grade_3：颜色区分第三级(output_color_grade_2\~output_color_grade_3)，最后是(output_color_grade_3\~无穷)
* outlier_remove：是否对扫描点云进行降噪
* outlier_radius：降噪算法参数，表示每个点的搜索半径
* outlier_min_neighbors：降噪算法参数，表示半径内最少点数（低于最少点数，视为噪点）

### 2.2 程序使用

调整好参数后，命令行进入到程序所在目录`x64\Release`后直接调用可执行程序

```shell
.\Registration_20201114.exe
```

### 2.3 结果统计

所有的统计结果均放在目录`x64\Release\data`下

* exc_bias.txt：包含偏差超过阈值点的信息
  * vertex amount：总点数
  * exc_vertex amount：超过阈值的点数
  * exc_ave_bias：超过阈值的偏差平均值
  * index_1：点在扫描点云中的index
  * index_2：扫描点云对应的最近点（模型点云中）的index
  * point_1：扫描点的点坐标
  * point_2：模型点云的点坐标
  * bias：偏差值
* total_bias.txt：所有点的偏差信息
  * vertex amount：总点数
  * exc_vertex amount：超过阈值的点数
  * total_ave_bias：所有偏差的平均值
  * index_1：点在扫描点云中的index
  * index_2：扫描点云对应的最近点（模型点云中）的index
  * point_1：扫描点的点坐标
  * point_2：模型点云的点坐标
  * bias：偏差值
* trans_mat.txt：配准时采用的旋转平移矩阵
  * has_converged：icp算法是否收敛（1收敛，0未收敛）
  * fitness_score：得分
  * trans_mat：旋转平移矩阵
* icp_res.ply：扫描点云配准后的结果（没有染色）
* icp_res_color.ply：扫描点云配准后，根据偏差染色后的结果（如效果图中所示）
* outlier_res_0.005.ply：扫描点云配准后，偏差超过阈值0.005的点云

## 3.效果图

没有差异的地方为蓝色

有较小差异的地方为绿色

有较大差异的地方为黄色

差异巨大的地方为红色

* 石柱扫描点云相较于原始模型差异结果1：

![](https://github.com/whwuhan/Registration/blob/master/%E6%95%88%E6%9E%9C%E5%9B%BE/result_1.png)

* 石柱扫描点云相较于原始模型差异结果2：

![](https://github.com/whwuhan/Registration/blob/master/%E6%95%88%E6%9E%9C%E5%9B%BE/result_2.png)

## 附录

还有一些可能会用到的程序，存放在目录`x64\Release`下，使用方法同上，要查看参数，直接运行程序即可查看所需参数

* normalization.exe：用于归一化点云数据
* origin.exe：用于将点云放置到原点位置
* pcl_obj2pcd_release.exe：obj格式点云转pcd格式
* pcl_obj2ply_release.exe：obj格式点云转ply格式
* pcl_mesh_sampling_release.exe：将mesh采样成点云数据

