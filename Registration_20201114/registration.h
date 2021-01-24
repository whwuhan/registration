#pragma once
#include "properties.h"
#include "stat_data.h"
#include "color.h"
#include <iostream>
#include <string>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <pcl/io/ply_io.h>              //ply文件读取
#include <pcl/point_types.h>            //点类型
#include <pcl/point_cloud.h>            //点云类型
#include <pcl/registration/icp.h>       //ICP算法
#include <boost/algorithm/string.hpp>   //split
#include <pcl/kdtree/kdtree_flann.h>    //kd树加速
#include <pcl/filters/radius_outlier_removal.h>//去噪

class Registration
{
public:
	PROPERTIES prop;							//属性
	std::unordered_map<std::string, int> format;//可选点云的类型
	

	Registration();								//构造函数
	void init();								//初始化
	void run();									//开始算法
	~Registration();							//析构函数
private:
	//计算两点的欧式距离
	float get_euclidean_distance(const pcl::PointXYZ& pointXYZ_1, const pcl::PointXYZ& pointXYZ_2);

	// 显示当前时间
	void show_time();

	//计算所有点的偏差
	std::vector<float> cal_total_bias(std::vector<std::vector<float> >& sqr_dis);

	//计算超过阈值的偏差 注意和上面的参数类型不同
	std::vector<float> cal_exc_bias
	(
		std::vector<float> &dis,
		std::vector<int> &flags
	);

	//将vector<vector<int> >转化为vector<int>
	std::vector<int> trans_knn_index_to_nn_index
	(
		std::vector<std::vector<int> > &knn_index
	);
};




