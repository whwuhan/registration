#pragma once
/*
	统计数据
*/
#include "properties.h"
#include <iostream>
#include <string>
#include <cstdlib>
#include <fstream>
#include <pcl/io/ply_io.h>              //ply文件读取
#include <pcl/point_types.h>            //点类型
#include <pcl/point_cloud.h>            //点云类型
#include <pcl/registration/icp.h>       //ICP算法
#include <boost/algorithm/string.hpp>   //split
#include <pcl/kdtree/kdtree_flann.h>    //kd树加速
#include <pcl/filters/radius_outlier_removal.h>//去噪


typedef struct StatData
{
	//需要保存的数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_icp_res_ptr;
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_origin_ptr;

	unsigned int total_amount;				//总的点数
	unsigned int exc_amount;				//超过阈值的点数
	std::vector<float> total_bias;			//所有点的偏差值
	std::vector<float> exc_bias;			//超过阈值的偏差值
	double total_ave_bias;					//所有点的平均误差
	double exc_ave_bias;					//超出阈值点的平均误差
	std::vector<int> nn_index;				//最近邻居的index
	std::vector<int> flags;					//是否超过阈值,1表示超过，0表示未超过
	int has_converged;						//是否收敛
	double fitness_score;					//匹配分数
	Eigen::Matrix4f trans_mat;				//变换矩阵
	
	//结构体方法
	void cal_ave_bais();					//计算平均误差
	void show_data();						//显示统计信息
	void save_data(Properties &prop);		//保存统计数据
} STATDATA;