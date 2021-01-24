#pragma once
#include <string>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <boost/algorithm/string.hpp>   //split
/**
 * 程序参数结构体
*/
typedef struct Properties
{  
	//参数

	//所有参数对应的map
	std::unordered_map<std::string, std::string> prop_in;

	double dis_threshold;                       //距离阈值
	int iter;									//icp迭代次数
	std::string input_format;                   //输入格式
	std::string input_file_scan;                //扫描点云
	std::string input_file_origin;              //模型点云
	std::string output_format;                  //输出格式
	std::string output_file_icp;                //icp配准结果
	std::string output_file_outlier;            //离群点结果
	std::string output_file_icp_color;			//带颜色的icp区分立群点的配准结果
	std::string output_file_total_bias;			//所有点的误差结果保存路径
	std::string output_file_exc_bias;			//超过阈值点的误差保存路径
	std::string output_file_trans_mat;			//变换矩阵
	float output_color_grade_1;			//颜色分级
	float output_color_grade_2;
	float output_color_grade_3;
	bool outlier_remove;						//是否需要做离群点检测
	double outlier_radius;						//离群点检测半径
	int outlier_min_neighbors;					//离群点检测最小邻居个数

	//读入配置信息
	int read_data(const std::string property_file = "registration.pcproperties");

	//显示配置信息
	void show_properties();

} PROPERTIES;
