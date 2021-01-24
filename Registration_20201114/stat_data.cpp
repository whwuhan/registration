#include "pch.h"
#include "stat_data.h"
using namespace std;

//计算平均误差
void StatData::cal_ave_bais()
{	
	//统计所有点的平均误差
	float sum = 0;
	for (float &item : total_bias)
	{
		sum += item;
	}
	total_ave_bias = sum / total_bias.size();

	//统计超过阈值的平均误差
	sum = 0;
	for (float &item : exc_bias)
	{
		sum += item;
	}
	exc_ave_bias = sum / exc_bias.size();
}

//显示统计信息
void StatData::show_data()
{
	std::cout << "=================统计信息===================" << std::endl;
	std::cout << "total_amount:"	<< total_amount << std::endl;
	std::cout << "exc_amount:" << exc_amount << std::endl;
	std::cout << "total_ave_bias:" << total_ave_bias << std::endl;
	std::cout << "exc_ave_bias:" << exc_ave_bias << std::endl;
	std::cout << "has_converged:" << has_converged << std::endl;
	std::cout << "fitness_score:" << fitness_score << std::endl;
	std::cout << "trans_mat:" << std::endl;
	std::cout << trans_mat << std::endl;
	std::cout << "=================统计信息结束=================" << std::endl;
}

//保存统计信息
void StatData::save_data(Properties &prop)
{
	std::ofstream file_out(prop.output_file_total_bias);
	//统计所有点的信息
	if (file_out)
	{
		file_out << "vertex amount:" << total_amount << std::endl;
		file_out << "exc_vertex amount:" << exc_amount << std::endl;
		file_out << "total_ave_bias:" << total_ave_bias << std::endl;
		file_out << std::setw(10) << std::setfill(' ') << "index_1" << " ";
		file_out << std::setw(10) << std::setfill(' ') << "index_2" << " ";
		file_out << std::setw(10) << std::setfill(' ') << "point_1" << " ";
		file_out << std::setw(30) << std::setfill(' ') << "point_2" << " ";
		file_out << std::setw(30) << std::setfill(' ') << "bias " << " ";
		file_out << std::endl;

		for (int i = 0; i < total_bias.size(); i++)
		{
			file_out << std::setw(10) << std::setfill(' ') << i << " ";
			file_out << std::setw(10) << std::setfill(' ') << nn_index[i] << " ";
			file_out << std::setw(10) << std::setfill(' ') << point_cloud_icp_res_ptr->points[i].x << " ";
			file_out << std::setw(10) << std::setfill(' ') << point_cloud_icp_res_ptr->points[i].y << " ";
			file_out << std::setw(10) << std::setfill(' ') << point_cloud_icp_res_ptr->points[i].z << " ";
			file_out << std::setw(10) << std::setfill(' ') << point_cloud_origin_ptr->points[nn_index[i]].x << " ";
			file_out << std::setw(10) << std::setfill(' ') << point_cloud_origin_ptr->points[nn_index[i]].y << " ";
			file_out << std::setw(10) << std::setfill(' ') << point_cloud_origin_ptr->points[nn_index[i]].z << " ";
			file_out << std::setw(10) << std::setfill(' ') << total_bias[i] << " ";
			file_out << std::endl;
		}
	}
	else
	{	
		std::cerr << prop.output_file_total_bias << "打开失败" << std::endl;
		return;
	}

	//统计超过阈值点的数据
	file_out.clear();
	file_out.close();	//重要，要先关闭文件流，再重新打开
	file_out.open(prop.output_file_exc_bias);
	if (file_out)
	{
		file_out << "vertex amount:" << total_amount << std::endl;
		file_out << "exc_vertex amount:" << exc_amount << std::endl;
		file_out << "exc_ave_bias:" << exc_ave_bias << std::endl;
		file_out << std::setw(10) << std::setfill(' ') << "index_1" << " ";
		file_out << std::setw(10) << std::setfill(' ') << "index_2" << " ";
		file_out << std::setw(10) << std::setfill(' ') << "point_1" << " ";
		file_out << std::setw(30) << std::setfill(' ') << "point_2" << " ";
		file_out << std::setw(30) << std::setfill(' ') << "bias " << " ";
		file_out << std::endl;

		for (int i = 0; i < flags.size(); i++)
		{
			if (flags[i])
			{
				file_out << std::setw(10) << std::setfill(' ') << i << " ";
				file_out << std::setw(10) << std::setfill(' ') << nn_index[i] << " ";
				file_out << std::setw(10) << std::setfill(' ') << point_cloud_icp_res_ptr->points[i].x << " ";
				file_out << std::setw(10) << std::setfill(' ') << point_cloud_icp_res_ptr->points[i].y << " ";
				file_out << std::setw(10) << std::setfill(' ') << point_cloud_icp_res_ptr->points[i].z << " ";
				file_out << std::setw(10) << std::setfill(' ') << point_cloud_origin_ptr->points[nn_index[i]].x << " ";
				file_out << std::setw(10) << std::setfill(' ') << point_cloud_origin_ptr->points[nn_index[i]].y << " ";
				file_out << std::setw(10) << std::setfill(' ') << point_cloud_origin_ptr->points[nn_index[i]].z << " ";
				file_out << std::setw(10) << std::setfill(' ') << total_bias[i] << " ";
				file_out << std::endl;
			}
		}
	}
	else
	{
		std::cerr << prop.output_file_exc_bias << "打开失败" << std::endl;
		return;
	}

	//统计旋转矩阵
	file_out.clear();
	file_out.close();
	file_out.open(prop.output_file_trans_mat);
	if (file_out)
	{
		file_out << "has_converged:" << has_converged << std::endl;
		file_out << "fitness_score:" << fitness_score << std::endl;
		file_out << "trans_mat:" << std::endl;
		file_out << std::setw(10) << std::setfill(' ') << trans_mat(0, 0) << " ";
		file_out << std::setw(10) << std::setfill(' ') << trans_mat(0, 1) << " ";
		file_out << std::setw(10) << std::setfill(' ') << trans_mat(0, 2) << " ";
		file_out << std::setw(10) << std::setfill(' ') << trans_mat(0, 3) << std::endl;

		file_out << std::setw(10) << std::setfill(' ') << trans_mat(1, 0) << " ";
		file_out << std::setw(10) << std::setfill(' ') << trans_mat(1, 1) << " ";
		file_out << std::setw(10) << std::setfill(' ') << trans_mat(1, 2) << " ";
		file_out << std::setw(10) << std::setfill(' ') << trans_mat(1, 3) << std::endl;

		file_out << std::setw(10) << std::setfill(' ') << trans_mat(2, 0) << " ";
		file_out << std::setw(10) << std::setfill(' ') << trans_mat(2, 1) << " ";
		file_out << std::setw(10) << std::setfill(' ') << trans_mat(2, 2) << " ";
		file_out << std::setw(10) << std::setfill(' ') << trans_mat(2, 3) << std::endl;

		file_out << std::setw(10) << std::setfill(' ') << trans_mat(3, 0) << " ";
		file_out << std::setw(10) << std::setfill(' ') << trans_mat(3, 1) << " ";
		file_out << std::setw(10) << std::setfill(' ') << trans_mat(3, 2) << " ";
		file_out << std::setw(10) << std::setfill(' ') << trans_mat(3, 3) << std::endl;

	}
	else
	{
		std::cerr << prop.output_file_trans_mat << "打开失败" << std::endl;
		return;
	}

	file_out.close();
}