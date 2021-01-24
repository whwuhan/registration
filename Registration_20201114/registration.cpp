#include "pch.h"
#include "registration.h"
using namespace std;

Registration::Registration()
{

}

//初始化
void Registration::init()
{
	//将参数放入Properties结构体
	PROPERTIES properties; //当前配置
	if (!properties.read_data())
	{
		std::cerr << "registration.properties文件参数设置有误，或者没有读取到文件..." << std::endl;
	}
	else
	{
		prop = properties;
		properties.show_properties();
	}

	//存储格式map
	std::unordered_map<std::string, int> format =
	{
		{"ply",1},
		{"obj",2},
		{"pcd",3},
		{"stl",4},
		{"pts",5}
	};
	this->format = format;
}

//开始算法
void Registration::run()
{
	std::cout << "开始时间：";
	show_time();
	//扫描点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr
		point_cloud_scan(new pcl::PointCloud<pcl::PointXYZ>);

	//原始点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr
		point_cloud_origin(new pcl::PointCloud<pcl::PointXYZ>);


	//读取点云数据
	switch (format[prop.input_format])
	{
	case 1:
		if (pcl::io::loadPLYFile(prop.input_file_scan, *point_cloud_scan) == -1) 
		{
			std::cerr << "读取扫描点云失败." << std::endl;
			return ;
		}
		if (pcl::io::loadPLYFile(prop.input_file_origin, *point_cloud_origin) == -1) 
		{
			std::cerr << "读取原始点云失败." << std::endl;
			return ;
		}
		//std::cerr << point_cloud_scan->size() << " " <<point_cloud_origin->size() << std::endl;
		break;  //不要忘了break!!!!!!!!
	default:
		std::cerr << "无法处理输入格式" << prop.input_format << "格式，请更改配置文件" << std::endl;
		return ;
	}

	//========================================================
	//去噪
	if (prop.outlier_remove)
	{	
		std::cout << "=========================开始降噪...===============================" << std::endl;
		std::cout << "时间：";
		show_time();
		pcl::PointCloud<pcl::PointXYZ>::Ptr
			point_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

		pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
		outrem.setInputCloud(point_cloud_scan);
		outrem.setRadiusSearch(prop.outlier_radius);
		outrem.setMinNeighborsInRadius(prop.outlier_min_neighbors);
		outrem.setKeepOrganized(false); //如果设为true，被标记为outlier的点会被置为nan，不会改变原始点云的大小
		outrem.filter(*point_cloud_filtered);
			
		unsigned int pre_size = point_cloud_scan->size();
		std::cout << "降噪前点云大小：" << pre_size << std::endl;
		point_cloud_scan = point_cloud_filtered;
		unsigned int pro_size = point_cloud_scan->size();
		std::cout << "降噪后点云大小: " << pro_size << std::endl;
		std::cout << "去除噪点个数：" << pre_size - pro_size << std::endl;

		std::cout << "时间：";
		show_time();
		std::cout << "=========================降噪结束===============================" << std::endl;
	}
	//去噪结束
	//========================================================


	
	//===============================================================
	//ICP配准
	std::cout << "=========================正在ICP配准...=================================" << std::endl;
	std::cout << "时间：";
	show_time();
	//创建ICP对象
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	//设置参数
	icp.setInputSource(point_cloud_scan);      //需要配准的点云
	icp.setInputTarget(point_cloud_origin);    //目标点云，就是不动的那个
	icp.setMaximumIterations(prop.iter);
	//保存结果
	pcl::PointCloud<pcl::PointXYZ>::Ptr
		point_cloud_icp_res(new pcl::PointCloud<pcl::PointXYZ>);
	icp.align(*point_cloud_icp_res);
	//hasConverged()返回是否收敛
	std::cout << "has converged:" << icp.hasConverged() << std::endl;
	std::cout << "score: " << icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	StatData stat_data;	//统计数据
	//保存icp统计数据
	stat_data.point_cloud_icp_res_ptr = point_cloud_icp_res;
	stat_data.point_cloud_origin_ptr = point_cloud_origin;
	stat_data.has_converged = icp.hasConverged();
	stat_data.fitness_score = icp.getFitnessScore();
	stat_data.trans_mat = icp.getFinalTransformation();
	switch (format[prop.output_format])
	{
	case 1:
		pcl::io::savePLYFile(prop.output_file_icp, *point_cloud_icp_res);
		break;
	default:
		std::cout << "无法处理输出格式" << prop.output_format << "格式，请更改配置文件" << std::endl;
		return ;
	}
	std::cout << "时间：";
	show_time();
	std::cout << "=========================icp配准结束=================================" << std::endl;
	//icp配准结束
	//============================================================


	//=========================================================
	//筛选超过阈值的点
	//创建一个kd_tree
	std::cout << "=========================开始筛选...==============================" << std::endl;
	std::cout << "时间：";
	show_time();
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	//设置输入点云
	kdtree.setInputCloud(point_cloud_origin);
	//k近邻居
	int k = 1;

	std::vector<int> point_knn_index(k);						//某一个点的k近邻居索引
	std::vector<float> point_knn_sqr_dis(k);					//某个点k近邻居的距离的平方
	std::vector<std::vector<int> > point_cloud_knn_index;		//配准点云所有点的k近邻居索引
	std::vector<std::vector<float> > point_cloud_knn_sqr_dis;	//配准点云所有点的k近邻居距离的平方
	for (int i = 0; i < point_cloud_icp_res->points.size(); i++)
	{
		if (kdtree.nearestKSearch(point_cloud_icp_res->points[i], k, point_knn_index, point_knn_sqr_dis) > 0)
		{
			point_cloud_knn_index.push_back(point_knn_index);
			point_cloud_knn_sqr_dis.push_back(point_knn_sqr_dis);
			point_knn_index.clear();
			point_knn_sqr_dis.clear();
		}
	}

	std::vector<int> flags(point_cloud_knn_sqr_dis.size(), 0);		//标志
	int size_count = 0;												//超出阈值点个数
	for (int i = 0; i < point_cloud_knn_sqr_dis.size(); i++)
	{
		//判断是否大于距离阈值
		if (point_cloud_knn_sqr_dis[i][0] > prop.dis_threshold * prop.dis_threshold)
		{
			flags[i] = 1;
			size_count++;
		}
	}

	
	stat_data.total_amount = point_cloud_scan->points.size();					//总的点数
	stat_data.exc_amount = size_count;											//超过阈值的点数
	stat_data.nn_index = trans_knn_index_to_nn_index(point_cloud_knn_index);	//最近邻居的index

	//计算所有点的偏差
	stat_data.total_bias = cal_total_bias(point_cloud_knn_sqr_dis);
	//计算超过阈值点的偏差 注意第一个参数
	stat_data.exc_bias = cal_exc_bias(stat_data.total_bias, flags);
	//对应点是否超过阈值
	stat_data.flags = flags;

	pcl::PointCloud<pcl::PointXYZ>::Ptr
		point_cloud_compare_res(new pcl::PointCloud<pcl::PointXYZ>); //对比结果
	point_cloud_compare_res->height = 1;
	point_cloud_compare_res->width = size_count;
	point_cloud_compare_res->resize(point_cloud_compare_res->height * point_cloud_compare_res->width);
	size_count = 0;
	for (int i = 0; i < point_cloud_knn_sqr_dis.size(); i++)
	{
		if (flags[i])
		{
			point_cloud_compare_res->points[size_count++] = point_cloud_icp_res->points[i];
		}
	}
	std::cout << "时间：";
	show_time();
	std::cout << "=========================筛选结束==============================" << std::endl;
	//筛选结束
	//==================================================

	//==================================================
	//添加颜色区分
	std::cout << "=========================添加颜色区分...================================" << std::endl;
	std::cout << "时间：";
	show_time();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		point_cloud_xyzrgb_icp_res(new pcl::PointCloud<pcl::PointXYZRGB>);
	point_cloud_xyzrgb_icp_res->width = point_cloud_icp_res->width;
	point_cloud_xyzrgb_icp_res->height = point_cloud_icp_res->height;
	point_cloud_xyzrgb_icp_res->resize(point_cloud_xyzrgb_icp_res->width * point_cloud_xyzrgb_icp_res->height);
	for (int i = 0; i < point_cloud_xyzrgb_icp_res->size(); i++)
	{
		point_cloud_xyzrgb_icp_res->points[i].x = point_cloud_icp_res->points[i].x;
		point_cloud_xyzrgb_icp_res->points[i].y = point_cloud_icp_res->points[i].y;
		point_cloud_xyzrgb_icp_res->points[i].z = point_cloud_icp_res->points[i].z;
		/*if (flags[i])
		{
			point_cloud_xyzrgb_icp_res->points[i].r = 255;
			point_cloud_xyzrgb_icp_res->points[i].g = 0;
			point_cloud_xyzrgb_icp_res->points[i].b = 0;
		}
		else
		{
			point_cloud_xyzrgb_icp_res->points[i].r = 0;
			point_cloud_xyzrgb_icp_res->points[i].g = 0;
			point_cloud_xyzrgb_icp_res->points[i].b = 255;
		}*/
		Color color = get_color(prop, stat_data.total_bias[i]);
		point_cloud_xyzrgb_icp_res->points[i].r = color.R;
		point_cloud_xyzrgb_icp_res->points[i].g = color.G;
		point_cloud_xyzrgb_icp_res->points[i].b = color.B;
	}
	std::cout << "时间：";
	show_time();
	std::cout << "=========================添加颜色区分结束========================" << std::endl;
	//添加颜色区分结束
	//==================================================
	
	stat_data.cal_ave_bais();//计算平均偏差
	stat_data.show_data();
	//保存结果
	std::cout << "=========================开始保存结果========================" << std::endl;
	std::cout << "时间：";
	show_time();
	switch (format[prop.output_format]) 
	{
	case 1:
		pcl::io::savePLYFile(prop.output_file_outlier, *point_cloud_compare_res);
		pcl::io::savePLYFile(prop.output_file_icp_color, *point_cloud_xyzrgb_icp_res);
		break;
	default:
		std::cout << "无法处理输出格式" << prop.output_format << "格式，请更改配置文件" << std::endl;
		return ;
	}
	stat_data.save_data(prop);
	std::cout << "时间：";
	show_time();
	std::cout << "=========================保存结果结束================================" << std::endl;
	std::cout << "结束时间：";
	show_time();
	
}



//计算所有点的偏差
std::vector<float> Registration::cal_total_bias(std::vector<std::vector<float> >& sqr_dis)
{
	std::vector<float> res(sqr_dis.size(), 0);
	for (int i = 0; i < sqr_dis.size(); i++)
	{
		res[i] = sqrt(sqr_dis[i][0]);
	}
	return res;
}

//计算超过阈值的偏差
std::vector<float> Registration::cal_exc_bias
(
	std::vector<float> &dis,
	std::vector<int> &flags
)
{
	std::vector<float> res;
	int count = 0;
	for (int i = 0; i < flags.size(); i++)
	{
		if (flags[i])
		{
			res.push_back(dis[i]);
		}
	}
	return res;
}

//将vector<vector<int> >转化为vector<int>
std::vector<int> Registration::trans_knn_index_to_nn_index
(
	std::vector<std::vector<int> > &knn_index
)
{
	vector<int> res(knn_index.size());
	for (int i = 0; i < knn_index.size(); i++)
	{
		res[i] = knn_index[i][0];
	}
	return res;
}

//析构函数
Registration::~Registration()
{
}

//计算欧式距离
float Registration::get_euclidean_distance(const pcl::PointXYZ& pointXYZ_1, const pcl::PointXYZ& pointXYZ_2)
{
	float euclidean_distance =
		(pointXYZ_1.x - pointXYZ_2.x) * (pointXYZ_1.x - pointXYZ_2.x) +
		(pointXYZ_1.y - pointXYZ_2.y) * (pointXYZ_1.y - pointXYZ_2.y) +
		(pointXYZ_1.z - pointXYZ_2.z) * (pointXYZ_1.z - pointXYZ_2.z);

	euclidean_distance = sqrt(euclidean_distance);
	return euclidean_distance;
}

//显示时间
void Registration::show_time()
{
	time_t now = time(0);			//获取当前时间 1970到现在的秒数
	tm *loc_time = localtime(&now);	//转化成当地时间
	std::cout << 1900 + loc_time->tm_year << "年" << 1 + loc_time->tm_mon << "月" << loc_time->tm_mday << "日";
	std::cout << std::setw(2) << std::setfill('0') << loc_time->tm_hour << ":";
	std::cout << std::setw(2) << std::setfill('0') << loc_time->tm_min << ":";
	std::cout << std::setw(2) << std::setfill('0') << loc_time->tm_sec << std::endl;
}

