// Registration_20200726.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include "pch.h"
#include "registration.h"
#include <iostream>

int main()
{
	Registration reg;
	reg.init();
	reg.run();

	/*pcl::PointCloud<pcl::PointXYZ> cloud_a;
	pcl::PointCloud<pcl::RGB> cloud_b;
	pcl::PointCloud<pcl::PointXYZRGB> cloud_c;

	cloud_a.width = cloud_b.width = 5;
	cloud_a.height = cloud_b.height = 1;
	cloud_a.points.resize(cloud_a.width * cloud_a.height);
	cloud_b.points.resize(cloud_b.width * cloud_b.height);

	for (std::size_t i = 0; i < cloud_a.size(); ++i)
	{
		cloud_a[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_a[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_a[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	for (std::size_t i = 0; i < cloud_b.size(); ++i)
	{
		cloud_b[i].r = (std::uint8_t)255;
		cloud_b[i].g = (std::uint8_t)255;
		cloud_b[i].b = (std::uint8_t)255;
	}

	for (auto it = cloud_b.begin(); it != cloud_b.end(); it++)
	{
		std::cout << (int)it->r << " " << (int)it->g << " " << (int)it->b << std::endl;
	}
	pcl::concatenateFields(cloud_a, cloud_b, cloud_c);
	std::cout << cloud_c << std::endl;*/
	return 0;
}

