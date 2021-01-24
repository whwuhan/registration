#include "pch.h"
#include "color.h"

Color::Color():R(0),G(0),B(0)
{}

Color::Color(int R, int G, int B) : R(R), G(G), B(B)
{}

//通过bias获取对应的颜色
Color get_color(Properties &prop, float bias)
{
	Color color;
	//颜色分级
	if (bias < prop.output_color_grade_1)
	{
		color.G = (int) 255 * (bias / prop.output_color_grade_1);
		color.B = (int) 255 * (1.0 - bias / prop.output_color_grade_1);
	}
	else if (bias < prop.output_color_grade_2)
	{
		color.G = 255;
		color.R = (int) 255 * (bias - prop.output_color_grade_1) / (prop.output_color_grade_2 - prop.output_color_grade_1);
	}
	else if (bias < prop.output_color_grade_3)
	{
		color.R = 255;
		color.G = 255 * (1.0 - (bias - prop.output_color_grade_2) / (prop.output_color_grade_3 - prop.output_color_grade_2));
	}
	else
	{
		color.R = 255;
	}
	return color;
}