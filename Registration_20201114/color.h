#pragma once

#include "properties.h"
#include <iostream>

typedef struct Color
{
	int R;
	int G;
	int B;

	Color();
	Color(int R, int G, int B);
} COLOR;

//通过bias获取对应的颜色
Color get_color(Properties &prop ,float bias);