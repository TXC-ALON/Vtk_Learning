#include "TriangleMesh.hpp"
#include <float.h>
#include "SVG.hpp"


namespace Slic3r {
/////////////////////////////////////////////////////////////////////////////////////////////////////////
///  数字标记点生成算法
////////////////////////////////////////////////////////////////////////////////////////////////////////

bool 
TriangleMesh::make_numPoints(size_t  spted_volume_id)
{
	this->numPoints.clear();
	ExPolygons exs = this->horizontal_projection_slm(90.0);
	LOGINFO("this exboundLoops size = %d", exs.size());
	if (exs.size() == 0)
	{
		LOGINFO("this is no exboundLoops");
		return false;
	}
	Pointf tempPoint = TriangleMesh::get_ExMidPoint(exs);
	SPT_pointf spt_pf(tempPoint, 0, this->get_tree_region_idx(spted_volume_id, 0));
	this->numPoints.push_back(spt_pf);

	return true;
}

ExPolygon 
TriangleMesh::get_maxEx(ExPolygons exs)
{
	double max_area = 0;
	int max_area_index = -1;
	for (int i = 0; i < exs.size(); i++)
	{
		if (max_area < exs.at(i).area())
		{
			max_area = exs.at(i).area();
			max_area_index = i;
		}
	}
	if (max_area_index == -1)
	{
		LOGINFO("this is no ExPolygons");
		return ExPolygon();
	}

	return exs.at(max_area_index);
}

// 计算二维图形的中心点
Pointf
TriangleMesh::get_ExMidPoint(ExPolygons exs)
{
	Pointf retval_pointf;
	ExPolygon max_ex = TriangleMesh::get_maxEx(exs);
	while (exs.size() != 0)
	{
		max_ex = TriangleMesh::get_maxEx(exs);
		exs = offset_ex(max_ex, scale_(-0.2));
	}

	retval_pointf.x = 0;
	retval_pointf.y = 0;
	if (max_ex.contour.points.size() == 0)
		return retval_pointf;

	for (int i = 0; i < max_ex.contour.points.size(); i++)
	{
		retval_pointf.x += unscale(max_ex.contour.points[i].x);
		retval_pointf.y += unscale(max_ex.contour.points[i].y);
	}
	retval_pointf.x /= max_ex.contour.points.size();
	retval_pointf.y /= max_ex.contour.points.size();

	LOGINFO("retval_pointf = %s", retval_pointf.dump_perl().c_str());
	return retval_pointf;
}

std::vector<Pointf> 
TriangleMesh::get_ExPoints(ExPolygons exs, double offset_val)
{
	std::vector<Pointf> retval_pointfs;
	ExPolygon max_ex = TriangleMesh::get_maxEx(exs);
	Pointf retval_pointf;
	while (exs.size() != 0)
	{
		Pointf minY_point, maxY_point;
		retval_pointf.x = 0;
		retval_pointf.y = 0;
		double minY = DBL_MAX;
		double maxY = -99999;
		max_ex = TriangleMesh::get_maxEx(exs);
		exs = offset_ex(max_ex, scale_(-offset_val));
		for (int i = 0; i < max_ex.contour.points.size(); i++)
		{
			double db_x = unscale(max_ex.contour.points[i].x);
			double db_y = unscale(max_ex.contour.points[i].y);
			retval_pointf.x += db_x;
			retval_pointf.y += db_y;
			if (minY > db_y)
			{
				minY = db_y;
				minY_point.x = db_x;
				minY_point.y = db_y;
			}
			if (maxY < db_y)
			{
				maxY = db_y;
				maxY_point.x = db_x;
				maxY_point.y = db_y;
			}
		}
		retval_pointf.x /= max_ex.contour.points.size();
		retval_pointf.y /= max_ex.contour.points.size();		
		retval_pointfs.push_back(minY_point);
		retval_pointfs.push_back(maxY_point);
	}
	retval_pointfs.push_back(retval_pointf);

	return retval_pointfs;
}

}