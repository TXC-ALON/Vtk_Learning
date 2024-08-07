#include "TriangleMesh.hpp"
#include <float.h>
#include "SVG.hpp"
#include "ClipperUtils.hpp"
#include "Geometry.hpp"
#include <cmath>
#include <deque>
#include <queue>
#include <set>
#include <vector>
#include <map>
#include <utility>
#include <algorithm>
#include <math.h>
#include <assert.h>
#include <stdexcept>
#include <Line.hpp>

namespace Slic3r {

//标记出面积小于阈值的面片
bool TriangleMesh::Area_Filter(double area, bool _in, bool _out, bool order)
{
	DWORD timecount = GetTickCount();
	LOGINFO("Start Edge_FilterFacets！！！！！！！！！！！！！！！！");

	//facet.extra突起面片初始标记
	for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
	{
		stl_facet &facet = this->stl.facet_start[i];
		//if ( acos(-facet.normal.z) * 180 < (90 * PI) )// 角度大于阈值
		{
			if (TriangleMesh::Calculate_Facet_Area(facet) < area && TriangleMesh::Calculate_MaxMinSide_Scale(facet) < 10)
				TriangleMesh::fact_set_mark(facet.extra, Tuqi_Fact);
			else
				TriangleMesh::fact_remove_mark(facet.extra, Tuqi_Fact);
		}
	}

	if (order)
	{
		if (_in)
		{
			this->tuqi_facets_index = this->Get_facets(Tuqi_Fact, false);
			this->other_facets_index = this->Get_facets(Tuqi_Fact, true);
			LOGINFO("tuqi_facets_index.size() = %d", this->tuqi_facets_index.size());
			LOGINFO("other_facets_index.size() = %d", this->other_facets_index.size());
			this->Area_FilterFacets(Tuqi_Fact, this->tuqi_facets_index);
		}
		if (_out)
		{
			this->tuqi_facets_index = this->Get_facets(Tuqi_Fact, false);
			this->other_facets_index = this->Get_facets(Tuqi_Fact, true);
			LOGINFO("tuqi_facets_index.size() = %d", this->tuqi_facets_index.size());
			LOGINFO("other_facets_index.size() = %d", this->other_facets_index.size());
			this->reArea_FilterFacets(Tuqi_Fact, this->other_facets_index);
		}
	}
	else
	{
		if (_out)
		{
			this->tuqi_facets_index = this->Get_facets(Tuqi_Fact, false);
			this->other_facets_index = this->Get_facets(Tuqi_Fact, true);
			LOGINFO("tuqi_facets_index.size() = %d", this->tuqi_facets_index.size());
			LOGINFO("other_facets_index.size() = %d", this->other_facets_index.size());
			this->reArea_FilterFacets(Tuqi_Fact, this->other_facets_index);
		}
		if (_in)
		{
			this->tuqi_facets_index = this->Get_facets(Tuqi_Fact, false);
			this->other_facets_index = this->Get_facets(Tuqi_Fact, true);
			LOGINFO("tuqi_facets_index.size() = %d", this->tuqi_facets_index.size());
			LOGINFO("other_facets_index.size() = %d", this->other_facets_index.size());
			this->Area_FilterFacets(Tuqi_Fact, this->tuqi_facets_index);
		}
	}

	this->tuqiLines_vec.clear();
	// 挑出所有含有突起边界的三角面片 生成突起线段集合
	for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
	{
		stl_facet& facet = this->stl.facet_start[i];
		if (TriangleMesh::fact_is_mark(facet.extra, Tuqi_Fact)) //挑出所有命中的面片
		{
			int facetNear_Num = 0;
			int Comedge_facetIndex = -1;
			for (int j = 0; j <= 2; j++)
			{
				int near_facet_index = this->stl.neighbors_start[i].neighbor[j];
				stl_facet& near_facet = this->stl.facet_start[near_facet_index];
				if (TriangleMesh::fact_is_mark(near_facet.extra, Tuqi_Fact))
					facetNear_Num++;
				else
					Comedge_facetIndex = near_facet_index;
			}

			if (facetNear_Num == 2) // 突起边缘三角面片
			{
				Linef3 comedge = this->GetFacets_OneEdge(facet, this->stl.facet_start[Comedge_facetIndex]);
				this->tuqiLines_vec.push_back(comedge);
			}
		}
	}

	// 扩展
	//for (size_t i = 0; i < 5; i++)
	//	this->ExpandMarkFacets(Tuqi_Fact);

	LOGINFO("End tuqi_FilterFacets! Times = %d !!!!!!!!!!!!!!!", GetTickCount() - timecount);

	return true;
}

//针对支架的处理流程，突起、边界和网格区域
bool TriangleMesh::make_tuqiLoops(
	size_t  spted_volume_id,
	int threshold, 
	double tuqiGridDistance, 
	double tuqiDistance, 
	double tuqi_width, 
	double min_length,
	double resolution,
	bool donot_makepoints
)
{
	DWORD timecount = GetTickCount();
	BoundLines_vec tuqiLines_temp = this->tuqiLines_vec;
	if (tuqiLines_temp.size() == 0)
		return false;

	// 数据初始化
	this->tuqiLoops.clear();
	this->extuqiLoops.clear();
	this->tuqiPoints.clear();
	this->tuqiLoops_vec.clear();
	// 突起成环处理
	while (tuqiLines_temp.size())
	{
		Polygon temp;
		if (this->find_onetuqiLoop(tuqiLines_temp, temp))
		{
			if (temp.points.size() > threshold)//抛弃顶点个数小于阈值的边缘环
			{
				this->tuqiLoops.push_back(temp);
				//LOGINFO("发现突起环 Polygon Points = %d", temp.points.size());
			}
		}
	}
	//LOGINFO("发现环 tuqiLoops_vec size = [%d]", this->tuqiLoops.size());

	//参数放大处理
	tuqiGridDistance = scale_(tuqiGridDistance);
	tuqiDistance = scale_(tuqiDistance);
	tuqi_width = scale_(tuqi_width);
	min_length = scale_(min_length);

	//SVG svg("tuqiLoops.svg");
	//svg.draw(this->tuqiLoops, "red");
	//svg.Close();
	this->extuqiLoops = union_ex(this->tuqiLoops); //intersection_ex(this->tuqiLoops, this->boundLoops);
	//计算突起支撑区域的支撑点
	if (!donot_makepoints)
		this->make_tuqiPoints(spted_volume_id, tuqiDistance, tuqi_width, min_length, resolution);
	//突起区域外扩一定尺寸，确保支撑不会添加到突起附近
	this->extuqiLoops = offset_ex(this->extuqiLoops, tuqiGridDistance);

	//LOGINFO("make_tuqiLoops times = %d", GetTickCount() - timecount);

	return true;
}

bool TriangleMesh::make_tuqiPoints(
	size_t  spted_volume_id,
	double tuqiDistance, 
	double tuqi_width, 
	double min_length,
	double resolution
)
{
	DWORD timecount = GetTickCount();
	ExPolygons expp = this->extuqiLoops; //突起支撑区域拷贝副本
	LOGINFO("this->extuqiLoops size = [%d]", this->extuqiLoops.size());
	//突起支撑区域化简，把polygons变成expolygons，再化简成polygons，再变成expolygons
	//确保简化系数不小于0.05；
	if (resolution < 0.05)
		resolution = 0.1;
	Polygons pp;
	for (ExPolygons::const_iterator ex = expp.begin(); ex != expp.end(); ++ex)
		ex->simplify_p(scale_(resolution), &pp);

	ExPolygons expp_p = union_ex(pp);
	double max = tuqi_width * 3;
	double min = tuqi_width * 0.05;
	//突起区域计算中线
	LOGINFO("expp_p size = %d", expp_p.size());
	for (ExPolygons::iterator ex = expp_p.begin(); ex != expp_p.end(); ) {
		ThickPolylines CentralLines;
		LOGINFO("[%d/%d] medial_axis_tuqi begin max = %f min = %f min_length = %f", 
			ex - expp_p.begin(), expp_p.size(), max, min, min_length);
		ex->medial_axis_tuqi(max, min, min_length, &CentralLines);
		LOGINFO("[%d/%d] CentralLines size = %d", ex - expp_p.begin(), expp_p.size(), CentralLines.size());
		unsigned int PointCount = 0;
		for (int i = 0; i < CentralLines.size(); i++) {
			unsigned int Line_PointCount = this->make_PolylinePoints(
				"stripe", 
				CentralLines[i], 
				tuqiDistance, 
				this->get_tree_region_idx(spted_volume_id));
			PointCount += Line_PointCount;
		}
		LOGINFO("[%d/%d] tuqiPoints_temp_coll size = %d", ex - expp_p.begin(), expp_p.size(), PointCount);
		if (PointCount == 0) {
			ex = expp_p.erase(ex);
		} else {
			ex++;
		}
	}

	this->extuqiLoops.clear();
	this->extuqiLoops = expp_p;

	LOGINFO("make_tuqiPoints times = %d", GetTickCount() - timecount);

	return true;
}

// 在线段池中挑出一个闭环的圈
bool TriangleMesh::find_onetuqiLoop(BoundLines_vec& Orgin_Lines, Polygon& oneLoop_polygon)
{
	if (Orgin_Lines.size() == 0)
		return true;

	// 边界环
	BoundLines_vec oneLoopLine_vec;
	oneLoopLine_vec.clear();

	// 取一点
	oneLoopLine_vec.push_back(Orgin_Lines.back());
	Orgin_Lines.pop_back();

	for (int index = 0; index < Orgin_Lines.size(); index++)
	{
		if (oneLoopLine_vec.back().b == Orgin_Lines[index].a) // 找到下一条
		{
			oneLoopLine_vec.push_back(Orgin_Lines[index]);
			Orgin_Lines.erase(Orgin_Lines.begin() + index);
			index = -1; // 重新开始循环
		}

		// 成环
		if (oneLoopLine_vec.back().b == oneLoopLine_vec.front().a)
		{
			// 边界环数据
			BoundLoop oneLoop;
			oneLoop.clear();
			oneLoop.push_back(oneLoopLine_vec.front().a);

			oneLoop_polygon.points.reserve(oneLoopLine_vec.size());
			for (int i = 0; i < oneLoopLine_vec.size(); i++)
			{
				// 边界环数据
				oneLoop.push_back(oneLoopLine_vec[i].b);
				// 多边形顶点进行放大处理
				Point tempPoint;
				tempPoint.x = scale_(oneLoopLine_vec[i].a.x);
				tempPoint.y = scale_(oneLoopLine_vec[i].a.y);
				oneLoop_polygon.points.push_back(tempPoint);
			}

			this->tuqiLoops_vec.push_back(oneLoop);

			return true;
		}

	}

	// 未成环
	LOGINFO("发现未成环的直线段组 size = %d  直线段被抛弃", oneLoopLine_vec.size());
	return false;
}

// 点要经过放大处理
// 判断点是否在圈内
int TriangleMesh::PointIn_tuqiLoops(Slic3r::Pointf checkPoint)
{
	if (this->extuqiLoops.empty())
		return -1;

	// 数据放大
	Point tempPoint;
	tempPoint.x = scale_(checkPoint.x);
	tempPoint.y = scale_(checkPoint.y);

	for (int i = 0; i < this->extuqiLoops.size(); i++)
	{
		if (this->extuqiLoops[i].contains_b(tempPoint))
			return (i + 1);
	}
	
	return -1;
}


}