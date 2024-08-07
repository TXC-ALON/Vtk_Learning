#include "TriangleMesh.hpp"
#include "ClipperUtils.hpp"
#include "SVG.hpp"
#include <math.h>

namespace Slic3r {

/////////////////////////////////////////////////////////////////////////////////////////////////////////
///  悬垂点、悬垂线、悬垂面
////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
法向角度低于阈值的面片作为悬垂面，只针对被退化的面片，其他悬垂面片已经包含在支撑面中
悬垂面的支撑方式：
面片面积大于一个网格面积采用网格法，面积过小则在面片中心点添加支撑
	
*/
bool 
TriangleMesh::Mark_OverhangFacet(double angle_throld, double step)
{
	LOGINFO("start Mark_OverhangFacet!!!");
	DWORD timecount = GetTickCount();
	Polygons pp;
	pp.clear();

	step = scale_(step);

	for (int i = 0; i < this->stl.stats.number_of_facets; ++i) {
		stl_facet &facet = this->stl.facet_start[i];
		// 找出所有在角度阈值内被退化掉的面片
		if (TriangleMesh::fact_is_mark(facet.extra, Area_Fact) == true &&
			TriangleMesh::fact_is_mark(facet.extra, Bound_Fact) == false)
		{
			Polygon p;
			p.points.resize(3);
			p.points[0] = Point(facet.vertex[0].x / SCALING_FACTOR, facet.vertex[0].y / SCALING_FACTOR);
			p.points[1] = Point(facet.vertex[1].x / SCALING_FACTOR, facet.vertex[1].y / SCALING_FACTOR);
			p.points[2] = Point(facet.vertex[2].x / SCALING_FACTOR, facet.vertex[2].y / SCALING_FACTOR);
			p.make_counter_clockwise();  // do this after scaling, as winding order might change while doing that
			pp.push_back(p);
		}
	}

	LOGINFO("pp.size() = %d!!!", pp.size());

	ExPolygons OverhangExboundLoops = union_ex(pp, true);
	ExPolygons exLoops;
	exLoops.assign(this->exboundLoops.begin(), this->exboundLoops.end());

// 	SVG svg1("OverhangExboundLoops.svg");
// 	svg1.draw(OverhangExboundLoops, "red");
// 	svg1.Close();

	for (auto temp : OverhangExboundLoops)
	{
		if (temp.contour.bounding_box().size().x >= step || temp.contour.bounding_box().size().y >= step)
		{
			//LOGINFO("add a OverhangExboundLoop, area is %f", temp.area());
			exLoops.push_back(temp);
		}
	}
// 	SVG svg2("exboundLoops_begin.svg");
// 	svg2.draw(this->exboundLoops, "red");
// 	svg2.Close();

	this->exboundLoops.clear();
	this->exboundLoops = union_ex(exLoops);

// 	SVG svg3("exboundLoops_end.svg");
// 	svg3.draw(this->exboundLoops, "red");
// 	svg3.Close();

	//LOGINFO("end Mark_OverhangFacet!!!");
	return true;
}

/*
计算悬垂线，在生成悬垂面之后进行,
STL模型中悬吊线的几何属性总结如下：
1）分层方向(xoy平面)的法向矢量与悬吊线夹角小于临界值；
2）共享一个边的三角面片都是不需要添加支撑的面片；
3）共享一个边的三角面片的单位法矢量相加后指向下；
4) 悬吊线的两个顶点之一必须是三角面片的最低点；???不一定要是最低点
5）包含悬吊线的带支撑面片必须至少有一个面片法相向下。
*/
bool
TriangleMesh::Mark_OverhangLines(double angel_throld)
{
	LOGINFO("start Mark_OverhangLines!!!");
	DWORD timecount = GetTickCount();

	for (int i = 0; i < this->stl.stats.number_of_facets; ++i) {
		stl_facet &facet = this->stl.facet_start[i];

		if (TriangleMesh::fact_is_mark(facet.extra, Area_Fact) == false)
			continue;
		if (facet.normal.z >= -EPSILON)
			continue;

		for (int j = 0; j < 3; j++)	{
			int neighbor_index = this->stl.neighbors_start[i].neighbor[j];
			stl_facet &neighbor_facet = this->stl.facet_start[neighbor_index];

			//该面片是支撑面
			if (TriangleMesh::fact_is_mark(facet.extra, Area_Fact) == false)
				continue;

			float z = facet.normal.z + neighbor_facet.normal.z;
			if (z > -EPSILON)//两个共边的面片法相相加不是向下，则其交线不可能是悬垂线
				continue;
			if (facet.normal.z >= -EPSILON && neighbor_facet.normal.z >= -EPSILON)//包含悬吊线的带支撑面片必须至少有一个面片法相向下。
				continue;
								
			Linef3 com_edge = this->GetFacets_OneEdge(facet, neighbor_facet);

// 			//悬吊线的两个顶点之一必须是三角面片的最低点
// 			float min_z = com_edge.a.z < com_edge.b.z ? com_edge.a.z : com_edge.b.z;
// 			stl_vertex facet_min = facet.vertex[this->get_zmin_vertex_of_facet(facet)] ;
// 			stl_vertex neighbor_facet_min = neighbor_facet.vertex[this->get_zmin_vertex_of_facet(neighbor_facet)];
// 
// 			if (facet_min.z != min_z || neighbor_facet_min.z != min_z)//判断最低点是否重合
// 			{
// // 				LOGINFO("facet_min.z != min_z || neighbor_facet_min.z != min_z");
// 				continue;
// 			}

			//分层方向(xoy平面)的法向矢量与悬吊线夹角小于临界值	
			//两面片法向量夹角大于一定角度，10度是否合适？？？
			if (this->get_angle_to_xoy(com_edge) < angel_throld && this->facet_angle(facet, neighbor_facet) > 10.0 / 180 * PI) 
			{
				this->OverhangLines.push_back(com_edge);
				// 悬垂线标记
				TriangleMesh::fact_set_mark(facet.extra, SSLine_Fact);
			}
		}
	}
	
	LOGINFO("this->OverhangLines.size() = %d", this->OverhangLines.size());
	LOGINFO("Mark_OverhangLines times =%d", GetTickCount() - timecount);

	return true;
}

//获取三角面片上某一条边与xoy平面的夹角(rad)
double
TriangleMesh::get_angle_to_xoy(const Linef3 edge)
{
// 	LOGINFO("start get_angle_to_xoy!!!");
	Pointf3 edge_normal = edge.a - edge.b;
	double z = fabs(edge_normal.z) / sqrt(edge_normal.x*edge_normal.x + edge_normal.y*edge_normal.y + edge_normal.z*edge_normal.z);
// 	LOGINFO("edge angle = %f", asin(z) / PI * 180);
	return asin(z) / PI * 180;
}

//计算悬垂线上的支撑点
bool
TriangleMesh::Make_OverhangLinesPoints(size_t  spted_volume_id, double overhang_distance)
{
	LOGINFO("start Make_OverhangLinesPoints!!!");
	DWORD timecount = GetTickCount();

	overhang_distance = scale_(overhang_distance);
	BoundLines_vec OverhangLines_temp = this->OverhangLines;     // 悬垂线集合拷贝副本
	if (OverhangLines_temp.size() == 0)
		return false;

	while (OverhangLines_temp.size())
	{
		Polyline temp;
		if (this->find_onePolyline(OverhangLines_temp, temp))
			if (temp.length() > overhang_distance)
				this->Overhang_polylines.push_back(temp);
	}

// 	SVG svg1("Overhang_polylines.svg");
// 	svg1.draw(this->Overhang_polylines);
// 	svg1.Close();

	for (int i = 0; i < this->Overhang_polylines.size(); i++)
	{
		if (this->Overhang_polylines.at(i).length() < overhang_distance)
			continue;
		this->make_PolylinePoints("overhangline", Overhang_polylines[i], overhang_distance, this->get_tree_region_idx(spted_volume_id));
	}

	LOGINFO("this->OverhangLinesPoints.size() = %d", this->OverhangLinesPoints.size());
	LOGINFO("Make_OverhangLinesPoints times =%d", GetTickCount() - timecount);

}

//将可能相连的悬垂线连接起来
bool
TriangleMesh::find_onePolyline(BoundLines_vec& Orgin_Lines, Polyline& onePolyline)
{
	if (Orgin_Lines.size() == 0)
		return true;

	if (Orgin_Lines.size() == 1)
	{
		onePolyline.points.reserve(2);
		Point temppoint;
		temppoint.x = scale_(Orgin_Lines[0].a.x);
		temppoint.y = scale_(Orgin_Lines[0].a.y);
		onePolyline.points.push_back(temppoint);

		temppoint.x = scale_(Orgin_Lines[0].b.x);
		temppoint.y = scale_(Orgin_Lines[0].b.y);
		onePolyline.points.push_back(temppoint);

		Orgin_Lines.pop_back();
		return true;
	}

	BoundLines_vec temp;
	temp.clear();

	temp.push_back(Orgin_Lines.back());
	Orgin_Lines.pop_back();

	for (int index = 0; index < Orgin_Lines.size(); index++)
	{
		if (temp.back().b == Orgin_Lines[index].a) // 找到下一条
		{
			temp.push_back(Orgin_Lines[index]);
			Orgin_Lines.erase(Orgin_Lines.begin() + index);
			index = -1; // 重新开始循环
		}
		else if (temp.back().b == Orgin_Lines[index].b)
		{
			Orgin_Lines[index].reverse();
			temp.push_back(Orgin_Lines[index]);
			Orgin_Lines.erase(Orgin_Lines.begin() + index);
			index = -1; // 重新开始循环
		}
	}
	//遍历完成，生成polyline
	onePolyline.points.reserve(temp.size());
	Point temppoint;
	temppoint.x = scale_(temp[0].a.x);
	temppoint.y = scale_(temp[0].a.y);
	onePolyline.points.push_back(temppoint);
	for (int i = 0; i < temp.size(); i++)
	{
		temppoint.x = scale_(temp[i].b.x);
		temppoint.y = scale_(temp[i].b.y);
		onePolyline.points.push_back(temppoint);
	}
// 	LOGINFO("发现多段线 size = %d", onePolyline.points.size());

	return true;
}

/*
计算悬垂点，在生成悬垂面和悬垂线之后进行
在STL模型中悬吊点的的几何属性总结如下：
1）若该点为悬吊点，则该点所在的三角面片中其他两个顶点z坐标值大于该悬吊点；
2）若该点是悬吊点，则所在边不为悬吊边；
3）若该点是悬吊点，则所在面不是悬吊面；
4）判断该点向下延伸与零件实体相交的交点个数或相交的第一个三角面片的法矢量，
   若法矢量向上，或者交点数是偶数，则该点是悬吊点；
5) 包含该最低点的所有三角面片法矢的z 向分量必须小于0，即法矢必须向下；
*/

bool
TriangleMesh::Mark_OverhangPoints(size_t  spted_volume_id, double angle_threshold)
{
	LOGINFO("start Mark_OverhangPoints!!!");
	DWORD timecount = GetTickCount();

	this->OverhangPoints.clear();
	for (int i = 0; i < this->stl.stats.number_of_facets; ++i) 
	{
		stl_facet &facet = this->stl.facet_start[i];

		if (TriangleMesh::fact_is_mark(facet.extra, SSFace_Fact) ||
			TriangleMesh::fact_is_mark(facet.extra, Tuqi_Fact)) //已经包含悬垂点的面片不再计算，花纹面片也不计算悬垂点
			continue;

		if (facet.normal.z >= 0)//三角面片法矢的z 向分量必须小于0
			continue;

		int lowest_index = this->get_zmin_vertex_of_facet(facet);
		stl_vertex lowest_vertex = facet.vertex[lowest_index];
		int next_index = lowest_index == 2 ? 0 : lowest_index + 1;
		stl_vertex next_vertex = facet.vertex[next_index];

		bool isoverhang = true;

		//寻找包含该最低点的所有面片
		std::vector<int> neighbor_indexs;
		neighbor_indexs.clear();
		
		int current_index = i;
		while (true)
		{
			//找到下一个面片
			int next_facet_index = this->index_of_neighbor_with_same_vertex(current_index, lowest_vertex, next_vertex);
// 			LOGINFO("i = %d, next_facet_index = %d", i, next_facet_index);
			if (next_facet_index < 0)
			{
// 				LOGINFO("cannot find next facet!!!");
				isoverhang = false;//没有找到围绕该点的一圈面片
				break;
			}

			//防止进入死循环
			if (find(neighbor_indexs.begin(), neighbor_indexs.end(), next_facet_index) != neighbor_indexs.end())
			{
				LOGINFO("find next facet ERROR!!!");
				isoverhang = false;//没有找到围绕该点的一圈面片
				break;
			}

			if (next_facet_index == i)
			{
// 				LOGINFO("find all facet of this vertex, neighbor_indexs.size = %d!!!", neighbor_indexs.size());
				break;
			}				
			neighbor_indexs.push_back(next_facet_index);

			//找到下一个顶点
			stl_facet &neighbor_facet = this->stl.facet_start[next_facet_index];
			for (int n = 0; n < 3; n++)
			{
				stl_vertex vertex_temp = neighbor_facet.vertex[n];

				//如果是另外两个共线顶点，退出当前循环，继续下一个顶点
// 				if (vertex_temp.x == lowest_vertex.x && vertex_temp.y == lowest_vertex.y && vertex_temp.z == lowest_vertex.z)
				if (std::abs(vertex_temp.x - lowest_vertex.x) < EPSILON_1 &&
					std::abs(vertex_temp.y - lowest_vertex.y) < EPSILON_1 &&
					std::abs(vertex_temp.z - lowest_vertex.z) < EPSILON_1)
				{
// 					LOGINFO("find lowest_vertex, vertex_temp.x = %f, vertex_temp.y = %f, vertex_temp.z = %f", vertex_temp.x, vertex_temp.y, vertex_temp.z);
					continue;
				}
// 				else if (vertex_temp.x == next_vertex.x && vertex_temp.y == next_vertex.y && vertex_temp.z == next_vertex.z)
				else if (std::abs(vertex_temp.x - next_vertex.x) < EPSILON_1 &&
					std::abs(vertex_temp.y - next_vertex.y) < EPSILON_1 &&
					std::abs(vertex_temp.z - next_vertex.z) < EPSILON_1)
				{
// 					LOGINFO("find other_vertex, vertex_temp.x = %f, vertex_temp.y = %f, vertex_temp.z = %f", vertex_temp.x, vertex_temp.y, vertex_temp.z);
					continue;
				}
				else
				{
// 					LOGINFO("find next_vertex, vertex_temp.x = %f, vertex_temp.y = %f, vertex_temp.z = %f", vertex_temp.x, vertex_temp.y, vertex_temp.z);
					next_vertex.x = vertex_temp.x;
					next_vertex.y = vertex_temp.y;
					next_vertex.z = vertex_temp.z;
					break;
				}
			}
			current_index = next_facet_index;
		}

		if (!isoverhang)//没有找到围绕该点的一圈面片
			continue;

		//判断该点所在的三角面片中其他两个顶点z坐标值大于该悬吊点
		for (int j = 0; j < neighbor_indexs.size(); j++)
		{
			stl_facet &neighbor_facet = this->stl.facet_start[neighbor_indexs.at(j)];

			if (TriangleMesh::fact_is_mark(facet.extra, SSFace_Fact) ||
				TriangleMesh::fact_is_mark(facet.extra, Tuqi_Fact))
			{
				isoverhang = false;
				break;
			}

			if (neighbor_facet.normal.z >= 0)//三角面片法矢的z 向分量必须小于0
			{
				isoverhang = false;
				break;
			}

			if (!this->facet_contain_vertex(neighbor_facet, lowest_vertex))//邻接面片是否包含最低点
			{
				isoverhang = false;
				break;
			}

			if (!this->is_zmin_vertex_of_facet(neighbor_facet, lowest_vertex))//该最低点是否也是邻接面片的最低点
			{
				isoverhang = false;
				break;
			}

			//滤除噪声点
			//共该顶点的所有三角面片，求得包含该点的所有边，求出边与x－y 平面的夹角，
			//如小于设定值，则该点可以自支撑，需将该点从悬吊点中去除。
			for (int k = 0; k < 3; k++)
			{
				stl_vertex vertex = neighbor_facet.vertex[k];
				if (lowest_vertex.x == vertex.x && lowest_vertex.y == vertex.y && lowest_vertex.z == vertex.z)
					continue;
				Linef3 com_edge;
				com_edge.a.x = lowest_vertex.x;
				com_edge.a.y = lowest_vertex.y;
				com_edge.a.z = lowest_vertex.z;
				com_edge.b.x = vertex.x;
				com_edge.b.y = vertex.y;
				com_edge.b.z = vertex.z;
				if (this->get_angle_to_xoy(com_edge) < angle_threshold) //分层方向(xoy平面)的法向矢量与悬吊线夹角小于临界值
				{
					isoverhang = false;
					break;
				}
			}
		}

		if (isoverhang)
		{
			TriangleMesh::fact_set_mark(facet.extra, SSFace_Fact);
			for (int j = 0; j < neighbor_indexs.size(); j++)
			{
				stl_facet &neighbor_facet = this->stl.facet_start[neighbor_indexs.at(j)];
				TriangleMesh::fact_set_mark(neighbor_facet.extra, SSFace_Fact);
			}

			Pointf temppoint;
			temppoint.x = lowest_vertex.x;
			temppoint.y = lowest_vertex.y;
			SPT_pointf spt_pf(temppoint, 0.0, this->get_tree_region_idx(spted_volume_id));
			this->OverhangPoints.push_back(spt_pf);
		}
	}

	LOGINFO("this->OverhangPoints.size() = %d", this->OverhangPoints.size());
	LOGINFO("Mark_OverhangPoints times =%d", GetTickCount() - timecount);

	return true;
}

bool
TriangleMesh::is_zmin_vertex_of_facet(const stl_facet facet, const stl_vertex vertex)
{
	//顶点是否是面片上的z轴最低点
	stl_vertex zmin_vertex = facet.vertex[this->get_zmin_vertex_of_facet(facet)];
// 	LOGINFO("zmin_vertex.x =%f, zmin_vertex.y =%f, zmin_vertex.z =%f", zmin_vertex.x, zmin_vertex.y, zmin_vertex.z);
// 	LOGINFO("vertex.x =%f, vertex.y =%f, vertex.z =%f", vertex.x, vertex.y, vertex.z);

// 	if (zmin_vertex.x == vertex.x && zmin_vertex.y == vertex.y && zmin_vertex.z == vertex.z)
	if (std::abs(zmin_vertex.x - vertex.x) < EPSILON_1 &&
		std::abs(zmin_vertex.y - vertex.y) < EPSILON_1 &&
		std::abs(zmin_vertex.z - vertex.z) < EPSILON_1)
		return true;

	return false;
}

//面片是否包含某个顶点
bool 
TriangleMesh::facet_contain_vertex(const stl_facet facet, const stl_vertex vertex)
{
	for (int i = 0; i < 3; i++)
	if (std::abs(facet.vertex[i].x - vertex.x) < EPSILON_1 &&
		std::abs(facet.vertex[i].y - vertex.y) < EPSILON_1 &&
		std::abs(facet.vertex[i].z - vertex.z) < EPSILON_1)
// 		if (facet.vertex[i].x == vertex.x && facet.vertex[i].y == vertex.y && facet.vertex[i].z == vertex.z)
			return true;

	return false;
}

//获取该面片上z值最小的顶点
int 
TriangleMesh::get_zmin_vertex_of_facet(const stl_facet facet)
{
	int result = 0;
	double z_min = facet.vertex[result].z;
	for (int i = 1; i < 3; i++)
		if (facet.vertex[i].z < z_min)
		{
			z_min = facet.vertex[i].z;
			result = i;
		}

	return result;
}

//找到共两个顶点的邻接面片
int 
TriangleMesh::index_of_neighbor_with_same_vertex(const int index, const stl_vertex vertex1, const stl_vertex vertex2)
{
	stl_facet &facet = this->stl.facet_start[index];
	if (!this->facet_contain_vertex(facet, vertex1) || !this->facet_contain_vertex(facet, vertex2))
		return -1;

	for (int i = 0; i < 3; i++)
	{
		int near_facet_index = this->stl.neighbors_start[index].neighbor[i];
		stl_facet &neighbor_facet = this->stl.facet_start[near_facet_index];
		if (this->facet_contain_vertex(neighbor_facet, vertex1) && this->facet_contain_vertex(neighbor_facet, vertex2))
			return near_facet_index;
	}

	return -1;
}

}