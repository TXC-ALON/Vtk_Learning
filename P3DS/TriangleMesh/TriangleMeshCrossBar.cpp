#include "TriangleMesh.hpp"
#include <float.h>
#include "SVG.hpp"


namespace Slic3r {


//自动计算横杆点
	bool TriangleMesh::make_auto_crossbar(
		double distance,                         // 主横杆水平切片层间距
		double minLength,                     // 横杆最小长度
		double minDistanceToRpd,       //  横杆距离支架最短距离
		double Crossbar_raduis,             // 横杆碰撞计算半径
		double slice_assist_dis,               //  辅助横杆采样点间距
	    double bound_assist_dis,           //   辅助横杆采样点间距
		int bound_constrat_loop           //   辅助横杆边界内缩量
)
{
	DWORD timecount = GetTickCount();
	LOGINFO("auto_crossbar distance = %f", distance);
	LOGINFO("auto_crossbar minLength = %f", minLength);
	LOGINFO("auto_crossbar minDistanceToRpd = %f", minDistanceToRpd);
	LOGINFO("auto_crossbar Crossbar_raduis = %f", Crossbar_raduis);
	LOGINFO("auto_crossbar slice_assist_dis = %f", slice_assist_dis);
	LOGINFO("auto_crossbar bound_assist_dis = %f", bound_assist_dis);
	LOGINFO("auto_crossbar bound_constrat_loop = %d", bound_constrat_loop);
	this->ShowLoops.clear();
	this->Crossbar_Points.clear(); // 横杆点的结果容器 
	this->Assit_Crossbar_Points.clear();
	// 重置标记
	ResetMark_RPDStrong();
	ResetMark_RPDWeak();
	// 找出自动横杆区域的面片
	std::vector<int> facets_index = this->Get_auto_crossbar_facets();
	// 找出自动横杆区域的边界线
	BoundSegments_loops bound_loops = this->Get_auto_crossbar_boundloops();
	// 边界为0 直接退出
	if (bound_loops.size() == 0) return false;
	// 输出显示
	if (false) {
		for (int i = 0; i < bound_loops.size(); i++)
		{
			BoundLoop sampling_boundloop = this->Sampling_SegmentsLoop2(bound_loops[i], 1.0);
			ShowLoops.push_back(sampling_boundloop);
		}
	}

	this->set_step_length(1, 1, 1);
	this->init_hash_facets();
	
	//通过边界计算横杆点
	std::map<double, PlaneZ_CrossBars> zBars_map_orgin;
	BoundingBoxf3 box = GetBoundingBox(bound_loops);
	const double zmin = box.min.z;
	const double zmax = box.max.z;
	LOGINFO("zmin = [%f], zmax = [%f] bound_loops.size() = %d", zmin, zmax, bound_loops.size());
	if (zmax <= zmin)
	{
		return false;
	}
	// 从高到低切片 获取主横杆 
	double slice_z = zmax;
	const double z_step = (zmax - zmin) / 50;
	if (distance < z_step) distance = z_step;
	while (slice_z >= zmin)
	{
		//过滤主横杆
		PlaneZ_CrossBars zCrossBars = this->Get_PlaneZ_CrossBars_ByZ(bound_loops, slice_z);
		CrossbarPointPairs Filter_Major_CrossBars;
		for (int i = 0; i < zCrossBars.Major_CrossBars.size(); i++)
		{
			CrossbarPointPair cb_pointpair = zCrossBars.Major_CrossBars[i];
			if (this->Is_CrossBar_CanUse(cb_pointpair, facets_index, minLength, 999, minDistanceToRpd, Crossbar_raduis))
			{
				/*
				// 轻微的倾斜横杆
				if (slice_z == zmax)
				{
					//cb_pointpair.first.z += 0.5;
					cb_pointpair.second.z -= 1.0;
				}
				else
				{
					cb_pointpair.first.z += 0.5;
					cb_pointpair.second.z -= 0.5;
				}
				*/

				Filter_Major_CrossBars.push_back(cb_pointpair);
				LOGINFO("PlaneZ[%f] find a pair of points in cp_Pairs, a=[%s], b=[%s]",
					slice_z,
					cb_pointpair.first.dump_perl().c_str(),
					cb_pointpair.second.dump_perl().c_str());
			}
		}
		if (Filter_Major_CrossBars.size())
		{
			zBars_map_orgin[slice_z] = zCrossBars;
			zBars_map_orgin[slice_z].Major_CrossBars = Filter_Major_CrossBars;
		}					
		slice_z -= z_step;
	}
	// 按distance间距来取横杆
	LOGINFO("zBars_map_orgin size = %d", zBars_map_orgin.size());
	std::map<double, PlaneZ_CrossBars> zBars_map;
	double Range_Zmax = zmax;
 	while (Range_Zmax > zmin) // 某一个区域循环
	{	
		//  
		double Range_Zmin = Range_Zmax - distance;
		//LOGINFO("Range_Zmax(%f), Range_Zmin(%f)", Range_Zmax, Range_Zmin);
		double Max_major_Len = DBL_MIN;
		double Min_z_dis = DBL_MAX;
		double Select_plane_z = 0;
		//
		std::map<double, PlaneZ_CrossBars>::iterator map_It = zBars_map_orgin.begin();
		for (; map_It != zBars_map_orgin.end(); map_It++)
		{
			double plane_z = map_It->first;
			if (plane_z > Range_Zmax || plane_z <= Range_Zmin)
				continue;
			if (Range_Zmax >= zmax || Range_Zmin <= zmin) // 第一层和最后一层
			{
				for (int i = 0; i < map_It->second.Major_CrossBars.size(); i++)
				{
					CrossbarPointPair cb_pointpair = map_It->second.Major_CrossBars.at(i);
					double major_len = cb_pointpair.first.distance_to(cb_pointpair.second);
					if (major_len > Max_major_Len)
					{
						Max_major_Len = major_len;
						Select_plane_z = plane_z;
					}
				}
			}
			else if(zmax - zmin > 2.0 * distance) // 中间层  去中间
			{
				double z_dis = std::abs((Range_Zmax + Range_Zmin) / 2 - plane_z);
				if (z_dis < z_step && Min_z_dis > z_dis) {
					Min_z_dis = z_dis;
					Select_plane_z = plane_z;
				}
			}
		}
		// 下一个迭代 数据准备
		LOGINFO("Range_Z [%f,%f], Select_plane_z = %f", Range_Zmin, Range_Zmax, Select_plane_z);
		Range_Zmax = Range_Zmin;
		//
		zBars_map[Select_plane_z] = zBars_map_orgin[Select_plane_z];		
		if (zBars_map[Select_plane_z].Major_CrossBars.size())
		{
			this->Crossbar_Points.insert(
				this->Crossbar_Points.end(),
				zBars_map[Select_plane_z].Major_CrossBars.begin(),
				zBars_map[Select_plane_z].Major_CrossBars.end());
		}
	}
	LOGINFO("zBars_map size = %d", zBars_map.size());
	
	std::map<double, PlaneZ_CrossBars>::iterator map_It = zBars_map.begin();
	for (; map_It != zBars_map.end(); map_It++)
	{
		double plane_z = map_It->first;
		//按水平切片的顶点收集辅助横杆点
		this->Add_ZSliceSegs(facets_index, zBars_map[plane_z]);	
		//提取水平辅助横杆
		if (this->General_Slice_Assist_CrossBar(zBars_map[plane_z], slice_assist_dis))
		{
			// 过滤结果集
			CrossbarPointPairs Filter_Assist_CrossBars;
			// 取第一条主横杆
			CrossbarPointPair major_pointpair = zBars_map[plane_z].Major_CrossBars.front();
			double maxLength = (major_pointpair.first.distance_to(major_pointpair.second)) / 2;
			// 按最小长度，离支架最短距离，碰撞检测进行过滤
			for (int i = 0; i < map_It->second.Slice_Assist_CrossBars.size(); i++)
			{
				CrossbarPointPair cb_pointpair = map_It->second.Slice_Assist_CrossBars.at(i);
				if (this->Is_CrossBar_CanUse(cb_pointpair, facets_index, minLength, maxLength, minDistanceToRpd, Crossbar_raduis))
				{
					Filter_Assist_CrossBars.push_back(cb_pointpair);
					LOGINFO("PlaneZ[%f] Slice_Assist_CrossBars find a pair of points in cp_Pairs, a=[%s], b=[%s]",
						map_It->first,
						cb_pointpair.first.dump_perl().c_str(),
						cb_pointpair.second.dump_perl().c_str());
				}
			}

			// 落地结果 
			map_It->second.Slice_Assist_CrossBars = Filter_Assist_CrossBars;
			if (map_It->second.Slice_Assist_CrossBars.size())
			{
				this->Assit_Crossbar_Points.insert(
					this->Assit_Crossbar_Points.end(),
					map_It->second.Slice_Assist_CrossBars.begin(),
					map_It->second.Slice_Assist_CrossBars.end());
			}
		}
	}
	LOGINFO("this->Assit_Crossbar_Points.size() = %d", this->Assit_Crossbar_Points.size());	
	
	
	// 重置标记
	ResetMark_RPDStrong();
	ResetMark_RPDWeak();
	// 重新标记
	this->Get_special_crossbar_facets(facets_index, bound_constrat_loop);
	bound_loops = this->Get_auto_crossbar_boundloops();
	//提取辅助横杆
	this->Add_SepcialSegLoops(bound_loops, zBars_map, bound_assist_dis);
	for (map_It = zBars_map.begin(); map_It != zBars_map.end(); map_It++)
	{
		// 过滤结果集
		CrossbarPointPairs Filter_Assist_CrossBars;
		// 取第一条主横杆
		double maxLength = 999;
		if (map_It->second.Major_CrossBars.size())
		{
			CrossbarPointPair major_pointpair = map_It->second.Major_CrossBars.front();
			maxLength = (major_pointpair.first.distance_to(major_pointpair.second)) / 2;
		}
		for (int i = 0; i < map_It->second.Boundary_Assist_CrossBars.size(); i++)
		{
			CrossbarPointPair cb_pointpair = map_It->second.Boundary_Assist_CrossBars[i];
			if (this->Is_CrossBar_CanUse(cb_pointpair, facets_index, minLength, maxLength, minDistanceToRpd, Crossbar_raduis))
			{
				Filter_Assist_CrossBars.push_back(cb_pointpair);
				LOGINFO("PlaneZ[%f] Boundary_Assist_CrossBars find a pair of points in cp_Pairs, a=[%s], b=[%s]",
					map_It->first,
					cb_pointpair.first.dump_perl().c_str(),
					cb_pointpair.second.dump_perl().c_str());
			}
		}
		map_It->second.Boundary_Assist_CrossBars = Filter_Assist_CrossBars;
		if (map_It->second.Boundary_Assist_CrossBars.size())
		{
			this->Assit_Crossbar_Points.insert(
				this->Assit_Crossbar_Points.end(),
				map_It->second.Boundary_Assist_CrossBars.begin(),
				map_It->second.Boundary_Assist_CrossBars.end());
		}
	}
	// 重置标记
	ResetMark_RPDStrong();
	ResetMark_RPDWeak();
	LOGINFO("this->Assit_Crossbar_Points.size() = %d", this->Assit_Crossbar_Points.size());
	

	return true;
}

bool TriangleMesh::Is_CrossBar_CanUse(
	CrossbarPointPair cb_pointpair,
	std::vector<int> facets_index,
	double minLength,
	double maxLength,
	double minDistanceToRpd,
	double Crossbar_raduis
)
{
	// 剔除长度过短的横杆
	double dis_cb = cb_pointpair.first.distance_to(cb_pointpair.second);
	if (dis_cb < minLength)
	{
		LOGINFO("CrossbarPointPair Len(%f) < minLength(%f)", dis_cb, minLength);
		return false;
	}
	if (dis_cb > maxLength)
	{
		LOGINFO("CrossbarPointPair Len(%f) > maxLength(%f)", dis_cb, maxLength);
		return false;
	}
	
	// 剔除到支架距离过短的横杆
	Pointf3 cb_midPoint((cb_pointpair.first.x + cb_pointpair.second.x) / 2,
		(cb_pointpair.first.y + cb_pointpair.second.y) / 2,
		(cb_pointpair.first.z + cb_pointpair.second.z) / 2);
	double dis_rpd = this->get_min_distance_point_to_facets(facets_index, cb_midPoint);
	if (dis_rpd < minDistanceToRpd)
	{
		LOGINFO("find a dis_rpd < minDistanceToRpd, dis_rpd = [%f], minDistanceToRpd = [%f] ", dis_rpd, minDistanceToRpd);
		return false;
	}
	
	// 剔除穿透支架的横杆
	if (this->is_intersect(cb_pointpair.first, cb_pointpair.second, Crossbar_raduis, 8))
	{
		LOGINFO("find a crossbar  is_intersect！cb_pointpair (%s)--(%s), Crossbar_raduis--%f, ringsize--%d", 
			cb_pointpair.first.dump_perl().c_str(), cb_pointpair.second.dump_perl().c_str(), Crossbar_raduis, 8);
		return false;
	}

	LOGINFO("CrossbarPointPair Is_CrossBar_CanUse Begin()~~~~~~~~~~~~~~");
	LOGINFO("CrossbarPointPair Len(%f) > minLength(%f)", dis_cb, minLength);
	LOGINFO("find a dis_rpd > minDistanceToRpd, dis_rpd = [%f], minDistanceToRpd = [%f] ",  dis_rpd,  minDistanceToRpd);
	LOGINFO("find a crossbar  is not intersect！cb_pointpair (%s)--(%s), Crossbar_raduis--%f, ringsize--%d",
		cb_pointpair.first.dump_perl().c_str(), cb_pointpair.second.dump_perl().c_str(), Crossbar_raduis, 8);
	LOGINFO("CrossbarPointPair Is_CrossBar_CanUse End()~~~~~~~~~~~~~~");

	return true;
}

//计算切片平面的辅助横杆
bool TriangleMesh::General_Slice_Assist_CrossBar(PlaneZ_CrossBars& _planeZ, double Point_distance)
{		
	if (_planeZ.Major_CrossBars.size() * _planeZ.Slice_loops.size() == 0) 
		return false;
	if (Point_distance < 1.0) 
		Point_distance = 1.0;
	// 取第一条主横杆
	CrossbarPointPair major_pointpair = _planeZ.Major_CrossBars.front();
	LOGINFO(" _planeZ.Slice_loops.size() = %d", _planeZ.Slice_loops.size());
	// 采样顶点集
	for (int i = 0; i < _planeZ.Slice_loops.size(); i++)
	{
		BoundSegments_loop one_loop = _planeZ.Slice_loops[i];
		// 采样顶点
		BoundLoop sampling_sliceloop = this->Sampling_SegmentsLoop2(one_loop, 1.0);
		ShowLoops.push_back(sampling_sliceloop); // 调试显出输出
		LOGINFO("Sampling_SegmentsLoop [%d] --> [%d]", one_loop.size(), sampling_sliceloop.size());
		double Count_dis = 0.0;
		for (int j = 0; j < sampling_sliceloop.size(); j++)
		{
			if (Count_dis < Point_distance)
			{
				LOGINFO("Count_dis [%f/%f]", Count_dis, Point_distance);
				Count_dis += 1.0;
				continue;
			}
			CrossbarPointPair assit_pointpair;
			if (Link_Assist_CrossBar(sampling_sliceloop[j], major_pointpair, assit_pointpair))
				_planeZ.Slice_Assist_CrossBars.push_back(assit_pointpair);
			Count_dis = 0;
		}
	}
	LOGINFO("_planeZ %f  Slice_Assist_CrossBars size = %d", _planeZ.slice_z, _planeZ.Slice_Assist_CrossBars.size());

	return _planeZ.Slice_Assist_CrossBars.size();
}


bool TriangleMesh::Add_SepcialSegLoops(
	BoundSegments_loops bound_loops,
	std::map<double, PlaneZ_CrossBars>& zBars,
	double Point_distance
	)
{
	const double step_distance = 1.0;
	// 距离保护
	if (Point_distance < step_distance)
		Point_distance = 1.0;
	// 裁剪边界线段
	BoundSegments_loops SepcialSegLoops_res = this->CutLoopsByFaceMark(bound_loops, RPDWeak_Fact);
	LOGINFO("CutLoopsByFaceMark [%d] --> [%d]", bound_loops.size(), SepcialSegLoops_res.size());
	// 简化
	for (int i = 0; i < SepcialSegLoops_res.size(); i++)
	{
		BoundSegments_loop one_loop = SepcialSegLoops_res[i];
		// 采样顶点2
		BoundLoop sampling_sliceloop = this->Sampling_SegmentsLoop2(one_loop, step_distance);
		ShowLoops.push_back(sampling_sliceloop); // 调试显示
		LOGINFO("sampling_sliceloop [%d] --> [%d]", one_loop.size(), sampling_sliceloop.size());
		double Count_dis = 0.0;
		for (int j = 0; j < sampling_sliceloop.size(); j++)
		{
			if (Count_dis < Point_distance)
			{
				LOGINFO("Count_dis [%f/%f]", Count_dis, Point_distance);
				Count_dis += 1.0;
				continue;
			}
			Pointf3 loop_point = sampling_sliceloop[j];
			// 找到距离最近的平面
			const double point_z = loop_point.z;
			double min_dis = DBL_MAX;
			double near_plane_z;
			CrossbarPointPair min_assit_pointpair;
			bool has_find_min_assit = false;
			std::map<double, PlaneZ_CrossBars>::iterator It_map = zBars.begin();
			for (;It_map != zBars.end();It_map++) {			
				double _z = It_map->first;
				if (It_map->second.Major_CrossBars.size() == 0) continue;
				for (int i = 0; i < It_map->second.Major_CrossBars.size(); i++)
				{
					CrossbarPointPair major_pointpair = It_map->second.Major_CrossBars[i];
					CrossbarPointPair assit_pointpair;
					if (Link_Assist_CrossBar(loop_point, major_pointpair, assit_pointpair))
					{
						double _dis = assit_pointpair.first.distance_to(assit_pointpair.second);
						if (min_dis > _dis)
						{
							min_dis = _dis;
							near_plane_z = _z;
							min_assit_pointpair = assit_pointpair;
							has_find_min_assit = true;
						}
					}
				}
			}
			if (has_find_min_assit) { // 找到了合适的连接
				zBars[near_plane_z].Boundary_Assist_CrossBars.push_back(min_assit_pointpair);
				LOGINFO("zBars[%f] Boundary_Assist_CrossBars size = %d", near_plane_z, zBars[near_plane_z].Boundary_Assist_CrossBars.size());
			}
			Count_dis = 0.0;
		}
	}

	return true;
}


bool
TriangleMesh::Link_Assist_CrossBar(
	Pointf3 start_p,
	CrossbarPointPair major_pointpair,
	CrossbarPointPair& assit_pointpair
)
{
	Linef3 major_line(major_pointpair.first, major_pointpair.second);
	Pointf3 major_line_mid = (major_line.b + major_line.a) / 2;
	assit_pointpair.first = start_p;
	Pointf3 foot_point;
	if (major_line.perpendicular_foot(start_p, foot_point))  // 求得垂足
	{	
		assit_pointpair.second =  3 * foot_point / 4 + major_line_mid / 4;
		return true;
	} 
	//else {  // 没求得垂足
	// Pointf3 major_line_ab = (major_line.b - major_line.a) / 10;
	//	Pointf3 major_line_a = major_line.a + major_line_ab;
	//	Pointf3 major_line_b = major_line.b - major_line_ab;
	//	double dis_1 = start_p.distance_to(major_line_a);
	//	double dis_2 = start_p.distance_to(major_line_b);

	//	if (dis_1 < dis_2)
	//		assit_pointpair.second = major_line_a;
	//	else
	//		assit_pointpair.second = major_line_b;
	//	return true
	//}

	return false;
}

//bool
//TriangleMesh::Link_Assist_CrossBar(
//	Pointf3 begin_p,
//	Pointf3 mid_p,
//	Pointf3 end_p,
//	CrossbarPointPair major_pointpair,
//	CrossbarPointPair& assit_pointpair
//)
//{
//	Pointf begin_2Dp = Pointf(begin_p.x, begin_p.y);
//	Pointf mid_2Dp = Pointf(mid_p.x, mid_p.y);
//	Pointf end_2Dp = Pointf(end_p.x, end_p.y);
//
//	Pointf vec_begin = mid_2Dp - begin_2Dp;
//	Pointf vec_end = mid_2Dp - end_2Dp;
//	vec_begin.normalize();
//	vec_end.normalize();
//	Pointf mid_begin = mid_2Dp + vec_begin;
//	Pointf mid_end = mid_2Dp + vec_end;
//	Pointf mid_other = (mid_begin + mid_end) / 2;
//	Pointf mid_project = (mid_other - mid_2Dp);
//	mid_project.normalize();
//	mid_project = 100 * mid_project;
//	Pointf mid_other_project = mid_2Dp - mid_project;
//	mid_project = mid_2Dp + mid_project;
//	
//	Line line_1(Point(scale_(mid_p.x), scale_(mid_p.y)), Point(scale_(mid_project.x), scale_(mid_project.y)));
//	Line line_11(Point(scale_(mid_p.x), scale_(mid_p.y)), Point(scale_(mid_other_project.x), scale_(mid_other_project.y)));
//	Line line_2(Point(scale_(major_pointpair.first.x), scale_(major_pointpair.first.y)), Point(scale_(major_pointpair.second.x), scale_(major_pointpair.second.y)));
//	Point insert_point;
//	Pointf3 Second_Point;
//	bool res = false;
//	if (line_2.intersection(line_1, &insert_point))
//	{
//		Second_Point = Pointf3(unscale(insert_point.x), unscale(insert_point.y), major_pointpair.first.z);
//		res = true;
//	}
//	else if (line_2.intersection(line_11, &insert_point))
//	{
//		Second_Point = Pointf3(unscale(insert_point.x), unscale(insert_point.y), major_pointpair.first.z);
//		res = true;
//	}
//
//	if (res)
//	{
//		CrossbarPointPair assit_pointpair_feet;
//		if (this->Link_Assist_CrossBar(mid_p, major_pointpair, assit_pointpair_feet))
//		{
//			Second_Point = (Second_Point + assit_pointpair_feet.second) / 2;
//		}
//		assit_pointpair.first = mid_p;
//		assit_pointpair.second = Second_Point;
//	}
//
//	return res;
//}



bool TriangleMesh::is_intersect(Pointf3 p1, Pointf3 p2, Pointf3& p3)//检测两点之间的连线是否与mesh有交点
{
	DWORD timecount = GetTickCount();

	//LOGINFO("p1= [%s], p2=[%s]", p1.dump_perl().c_str(), p2.dump_perl().c_str());

	Pointf3 p = p1 - p2;

	Vector3 from(p.x, p.y, p.z);
	from.normalize();
	Vector3 to(0, 0, 1);

	//计算旋转矩阵
	Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
	Eigen::Vector3d v_from(from.x, from.y, from.z);
	Eigen::Vector3d v_to(to.x, to.y, to.z);
	rotation_matrix = Eigen::Quaterniond::FromTwoVectors(v_from, v_to).toRotationMatrix();

	std::vector<XYZ_Index> grids = this->get_line_grids(Linef3(p1, p2));
	std::vector<int> facets = this->get_grids_facets(grids);
	//LOGINFO("GetSubMesh facets = [%d]", facets.size());
	TriangleMesh mesh = this->GetSubMesh(facets);
	//mesh.write_ascii("haha1.stl");
	mesh.rotate_self(rotation_matrix);
	mesh.set_step_length(1, 1, 1);
	mesh.init_hash_facets();
	//mesh.write_ascii("haha2.stl");

	Eigen::Vector3d p11(p1.x, p1.y, p1.z);
	Eigen::Vector3d p21(p2.x, p2.y, p2.z);

	Eigen::Vector3d p12 = rotation_matrix * p11;
	Eigen::Vector3d p22 = rotation_matrix * p21;

	Pointf3 p13 = Pointf3(p12(0), p12(1), p12(2));
	Pointf3 p23 = Pointf3(p22(0), p22(1), p22(2));

	Pointf3 pp(0, 0, 0);
	bool result = mesh.is_intersect_new(p13, p23, pp);

	if (result == true)
		p3 = mesh.rotate_pointf3(rotation_matrix.inverse(), pp);

	mesh.ClearMesh();

	//LOGINFO("is_intersect(Pointf3 p1, Pointf3 p2) times =%d", GetTickCount() - timecount);
	return result;
}

//检测两点之间的连线半径为r的范围内，是否与mesh有交点
bool TriangleMesh::is_intersect(Pointf3 p1, Pointf3 p2, double r, int ringSize)
{
	DWORD timecount = GetTickCount();

	LOGINFO("TriangleMesh::is_intersect p1= [%s], p2=[%s]", p1.dump_perl().c_str(), p2.dump_perl().c_str());

	Pointf3 p = p1 - p2;

	Vector3 from(p.x, p.y, p.z);
	from.normalize();
	Vector3 to(0, 0, 1);

	//计算旋转矩阵
	Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
	Eigen::Vector3d v_from(from.x, from.y, from.z);
	Eigen::Vector3d v_to(to.x, to.y, to.z);
	rotation_matrix = Eigen::Quaterniond::FromTwoVectors(v_from, v_to).toRotationMatrix();

	std::vector<XYZ_Index> grids = this->get_line_grids(Linef3(p1, p2));
	std::vector<int> facets = this->get_grids_facets(grids);
	LOGINFO("GetSubMesh facets = [%d]", facets.size());
	TriangleMesh mesh = this->GetSubMesh(facets);
	//mesh.write_ascii("haha1.stl");
	mesh.rotate_self(rotation_matrix);
	mesh.set_step_length(1, 1, 1);
	mesh.init_hash_facets();
	//mesh.write_ascii("haha2.stl");

	Eigen::Vector3d p11(p1.x, p1.y, p1.z);
	Eigen::Vector3d p21(p2.x, p2.y, p2.z);

	Eigen::Vector3d p12 = rotation_matrix * p11;
	Eigen::Vector3d p22 = rotation_matrix * p21;

	Pointf3 p13 = Pointf3(p12(0), p12(1), p12(2));
	Pointf3 p23 = Pointf3(p22(0), p22(1), p22(2));

	//生成一个圆上的点
	//Pointf3s tops;
	//Pointf3s downs;

	for (int i = 0; i < ringSize; i++)
	{
		double angle = 2 * PI / ringSize * i;
		LOGINFO("make a circle, i = [%d], angle = [%f]", i, angle);
		double x1 = p13.x + r * cos(angle);
		double y1 = p13.y + r * sin(angle);
		Pointf3 top(x1, y1, p13.z - 1.0);

		double x2 = p13.x + r * cos(angle);
		double y2 = p13.y + r * sin(angle);
		Pointf3 down(x2, y2, p23.z + 1.0);
		LOGINFO("TriangleMesh::is_intersect top= [%s], down=[%s]", top.dump_perl().c_str(), down.dump_perl().c_str());

		Pointf3 pp(0, 0, 0);
		if (mesh.is_intersect_new(top, down, pp))
		{
			mesh.ClearMesh();
			LOGINFO("TriangleMesh::is_intersect i = [%d]", i);
			return true;
		}
	}

	mesh.ClearMesh();

	//LOGINFO("is_intersect(Pointf3 p1, Pointf3 p2) times =%d", GetTickCount() - timecount);
	return false;
}

//P1, P2要有相同的x, y值，否则失效
bool TriangleMesh::is_intersect_new(Pointf3 p1, Pointf3 p2, Pointf3& p3)
{
	//LOGINFO("start is_intersect(TriangleMesh mesh, Pointf3 p1, Pointf3 p2)");

	bool result = true;
	assert((p1.x == p2.x) && (p1.y == p2.y));

	double x = p1.x, y = p1.y;
	double z1 = p1.z, z2 = p2.z;

	if (p1.z < p2.z)
		return false;

	assert(p1.z > p2.z);

	// 计算碰撞的三角面片
	std::vector<int> facet_ids = this->GetFacetIdxByZaxisFast(p1, p2);

	if (facet_ids.size() == 0)
	{
		//LOGINFO("not intersect!!");
		result = false;
	}
	else
	{
		// 记录碰撞信息
		MapCollsionInfo collsionZ_map;
		collsionZ_map.clear();
		for (int i = 0; i < facet_ids.size(); i++)
		{
			double collsion_z = this->GetPointZByZaxis(facet_ids[i], x, y);
			collsionZ_map[collsion_z] = facet_ids[i];
		}

		//LOGINFO("intersect point num = [%d]!!", collsionZ_map.size());
		MapCollsionInfo::iterator p_It = collsionZ_map.begin();
		int p1_index = 0, p2_index = 0;
		double z11 = DBL_MIN, z21 = DBL_MIN;
		while (p_It != collsionZ_map.end())
		{
			if (z1 > p_It->first)
			{
				p1_index++;
				z11 = p_It->first;
			}
			if (z2 > p_It->first)
			{
				p2_index++;
				z21 = p_It->first;
			}

			if (z1 < p_It->first && z2 < p_It->first)
				break;

			p_It++;
		}

		MapCollsionInfo().swap(collsionZ_map);

		if (p1_index == p2_index)
			result = false;
		else
			p3 = Pointf3(x, y, z11);

	}
	//LOGINFO("end is_intersect(TriangleMesh mesh, Pointf3 p1, Pointf3 p2)");
	std::vector<int>().swap(facet_ids);

	return result;

}

Pointf3 TriangleMesh::rotate_pointf3(Eigen::Matrix3d rotation_matrix, Pointf3 p)
{
	Eigen::Vector3d p1(p.x, p.y, p.z);
	Eigen::Vector3d p2 = rotation_matrix * p1;

	return Pointf3(p2(0), p2(1), p2(2));
}


// 获取自动横杆区域的面片
std::vector<int> TriangleMesh::Get_auto_crossbar_facets()
{
	const int Expand_WeakCount = 10;
	const int Expand_StrongCount = 1000;
	DWORD timecount = GetTickCount();
	// 加强区域
	this->MarkFacetsByRPDPart(Upper_Face, MajorConnector_Part, -1, RPDStrong_Fact, false);
	this->MarkFacetsByRPDPart(Upper_Face, StippledWax_Part, -1, RPDStrong_Fact, false);
	this->MarkFacetsByRPDPart(Upper_Face, LingualBar_Part, -1, RPDStrong_Fact, false);
	this->MarkFacetsByRPDPart(Upper_Face, MinorConnector_Part, -1, RPDStrong_Fact, false);
	this->MarkFacetsByRPDPart(Upper_Face, FinishingLine_Part, -1, RPDStrong_Fact, false);
	this->MarkFacetsByRPDPart(Lower_Face, FinishingLine_Part, -1, RPDStrong_Fact, false);
	// 弱化区域
	this->MarkFacetsByRPDPart(Upper_Face, FinishingLine_Part, -1, RPDWeak_Fact, false);
	this->MarkFacetsByRPDPart(Lower_Face, FinishingLine_Part, -1, RPDWeak_Fact, false);
	this->MarkFacetsByRPDPart(Side_Face, FinishingLine_Part, -1, RPDWeak_Fact, false);
	this->MarkFacetsByRPDPart(Upper_Face, RetentionMesh_Part, -1, RPDWeak_Fact, false);
	this->MarkFacetsByRPDPart(Lower_Face, RetentionMesh_Part, -1, RPDWeak_Fact, false);
	this->MarkFacetsByRPDPart(Side_Face, RetentionMesh_Part, -1, RPDWeak_Fact, false);

	// 标记和退化面片
	std::vector<int> Strong_facets_index = this->Get_facets(RPDStrong_Fact, false);
	// 没有获取到标记面片数 直接退出
	if (Strong_facets_index.size() == 0)
	{
		return Strong_facets_index;
	}
	//	扩张弱化区域的面积
	int Expand_count = 0;
	while (Expand_count <= Expand_WeakCount)
	{
		this->ExpandMarkFacets_Anyway(RPDWeak_Fact);
		Expand_count++;
	}
	while (Expand_count > 0)
	{
		this->ContractMarkFacets(RPDWeak_Fact);
		Expand_count--;
	}
	std::vector<int> Weak_facets_index = this->Get_facets(RPDWeak_Fact, false);
	//TriangleMesh Weak_Mesh = GetSubMesh(Weak_facets_index);
	//Weak_Mesh.write_binary("F:\\Desk\\Weak_Mesh.stl");
	//内缩，直到消除所有FinishingLine_Part
	int  Contract_Count = 0;
	int Left_faceCount = this->Get_facet_mark_count(Strong_facets_index, RPDWeak_Fact);
	while (Left_faceCount > 0)
	{
		this->ContractMarkFacets(RPDStrong_Fact);
		Strong_facets_index = this->Get_facets(RPDStrong_Fact, false);
		Left_faceCount = this->Get_facet_mark_count(Strong_facets_index, RPDWeak_Fact);
		Contract_Count++;
	}
	//TriangleMesh Strong_Mesh = GetSubMesh(Strong_facets_index);
	//Strong_Mesh.write_binary("F:\\Desk\\Strong_Mesh1.stl");
	Contract_Count += Expand_StrongCount;
	LOGINFO("Strong_facets_index = [%d], Need Contract_Count = [%d]", Strong_facets_index.size(), Contract_Count);
	while (Contract_Count > 0)
	{
		// 不扩张FinishingLine_Part
		this->ExpandMarkFacets_Special(RPDStrong_Fact);
		Strong_facets_index = this->Get_facets(RPDStrong_Fact, false);
		Contract_Count--;
	}
	// 递归调用直到消除所有的退化面片
	this->Area_FilterFacets(RPDStrong_Fact, Strong_facets_index);
	LOGINFO("Area_FilterFacets =%d", GetTickCount() - timecount);
	Strong_facets_index = this->Get_facets(RPDStrong_Fact, false);
	//Strong_Mesh = GetSubMesh(Strong_facets_index);
	//Strong_Mesh.write_binary("F:\\Desk\\Strong_Mesh2.stl");

	return Strong_facets_index;
}


// 获取自动横杆区域的面片
std::vector<int> TriangleMesh::Get_special_crossbar_facets(std::vector<int> facets_index, int Max_Contract_Count)
{	
    //std::vector<int> facets_index = this->Get_facets(RPDStrong_Fact, false);
	// 没有获取到标记面片数 直接退出
	if (facets_index.size() == 0)
	{
		return facets_index;
	}
	// 标记和退化面片
	for (int i = 0; i < facets_index.size(); i++)
	{
		int facet_index = facets_index[i];
		stl_facet &facet = this->stl.facet_start[facet_index];
		TriangleMesh::fact_set_mark(facet.extra, RPDStrong_Fact);
	}

	//内缩，Max_Contract_Count
	int  Contract_Count = 0;
	while (Max_Contract_Count > Contract_Count)
	{
		this->ContractMarkFacets(RPDStrong_Fact);
		Contract_Count++;
	}
	LOGINFO("Contract facets_index = [%d], Contract_Count = [%d]", facets_index.size(), Contract_Count);
	// 扩张FinishingLine_Part
	this->MarkFacetsByRPDPart(Upper_Face, FinishingLine_Part, -1, RPDWeak_Fact, false);
	this->MarkFacetsByRPDPart(Lower_Face, FinishingLine_Part, -1, RPDWeak_Fact, false);
	while (Contract_Count > 0)
	{
		this->ExpandMarkFacets_Anyway(RPDWeak_Fact);
		Contract_Count--;
	}
	// 递归调用直到消除所有的退化面片
	this->Area_FilterFacets(RPDStrong_Fact, facets_index);
	facets_index = this->Get_facets(RPDStrong_Fact, false);

	return facets_index;
}

//获取自动横杆区域的边界
BoundSegments_loops TriangleMesh::Get_auto_crossbar_boundloops()
{
	BoundSegments_loops bound_loops;
	// 边界无序线段提取
	DWORD timecount = GetTickCount();
	BoundSegments_vec boundline_segs;
	boundline_segs = this->Get_BoundSegments(RPDStrong_Fact);
	LOGINFO("Get_BoundSegments cost time %d", GetTickCount() - timecount);

	// 成环处理
	while (boundline_segs.size())
	{
		BoundSegments_loop boundline_oneloop = this->find_oneLoop(boundline_segs);
		if (boundline_oneloop.size() > 1) // 存在连续的直线段集合
		{
			bound_loops.push_back(boundline_oneloop);
		}
		else if (boundline_oneloop.size() == 1) // 有孤立线段存在  
		{
			LOGINFO("find_oneLoop abondon seg");
		}
		else
		{
			break;
		}
	}
	LOGINFO("find_oneLoops cost time [%d] bound_loops.size = %d", GetTickCount() - timecount, bound_loops.size());
	return bound_loops;
}

// 获取切片平面的相交线段
bool
TriangleMesh::Add_ZSliceSegs(std::vector<int> facets_index, PlaneZ_CrossBars& _zBars)
{
	double plane_z = _zBars.slice_z;
	if (facets_index.size() == 0)
		return false;
	BoundSegments_vec seg_vec;
	TriangleMeshSlicer<Z> mesh_slicer(&*this);
	// 挑出所有含有边界的三角面片 生成边界线段集合
	for (int i = 0; i < facets_index.size(); ++i)
	{
		int facet_index = facets_index[i];
		stl_facet& facet = this->stl.facet_start[facet_index];
		Pointf3s points;
		for (int j = 0; j < 3; ++j)
		{
			points.push_back(Pointf3(facet.vertex[j].x, facet.vertex[j].y, facet.vertex[j].z));
		}
		BoundingBoxf3 fbox(points);
		// 排除不在切片平面以外的三角面片
		if (fbox.max.z < plane_z || fbox.min.z > plane_z)
			continue;
		//
		//LOGINFO("facet_index = %d plane_z = %f", facet_index, plane_z);
		std::vector<IntersectionLine> intersection_lines;
		mesh_slicer.slice_facet_nonsolid(scale_(plane_z), facet, facet_index, fbox.min.z, fbox.max.z, &intersection_lines);
		if (intersection_lines.size() == 0) continue;
		IntersectionLine _line = intersection_lines.front();
		BoundSegment _Seg;
		_Seg.FacetID.Mark_facetIndex = facet_index;
		_Seg.FacetID.NoMark_facetIndex = facet_index;
		_Seg.Segment_line.a.x = unscale(_line.a.x);
		_Seg.Segment_line.a.y = unscale(_line.a.y);
		_Seg.Segment_line.a.z = plane_z;
		_Seg.Segment_line.b.x = unscale(_line.b.x);
		_Seg.Segment_line.b.y = unscale(_line.b.y);
		_Seg.Segment_line.b.z = plane_z;
		seg_vec.push_back(_Seg);
	}
	LOGINFO("Add_ZSliceSegs plane_z = %f, seg_vec size = %d", plane_z, seg_vec.size());

	//  
	if (seg_vec.size() != 0) {
		// 成环处理
		while (seg_vec.size())
		{
			BoundSegments_loop temp_loop = this->find_oneLoop(seg_vec);
			if (temp_loop.size() > 0)
			{
				_zBars.Slice_loops.push_back(temp_loop);
			}
		}
	}
	LOGINFO("Add_ZSliceSegs plane_z = %f, _zBars.Slice_loops size = %d", plane_z, _zBars.Slice_loops.size());

	return true;
}


// 同一水平面  取长度最长的作为主横杆
PlaneZ_CrossBars TriangleMesh::Get_PlaneZ_CrossBars_ByZ(BoundSegments_loops bound_loops, double z)
{
	PlaneZ_CrossBars zBars;
	zBars.slice_z = z;
	double max_len = 0.0;
	CrossbarPointPair pp;

	for (int i = 0; i < bound_loops.size(); i++)
	{
		// 找出相交的所有线段
		BoundSegments_vec z_loops = getLoops(bound_loops[i], z);
		if (z_loops.size() == 0) continue;
		Pointf3s points;
		//计算交点
		for (int i = 0; i < z_loops.size(); i++)
		{
			if (std::abs(z_loops.at(i).Segment_line.a.z - z_loops.at(i).Segment_line.b.z) < 0.0001)
				continue;

			Pointf3 point = z_loops.at(i).Segment_line.intersect_plane(z);
			points.push_back(point);
		}

		//预计交点不会很多，直接暴力求解
		if(points.size() == 0) continue;
		for (int i = 0; i < points.size() - 1; i++)
		{
			for (int j = i + 1; j < points.size(); j++)
			{
				double ddist = points.at(i).distance_to(points.at(j));
				if (ddist > max_len)
				{
					pp.first = points.at(i);
					pp.second = points.at(j);
					max_len = ddist;
				}
			}
		}
	}
	zBars.Major_CrossBars.push_back(pp);

	return zBars;
}


//寻找点到一组面片的最短距离
double TriangleMesh::get_min_distance_point_to_facets(std::vector<int> facets_index, Pointf3 point)
{
	if (facets_index.empty())
		return 0.0;

	double dis = DBL_MAX;
	for (int i = 0; i < facets_index.size(); i++)
	{
		stl_facet &facet = this->stl.facet_start[facets_index[i]];
		double x = 0.0, y = 0.0, z = 0.0;
		for (int j = 0; j < 3; j++)
		{
			x += facet.vertex[j].x;
			y += facet.vertex[j].y;
			z += facet.vertex[j].z;
		}
		x = x / 3;
		y = y / 3;
		z = z / 3;

		double d = (point.x - x)*(point.x - x) + (point.y - y)*(point.y - y) + (point.z - z)*(point.z - z);
		if (d < dis)
			dis = d;
	}

	return std::sqrt(dis);
}


double TriangleMesh::DistanceBetween2Line(Linef3 line1, Linef3 line2)
{
	// 解析几何通用解法，可以求出点的位置，判断点是否在线段上
	// 算法描述：设两条无限长度直线s、t,起点为s0、t0，方向向量为u、v
	// 最短直线两点：在s1上为s0+sc*u，在t上的为t0+tc*v
	// 记向量w为(s0+sc*u)-(t0+tc*v),记向量w0=s0-t0
	// 记a=u*u，b=u*v，c=v*v，d=u*w0，e=v*w0——(a)；
	// 由于u*w=、v*w=0，将w=-tc*v+w0+sc*u带入前两式得：
	// (u*u)*sc - (u*v)*tc = -u*w0  (公式2)
	// (v*u)*sc - (v*v)*tc = -v*w0  (公式3)
	// 再将前式(a)带入可得sc=(be-cd)/(ac-b2)、tc=(ae-bd)/(ac-b2)——（b）
	// 注意到ac-b2=|u|2|v|2-(|u||v|cosq)2=(|u||v|sinq)2不小于0
	// 所以可以根据公式（b）判断sc、tc符号和sc、tc与1的关系即可分辨最近点是否在线段内
	// 当ac-b2=0时，(公式2)(公式3)独立，表示两条直线平行。可令sc=0单独解出tc
	// 最终距离d（L1、L2）=|（P0-Q0)+[(be-cd)*u-(ae-bd)v]/(ac-b2)|
	double ux = line1.b.x - line1.a.x;
	double uy = line1.b.y - line1.a.y;
	double uz = line1.b.z - line1.a.z;

	double vx = line2.b.x - line2.a.x;
	double vy = line2.b.y - line2.a.y;
	double vz = line2.b.z - line2.a.z;

	double wx = line1.a.x - line2.a.x;
	double wy = line1.a.y - line2.a.y;
	double wz = line1.a.z - line2.a.z;

	double a = (ux * ux + uy * uy + uz * uz); //u*u
	double b = (ux * vx + uy * vy + uz * vz); //u*v
	double c = (vx * vx + vy * vy + vz * vz); //v*v
	double d = (ux * wx + uy * wy + uz * wz); //u*w 
	double e = (vx * wx + vy * wy + vz * wz); //v*w
	double dt = a * c - b * b;

	double sd = dt;
	double td = dt;

	double sn = 0.0;//sn = be-cd
	double tn = 0.0;//tn = ae-bd

	if (std::abs(dt) < EPSILON_1)
	{
		//两直线平行
		sn = 0.0;    //在s上指定取s0
		sd = 1.00;   //防止计算时除0错误

		tn = e;      //按(公式3)求tc
		td = c;
	}
	else
	{
		sn = (b * e - c * d);
		tn = (a * e - b * d);
		if (sn < 0.0)
		{
			//最近点在s起点以外，同平行条件
			sn = 0.0;
			tn = e;
			td = c;
		}
		else if (sn > sd)
		{
			//最近点在s终点以外(即sc>1,则取sc=1)
			sn = sd;
			tn = e + b; //按(公式3)计算
			td = c;
		}
	}
	if (tn < 0.0)
	{
		//最近点在t起点以外
		tn = 0.0;
		if (-d < 0.0) //按(公式2)计算，如果等号右边小于0，则sc也小于零，取sc=0
			sn = 0.0;
		else if (-d > a) //按(公式2)计算，如果sc大于1，取sc=1
			sn = sd;
		else
		{
			sn = -d;
			sd = a;
		}
	}
	else if (tn > td)
	{
		tn = td;
		if ((-d + b) < 0.0)
			sn = 0.0;
		else if ((-d + b) > a)
			sn = sd;
		else
		{
			sn = (-d + b);
			sd = a;
		}
	}

	double sc = 0.0;
	double tc = 0.0;

	if (std::abs(sn) < EPSILON_1)
		sc = 0.0;
	else
		sc = sn / sd;

	if (std::abs(tn) < EPSILON_1)
		tc = 0.0;
	else
		tc = tn / td;

	double dx = wx + (sc * ux) - (tc * vx);
	double dy = wy + (sc * uy) - (tc * vy);
	double dz = wz + (sc * uz) - (tc * vz);
	return dx * dx + dy * dy + dz * dz;
}


BoundingBoxf3 TriangleMesh::GetBoundingBox(BoundSegments_loops loops)
{
	BoundingBoxf3 box;

	for (int i = 0; i < loops.size(); i++)
	{
		for (int j = 0; j < loops.at(i).size(); j++)
		{
			box.merge(loops.at(i).at(j).Segment_line.a);
			box.merge(loops.at(i).at(j).Segment_line.b);
		}
	}

	return box;
}

BoundSegments_vec TriangleMesh::getLoops(BoundSegments_loop loop, double z)
{
	BoundSegments_vec z_Segs;
	z_Segs.clear();

	for (int i = 0; i < loop.size(); i++)
	{
		BoundSegment _seg = loop.at(i);
		if ((_seg.Segment_line.a.z <= z && _seg.Segment_line.b.z >= z) ||
			(_seg.Segment_line.a.z >= z && _seg.Segment_line.b.z <= z))
			z_Segs.push_back(_seg);
	}

	return z_Segs;
}


}