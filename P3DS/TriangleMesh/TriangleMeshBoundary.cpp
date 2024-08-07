#include "TriangleMesh.hpp"
#include "ClipperUtils.hpp"
#include "SVG.hpp"

using namespace std;

namespace Slic3r
{
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	///  网格点区域限制
	////////////////////////////////////////////////////////////////////////////////////////////////////////

	std::vector<int> TriangleMesh::MarkFacetsByAngle(const double angel_throld, const char fact_mark, const bool all_reset)
	{
		std::vector<int> Pro_facets_index;
		Pro_facets_index.clear();
		for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
		{
			stl_facet &facet = this->stl.facet_start[i];
			if (all_reset)
				TriangleMesh::fact_reset(facet.extra); // 重置所有面片的mark信息
			// 用角度阈值标记不同的颜色
			if (acos(-facet.normal.z) * 180 <= (angel_throld * PI)) // 角度大于阈值
			{
				TriangleMesh::fact_set_mark(facet.extra, Area_Fact);
				TriangleMesh::fact_set_mark(facet.extra, fact_mark); // Bound_Fact还需要退化
				Pro_facets_index.push_back(i);
			}
		}

		return Pro_facets_index;
	}

	// 用法相角度阈值对三角面片进行分类标记
	bool TriangleMesh::Mark_FilterFacets(double angel_throld)
	{
		DWORD timecount = GetTickCount();
		std::vector<int> Pro_facets_index = MarkFacetsByAngle(angel_throld, Bound_Fact);
		// ContractMarkFacets(Bound_Fact);
		this->Area_FilterFacets(Bound_Fact, Pro_facets_index); // 递归调用直到消除所有的退化面片
		LOGINFO("Area_FilterFacets time cost =%d", GetTickCount() - timecount);
		timecount = GetTickCount();
		this->General_BoundLoop_Line(Bound_Fact, this->boundLines_vec);
		LOGINFO("General_BoundLoop_Line time cost=%d", GetTickCount() - timecount);
		return true;
	}

	// LatticeSPT_Pointf3 --> SPT_pointf
	int TriangleMesh::Lspt2SPT(std::string _type)
	{
		int _ret = 0;
		std::vector<SPT_pointf> *_points_vex = this->get_points(_type);
		if (_points_vex == NULL)
		{
			LOGINFO("[%s]_Lsptpoints_vex == NULL", _type.c_str());
			return _ret;
		}
		if (this->Lspt_Points.size() == 0)
		{
			LOGINFO("Lspt_Points size == 0");
			return _ret;
		}

		for (int i = 0; i < this->Lspt_Points.size(); i++)
		{
			LatticeSPT_Pointf3 _Lspt = this->Lspt_Points.at(i);
			SPT_pointf spt_pf(Pointf(_Lspt.UpPoint.x, _Lspt.UpPoint.y), 0.0, 1);
			_points_vex->push_back(spt_pf);
			_ret++;
		}

		return _ret;
	}

	// SPT_pointf --> LatticeSPT_Pointf3
	int TriangleMesh::SPT2Lspt(std::string _type)
	{
		int _ret = 0;
		std::vector<SPT_pointf> *_points_vex = this->get_points(_type);
		if (_points_vex == NULL)
		{
			LOGINFO("[%s]_Lsptpoints_vex == NULL", _type.c_str());
			return _ret;
		}
		if (_points_vex->size() == 0)
		{
			LOGINFO("[%s]_Lsptpoints_vex size == 0", _type.c_str());
			return _ret;
		}

		for (int i = 0; i < _points_vex->size(); i++)
		{
			if (_points_vex->at(i).IsCollsion())
			{
				LatticeSPT_Pointf3 _Lspt(0.0, 0.5, 0.2, 1.0);
				_Lspt.UpPoint = _points_vex->at(i).Get_upPoint();
				_Lspt.DwPoint = _points_vex->at(i).Get_dwPoint();
				_Lspt.x_num = 0;
				_Lspt.y_num = 0;
				_Lspt.Form_FaceID = _points_vex->at(i).get_mincollsionFaceID();
				_Lspt.spt_type = LatticeSPT_Type::LSPT_Other;
				_Lspt.Weight = 1;
				this->Lspt_Points.push_back(_Lspt);
				_ret++;
			}
		}
		LOGINFO("%s [%d] SPT2Lspt [%d]", _type.c_str(), _points_vex->size(), _ret);

		return _ret;
	}

	// 从"boundary"，"grid"，"overhang"，"overhangline"中获取晶格支撑
	int TriangleMesh::Get_LsptPoints()
	{
		int _ret = 0;
		// 清空历史数据
		this->Lspt_Points.clear();
		_ret += SPT2Lspt("boundary");
		_ret += SPT2Lspt("grid");
		_ret += SPT2Lspt("overhang");
		_ret += SPT2Lspt("overhangline");
		return _ret;
	}

	// 通过获取不同角度的边界，提取一定重复数以上的线段集合
	int TriangleMesh::Get_SuspendPoints(
		const int angel_step,
		const int boundNumMin,
		const int suspend_angel)
	{
		// 清空历史数据
		this->Lspt_Points.clear();
		// 按不同的步长进行边界标记
		BoundSegments_vec bounds_vec;
		FacetSegments_map facetSegments_map;
		facetSegments_map.clear();
		double angel_start = angel_step;
		while (angel_start < 90)
		{
			std::vector<int> Pro_facets_index = MarkFacetsByAngle(angel_start, Bound_Fact);
			this->Area_FilterFacets(Bound_Fact, Pro_facets_index);
			this->Get_BoundSegments(Bound_Fact, angel_start, facetSegments_map);
			angel_start += angel_step;
		}

		// 筛选悬垂边
		FacetSegments_map::iterator map_It = facetSegments_map.begin();
		while (map_It != facetSegments_map.end())
		{
			BoundSegment _seg = map_It->second.front();
			stl_facet &mark_facet = this->stl.facet_start[_seg.FacetID.Mark_facetIndex];
			stl_facet &nomark_facet = this->stl.facet_start[_seg.FacetID.NoMark_facetIndex];
			_seg.weight = map_It->second.size(); // 记录重复边次数作为权重
			if (map_It->second.size() >= boundNumMin
				// 过滤花纹面
				&& TriangleMesh::fact_is_mark(mark_facet.extra, Tuqi_Fact) == false && TriangleMesh::fact_is_mark(nomark_facet.extra, Tuqi_Fact) == false
				// 过滤角度阈值
				//&& TriangleMesh::fact_is_mark(mark_facet.extra, SSFace_Fact)
				//&& TriangleMesh::fact_is_mark(nomark_facet.extra, SSFace_Fact)
			)
			{
				bounds_vec.push_back(_seg);
				// 进行悬垂线标记
				TriangleMesh::fact_set_mark(mark_facet.extra, SSLine_Fact);
				TriangleMesh::fact_set_mark(nomark_facet.extra, SSLine_Fact);
			}
			map_It++;
		}

		//////////////////////////////////////////////////////////////////
		///  计算悬垂面
		//////////////////////////////////////////////////////////////////
		// 标记悬垂面面片
		std::vector<int> Suspend_facetsID = MarkFacetsByAngle(suspend_angel, SSFace_Fact, false);
		int CE_Num = floor(suspend_angel / 10) + 1;
		for (size_t i = 0; i < CE_Num; i++)
			this->ContractMarkFacets(SSFace_Fact);
		for (size_t i = 0; i < CE_Num; i++)
			this->ExpandMarkFacets(SSFace_Fact);

		for (int i = 0; i < Suspend_facetsID.size(); i++)
		{
			int Suspend_facetID = Suspend_facetsID[i];
			stl_facet &suspend_facet = this->stl.facet_start[Suspend_facetID];
			// 排除花纹区域和悬垂线区域
			if (TriangleMesh::fact_is_mark(suspend_facet.extra, SSLine_Fact) || TriangleMesh::fact_is_mark(suspend_facet.extra, Tuqi_Fact))
			{
				continue;
			}
			if (TriangleMesh::fact_is_mark(suspend_facet.extra, SSFace_Fact) == false)
			{
				continue;
			}
			Pointf3 face_a = Pointf3(suspend_facet.vertex[0].x, suspend_facet.vertex[0].y, suspend_facet.vertex[0].z);
			Pointf3 face_b = Pointf3(suspend_facet.vertex[1].x, suspend_facet.vertex[1].y, suspend_facet.vertex[1].z);
			Pointf3 face_c = Pointf3(suspend_facet.vertex[2].x, suspend_facet.vertex[2].y, suspend_facet.vertex[2].z);
			LatticeSPT_Pointf3 _Lspt(0.0, 0.5, 0.2, 1.0);
			_Lspt.UpPoint = (face_a + face_b + face_c) / 3;
			_Lspt.DwPoint = Pointf3(_Lspt.UpPoint.x, _Lspt.UpPoint.y, 0);
			_Lspt.x_num = 0;
			_Lspt.y_num = 0;
			_Lspt.Form_FaceID = Suspend_facetID;
			_Lspt.spt_type = LatticeSPT_Type::Suspend_Face;
			_Lspt.Weight = 1;
			this->Lspt_Points.push_back(_Lspt);
			// 对边进行分析
			BoundSegment _seg;
			_seg.angle = suspend_angel;
			_seg.FacetID.Mark_facetIndex = Suspend_facetID;
			_seg.FacetID.NoMark_facetIndex = Suspend_facetID;
			_seg.weight = 1;
			Linef3 bound_ab(face_a, face_b);
			if (bound_ab.Length() > 2)
			{
				_seg.Segment_line = bound_ab;
				bounds_vec.push_back(_seg);
			}
			Linef3 bound_bc(face_b, face_c);
			if (bound_bc.Length() > 2)
			{
				_seg.Segment_line = bound_bc;
				bounds_vec.push_back(_seg);
			}
			Linef3 bound_ac(face_a, face_c);
			if (bound_ac.Length() > 2)
			{
				_seg.Segment_line = bound_ac;
				bounds_vec.push_back(_seg);
			}
		}

		////////////////////////////////////////////////////
		///  压入所有的悬垂边中点
		////////////////////////////////////////////////////
		TriangleMesh mesh_lspt;
		mesh_lspt.checkonly = true;
		for (int i = 0; i < bounds_vec.size(); i++)
		{
			BoundSegment _seg = bounds_vec[i];
			Pointf3s ps_vec = _seg.Segment_line.GetSegLines(0.1);
			for (int j = 0; j < ps_vec.size(); j++)
			{
				LatticeSPT_Pointf3 _Lspt(0.0, 0.5, 0.2, 1.0);
				_Lspt.UpPoint = ps_vec[j];
				_Lspt.DwPoint = Pointf3(_Lspt.UpPoint.x, _Lspt.UpPoint.y, 0);
				_Lspt.x_num = 0;
				_Lspt.y_num = 0;
				_Lspt.Form_FaceID = _seg.FacetID.Mark_facetIndex;
				_Lspt.spt_type = LatticeSPT_Type::Suspend_Bound;
				_Lspt.Weight = _seg.weight;
				this->Lspt_Points.push_back(_Lspt);
			}
		}

		// 重置所有面片的mark信息
		for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
		{
			stl_facet &facet = this->stl.facet_start[i];
			TriangleMesh::fact_reset(facet.extra);
		}

		return this->Lspt_Points.size();
	}

	LatticeSPT_Pointf3s
	TriangleMesh::Filter_LatticeSPT_byRPDPart(stl_face_type _type, bool remove)
	{
		LatticeSPT_Pointf3s _res;
		LatticeSPT_Pointf3s::iterator _It = this->Lspt_Points.begin();
		while (_It != this->Lspt_Points.end())
		{
			LatticeSPT_Pointf3 _spt = *_It;
			if (_spt.Form_FaceID == -1 || _spt.Form_FaceID >= this->facets_count()) // 面片id信息无效，无法过滤
			{
				_It++;
				continue;
			}
			else
			{
				stl_facet spt_face = this->stl.facet_start[_spt.Form_FaceID];
				if (spt_face.face_type == 0) // 面片标记信息无效，无法过滤
				{
					_It++;
					continue;
				}
				else
				{
					if (spt_face.face_type == _type) // 选中了面片
					{
						_res.push_back(_spt);
						if (remove)
						{
							_It = this->Lspt_Points.erase(_It);
							continue;
						}
						else
						{
							_It++;
							continue;
						}
					}
					else
					{
						_It++;
						continue;
					}
				}
			}
		}

		return _res;
	}

	bool TriangleMesh::Simply_SuspendPoints(
		const coordf_t Xcell_Length,
		const coordf_t Ycell_Length,
		const coordf_t Zcell_Length,
		const double reten_factor,
		const double tuqi_factor,
		const bool IsShowCell)
	{
		if (Xcell_Length < 0.01 || Ycell_Length < 0.01)
		{
			LOGINFO("Simply_SuspendPoints cell Length error [%d, %d]", Xcell_Length, Ycell_Length);
			return false;
		}
		if (this->Lspt_Points.size() == 0)
		{
			LOGINFO("Simply_SuspendPoints Lspt_Points size = 0");
			return false;
		}

		LOGINFO("------------Simply_SuspendPoints begin--------------");
		int befort_count = this->Lspt_Points.size();
		// 对主体进行简化
		LatticeSPT_Simply SPT_simply = LatticeSPT_Simply(Xcell_Length, Ycell_Length);
		// 调试辅助代码
		if (IsShowCell)
			this->boundLines_vec = SPT_simply.GetSimplyCellLines(this->Lspt_Points);
		if (Zcell_Length >= 0.1)
			SPT_simply.Set_Zstep(Zcell_Length);
		this->Lspt_Points = SPT_simply.GetSimplyVec(this->Lspt_Points);

		// 分类固位网区域支撑
		LatticeSPT_Pointf3s reten_spts;
		reten_spts.clear();
		LatticeSPT_Pointf3s temp_spts;
		temp_spts.clear();
		temp_spts = this->Filter_LatticeSPT_byRPDPart(RetentionMesh_Part, true);
		reten_spts.insert(reten_spts.end(), temp_spts.begin(), temp_spts.end());
		temp_spts = this->Filter_LatticeSPT_byRPDPart(Fulcrum_Part, true);
		reten_spts.insert(reten_spts.end(), temp_spts.begin(), temp_spts.end());
		temp_spts = this->Filter_LatticeSPT_byRPDPart(Nail_Part, true);
		reten_spts.insert(reten_spts.end(), temp_spts.begin(), temp_spts.end());
		LOGINFO("this->Lspt_Points size[%d] reten_spts size[%d]", this->Lspt_Points.size(), reten_spts.size());
		// 分离花纹区域支撑
		LatticeSPT_Pointf3s tuqi_spts;
		tuqi_spts = this->Filter_LatticeSPT_byRPDPart(StippledWax_Part, true);
		LOGINFO("this->Lspt_Points size[%d] tuqi_spts size[%d]", this->Lspt_Points.size(), tuqi_spts.size());

		// 对固位网进行简化
		if (reten_factor < 5.0 && reten_spts.size() > 0)
		{
			int reten_spts_befort_count = reten_spts.size();
			LatticeSPT_Simply SPT_simply_reten = LatticeSPT_Simply(Xcell_Length * reten_factor, Ycell_Length * reten_factor);
			if (Zcell_Length >= 0.1)
				SPT_simply_reten.Set_Zstep(Zcell_Length * reten_factor);
			reten_spts = SPT_simply_reten.GetSimplyVec(reten_spts, true);
			LOGINFO("reten_spts simply[%d/%d]", reten_spts.size(), reten_spts_befort_count);
			this->Lspt_Points.insert(this->Lspt_Points.end(), reten_spts.begin(), reten_spts.end());
		}
		// 对花纹进行简化
		if (tuqi_factor < 5.0 && tuqi_spts.size() > 0)
		{
			int tuqi_spts_befort_count = tuqi_spts.size();
			LatticeSPT_Simply SPT_simply_tuqi = LatticeSPT_Simply(Xcell_Length * tuqi_factor, Ycell_Length * tuqi_factor);
			if (Zcell_Length >= 0.1)
				SPT_simply_tuqi.Set_Zstep(Zcell_Length * tuqi_factor);
			tuqi_spts = SPT_simply_tuqi.GetSimplyVec(tuqi_spts, true);
			LOGINFO("tuqi_spts simply[%d/%d]", tuqi_spts.size(), tuqi_spts_befort_count);
			this->Lspt_Points.insert(this->Lspt_Points.end(), tuqi_spts.begin(), tuqi_spts.end());
		}

		int after_count = this->Lspt_Points.size();
		LOGINFO("Simply_SuspendPoints points [%d, %d]", befort_count, after_count);

		LOGINFO("------------Simply_SuspendPoints end--------------");

		return true;
	}

	// 对支撑结果进行修正
	bool TriangleMesh::Modify_SuspendPoints(
		const double Xcell_Length,
		const double Ycell_Length,
		const double Zcell_Length)
	{
		if (this->Lspt_Points.size() == 0)
		{
			return false;
		}

		// 建立hash抽屉
		this->set_step_length(Xcell_Length, Ycell_Length, Zcell_Length);
		this->init_hash_facets();
		LOGINFO("Modify_SuspendPoints set_step_length[%f, %f, %f] hash_facets.size() = %d, Lspt_Points.size() = [%d]",
				Xcell_Length, Ycell_Length, Zcell_Length, this->hash_facets.size(), this->Lspt_Points.size());
		//
		boost::thread_group *p_work_group = NULL;
		LOGINFO("------------Modify_SuspendPoints begin--------------");
		parallelize<size_t>(
			0,
			this->Lspt_Points.size() - 1,
			boost::bind(&TriangleMesh::_Modify_SuspendPoints, this, _1, Xcell_Length, Ycell_Length, Zcell_Length),
			p_work_group,
			boost::thread::hardware_concurrency() / 2);
		LOGINFO("-------------Modify_SuspendPoints end-------------");

		return true;
	}

	void TriangleMesh::_Modify_SuspendPoints(
		size_t spt_id,
		const double Xcell_Length,
		const double Ycell_Length,
		const double Zcell_Length)
	{
		if (spt_id >= this->Lspt_Points.size())
		{
			LOGINFO("_Modify_SuspendPoints Spt_id ERROR [%d] this->Lspt_Points.size() = %d",
					spt_id, this->Lspt_Points.size());
		}
		Pointf3 orgin_sptPoint = this->Lspt_Points[spt_id].UpPoint; // 原始支撑点
		std::vector<int> near_facetsID = this->get_Pointf3_gridfactes(orgin_sptPoint, 1);
		// LOGINFO("orgin_sptPoint[%s] near_facetsID size = %d", orgin_sptPoint.dump_perl().c_str(), near_facetsID.size());
		if (near_facetsID.size() == 0)
			return;
		Pointf3 modify_sptPoint = Pointf3(0, 0, DBL_MAX);
		int modify_faceId = -1;
		// 遍历附近所有的面片  找到最低
		for (int i = 0; i < near_facetsID.size(); i++)
		{
			const int near_facet_id = near_facetsID[i];
			const stl_facet &near_facet = this->stl.facet_start[near_facet_id];
			for (int j = 0; j < 3; ++j)
			{
				Pointf3 facet_point = Pointf3(near_facet.vertex[j].x, near_facet.vertex[j].y, near_facet.vertex[j].z);
				if (
					facet_point.x >= orgin_sptPoint.x - Xcell_Length && facet_point.x <= orgin_sptPoint.x + Xcell_Length && facet_point.y >= orgin_sptPoint.y - Ycell_Length && facet_point.y <= orgin_sptPoint.y + Ycell_Length && facet_point.z >= orgin_sptPoint.z - Zcell_Length && facet_point.z <= orgin_sptPoint.z && modify_sptPoint.z > facet_point.z)
				{
					modify_sptPoint = facet_point;
					modify_faceId = near_facet_id;
				}
			}
		}
		// LOGINFO("orgin_sptPoint[%s] modify_sptPoint[%s] ", orgin_sptPoint.dump_perl().c_str(), modify_sptPoint.dump_perl().c_str());
		if (modify_sptPoint.z < orgin_sptPoint.z)
		{
			this->Lspt_Points[spt_id].UpPoint = modify_sptPoint;
			this->Lspt_Points[spt_id].Form_FaceID = modify_faceId;
		}
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	///  边界点生成算法
	////////////////////////////////////////////////////////////////////////////////////////////////////////

	// 生成边界线段
	bool TriangleMesh::General_BoundLoop_Line(char facet_type, BoundLines_vec &boundlines)
	{
		DWORD timecount = GetTickCount();
		bool retval = false;
		boundlines.clear();
		// 挑出所有含有边界的三角面片 生成边界线段集合
		for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
		{
			stl_facet &facet = this->stl.facet_start[i];
			if (TriangleMesh::fact_is_mark(facet.extra, facet_type)) // 挑出所有命中的面片
			{
				int facetNear_Num = 0;
				int Comedge_facetIndex = -1;
				for (int j = 0; j <= 2; j++)
				{
					int near_facet_index = this->stl.neighbors_start[i].neighbor[j];
					stl_facet &near_facet = this->stl.facet_start[near_facet_index];
					if (TriangleMesh::fact_is_mark(near_facet.extra, facet_type))
						facetNear_Num++;
					else
						Comedge_facetIndex = near_facet_index;
				}

				if (facetNear_Num == 2) // 边界三角面片
				{
					Linef3 comedge = this->GetFacets_OneEdge(facet, this->stl.facet_start[Comedge_facetIndex]);
					boundlines.push_back(comedge);
				}
			}
		}

		LOGINFO("boundlines.size = %d", boundlines.size());
		LOGINFO("General_BoundLoop times = %d", GetTickCount() - timecount);

		return retval;
	}

	// 获取某一个边界合集
	bool TriangleMesh::Get_BoundSegments(char facet_type, double angle, FacetSegments_map &boundsegs_map)
	{
		DWORD timecount = GetTickCount();
		bool retval = false;
		// 挑出所有含有边界的三角面片 生成边界线段集合
		for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
		{
			stl_facet &facet = this->stl.facet_start[i];
			if (TriangleMesh::fact_is_mark(facet.extra, facet_type)) // 挑出所有命中的面片
			{
				int facetNear_Num = 0;
				int Comedge_facetIndex = -1;
				for (int j = 0; j <= 2; j++)
				{
					int near_facet_index = this->stl.neighbors_start[i].neighbor[j];
					stl_facet &near_facet = this->stl.facet_start[near_facet_index];
					if (TriangleMesh::fact_is_mark(near_facet.extra, facet_type))
						facetNear_Num++;
					else
						Comedge_facetIndex = near_facet_index;
				}

				if (facetNear_Num == 2) // 边界三角面片
				{
					BoundSegment boundseg;
					boundseg.Segment_line = this->GetFacets_OneEdge(facet, this->stl.facet_start[Comedge_facetIndex]);
					boundseg.angle = angle;
					boundseg.FacetID.Mark_facetIndex = i;
					boundseg.FacetID.NoMark_facetIndex = Comedge_facetIndex;
					boundsegs_map[boundseg.FacetID.union_id].push_back(boundseg);
				}
			}
		}

		LOGINFO("boundlines.size = %d", boundsegs_map.size());
		LOGINFO("General_BoundLoop times = %d", GetTickCount() - timecount);

		return retval;
	}

	// 获取相邻两个三角面片的公共边
	Linef3 TriangleMesh::GetFacets_OneEdge(const stl_facet &Selected_facet, const stl_facet &unSelected_facet)
	{
		Linef3 retval;
		int unSelected_point = -1;
		for (int i = 0; i < 3; i++)
		{
			bool IsIn = false;
			for (int j = 0; j < 3; j++)
			{
				if ((Selected_facet.vertex[i].x == unSelected_facet.vertex[j].x) &&
					(Selected_facet.vertex[i].y == unSelected_facet.vertex[j].y) &&
					(Selected_facet.vertex[i].z == unSelected_facet.vertex[j].z))
				{
					IsIn = true;
					break;
				}
			}
			// 记录非邻接点索引
			if (IsIn == false)
			{
				unSelected_point = i;
			}
		}

		if (unSelected_point == -1)
		{
			LOGINFO("两个邻接面片找不到相异点！！！");
		}
		else
		{
			retval.a.x = Selected_facet.vertex[(unSelected_point + 1) % 3].x;
			retval.a.y = Selected_facet.vertex[(unSelected_point + 1) % 3].y;
			retval.a.z = Selected_facet.vertex[(unSelected_point + 1) % 3].z;
			retval.b.x = Selected_facet.vertex[(unSelected_point + 2) % 3].x;
			retval.b.y = Selected_facet.vertex[(unSelected_point + 2) % 3].y;
			retval.b.z = Selected_facet.vertex[(unSelected_point + 2) % 3].z;
		}

		return retval;
	}

	// 在线段池中挑出一个闭环的圈
	BoundLoop
	TriangleMesh::find_oneLoop(BoundLines_vec &Orgin_Lines)
	{
		if (Orgin_Lines.size() == 0)
			return BoundLoop();

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
			if ((oneLoopLine_vec.size() > 2) && (oneLoopLine_vec.back().b == oneLoopLine_vec.front().a))
			{
				// 边界环数据
				BoundLoop oneLoop;
				oneLoop.clear();
				oneLoop.push_back(oneLoopLine_vec.front().a);
				for (int i = 0; i < oneLoopLine_vec.size(); i++)
				{
					// 边界环数据
					oneLoop.push_back(oneLoopLine_vec[i].b);
				}
				return oneLoop;
			}
		}

		// 未成环
		LOGINFO("发现未成环的直线段组 size = %d  直线段被抛弃", oneLoopLine_vec.size());
		return BoundLoop();
	}

	// 在线段池中挑出一个闭环的圈
	Polygon
	TriangleMesh::find_oneLoop_polygon(BoundLines_vec &Orgin_Lines)
	{
	    //算法1，老算法，对于10W条边界线计算时长大概36秒
		if (Orgin_Lines.size() == 0)
			return Polygon();
		Polygon oneLoop_polygon;
		// 边界环
		BoundLines_vec oneLoopLine_vec;
		oneLoopLine_vec.clear();

		// 取一点
		oneLoopLine_vec.push_back(Orgin_Lines.back());
		Orgin_Lines.pop_back();

		for (int index = 0; index < Orgin_Lines.size(); index++)
		{
			if (oneLoopLine_vec.back().b.EslipEqual(Orgin_Lines[index].a)) // 找到下一条
			{
				oneLoopLine_vec.push_back(Orgin_Lines[index]);
				Orgin_Lines.erase(Orgin_Lines.begin() + index);
				index = -1; // 重新开始循环
			}
			else if (oneLoopLine_vec.front().a.EslipEqual(Orgin_Lines[index].b)) // 找到下一条
			{
				oneLoopLine_vec.insert(oneLoopLine_vec.begin(), Orgin_Lines[index]);
				Orgin_Lines.erase(Orgin_Lines.begin() + index);
				index = -1; // 重新开始循环
			}

			// 成环
			if ((oneLoopLine_vec.size() > 2) && (oneLoopLine_vec.back().b.EslipEqual(oneLoopLine_vec.front().a)))
			{
				oneLoop_polygon.points.reserve(oneLoopLine_vec.size());
				for (int i = 0; i < oneLoopLine_vec.size(); i++)
				{
					// 多边形顶点进行放大处理
					Point tempPoint;
					tempPoint.x = scale_(oneLoopLine_vec[i].a.x);
					tempPoint.y = scale_(oneLoopLine_vec[i].a.y);
					oneLoop_polygon.points.push_back(tempPoint);
				}
				// 反转
				oneLoop_polygon.reverse();
				return oneLoop_polygon;
			}
		}

		// 未成环
		// LOGINFO("发现未成环的直线段组 size = %d", oneLoopLine_vec.size());
		oneLoop_polygon.points.reserve(oneLoopLine_vec.size());
		for (int i = 0; i < oneLoopLine_vec.size(); i++)
		{
			// 多边形顶点进行放大处理
			Point tempPoint;
			tempPoint.x = scale_(oneLoopLine_vec[i].a.x);
			tempPoint.y = scale_(oneLoopLine_vec[i].a.y);
			oneLoop_polygon.points.push_back(tempPoint);
		}
		// 反转
		oneLoop_polygon.reverse();
		return oneLoop_polygon;
//--------------------------------------------------------------------------------------------------------------------------
            //算法2，对于10W条边界线计算时长大概27秒byGuoXiao
//            std::cout<<"Orgin_Lines.size():"<<Orgin_Lines.size()<<std::endl;
////         // 如果原始边界线集合为空，则返回一个空的多边形
//            if (Orgin_Lines.empty())
//                return Polygon();
//
//            Polygon oneLoop_polygon;
//            BoundLines_vec oneLoopLine_vec;
//
//            // 取一点作为起始点
//            oneLoopLine_vec.push_back(Orgin_Lines.back());
//            Orgin_Lines.pop_back();
//
//            // 循环处理边界线集合
//            while (!Orgin_Lines.empty())
//            {
//                auto it = Orgin_Lines.begin();
//                bool found = false;
//
//                // 尝试将边界线加入到 oneLoopLine_vec 中以形成闭合环
//                for (; it != Orgin_Lines.end(); ++it)
//                {
//                    if (oneLoopLine_vec.back().b.EslipEqual(it->a))
//                    {
//                        oneLoopLine_vec.push_back(*it);
//                        Orgin_Lines.erase(it);
//                        found = true;
//                        break;
//                    }
//                    else if (oneLoopLine_vec.front().a.EslipEqual(it->b))
//                    {
//                        oneLoopLine_vec.insert(oneLoopLine_vec.begin(), *it);
//                        Orgin_Lines.erase(it);
//                        found = true;
//                        break;
//                    }
//                }
//
//                // 如果未找到符合条件的边界线，则退出循环
//                if (!found)
//                    break;
//            }
//
//             std::cout<<"oneLoopLine_vec.size():"<<oneLoopLine_vec.size()<<std::endl;
//            // 如果未形成闭合环，则直接返回包含顶点的多边形
//            if (oneLoopLine_vec.size() < 3 || !oneLoopLine_vec.back().b.EslipEqual(oneLoopLine_vec.front().a))
//            {
//                oneLoop_polygon.points.reserve(oneLoopLine_vec.size());
//                for (const auto &line : oneLoopLine_vec)
//                {
//                    Point tempPoint;
//                    tempPoint.x = scale_(line.a.x);
//                    tempPoint.y = scale_(line.a.y);
//                    oneLoop_polygon.points.push_back(tempPoint);
//                }
//            }
//            // 否则，从边界线集合中提取多边形的顶点，并返回多边形
//            else
//            {
//                oneLoop_polygon.points.reserve(oneLoopLine_vec.size());
//                for (const auto &line : oneLoopLine_vec)
//                {
//                    Point tempPoint;
//                    tempPoint.x = scale_(line.a.x);
//                    tempPoint.y = scale_(line.a.y);
//                    oneLoop_polygon.points.push_back(tempPoint);
//                }
//                // 返回多边形，无需反转
//                return oneLoop_polygon;
//            }
//
//            // 返回多边形，无需反转
//            return oneLoop_polygon;
//--------------------------------------------------------------------------------------------------------------------------
//       //算法3 byGuoXiao
//	   std::cout<<"Orgin_Lines.size():"<<Orgin_Lines.size()<<std::endl;
//	   if (Orgin_Lines.size() == 0)
//            return Polygon();
//
//       std::vector<int> cyclePath;
//       if (hasCycle(Orgin_Lines, cyclePath,Orgin_Lines.size()-1))
//       {
//           std::cout<<"cyclePath.size():"<<cyclePath.size()<<std::endl;
//       }
//
//       if(cyclePath.size()>0)
//       {
//            removeCycleLines(Orgin_Lines, cyclePath);
//       }
//       else
//       {
//            Orgin_Lines.pop_back();
//       }
//       return Polygon();
	}

	bool TriangleMesh::make_ExboundPoints(ExPolygons exps, double Points_distance, size_t spted_volume_id)
	{
		if (exps.size() == 0)
			return false;
		for (int i = 0; i < exps.size(); i++)
		{
			this->make_PolylinePoints("boundary", exps[i].contour.split_at_first_point(), Points_distance, this->get_tree_region_idx(spted_volume_id, i + 1));
			for (int j = 0; j < exps[i].holes.size(); j++)
				this->make_PolylinePoints("boundary", exps[i].holes[j].split_at_first_point(), Points_distance, this->get_tree_region_idx(spted_volume_id, i + 1, j + 1));
		}

		return true;
	}

	bool TriangleMesh::Make_SelectedFacets_ThinLines(
		size_t spted_volume_id,
		double Points_distance)
	{
		this->SelectedPoints.clear();
		// 参数要经过放大处理
		Points_distance = scale_(Points_distance);
		// 为选择的面片构造墙
		ExPolygons selected_exs = this->Get_SelectFacets_Ex();
		LOGINFO("selected_exs.size = %d", selected_exs.size());
		if (false)
		{
			SVG svg("selected_exs.svg");
			svg.draw(selected_exs, "red");
			svg.Close();
		}
		if (selected_exs.size())
		{
			ExPolygons simply_expp;
			for (ExPolygons::const_iterator it = selected_exs.begin(); it != selected_exs.end(); ++it)
			{
				ExPolygons simply_expp_one;
				it->simplify(scale_(0.2), &simply_expp_one);
				simply_expp.insert(simply_expp.end(), simply_expp_one.begin(), simply_expp_one.end());
			}
			if (false)
			{
				SVG svg("selected_exs_simply.svg");
				svg.draw(simply_expp, "blue");
				svg.Close();
			}
			this->Make_ExCenterLine(simply_expp, "selected", spted_volume_id, Points_distance);
		}
		// 重置所有面片的Selected_Fact信息
		for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
		{
			stl_facet &facet = this->stl.facet_start[i];
			TriangleMesh::fact_remove_mark(facet.extra, Selected_Fact);
		}
		return true;
	}
	bool TriangleMesh::Make_SelectedFacets_ThinBounds(
		size_t spted_volume_id,
		double Points_distance)
	{
		LOGINFO("0724 1");
		this->SelectedPoints.clear();
		this->custom_simply_boundary.clear();
		// 参数要经过放大处理
		Points_distance = scale_(Points_distance);
		// 为选择的面片构造墙
		ExPolygons selected_exs = this->Get_SelectFacets_Ex();
		LOGINFO("selected_exs.size = %d", selected_exs.size());
		if (false)
		{
			SVG svg("selected_exs.svg");
			svg.draw(selected_exs, "red");
			svg.Close();
		}
		LOGINFO("0724 2");
		if (selected_exs.size())
		{
			ExPolygons simply_expp;
			for (ExPolygons::const_iterator it = selected_exs.begin(); it != selected_exs.end(); ++it)
			{
				ExPolygons simply_expp_one;
				it->simplify(scale_(0.02), &simply_expp_one);
				simply_expp.insert(simply_expp.end(), simply_expp_one.begin(), simply_expp_one.end());
				simply_expp = offset_ex(simply_expp, scale_(-0.1));
			}
			this->custom_simply_boundary = simply_expp;
			if (false)
			{
				SVG svg("selected_exs_simply.svg");
				svg.draw(simply_expp, "blue");
				svg.Close();
			}
			this->Make_ExCenterLine(simply_expp, "boundary_hexhole_wall", spted_volume_id, Points_distance); // 暂时只有六边形孔墙有这个特殊性
		}
		LOGINFO("0724 3");
		// 重置所有面片的Selected_Fact信息
		for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
		{
			stl_facet &facet = this->stl.facet_start[i];
			TriangleMesh::fact_remove_mark(facet.extra, Selected_Fact);
		}
		LOGINFO("0724 4");
		return true;
	}
	void removeClosePoints(std::vector<SPT_pointf> &points, double threshold)
	{
		auto it = points.begin();
		while (it != points.end())
		{
			auto jt = it + 1;
			while (jt != points.end())
			{
				if (it->hitpoint.distance_to(jt->hitpoint) < threshold)
				{
					jt = points.erase(jt);
				}
				else
				{
					++jt;
				}
			}
			++it;
		}
	}
	// 和支托独立加强
	bool TriangleMesh::Make_Occlusa_ThinLines(
		size_t spted_volume_id,
		double Points_distance,
		double Offset_distance,
		std::string type,
		double tree_distance)
	{
		bool nofilter = false;
		if (type == "custom_inner")
		{ // 卡环取中轴线的点生成树支撑，不考虑区域id
			nofilter = true;
		}
		this->get_points(type)->clear();
		// 参数要经过放大处理
		Points_distance = scale_(Points_distance);
		// 为选择的面片构造墙
		ExPolygons selected_exs = this->Get_SelectFacets_Ex();
		selected_exs = offset_ex(selected_exs, scale_(-Offset_distance));
		LOGINFO("selected_exs.size = %d", selected_exs.size());
		if (false)
		{
			SVG svg("selected_exs.svg");
			svg.draw(selected_exs, "red");
			svg.Close();
		}
		if (selected_exs.size())
		{
			ExPolygons simply_expp;
			for (ExPolygons::const_iterator it = selected_exs.begin(); it != selected_exs.end(); ++it)
			{
				ExPolygons simply_expp_one;
				it->simplify(scale_(0.2), &simply_expp_one);
				simply_expp.insert(simply_expp.end(), simply_expp_one.begin(), simply_expp_one.end());
			}
			if (false)
			{
				SVG svg("selected_exs_simply.svg");
				svg.draw(simply_expp, "blue");
				svg.Close();
			}
			unsigned int CentralLine_index = 1;
			for (ExPolygons::iterator ex = simply_expp.begin(); ex != simply_expp.end(); ex++)
			{
				Polylines CentralLines;
				ex->medial_axis_tuqi(scale_(5), scale_(0.015), scale_(0.00001), &CentralLines);
				// 遍历薄壁线上面的点
				for (int i = 0; i < CentralLines.size(); i++)
				{
					unsigned int region_id = nofilter ? 10000 : this->get_tree_region_idx(spted_volume_id, 0, CentralLine_index++);
					this->make_PolylinePoints(type, CentralLines[i], Points_distance, region_id);
				}
			}
			if (nofilter)
			{
				std::vector<SPT_pointf> tempPoints = *this->get_points(type);
				removeClosePoints(tempPoints, tree_distance);
				LOGINFO("1120 remove .size = %d", this->get_points(type)->size() - tempPoints.size());
				this->get_points(type)->clear();
				for (auto it : tempPoints)
				{
					this->get_points(type)->push_back(it);
				}
			}
		}
		// 重置所有面片的Selected_Fact信息
		for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
		{
			stl_facet &facet = this->stl.facet_start[i];
			TriangleMesh::fact_remove_mark(facet.extra, Selected_Fact);
		}
		return true;
	}

	void TriangleMesh::filter_ex_holes(double fliter_hole_area, ExPolygons &exs)
	{
		for (size_t i = 0; i < exs.size(); i++)
		{
			for (Polygons::iterator py_It = exs[i].holes.begin(); py_It != exs[i].holes.end();)
			{
				double hole_area = py_It->abs_area();
				hole_area = unscale(unscale(hole_area));
				LOGINFO("outline_exs[%d].holes[%d].abs_area = %f", i, (py_It - exs[i].holes.begin()), hole_area);
				if (hole_area <= fliter_hole_area)
				{
					py_It = exs[i].holes.erase(py_It);
				}
				else
				{
					py_It++;
				}
			}
		}
	}

	void TriangleMesh::filter_ex_conters(double flite_area, ExPolygons &exs)
	{
		for (ExPolygons::iterator py_It = exs.begin(); py_It != exs.end();)
		{
			double conter_area = py_It->contour.abs_area();
			double conter_len = py_It->contour.abs_length();
			conter_area = unscale(unscale(conter_area));
			conter_len = unscale(conter_len);
			if (conter_area <= flite_area)
			{
				LOGINFO("conter_area[%f], conter_len[%f] sqrt(flite_area * PI)[%f]", conter_area, conter_len, 4 * sqrt(conter_area * PI));
			}
			if (conter_area <= flite_area / 2)
			{
				py_It = exs.erase(py_It);
			}
			else if (conter_area <= flite_area && conter_len > 4 * sqrt(conter_area * PI))
			{
				py_It = exs.erase(py_It);
			}
			else
			{
				py_It++;
			}
		}
	}

	// 生成边界间隔采样点
	bool TriangleMesh::Make_BoundLoops(
		size_t spted_volume_id,
		double angle_facts,
		double Points_distance,
		double boundary_offset,
		double boundgrid_distance,
		bool make_outline,
		bool make_boundary,
		bool make_thinwalls)
	{
		// 数据初始化
		this->boundLoops.clear();
		this->exboundLoops.clear();
		this->boundLoops_vec.clear();
		this->BoundaryPoints.clear();
		// 参数要经过放大处理
		Points_distance = scale_(Points_distance);
		boundary_offset = scale_(boundary_offset);
		boundgrid_distance = scale_(boundgrid_distance);
		double merge_width = scale_(0.4);
		DWORD timecount = GetTickCount();

		// 支架的全轮廓线
		ExPolygons outline_exs;
		outline_exs.clear();
		if (make_outline)
		{
			outline_exs = this->horizontal_projection_slm(90.0);
			outline_exs = offset_ex(outline_exs, merge_width);
			outline_exs = offset_ex(outline_exs, -boundary_offset - merge_width);
			// 过滤金塑结合区的孔洞
			TriangleMesh::filter_ex_holes(20, outline_exs);
			// 生成薄壁
			if (make_thinwalls)
				outline_exs = this->Make_ThinWalls(outline_exs, spted_volume_id, "boundary", 2, unscale(Points_distance));
		}

		// 组成expolygons
		this->exboundLoops = this->horizontal_projection_slm(angle_facts);
		// TriangleMesh::filter_ex_conters(0.3, this->exboundLoops); // 过滤内圈
		TriangleMesh::filter_ex_holes(20, this->exboundLoops); // 过滤内圈
		if (false)
		{
			static int ex_count = 0;
			char _name[255];
			sprintf(_name, "horizontal_ex_slice[%d].svg", ex_count++);
			SVG _svg(_name);
			_svg.draw(this->exboundLoops, "red");
			_svg.Close();
		}

		if (make_outline)
		{ // 做了外圈之后，exboundLoops只作为gird的约束
			TriangleMesh::filter_ex_holes(20, this->exboundLoops);
			this->exboundLoops = offset_ex(this->exboundLoops, -boundary_offset);
		}
		else
		{ // 向外扩之后再内缩
			this->exboundLoops = offset_ex(this->exboundLoops, merge_width);
			this->exboundLoops = offset_ex(this->exboundLoops, -boundary_offset - merge_width);
		}

		// 融合两种类型的边界
		if (make_boundary)
			outline_exs.insert(outline_exs.end(), this->exboundLoops.begin(), this->exboundLoops.end());
		// 从边界上按距离截取并计算边界支撑点
		this->make_ExboundPoints(outline_exs, Points_distance, spted_volume_id);

		// 再缩，网格算法用
		this->exboundLoops = offset_ex(this->exboundLoops, -boundgrid_distance);

		if (false)
		{
			char svg_name[255];
			sprintf(svg_name, "exboundLoops.svg");
			SVG svg(svg_name);
			svg.draw(this->exboundLoops, "red");
			svg.Close();
		}

		LOGINFO("发现直线环组 boundLoops size = %d", this->boundLoops.size());
		LOGINFO("共产生采样间隔点%d,  间隔%f", this->BoundaryPoints.size(), Points_distance / 2);
		LOGINFO("make_boundLoops times = %d", GetTickCount() - timecount);

		return true;
	}

	ExPolygons TriangleMesh::Make_ThinWalls(ExPolygons exps, size_t spted_volume_id, std::string _type, double _wedth, double _step)
	{
		_wedth = scale_(_wedth);
		_step = scale_(_step);
		double simplify_step = scale_(0.3);

		// 计算边界中比较薄的区域
		ExPolygons new_exBL;
		ExPolygons thinLoops;
		for (ExPolygons::const_iterator ex = exps.begin(); ex != exps.end(); ++ex)
		{
			Polygons pp;
			ex->simplify_p(simplify_step, &pp);
			ExPolygons expp_p = union_ex(pp);
			ExPolygons _new_exBL = offset2_ex(pp, -_wedth, _wedth);
			ExPolygons _thinLoops = diff_ex(expp_p, new_exBL);
			new_exBL.insert(new_exBL.end(), _new_exBL.begin(), _new_exBL.end());
			thinLoops.insert(thinLoops.end(), _thinLoops.begin(), _thinLoops.end());
		}

		this->Make_ExCenterLine(thinLoops, _type, spted_volume_id, _step);
		// 计算相交的部分
		return intersection_ex(new_exBL, exps);
	}

	// 生成薄壁线
	bool TriangleMesh::Make_ExCenterLine(ExPolygons thinLoops, std::string _type, size_t spted_volume_id, double _step)
	{

		// SVG svg("thinLoops.svg");
		// svg.draw(thinLoops, "green");
		LOGINFO("0720 Make_ExCenterLine _type = %s", _type.c_str());
		unsigned int CentralLine_index = 1;
		std::string cur_type = _type;
		if (_type == "boundary_hexhole_wall")
		{
			cur_type = "selected";
		}
		for (ExPolygons::iterator ex = thinLoops.begin(); ex != thinLoops.end(); ex++)
		{
			Polylines CentralLines;

			if (_type == "boundary_hexhole_wall")
			{
				CentralLines.push_back(ex->contour.split_at_index(0));
				for (int j = 0; j < ex->holes.size(); j++)
				{
					CentralLines.push_back(ex->holes[j].split_at_index(0));
				}
			}
			else
			{
				ex->medial_axis_tuqi(scale_(5), scale_(0.015), scale_(0.00001), &CentralLines);
			}

			// svg.draw(CentralLines, "red", scale_(0.1));
			//  遍历薄壁线上面的点

			for (int i = 0; i < CentralLines.size(); i++)
			{
				unsigned int region_id = this->get_tree_region_idx(spted_volume_id, 0, CentralLine_index++);
				this->make_PolylinePoints(cur_type, CentralLines[i], _step, region_id);
			}
		}
		// svg.Close();
		return true;
	}

	// 生成墙支撑顶点
	bool TriangleMesh::Make_WallLoops(std::string _type)
	{
		this->wallLoops_vec.clear();
		std::map<size_t, std::vector<SPT_pointf>> wallLoop_map;
		std::vector<SPT_pointf> *wallpoints_vex = this->get_points(_type);
		if (wallpoints_vex == NULL)
		{
			LOGINFO("wallpoints_vex == NULL");
			return false;
		}

		// 按loop_id来分拣
		for (unsigned int i = 0; i < wallpoints_vex->size(); i++)
		{
			size_t loop_id = wallpoints_vex->at(i).region_id;
			if (wallLoop_map.find(loop_id) == wallLoop_map.end())
				wallLoop_map[loop_id] = std::vector<SPT_pointf>();
			wallLoop_map.find(loop_id)->second.push_back(wallpoints_vex->at(i));
		}
		// 遍历各个loop_id
		std::map<size_t, std::vector<SPT_pointf>>::iterator wallLoop_mapIt = wallLoop_map.begin();
		while (wallLoop_mapIt != wallLoop_map.end())
		{
			if (_type == "boundary")
			{
				BoundLoop new_wallLoop;
				for (unsigned int j = 0; j < wallLoop_mapIt->second.size(); j++)
				{
					int face_ID = wallLoop_mapIt->second.at(j).get_mincollsionFaceID();
					stl_facet &_facet = this->stl.facet_start[face_ID];
					//种植桥架等种植孔里不需要加支撑
					auto iterNoSupportFaces = this->NoSupportFaces.find(face_ID);
					if (iterNoSupportFaces != this->NoSupportFaces.end())
                    {
                        continue;
                    }
					if (_facet.face_type == RetentionMesh_Part) // 边界墙避开固位网
					{
						if (new_wallLoop.size() > 0)
							this->wallLoops_vec.push_back(new_wallLoop);
						new_wallLoop.clear();
					}
					else
					{
						Pointf3 new_point;
						new_point.x = wallLoop_mapIt->second.at(j).hitpoint.x;
						new_point.y = wallLoop_mapIt->second.at(j).hitpoint.y;
						new_point.z = wallLoop_mapIt->second.at(j).get_mincollsionZ();
						new_wallLoop.push_back(new_point);
					}
				}
				if (new_wallLoop.size() > 0)
					this->wallLoops_vec.push_back(new_wallLoop);
			}
			else
			{
				BoundLoop new_wallLoop;
				for (unsigned int j = 0; j < wallLoop_mapIt->second.size(); j++)
				{
					Pointf3 new_point;
					new_point.x = wallLoop_mapIt->second.at(j).hitpoint.x;
					new_point.y = wallLoop_mapIt->second.at(j).hitpoint.y;
					new_point.z = wallLoop_mapIt->second.at(j).get_mincollsionZ();
					new_wallLoop.push_back(new_point);
				}
				if (new_wallLoop.size() > 0)
					this->wallLoops_vec.push_back(new_wallLoop);
			}

			wallLoop_mapIt++;
		}
		LOGINFO("this->wallLoops_vec.size() == %d", this->wallLoops_vec.size());

		return !(this->wallLoops_vec.empty());
	}

	// 点要经过放大处理
	// 判断点是否在圈内
	int TriangleMesh::PointIn_exboundLoops(Slic3r::Pointf checkPoint)
	{
		if (this->exboundLoops.empty())
			return -1;

		// 数据放大
		Point tempPoint;
		tempPoint.x = scale_(checkPoint.x);
		tempPoint.y = scale_(checkPoint.y);

		for (int i = 0; i < this->exboundLoops.size(); i++)
		{
			if (this->exboundLoops[i].contains_b(tempPoint))
				return (i + 1);
		}

		return -1;
	}
	int TriangleMesh::PointIn_customboundLoops(Slic3r::Pointf checkPoint)
	{
		LOGINFO("0912 PointIn_customboundLoops custom_simply_boundary.size = %d", custom_simply_boundary.size());
		if (this->custom_simply_boundary.empty())
			return -1;

		// 数据放大
		Point tempPoint;
		tempPoint.x = scale_(checkPoint.x);
		tempPoint.y = scale_(checkPoint.y);

		for (int i = 0; i < this->custom_simply_boundary.size(); i++)
		{
			if (this->custom_simply_boundary[i].contains_b(tempPoint))
				return (i + 1);
		}

		return -1;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RPD Part
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 将支架转换为face mark；
	std::vector<int> TriangleMesh::MarkFacetsByRPDPart(
		const stl_face_side _side,	// 支架上下表面标记
		const stl_face_type _type,	// 支架的各部件标记
		const stl_face_type_id _id, // 支架的各部件id
		const char target_fact_mark,
		bool IsRemoveOther)
	{
		std::vector<int> Pro_facets_index;
		Pro_facets_index.clear();
		for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
		{
			stl_facet &facet = this->stl.facet_start[i];

			if ((_side == -1 || _side == facet.face_side) &&
				(_type == -1 || _type == facet.face_type) &&
				(_id == -1 || _id == facet.face_type_id))
			{
				if ((
						_type == Clasp_Part ||
						_type == OcclusalRest_Part ||
						_type == Crossbar_Part) &&
					_side == Upper_Face)
				{ // 卡环 颌支托 上表面需要过滤角度
					double _facet_angle = stl_calculate_faceangle(&facet);
					if (_facet_angle <= 90)
					{
						TriangleMesh::fact_set_mark(facet.extra, target_fact_mark);
						Pro_facets_index.push_back(i);
					}
				}
				else
				{
					TriangleMesh::fact_set_mark(facet.extra, target_fact_mark);
					Pro_facets_index.push_back(i);
				}
			}
			else if (IsRemoveOther)
			{
				TriangleMesh::fact_remove_mark(facet.extra, target_fact_mark);
			}
		}
		return Pro_facets_index;
	}

	// 获取某一个边界合集
	BoundSegments_vec TriangleMesh::Get_BoundSegments(char facet_type)
	{
		BoundSegments_vec seg_vec;
		DWORD timecount = GetTickCount();
		// 挑出所有含有边界的三角面片 生成边界线段集合
		for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
		{
			stl_facet &facet = this->stl.facet_start[i];
			if (TriangleMesh::fact_is_mark(facet.extra, facet_type)) // 挑出所有命中的面片
			{
				int facetNear_Num = 0;
				int Comedge_facetIndex = -1;
				for (int j = 0; j <= 2; j++)
				{
					int near_facet_index = this->stl.neighbors_start[i].neighbor[j];
					stl_facet &near_facet = this->stl.facet_start[near_facet_index];
					if (TriangleMesh::fact_is_mark(near_facet.extra, facet_type))
						facetNear_Num++;
					else
						Comedge_facetIndex = near_facet_index;
				}

				if (facetNear_Num == 2) // 边界三角面片
				{
					BoundSegment boundseg;
					boundseg.Segment_line = this->GetFacets_OneEdge(facet, this->stl.facet_start[Comedge_facetIndex]);
					boundseg.angle = 0;
					boundseg.FacetID.Mark_facetIndex = i;
					boundseg.FacetID.NoMark_facetIndex = Comedge_facetIndex;
					seg_vec.push_back(boundseg);
				}
			}
		}

		LOGINFO("seg_vec.size = %d", seg_vec.size());
		LOGINFO("Get_BoundSegments times = %d", GetTickCount() - timecount);

		// 删除借助的临时标记
		for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
		{
			stl_facet &facet = this->stl.facet_start[i];
			TriangleMesh::fact_remove_mark(facet.extra, RPDStrong_Fact);
		}

		return seg_vec;
	}

	//
	// 在线段池中选出连续的集合
	BoundSegments_loop
	TriangleMesh::find_oneLoop(BoundSegments_vec &Orgin_Segs)
	{
		// 边界环
		BoundSegments_loop oneLoopLine_vec;
		oneLoopLine_vec.clear();
		if (Orgin_Segs.size() == 0)
			return oneLoopLine_vec;

		// 取最后一个点
		oneLoopLine_vec.push_back(Orgin_Segs.back());
		Orgin_Segs.pop_back();

		for (int index = 0; index < Orgin_Segs.size(); index++)
		{
			if (oneLoopLine_vec.back().Segment_line.b.EslipEqual(Orgin_Segs[index].Segment_line.a)) // 接上尾巴
			{
				oneLoopLine_vec.push_back(Orgin_Segs[index]);
				Orgin_Segs.erase(Orgin_Segs.begin() + index);
				index = -1; // 重新开始循环
			}
			else if (oneLoopLine_vec.front().Segment_line.a.EslipEqual(Orgin_Segs[index].Segment_line.b)) // 接上头
			{
				BoundSegments_loop oneLoopLine_vec_new;
				oneLoopLine_vec_new.push_back(Orgin_Segs[index]);
				oneLoopLine_vec_new.insert(oneLoopLine_vec_new.end(), oneLoopLine_vec.begin(), oneLoopLine_vec.end());
				oneLoopLine_vec = oneLoopLine_vec_new;
				Orgin_Segs.erase(Orgin_Segs.begin() + index);
				index = -1; // 重新开始循环
			}

			// 成环
			if ((oneLoopLine_vec.size() > 2) && (oneLoopLine_vec.back().Segment_line.b.EslipEqual(oneLoopLine_vec.front().Segment_line.a)))
			{
				LOGINFO("发现(成环)的直线段组 size = %d;", oneLoopLine_vec.size());
				return oneLoopLine_vec;
			}
		}

		// 未成环
		LOGINFO("发现(未成环)的直线段组 size = %d;", oneLoopLine_vec.size());
		return oneLoopLine_vec;
	}

	// 按步长采样三维曲线上的点
	BoundLoop
	TriangleMesh::Sampling_SegmentsLoop(BoundSegments_loop Seg_loop, double Points_distance)
	{
		BoundLoop new_wallLoop;
		// 起点
		if (Seg_loop.size())
		{
			new_wallLoop.push_back(Seg_loop.front().Segment_line.a);
		}
		double add_distance = 0.0;
		for (size_t i = 0; i < Seg_loop.size(); i++)
		{
			Linef3 Seg_Line = Seg_loop[i].Segment_line;
			if (Points_distance <= 0.0)
			{ // 间距过小保护
				new_wallLoop.push_back(Seg_Line.b);
			}
			else
			{
				// 初始量
				Pointf3 Linef3_start = Seg_Line.a;
				Pointf3 Linef3_end = Seg_Line.b;
				double Linef_Len = Linef3_start.distance_toXY(Linef3_end); // 降为2维计算长度
				while (add_distance + Linef_Len >= Points_distance)		   // 长度大于采样步进
				{
					// 计算切割点
					double cutPoint_step = Points_distance - add_distance;
					Pointf3 vecPointf3 = Linef3_end - Linef3_start;
					Pointf3 cutPointf3 = Linef3_start + (cutPoint_step / Points_distance) * vecPointf3;
					Pointf tempPoint = Pointf(cutPointf3.x, cutPointf3.y);
					Pointf3 xyPointNormal;
					bool ishit = this->Cale_CollsionZ_byXY(tempPoint, cutPointf3, xyPointNormal); // 进行一下检测,避免发生穿模情况
					new_wallLoop.push_back(cutPointf3);

					// 重置初始条件
					add_distance = 0.0;
					Linef3_start = cutPointf3;
					Linef_Len = Linef3_start.distance_toXY(Linef3_end);
				}

				//  长度小于采样步进
				add_distance += Linef_Len;
			}
		}
		// 终点
		if (Seg_loop.size())
		{
			new_wallLoop.push_back(Seg_loop.back().Segment_line.b);
		}

		// if (Seg_loop.size() == 2) {
		//	for (int i = 0; i < new_wallLoop.size(); i++) {
		//		LOGINFO("new_wallLoop[%d] = %s", i , new_wallLoop[i].dump_perl().c_str());
		//	}
		// }

		return new_wallLoop;
	}

	// 按步长采样三维曲线上的点
	BoundLoop
	TriangleMesh::Sampling_SegmentsLoop2(BoundSegments_loop Seg_loop, double Points_distance)
	{
		BoundLoop _ret;
		// 起点
		if (Seg_loop.size() == 0)
		{
			return _ret;
		}

		_ret.push_back(Seg_loop.front().Segment_line.a);
		double add_distance = 0.0;
		for (size_t i = 0; i < Seg_loop.size(); i++)
		{
			Linef3 Seg_Line = Seg_loop[i].Segment_line;
			if (Points_distance <= 0.1)
			{ // 间距过小保护
				_ret.push_back(Seg_Line.b);
			}
			else
			{
				// 初始量
				Pointf3 Linef3_start = Seg_Line.a;
				Pointf3 Linef3_end = Seg_Line.b;
				double Linef_Len = Linef3_start.distance_to(Linef3_end);

				while (add_distance + Linef_Len >= Points_distance) // 长度大于采样步进
				{
					// 计算切割点
					double cutPoint_step = Points_distance - add_distance;
					Pointf3 vecPointf3 = Linef3_end - Linef3_start;
					Pointf3 cutPointf3 = Linef3_start + (cutPoint_step / Points_distance) * vecPointf3;
					_ret.push_back(cutPointf3);
					// 重置初始条件
					add_distance = 0.0;
					Linef3_start = cutPointf3;
					Linef_Len = Linef3_start.distance_to(Linef3_end);
				}

				//  长度小于采样步进
				add_distance += Linef_Len;
			}
		}

		// 终点
		// if (Seg_loop.size()) {
		//	Pointf3 Last_Point = Seg_loop.back().Segment_line.b;
		//	if(Last_Point.distance_to(_ret.back()) > Points_distance / 2)
		//		_ret.push_back(Last_Point);
		//}
		_ret.push_back(Seg_loop.back().Segment_line.b);
		if (_ret.size() <= 2)
		{
			return BoundLoop();
		}

		return _ret;
	}

	// 按曲率和步长采样三维曲线上的点  BoundSegments_loop间隔小于Points_distance
	BoundLoop
	TriangleMesh::Sampling_SegmentsLoop3(BoundLoop Seg_loop, double Points_distance)
	{
		BoundLoop _ret;
		// 起点
		if (Seg_loop.size() == 0)
		{
			return _ret;
		}

		double add_distance = 0.0;
		double min_curvature = 1.0;
		Pointf3 Selected_Pointf3;
		for (size_t i = 0; i < Seg_loop.size(); i++)
		{
			Pointf3 Linef3_mid = Seg_loop[i];
			if (i == 0)
				continue; // 跳过第一个点
			if (Points_distance <= 0.1)
			{ // 间距过小保护
				_ret.push_back(Linef3_mid);
			}
			else
			{
				double curvature = 1.0;
				Pointf3 Linef3_last = Seg_loop[i - 1];
				if (i != Seg_loop.size() - 1) // 非最后一个点
				{
					// 初始量
					Pointf3 Linef3_next = Seg_loop[i + 1];
					Pointf3 Linef3_vector = Linef3_mid - Linef3_last;
					Linef3_vector.normalize();
					Pointf3 Next_Linef3_vector = Linef3_next - Linef3_mid;
					Next_Linef3_vector.normalize();
					curvature = Linef3_vector.x * Next_Linef3_vector.x + Linef3_vector.y * Next_Linef3_vector.y + Linef3_vector.z * Next_Linef3_vector.z;
					LOGINFO("Linef3_last = %s, Linef3_mid = %s, Linef3_next = %s, curvature = %f",
							Linef3_last.dump_perl().c_str(),
							Linef3_mid.dump_perl().c_str(),
							Linef3_next.dump_perl().c_str(),
							curvature);
				}

				if (curvature < min_curvature)
				{
					min_curvature = curvature;
					Selected_Pointf3 = Linef3_mid;
				}

				// 计算线段长度
				double Linef_Len = Linef3_last.distance_to(Linef3_mid);
				// 长度大于采样步进
				if (add_distance + Linef_Len >= Points_distance)
				{
					LOGINFO("Selected_Pointf3 = %s, min_curvature = %f", Selected_Pointf3.dump_perl().c_str(), min_curvature);
					// 计算切割点
					_ret.push_back(Selected_Pointf3);
					// 重置初始条件
					min_curvature = 1.0;
					add_distance = 0.0;
					Linef_Len = 0.0;
				}
				//  长度小于采样步进
				add_distance += Linef_Len;
			}
		}

		return _ret;
	}

	//
	int TriangleMesh::Mark_RPDPart(
		stl_face_side _side,
		stl_face_type _type,
		stl_face_type_id _id)
	{
		// 将某个部件标记成RPDStrong_Fact
		std::vector<int> facets_index = this->MarkFacetsByRPDPart(_side, _type, _id, RPDStrong_Fact, false);
		return facets_index.size();
	}

	int TriangleMesh::Mark_RPDPart_Selected(
		stl_face_side _side,
		stl_face_type _type,
		stl_face_type_id _id)
	{
		// 将某个部件标记成Selected_Fact
		std::vector<int> facets_index = this->MarkFacetsByRPDPart(_side, _type, _id, Selected_Fact, false);
		return facets_index.size();
	}

	int TriangleMesh::UnMark_RPDPart_Selected( //
		stl_face_side _side,
		stl_face_type _type,
		stl_face_type_id _id)
	{
		int num = 0;
		// 从Selected_Fact将某个部件排除标记
		for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
		{
			stl_facet &facet = this->stl.facet_start[i];

			if ((_side == -1 || _side == facet.face_side) &&
				(_type == -1 || _type == facet.face_type) &&
				(_id == -1 || _id == facet.face_type_id))
			{

				TriangleMesh::fact_remove_mark(facet.extra, Selected_Fact);
				num++;
			}
		}
		return num;
	}

	void TriangleMesh::ResetMark_RPDStrong()
	{
		// 移除标记
		for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
		{
			stl_facet &facet = this->stl.facet_start[i];
			TriangleMesh::fact_remove_mark(facet.extra, RPDStrong_Fact);
		}
	}

	void TriangleMesh::ResetMark_Selected()
	{
		// 移除标记
		this->ResetMark_By_Type(Selected_Fact);
	}

	void TriangleMesh::ResetMark_By_Type(const char _type)
	{
		// 移除标记
		for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
		{
			stl_facet &facet = this->stl.facet_start[i];
			TriangleMesh::fact_remove_mark(facet.extra, _type);
		}
	}
#define Debug_Test_Svg false
#define Debug_Test_Point_Svg false
	void TriangleMesh::Set_facets_Selected(std::vector<int> facets, bool remove_selected)
	{

		LOGINFO("Set_facets_Selected size is [%d]", facets.size());
		for (int i = 0; i < facets.size(); ++i)
		{
			int facet_id = facets[i];
			stl_facet &facet = this->stl.facet_start[facet_id];
			if (!remove_selected)
			{
				TriangleMesh::fact_set_mark(facet.extra, Selected_Fact);
			}
			else
			{
				TriangleMesh::fact_remove_mark(facet.extra, Selected_Fact);
			}
		}
	}
	void TriangleMesh::Set_2dimfacets_Selected(int dim, bool remove_selected, double filter_angle)
	{
		if (this->RPD_selected_facets.empty())
		{
			return;
		}
		LOGINFO("Set_2dimfacets_Selected[%d] size is [%d]", dim, this->RPD_selected_facets[dim].size());
		if (!this->RPD_selected_facets[dim].empty())
		{
			this->Set_facets_Selected(this->RPD_selected_facets[dim], remove_selected);
		}
		this->Angle_FilterFacets(Selected_Fact, this->RPD_selected_facets[dim], filter_angle); // 先用角度阈值过滤
	}
	void TriangleMesh::ResetMark_RPDWeak()
	{
		// 移除标记
		for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
		{
			stl_facet &facet = this->stl.facet_start[i];
			TriangleMesh::fact_remove_mark(facet.extra, RPDWeak_Fact);
		}
	}

	// 构造支架特殊区域
	int TriangleMesh::Make_RPDPart_BoundWall(double Points_distance, double filter_angle)
	{
		int _ret = 0;
		// 标记和退化面片
		DWORD timecount = GetTickCount();
		std::vector<int> facets_index = this->Get_facets(RPDStrong_Fact, false);
		// this->Angle_FilterFacets(RPDStrong_Fact, facets_index, filter_angle); // 先用角度阈值过滤
		this->Area_FilterFacets(RPDStrong_Fact, facets_index); // 递归调用直到消除所有的退化面片
		LOGINFO("Area_FilterFacets =%d", GetTickCount() - timecount);

		// 边界无序线段提取
		timecount = GetTickCount();
		BoundSegments_vec boundline_segs;
		boundline_segs = this->Get_BoundSegments(RPDStrong_Fact);
		LOGINFO("Get_BoundSegments cost time %d", GetTickCount() - timecount);

		// 边界成环
		timecount = GetTickCount();
		BoundSegments_loops bound_loops;
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
				LOGINFO("find_oneLoop abondon seg!!!!!!!");
			}
			else
			{
				break;
			}
		}
		LOGINFO("find_oneLoops cost time [%d] bound_loops.size = %d Left boundline_segs size = %d",
				GetTickCount() - timecount,
				bound_loops.size(),
				boundline_segs.size());

		// 生成墙支撑点
		if (bound_loops.size() == 0)
		{
			return _ret;
		}

		// 对最大的外圈和最大的内圈做处理
		bound_loops = this->CutLoopsByMajorLoopAndHole_oldversion(bound_loops, Linef3(Pointf3(0, 0, 0), Pointf3(0, 1, 0)));
		// 过滤特定类型的边界
		bound_loops = this->CutLoopsByRPDType(bound_loops, RetentionMesh_Part);
		bound_loops = this->CutLoopsByRPDType(bound_loops, OcclusalRest_Part);
		bound_loops = this->CutLoopsByRPDType(bound_loops, FinishingLine_Part);
		bound_loops = this->CutLoopsByRPDType(bound_loops, Clasp_Part);
		// 按loop顶点个数进行过滤
		// bound_loops = this->CutLoopsBySize(bound_loops, 50);
		// 过滤角度
		// bound_loops = this->CutLoopsByFaceAngle(bound_loops, filter_angle);
		// 按封闭圈过滤
		// bound_loops = this->CutLoopsByLoop(bound_loops);

		if (bound_loops.size() == 0)
		{
			return _ret;
		}
		this->wallLoops_vec.clear();
		for (unsigned int i = 0; i < bound_loops.size(); i++)
		{
			BoundSegments_loop Seg_loop = bound_loops[i];
			BoundLoop new_wallLoop = this->Sampling_SegmentsLoop(Seg_loop, Points_distance);
			this->wallLoops_vec.push_back(new_wallLoop);
		}
		LOGINFO("this->wallLoops_vec.size() == %d", this->wallLoops_vec.size());
		this->ResetMark_RPDStrong();

		return _ret;
	}
	void TriangleMesh::FilterLoopsByIPD(BoundSegments_loops &bound_loops)
	{
		std::string RetentionMesh_Partname = "RetentionMesh_Part.svg";
		std::string FinishingLine_Partname = "FinishingLine_Part.svg";
		std::string Clasp_Partname = "Clasp_Part.svg";
		std::string CutLoopsBySizename = "CutLoopsBySize.svg";

		bound_loops = this->CutLoopsByRPDType(bound_loops, RetentionMesh_Part);
		// bound_loops = this->CutLoopsByRPDType(bound_loops, OcclusalRest_Part);
		bound_loops = this->CutLoopsByRPDType(bound_loops, FinishingLine_Part);
		bound_loops = this->CutLoopsByRPDType(bound_loops, Clasp_Part);
		// draw_svg_byBoundSegments_loops(bound_loops, RetentionMesh_Partname, 1);
		// draw_svg_byBoundSegments_loops(bound_loops, FinishingLine_Partname, 2);
		// draw_svg_byBoundSegments_loops(bound_loops, Clasp_Partname, 3);
		//  按loop顶点个数进行过滤
		bound_loops = this->CutLoopsBySize(bound_loops, 100);
		// draw_svg_byBoundSegments_loops(bound_loops, CutLoopsBySizename, 4);
	}

	void TriangleMesh::FilterFacetsByIPD(std::vector<int> Selected_Facets, double filter_angle)
	{
		LOGINFO("0901 filter_angle is %f", filter_angle);
		// std::vector<int> Selected_Facets = this->Get_facets(Selected_Fact, false);
		LOGINFO("0901 Selected_Facets size[%d]", Selected_Facets.size());
		this->Angle_FilterFacets(Selected_Fact, Selected_Facets, filter_angle); // 先用角度阈值过滤
		LOGINFO("0901 Angle_Filter Selected_Facets size[%d]", Selected_Facets.size());
		// this->ResetMark_Selected();
		//   开始标记
		for (int i = 0; i < Selected_Facets.size(); i++)
		{
			int fact_index = Selected_Facets[i];
			stl_facet &facet = this->stl.facet_start[fact_index];
			if (TriangleMesh::is_face_rpd_mark(facet.face_type, Clasp_Part) || TriangleMesh::is_face_rpd_mark(facet.face_type, RetentionMesh_Part) || TriangleMesh::is_face_rpd_mark(facet.face_type, FinishingLine_Part))
			{
				TriangleMesh::fact_remove_mark(facet.extra, Selected_Fact);
			}
			else
			{
				TriangleMesh::fact_set_mark(facet.extra, Selected_Fact);
			}
		}
		LOGINFO("0901 IPD_Filter Selected_Facets size[%d]", this->Get_facets(Selected_Fact, false).size());
	}

	// 将selected的区域作为生成网格点的区域，进行预处理
	int TriangleMesh::Make_Selected_Region_GridPoints(double filter_angle, bool is_filter_angle)
	{
		LOGINFO("Make_Selected_Region_GridPoint\n");

		LOGINFO("0828 1 filter_angle is %f", filter_angle);
		this->SelectedPoints.clear();
		this->custom_GridPoints.clear();
		this->custom_simply_boundary.clear();
		std::vector<int> facets_index = this->Get_facets(Selected_Fact, false);
		if (is_filter_angle == true)
		{
			LOGINFO("1110 filter_angle is %f", filter_angle);
			this->Angle_FilterFacets(Selected_Fact, facets_index, filter_angle); // 用角度阈值过滤
		}
		else
		{
			this->Angle_FilterFacets(Selected_Fact, facets_index, 90.0); // 用角度阈值过滤
		}

		ExPolygons selected_exs = this->Get_SelectFacets_Ex();
		LOGINFO("selected_exs.size = %d", selected_exs.size());
		DWORD timecount = GetTickCount();
		LOGINFO("timecount is = %d", timecount);
		std::string time = boost::to_string(timecount);
		if (Debug_Test_Svg)
		{
			std::string allfacets = time + "_0912-1Region_Grid.svg";
			this->draw_svg_byfacets(facets_index, allfacets, 1);
		}

		LOGINFO("0821 2");
		if (selected_exs.size())
		{
			ExPolygons simply_expp;
			for (ExPolygons::const_iterator it = selected_exs.begin(); it != selected_exs.end(); ++it)
			{
				ExPolygons simply_expp_one;
				it->simplify(scale_(0.02), &simply_expp_one);
				simply_expp.insert(simply_expp.end(), simply_expp_one.begin(), simply_expp_one.end());
			}
			if (Debug_Test_Svg)
			{
				std::string name = time + "_0912-2simply_expp.svg";
				SVG svg(name.c_str());
				svg.draw(simply_expp, "green");
				svg.Close();
			}
			simply_expp = offset_ex(simply_expp, scale_(-0.1));
			this->custom_simply_boundary = simply_expp;
			if (Debug_Test_Svg)
			{
				std::string name = time + "_0912-3custom_simply_boundary.svg";
				SVG svg(name.c_str());
				svg.draw(this->custom_simply_boundary, "orange");
				svg.Close();
			}
		}
		LOGINFO("0821 3");
		// 重置所有面片的Selected_Fact信息
		for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
		{
			stl_facet &facet = this->stl.facet_start[i];
			TriangleMesh::fact_remove_mark(facet.extra, Selected_Fact);
		}

		LOGINFO("0821 4");
		return true;
	}
	// 为面片绘制SVG图，以供调试
	void TriangleMesh::draw_svg_byfacets(std::vector<int> Selected_Facets, std::string name, int color)
	{
		LOGINFO("0828 - draw_svg_byfacets ");
		//  清空标记
		this->ResetMark_Selected();

		// 开始标记
		for (int i = 0; i < Selected_Facets.size(); i++)
		{
			int fact_index = Selected_Facets[i];
			stl_facet &facet = this->stl.facet_start[fact_index];
			TriangleMesh::fact_set_mark(facet.extra, Selected_Fact);
		}

		ExPolygons selected_exs = this->Get_SelectFacets_Ex();
		LOGINFO("selected_exs.size = %d", selected_exs.size());
		LOGINFO("0828 - 1 ");
		SVG svg(name.c_str());
		if (color == 1)
		{
			svg.draw(selected_exs, "red");
		}
		else if (color == 2)
		{
			svg.draw(selected_exs, "yellow");
		}
		else if (color == 3)
		{
			svg.draw(selected_exs, "green");
		}
		else if (color == 4)
		{
			svg.draw(selected_exs, "blue");
		}

		svg.Close();
		LOGINFO("0828 - 2 ");
		this->ResetMark_Selected();
	}
	// 获得需要强化的区域的面片并放入RPD_selected_facets

	void TriangleMesh::draw_svg_byBoundSegments_loops(BoundSegments_loops Seg_loops, std::string name, int color)
	{
		LOGINFO("1109 - [%s] draw_svg_byPoints,Seg_loops.size() [%d]", name.c_str(), Seg_loops.size());
		std::vector<Point> points;
		int num = 0;
		for (int j = 0; j < Seg_loops.size(); j++)
		{
			for (int i = 0; i < Seg_loops[j].size(); i++)
			{
				Pointf3 tempPointf3 = Seg_loops[j][i].Segment_line.MidPoint();
				Point tempPoint = Point(scale_(tempPointf3.x), scale_(tempPointf3.y));
				points.push_back(tempPoint);
				num++;
			}
		}
		LOGINFO("1109 - [%s] draw_svg_byPoints,Points num = [%d]", name.c_str(), num);
		DWORD timecount = GetTickCount();
		LOGINFO("timecount is = %d", timecount);
		std::string time = boost::to_string(timecount);
		std::string newname = time + name;
		SVG svg(newname.c_str());
		if (color == 1)
		{
			svg.draw(points, "red", scale_(0.1));
		}
		else if (color == 2)
		{
			svg.draw(points, "yellow", scale_(0.1));
		}
		else if (color == 3)
		{
			svg.draw(points, "green", scale_(0.1));
		}
		else if (color == 4)
		{
			svg.draw(points, "blue", scale_(0.1));
		}

		svg.Close();
	}

	bool TriangleMesh::Get_RPDRegion_By_Facets(double filter_angle, int expand_degree, int back_expand_degree)
	{
		int _ret = 0;
		int expandnum = expand_degree;
		// 标记和退化面片
		DWORD timecount = GetTickCount();
		std::vector<int> facets_index = this->Get_facets(RPDStrong_Fact, false);
		this->Angle_FilterFacets(RPDStrong_Fact, facets_index, filter_angle); // 先用角度阈值过滤
		this->Area_FilterFacets(RPDStrong_Fact, facets_index);				  // 递归调用直到消除所有的退化面片
		LOGINFO("Area_FilterFacets =%d", GetTickCount() - timecount);

		// 边界无序线段提取
		timecount = GetTickCount();
		BoundSegments_vec boundline_segs;
		boundline_segs = this->Get_BoundSegments(RPDStrong_Fact);
		LOGINFO("Get_BoundSegments cost time %d", GetTickCount() - timecount);

		// 边界成环
		timecount = GetTickCount();
		BoundSegments_loops bound_loops;
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
				LOGINFO("find_oneLoop abondon seg!!!!!!!");
			}
			else
			{
				break;
			}
		}
		LOGINFO("find_oneLoops cost time [%d] bound_loops.size = %d Left boundline_segs size = %d",
				GetTickCount() - timecount,
				bound_loops.size(),
				boundline_segs.size());

		// 生成墙支撑点
		if (bound_loops.size() == 0)
		{
			return _ret;
		}
		// 对最大的外圈和最大的内圈做处理

		int MaxLoop_index = -1;
		int MaxHole_index = -1;
		this->GetMaxLoopAndHole(bound_loops, MaxLoop_index, MaxHole_index);
		LOGINFO("MaxLoop_index[%d] MaxHole_index[%d]", MaxLoop_index, MaxHole_index);
		BoundSegments_loops MaxLoop_head;
		BoundSegments_loops MaxLoop_back;
		BoundSegments_loops MaxHole_back;
		BoundSegments_loops tempMaxLoop_head;
		BoundSegments_loops tempMaxLoop_back;
		BoundSegments_loops tempMaxHole_head;
		BoundSegments_loops tempMaxHole_back;
		std::vector<int> MaxLoop_head_facets;
		std::vector<int> MaxLoop_back_facets;
		std::vector<int> MaxHole_back_facets;
		Linef3 midLinef3 = Linef3(Pointf3(0, 0, 0), Pointf3(0, 1, 0));
		if (MaxLoop_index != -1)
		{
			BoundSegments_loop MaxLoop = bound_loops[MaxLoop_index]; // 外圈
			LOGINFO("MaxLoop.size()[%d]", MaxLoop.size());
			if (MaxLoop.size()) // 对最大外圈进行处理
			{
				LOGINFO("MaxLoop CutLoopByMajorLoopAndHole before!");
				tempMaxLoop_head = this->CutLoopByMajorLoopAndHole(MaxLoop, midLinef3, true, true);
				tempMaxLoop_back = this->CutLoopByMajorLoopAndHole(MaxLoop, midLinef3, true, false);

				double MaxLoop_max_Y_head = this->Get_BoundLoops_Max_Y(tempMaxLoop_head);
				double MaxLoop_max_Y_back = this->Get_BoundLoops_Max_Y(tempMaxLoop_back);
				LOGINFO("1103 MaxLoop_max_Y_head[%f] MaxLoop_max_Y_back [%f]", MaxLoop_max_Y_head, MaxLoop_max_Y_back);
				if (Debug_Test_Point_Svg)
				{
					std::string headname = "MaxLoop_head_before.svg";
					std::string backname = "MaxLoop_back_before.svg";
					draw_svg_byBoundSegments_loops(tempMaxLoop_head, headname, 1);
					draw_svg_byBoundSegments_loops(tempMaxLoop_back, backname, 2);
				}
				if (MaxLoop_max_Y_head < MaxLoop_max_Y_back)
				{
					MaxLoop_head = tempMaxLoop_back;
					MaxLoop_back = tempMaxLoop_head;
				}
				else
				{
					MaxLoop_head = tempMaxLoop_head;
					MaxLoop_back = tempMaxLoop_back;
				}

				if (Debug_Test_Point_Svg)
				{
					std::string headname = "MaxLoop_head.svg";
					std::string backname = "MaxLoop_back.svg";
					draw_svg_byBoundSegments_loops(MaxLoop_head, headname, 3);
					draw_svg_byBoundSegments_loops(MaxLoop_back, backname, 4);
				}

				LOGINFO("1 MaxLoop_head.size()[%d]", MaxLoop_head.size());
				// 过滤特定类型的边界
				this->FilterLoopsByIPD(MaxLoop_head);
				MaxLoop_head_facets = this->ExpandSegLine2Facets(MaxLoop_head, expandnum);
				LOGINFO("MaxLoop_head_facets size = %d", MaxLoop_head_facets.size());

				LOGINFO("2 MaxLoop_back.size()[%d]", MaxLoop_back.size());
				this->FilterLoopsByIPD(MaxLoop_back);
				MaxLoop_back_facets = this->ExpandSegLine2Facets(MaxLoop_back, back_expand_degree);
				LOGINFO("MaxLoop_back_facets size = %d", MaxLoop_back_facets.size());
			}
		}

		if (MaxHole_index != -1)
		{

			BoundSegments_loop MaxHole = bound_loops[MaxHole_index]; // 内圈
			LOGINFO("MaxHole.size()[%d]", MaxHole.size());
			if (MaxHole.size()) // 对最大内圈进行处理
			{
				tempMaxHole_head = this->CutLoopByMajorLoopAndHole(MaxHole, midLinef3, false, true);
				tempMaxHole_back = this->CutLoopByMajorLoopAndHole(MaxHole, midLinef3, false, false);
				double MaxHole_max_Y_head = this->Get_BoundLoops_Max_Y(tempMaxHole_head);
				double MaxHole_max_Y_back = this->Get_BoundLoops_Max_Y(tempMaxHole_back);
				LOGINFO("1103-2 MaxHole_max_Y_head[%f] MaxHole_max_Y_back [%f]", MaxHole_max_Y_head, MaxHole_max_Y_back);
				if (MaxHole_max_Y_head < MaxHole_max_Y_back)
				{
					MaxHole_back = tempMaxHole_head;
				}
				else
				{
					MaxHole_back = tempMaxHole_back;
				}

				LOGINFO("MaxHole CutLoopByMajorLoopAndHole before!");
				this->FilterLoopsByIPD(MaxHole_back);
				MaxHole_back_facets = this->ExpandSegLine2Facets(MaxHole_back, expandnum);

				LOGINFO("MaxHole_back_facets size = %d", MaxHole_back_facets.size());
			}
		}
		if (Debug_Test_Svg)
		{
			std::vector<int> facets;
			facets.insert(facets.end(), MaxHole_back_facets.begin(), MaxHole_back_facets.end());
			facets.insert(facets.end(), MaxLoop_back_facets.begin(), MaxLoop_back_facets.end());
			facets.insert(facets.end(), MaxLoop_head_facets.begin(), MaxLoop_head_facets.end());
			DWORD timecount = GetTickCount();
			LOGINFO("timecount is = %d", timecount);
			std::string time = boost::to_string(timecount);
			std::string MaxLoop_heads = time + "_0828MaxLoop_head.svg";
			std::string MaxLoop_backs = time + "_0828MaxLoop_back.svg";
			std::string MaxHole_backs = time + "_0828MaxHole_back.svg";
			std::string allfacets = time + "_0828regionfacets.svg";
			this->draw_svg_byfacets(MaxLoop_head_facets, MaxLoop_heads, 1);
			this->draw_svg_byfacets(MaxLoop_back_facets, MaxLoop_backs, 2);
			this->draw_svg_byfacets(MaxHole_back_facets, MaxHole_backs, 3);
			this->draw_svg_byfacets(facets, allfacets, 4);
		}
		LOGINFO("0906 MaxLoop_head_facets [%d] size is [%d]", 0, MaxLoop_head_facets.size());
		LOGINFO("0906 MaxHole_back_facets [%d] size is [%d]", 1, MaxLoop_back_facets.size());
		LOGINFO("0906 MaxLoop_back_facets [%d] size is [%d]", 2, MaxLoop_back_facets.size());
		this->FilterFacetsByIPD(MaxLoop_head_facets);
		this->FilterFacetsByIPD(MaxLoop_back_facets);
		this->FilterFacetsByIPD(MaxHole_back_facets);
		LOGINFO("0906 MaxLoop_head_facets [%d] size is [%d]", 0, MaxLoop_head_facets.size());
		LOGINFO("0906 MaxHole_back_facets [%d] size is [%d]", 1, MaxLoop_back_facets.size());
		LOGINFO("0906 MaxLoop_back_facets [%d] size is [%d]", 2, MaxLoop_back_facets.size());
		if (Debug_Test_Svg)
		{
			std::vector<int> facets;
			facets.insert(facets.end(), MaxHole_back_facets.begin(), MaxHole_back_facets.end());
			facets.insert(facets.end(), MaxLoop_back_facets.begin(), MaxLoop_back_facets.end());
			facets.insert(facets.end(), MaxLoop_head_facets.begin(), MaxLoop_head_facets.end());
			DWORD timecount = GetTickCount();
			LOGINFO("timecount is = %d", timecount);
			std::string time = boost::to_string(timecount);
			std::string MaxLoop_heads = time + "_0901MaxLoop_head.svg";
			std::string MaxLoop_backs = time + "_0901MaxLoop_back.svg";
			std::string MaxHole_backs = time + "_0901MaxHole_back.svg";
			std::string allfacets = time + "_0901regionfacets.svg";
			this->draw_svg_byfacets(MaxLoop_head_facets, MaxLoop_heads, 1);
			this->draw_svg_byfacets(MaxLoop_back_facets, MaxLoop_backs, 2);
			this->draw_svg_byfacets(MaxHole_back_facets, MaxHole_backs, 3);
			this->draw_svg_byfacets(facets, allfacets, 4);
		}
		//  清空标记
		this->ResetMark_Selected();
		this->RPD_selected_facets.clear();
		this->RPD_selected_facets.push_back(MaxLoop_head_facets);
		this->RPD_selected_facets.push_back(MaxHole_back_facets);
		this->RPD_selected_facets.push_back(MaxLoop_back_facets);
		LOGINFO("0830 MaxLoop_head_facets [%d] size is [%d]", 0, this->RPD_selected_facets[0].size());
		LOGINFO("0830 MaxHole_back_facets [%d] size is [%d]", 1, this->RPD_selected_facets[1].size());
		LOGINFO("0830 MaxLoop_back_facets [%d] size is [%d]", 2, this->RPD_selected_facets[2].size());

		return true;
	}

	bool TriangleMesh::General_BoundWall_By_SelectedFacets(double Points_distance)
	{
		LOGINFO("0906 1");
		this->SelectedPoints.clear(); // 支撑点
		// this->ResetMark_Selected();

		//  参数要经过放大处理
		Points_distance = scale_(Points_distance);
		// 为选择的面片构造墙

		std::vector<int> facets_index = this->Get_facets(Selected_Fact, false);
		LOGINFO("1110 selected_facets_index1.size = %d", facets_index.size());
		this->Angle_FilterFacets(Selected_Fact, facets_index, 90.0); // 先用角度阈值过滤
		LOGINFO("selected_facets_index2.size = %d", facets_index.size());
		ExPolygons selected_exs = this->Get_SelectFacets_Ex();
		if (false)
		{
			SVG svg("selected_exs.svg");
			svg.draw(selected_exs, "red");
			svg.Close();
		}
		DWORD timecount = GetTickCount();
		LOGINFO("timecount is =  %d", timecount);
		if (Debug_Test_Svg)
		{
			std::string time = boost::to_string(timecount);
			std::string allfacets = time + "_0908_selected_exs.svg";
			SVG svg(allfacets.c_str());
			svg.draw(selected_exs, "red");
			svg.Close();
		}
		LOGINFO("0908 selected_exs.size() is = %d", selected_exs.size());
		if (selected_exs.size())
		{
			ExPolygons simply_expp;
			for (ExPolygons::const_iterator it = selected_exs.begin(); it != selected_exs.end(); ++it)
			{
				ExPolygons simply_expp_one;
				it->simplify(scale_(0.02), &simply_expp_one);

				if (unscale(it->area()) > scale_(1))
				{
					LOGINFO("0908 it->area() is = %f", unscale(it->area()));
					simply_expp.insert(simply_expp.end(), simply_expp_one.begin(), simply_expp_one.end());
				}
			}
			if (Debug_Test_Svg)
			{
				std::string time = boost::to_string(timecount);
				std::string allfacets = time + "_0908_simplify_exs.svg";
				SVG svg(allfacets.c_str());
				svg.draw(simply_expp, "orange");
				svg.Close();
			}
			simply_expp = offset_ex(simply_expp, scale_(-0.1));
			if (Debug_Test_Svg)
			{
				std::string time = boost::to_string(timecount);
				std::string allfacets = time + "_0912_offset_exs.svg";
				SVG svg(allfacets.c_str());
				svg.draw(simply_expp, "blue");
				svg.Close();
			}
			this->Make_ExCenterLine(simply_expp, "boundary_hexhole_wall", 0, Points_distance); // 暂时只有六边形孔墙有这个特殊性
		}
		// 重置所有面片的Selected_Fact信息
		this->ResetMark_Selected();
		return true;
	}

	// 按loop顶点个数进行过滤
	BoundSegments_loops
	TriangleMesh::CutLoopsBySize(BoundSegments_loops Seg_loops, int min_size)
	{
		BoundSegments_loops Filter_loops;
		for (unsigned int i = 0; i < Seg_loops.size(); i++)
		{
			BoundSegments_loop Seg_loop = Seg_loops[i];
			if (Seg_loop.size() >= min_size)
			{
				Filter_loops.push_back(Seg_loop);
			}
		}
		return Filter_loops;
	}

	// 按RPD type进行过滤
	BoundSegments_loops
	TriangleMesh::CutLoopsByRPDType(BoundSegments_loops Seg_loops, const stl_face_type _type)
	{
		BoundSegments_loops Filter_loops;
		for (unsigned int i = 0; i < Seg_loops.size(); i++)
		{
			BoundSegments_loop Seg_loop = Seg_loops[i];
			BoundSegments_loops Cut_Segloops = this->CutLoopByRPDType(Seg_loop, _type);
			Filter_loops.insert(Filter_loops.end(), Cut_Segloops.begin(), Cut_Segloops.end());
		}
		return Filter_loops;
	}

	// 按Face Mark进行过滤
	BoundSegments_loops
	TriangleMesh::CutLoopsByFaceMark(BoundSegments_loops Seg_loops, const char _type)
	{
		BoundSegments_loops Filter_loops;
		for (unsigned int i = 0; i < Seg_loops.size(); i++)
		{
			BoundSegments_loop Seg_loop = Seg_loops[i];
			BoundSegments_loops Cut_Segloops = this->CutLoopByFaceMark(Seg_loop, _type);
			Filter_loops.insert(Filter_loops.end(), Cut_Segloops.begin(), Cut_Segloops.end());
		}
		return Filter_loops;
	}

	BoundSegments_loops
	TriangleMesh::CutLoopByRPDType(BoundSegments_loop Seg_loop, const stl_face_type _type)
	{
		BoundSegments_loops _loops;
		if (Seg_loop.size() == 0)
			return _loops;
		BoundSegments_loop temp_one_loop;
		for (int i = 0; i < Seg_loop.size(); i++)
		{
			BoundSegment loopSeg = Seg_loop[i];
			if (this->IsSegRPDType(loopSeg, _type) == false)
			{
				temp_one_loop.push_back(loopSeg);
			}
			else if (temp_one_loop.size())
			{
				LOGINFO("temp_one_loop size = %d", temp_one_loop.size());
				_loops.push_back(temp_one_loop);
				temp_one_loop.clear();
			}
		}
		// 最后补一次
		if (temp_one_loop.size())
		{
			_loops.push_back(temp_one_loop);
			temp_one_loop.clear();
		}

		LOGINFO("CutLoopByRPDType loops size = %d", _loops.size());
		return _loops;
	}

	BoundSegments_loops
	TriangleMesh::CutLoopByFaceMark(BoundSegments_loop Seg_loop, const char _type)
	{
		BoundSegments_loops _loops;
		if (Seg_loop.size() == 0)
			return _loops;
		BoundSegments_loop temp_one_loop;
		for (int i = 0; i < Seg_loop.size(); i++)
		{
			BoundSegment loopSeg = Seg_loop[i];
			if (this->IsSegFaceMark(loopSeg, _type) == false)
			{
				temp_one_loop.push_back(loopSeg);
			}
			else if (temp_one_loop.size())
			{
				LOGINFO("temp_one_loop size = %d", temp_one_loop.size());
				_loops.push_back(temp_one_loop);
				temp_one_loop.clear();
			}
		}
		// 最后补一次
		if (temp_one_loop.size())
		{
			_loops.push_back(temp_one_loop);
			temp_one_loop.clear();
		}

		LOGINFO("CutLoopByFaceMark loops size = %d", _loops.size());
		return _loops;
	}

	bool
	TriangleMesh::IsFaceNerRPDType(int Mark_faceIndex, const stl_face_type _type)
	{
		for (int j = 0; j <= 2; j++)
		{
			int near_facet_index = this->stl.neighbors_start[Mark_faceIndex].neighbor[j];
			stl_facet &near_facet = this->stl.facet_start[near_facet_index];
			if (near_facet.face_type == _type)
				return true;
		}
		return false;
	}

	bool
	TriangleMesh::IsFaceNerFaceMark(int Mark_faceIndex, const char _type)
	{
		for (int j = 0; j <= 2; j++)
		{
			int near_facet_index = this->stl.neighbors_start[Mark_faceIndex].neighbor[j];
			stl_facet &near_facet = this->stl.facet_start[near_facet_index];
			if (TriangleMesh::fact_is_mark(near_facet.extra, _type))
				return true;
		}
		return false;
	}

	//
	bool
	TriangleMesh::IsSegRPDType(BoundSegment loopSeg, const stl_face_type _type)
	{
		int Mark_faceIndex = loopSeg.FacetID.Mark_facetIndex;
		int NoMark_faceIndex = loopSeg.FacetID.NoMark_facetIndex;
		stl_facet &facet_Mark = this->stl.facet_start[Mark_faceIndex];
		stl_facet &facet_NoMark = this->stl.facet_start[NoMark_faceIndex];
		if (facet_Mark.face_type == _type || facet_NoMark.face_type == _type)
			return true;
		if (this->IsFaceNerRPDType(Mark_faceIndex, _type))
			return true;
		if (this->IsFaceNerRPDType(NoMark_faceIndex, _type))
			return true;

		return false;
	}

	bool
	TriangleMesh::IsSegFaceMark(BoundSegment loopSeg, const char _type)
	{
		int Mark_faceIndex = loopSeg.FacetID.Mark_facetIndex;
		int NoMark_faceIndex = loopSeg.FacetID.NoMark_facetIndex;
		stl_facet &facet_Mark = this->stl.facet_start[Mark_faceIndex];
		stl_facet &facet_NoMark = this->stl.facet_start[NoMark_faceIndex];
		if (TriangleMesh::fact_is_mark(facet_Mark.extra, _type) || TriangleMesh::fact_is_mark(facet_NoMark.extra, _type))
			return true;
		if (this->IsFaceNerFaceMark(Mark_faceIndex, _type))
			return true;
		if (this->IsFaceNerFaceMark(NoMark_faceIndex, _type))
			return true;

		return false;
	}

	// 按封闭圈过滤
	BoundSegments_loops
	TriangleMesh::CutLoopsByLoop(BoundSegments_loops Seg_loops)
	{
		BoundSegments_loops Filter_loops;
		for (unsigned int i = 0; i < Seg_loops.size(); i++)
		{
			BoundSegments_loop Seg_loop = Seg_loops[i];
			const bool IsCircle = (Seg_loop.back().Segment_line.b == Seg_loop.front().Segment_line.a);
			if (IsCircle)
			{
				BoundSegments_loops Cut_Segloops = this->CutLoop(Seg_loop);
				Filter_loops.insert(Filter_loops.end(), Cut_Segloops.begin(), Cut_Segloops.end());
			}
			else
			{
				Filter_loops.push_back(Seg_loop);
			}
		}
		return Filter_loops;
	}

	BoundSegments_loops
	TriangleMesh::CutLoop(BoundSegments_loop Seg_loop)
	{
		BoundSegments_loops _loops;
		int loopPoint_size = Seg_loop.size();
		if (loopPoint_size == 0)
			return _loops;
		BoundSegments_loop temp_one_loop;
		int loopPoint_end = loopPoint_size;
		if (loopPoint_size > 5)
		{
			loopPoint_end -= 5;
		}
		else
		{
			loopPoint_end -= 1;
		}
		for (int i = 0; i < loopPoint_end; i++)
		{
			BoundSegment loopSeg = Seg_loop[i];
			int Mark_faceIndex = loopSeg.FacetID.Mark_facetIndex;
			int NoMark_faceIndex = loopSeg.FacetID.NoMark_facetIndex;
			stl_facet &facet_Mark = this->stl.facet_start[Mark_faceIndex];
			stl_facet &facet_NoMark = this->stl.facet_start[NoMark_faceIndex];
			temp_one_loop.push_back(loopSeg);
		}
		// 最后补一次
		if (temp_one_loop.size())
		{
			_loops.push_back(temp_one_loop);
			temp_one_loop.clear();
		}

		LOGINFO("CutLoopByRPDTypebound loops size = %d", _loops.size());

		return _loops;
	}

	// 按Face angle进行过滤
	BoundSegments_loops
	TriangleMesh::CutLoopsByFaceAngle(BoundSegments_loops Seg_loops, double filter_angle)
	{
		BoundSegments_loops Filter_loops;
		for (unsigned int i = 0; i < Seg_loops.size(); i++)
		{
			BoundSegments_loop Seg_loop = Seg_loops[i];
			// 按角度阈值切割loop
			BoundSegments_loops Cut_Segloops = this->CutLoopByFaceAngle(Seg_loop, filter_angle);
			Filter_loops.insert(Filter_loops.end(), Cut_Segloops.begin(), Cut_Segloops.end());
		}
		return Filter_loops;
	}

	BoundSegments_loops
	TriangleMesh::CutLoopByFaceAngle(BoundSegments_loop Seg_loop, double filter_angle)
	{
		BoundSegments_loops _loops;
		if (Seg_loop.size() == 0)
			return _loops;
		BoundSegments_loop temp_one_loop;
		for (int i = 0; i < Seg_loop.size(); i++)
		{
			BoundSegment loopSeg = Seg_loop[i];
			int Mark_faceIndex = loopSeg.FacetID.Mark_facetIndex;
			int NoMark_faceIndex = loopSeg.FacetID.NoMark_facetIndex;
			stl_facet &facet_Mark = this->stl.facet_start[Mark_faceIndex];
			stl_facet &facet_NoMark = this->stl.facet_start[NoMark_faceIndex];
			double facet_Mark_angle = stl_calculate_faceangle(&facet_Mark);
			double facet_NoMark_angle = stl_calculate_faceangle(&facet_NoMark);

			// if (facet_Mark_angle < filter_angle || facet_NoMark_angle < filter_angle  ||
			//	facet_Mark.face_type == OcclusalRest_Part || facet_NoMark.face_type == OcclusalRest_Part ||
			//	facet_Mark.face_type == MinorConnector_Part || facet_NoMark.face_type == MinorConnector_Part)
			if (facet_Mark_angle < filter_angle || facet_NoMark_angle < filter_angle)
			{
				temp_one_loop.push_back(loopSeg);
			}
			else if (temp_one_loop.size())
			{
				LOGINFO("facet_Mark out of angle rangle, normal[%f,%f,%f], angle = %f",
						facet_Mark.normal.x,
						facet_Mark.normal.y,
						facet_Mark.normal.z,
						facet_Mark_angle);
				LOGINFO("facet_NoMark out of angle rangle, normal[%f,%f,%f], angle = %f",
						facet_NoMark.normal.x,
						facet_NoMark.normal.y,
						facet_NoMark.normal.z,
						facet_NoMark_angle);
				_loops.push_back(temp_one_loop);
				temp_one_loop.clear();
			}
		}
		// 最后补一次
		if (temp_one_loop.size())
		{
			_loops.push_back(temp_one_loop);
			temp_one_loop.clear();
		}

		LOGINFO("CutLoopByFaceAngle loops size = %d", _loops.size());

		return _loops;
	}

	// 判断曲线在z向是顺时针还是逆时针
	bool
	TriangleMesh::Is_ClockWise(BoundSegments_loop Seg_loop)
	{
		// 构造Polygon
		Polygon poly_temp;
		for (int i = 0; i < Seg_loop.size(); i++)
		{
			Linef3 SegLine = Seg_loop[i].Segment_line;
			Point temp_Point;
			temp_Point.x = scale_(SegLine.a.x);
			temp_Point.y = scale_(SegLine.a.y);
			poly_temp.points.push_back(temp_Point);
		}

		Point temp_Point;
		temp_Point.x = scale_(Seg_loop.back().Segment_line.b.x);
		temp_Point.y = scale_(Seg_loop.back().Segment_line.b.y);
		poly_temp.points.push_back(temp_Point);

		return poly_temp.is_clockwise();
	}

	bool
	TriangleMesh::GetMaxLoopAndHole(BoundSegments_loops Seg_loops, int &MaxLoop_index, int &MaxHole_index)
	{
		int MaxLoopSize = 0;
		int MaxHoleSize = 0;
		MaxLoop_index = -1;
		MaxHole_index = -1;
		if (Seg_loops.size() == 0)
			return false;
		for (int i = 0; i < Seg_loops.size(); i++)
		{
			BoundSegments_loop Seg_loop = Seg_loops[i];
			if (this->Is_ClockWise(Seg_loop))
			{
				// LOGINFO("BoundSegments_loop[%d] is clockWise", Seg_loop.size());
				if (Seg_loop.size() > MaxLoopSize)
				{
					MaxLoopSize = Seg_loop.size();
					MaxLoop_index = i;
				}
			}
			else
			{
				// LOGINFO("BoundSegments_loop[%d] is Count clockWise", Seg_loop.size());
				if (Seg_loop.size() > MaxHoleSize)
				{
					MaxHoleSize = Seg_loop.size();
					MaxHole_index = i;
				}
			}
		}
		return true;
	}

	// 对主loop和hole进行删减
	BoundSegments_loops
	TriangleMesh::CutLoopsByMajorLoopAndHole(BoundSegments_loops Seg_loops, Linef3 midLinef3)
	{
		//
		BoundSegments_loops _loops;
		if (Seg_loops.size() == 0)
			return _loops;
		int MaxLoop_index = -1;
		int MaxHole_index = -1;
		this->GetMaxLoopAndHole(Seg_loops, MaxLoop_index, MaxHole_index);

		/*
		// 拷贝剩下的圈数
		for (int i = 0; i < Seg_loops.size(); i++)
		{
			if (i == MaxHole_index || i == MaxLoop_index)
				continue;
			_loops.push_back(Seg_loops[i]);
		}
		*/

		if (MaxLoop_index != -1)
		{
			BoundSegments_loop MaxLoop = Seg_loops[MaxLoop_index]; // 外圈
			if (MaxLoop.size())									   // 对最大外圈进行处理
			{
				LOGINFO("MaxLoop CutLoopByMajorLoopAndHole before!");
				BoundSegments_loops _res = this->CutLoopByMajorLoopAndHole(MaxLoop, midLinef3, true, true);
				_loops.insert(_loops.end(), _res.begin(), _res.end());
				LOGINFO("MaxLoop CutLoopByMajorLoopAndHole after! _res size = %d", _res.size());
				_res = this->CutLoopByMajorLoopAndHole(MaxLoop, midLinef3, true, false);
				_loops.insert(_loops.end(), _res.begin(), _res.end());
				LOGINFO("MaxLoop CutLoopByMajorLoopAndHole after! _res size = %d", _res.size());
			}
		}

		if (MaxHole_index != -1)
		{
			BoundSegments_loop MaxHole = Seg_loops[MaxHole_index]; // 内圈
			if (MaxHole.size())									   // 对最大内圈进行处理
			{
				LOGINFO("MaxHole CutLoopByMajorLoopAndHole before!");
				BoundSegments_loops _res = this->CutLoopByMajorLoopAndHole(MaxHole, midLinef3, false, false);
				_loops.insert(_loops.end(), _res.begin(), _res.end());
				LOGINFO("MaxHole CutLoopByMajorLoopAndHole after! _res size = %d", _res.size());
			}
		}

		return _loops;
	}

	// 对主loop和hole进行删减 用于自动加强支撑
	BoundSegments_loops
	TriangleMesh::CutLoopsByMajorLoopAndHole_oldversion(BoundSegments_loops Seg_loops, Linef3 midLinef3)
	{
		//
		BoundSegments_loops _loops;
		if (Seg_loops.size() == 0)
			return _loops;
		int MaxLoop_index = -1;
		int MaxHole_index = -1;
		this->GetMaxLoopAndHole(Seg_loops, MaxLoop_index, MaxHole_index);
		// 拷贝剩下的圈数
		for (int i = 0; i < Seg_loops.size(); i++)
		{
			if (i == MaxHole_index || i == MaxLoop_index)
				continue;
			_loops.push_back(Seg_loops[i]);
		}

		if (MaxLoop_index != -1)
		{
			BoundSegments_loop MaxLoop = Seg_loops[MaxLoop_index]; // 外圈
			if (MaxLoop.size())									   // 对最大外圈进行处理
			{
				LOGINFO("MaxLoop CutLoopByMajorLoopAndHole before!");
				BoundSegments_loops MaxLoop_head;
				BoundSegments_loops _res = this->CutLoopByMajorLoopAndHole_oldversion(MaxLoop, midLinef3, true);
				BoundSegments_loops _res2 = this->CutLoopByMajorLoopAndHole_oldversion(MaxLoop, midLinef3, false);
				double MaxLoop_max_Y_head = this->Get_BoundLoops_Max_Y(_res);
				double MaxLoop_max_Y_back = this->Get_BoundLoops_Max_Y(_res2);
				if (MaxLoop_max_Y_head < MaxLoop_max_Y_back)
				{
					MaxLoop_head = _res2;
				}
				else
				{
					MaxLoop_head = _res;
				}
				_loops.insert(_loops.end(), MaxLoop_head.begin(), MaxLoop_head.end());
				LOGINFO("MaxLoop CutLoopByMajorLoopAndHole after! _res size = %d", MaxLoop_head.size());
			}
		}

		if (MaxHole_index != -1)
		{
			BoundSegments_loop MaxHole = Seg_loops[MaxHole_index]; // 内圈
			if (MaxHole.size())									   // 对最大内圈进行处理
			{
				LOGINFO("MaxHole CutLoopByMajorLoopAndHole before!");
				BoundSegments_loops _res = this->CutLoopByMajorLoopAndHole_oldversion(MaxHole, midLinef3, false);
				_loops.insert(_loops.end(), _res.begin(), _res.end());
				LOGINFO("MaxHole CutLoopByMajorLoopAndHole after! _res size = %d", _res.size());
			}
		}

		return _loops;
	}

	BoundSegments_loops TriangleMesh::ExpandSegLine_by_Facets(std::vector<int> region_facets, int Expend_Num)
	{
		BoundSegments_loops _loops;
		if (region_facets.size() == 0)
			return _loops;
		std::vector<int> ExpandedFacets = region_facets;
		LOGINFO("0818 ExpandedFacets = %d", ExpandedFacets.size());
		BoundSegments_loops Seg_loop_Expand = this->ExpandedFacets2SegLine(ExpandedFacets);
		_loops.insert(_loops.end(), Seg_loop_Expand.begin(), Seg_loop_Expand.end());
		LOGINFO("0818 expanded_loops = %d", _loops.size());
		return _loops;
	}
	std::vector<int> TriangleMesh::ExpandSegLine2Facets(BoundSegments_loops Seg_loops, int Expend_Num)
	{
		LOGINFO("0818 _loops = %d", Seg_loops.size());
		std::vector<int> resExpandedFacets;
		if (Seg_loops.size() == 0)
			return resExpandedFacets;

		for (int i = 0; i < Seg_loops.size(); i++)
		{
			BoundSegments_loop Seg_loop = Seg_loops[i];
			std::vector<int> ExpandedFacets = ExpandSegLine2Facets_Region(Seg_loop, Expend_Num);
			LOGINFO("0818 [%d] ExpandedFacets = %d", i, ExpandedFacets.size());
			resExpandedFacets.insert(resExpandedFacets.end(), ExpandedFacets.begin(), ExpandedFacets.end());
			LOGINFO("0818 [%d] resExpandedFacets = %d", i, resExpandedFacets.size());
		}
		LOGINFO("0818 resExpandedFacets = %d", resExpandedFacets.size());
		return resExpandedFacets;
	}
	BoundSegments_loops TriangleMesh::ExpandedFacets2SegLine(std::vector<int> ExpandedFacets)
	{
		// 清空标记
		for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
		{
			stl_facet &facet = this->stl.facet_start[i];
			TriangleMesh::fact_remove_mark(facet.extra, Selected_Fact);
		}

		// 开始标记
		for (int i = 0; i < ExpandedFacets.size(); i++)
		{
			int fact_index = ExpandedFacets[i];
			stl_facet &facet = this->stl.facet_start[fact_index];
			TriangleMesh::fact_set_mark(facet.extra, Selected_Fact);
		}

		std::vector<int> facets_index = this->Get_facets(Selected_Fact, false);
		LOGINFO("0818 facets_index = %d", facets_index.size());

		// 边界无序线段提取
		BoundSegments_vec boundline_segs;
		boundline_segs = this->Get_BoundSegments(Selected_Fact);
		// 边界成环

		BoundSegments_loops bound_loops;
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
				LOGINFO("find_oneLoop abondon seg!!!!!!!");
			}
			else
			{
				break;
			}
		}
		// 移除标记
		for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
		{
			stl_facet &facet = this->stl.facet_start[i];
			TriangleMesh::fact_remove_mark(facet.extra, Selected_Fact);
		}

		LOGINFO("0818 find_oneLoops -- bound_loops.size = %d Left boundline_segs size = %d",
				bound_loops.size(),
				boundline_segs.size());
		return bound_loops;
	}

	std::pair<int, int>
	TriangleMesh::CutBoundLoop_by_MidPoint(BoundSegments_loop Seg_loop, Linef3 midLinef3, bool IsLoop)
	{
		if (Seg_loop.size() == 0)
			return std::pair<int, int>(-1, -1);
		const bool IsCircle = (Seg_loop.back().Segment_line.b == Seg_loop.front().Segment_line.a);
		// 计算loop中心起点 降维计算
		Pointf midLine_vector(midLinef3.b.x - midLinef3.a.x, midLinef3.b.y - midLinef3.a.y);
		midLine_vector.normalize(); // 中线方向
		Pointf OtherLine_vector = midLine_vector;
		OtherLine_vector.rotate(PI / 2); // 垂直于中线方向
		OtherLine_vector.normalize();
		// 中线最低点索引
		int LowestPointIndex = -1;
		double LowestPointVal = 99999;
		for (int i = 0; i < Seg_loop.size(); i++)
		{
			Linef3 segline = Seg_loop[i].Segment_line;
			Pointf3 midPoint = segline.MidPoint();
			// a,b 沿水平方向的分量
			double a_OtherVal = segline.a.x * OtherLine_vector.x + segline.a.y * OtherLine_vector.y;
			double b_OtherVal = segline.b.x * OtherLine_vector.x + segline.b.y * OtherLine_vector.y;
			// line mid 沿垂直方向分量
			double midPoint_MidVal = midPoint.x * midLine_vector.x + midPoint.y * midLine_vector.y;
			// 线段越过中线	且最低
			if (a_OtherVal * b_OtherVal < 0 && midPoint_MidVal < LowestPointVal)
			{
				LowestPointVal = midPoint_MidVal;
				LowestPointIndex = i;
			}
		}
		// 未找到中线分界点
		if (LowestPointIndex == -1)
		{
			LOGINFO("LowestPointIndex == -1");
			return std::pair<int, int>(0, 0);
		}
		LOGINFO("LowestPointIndex = %d [%f,%f,%f]",
				LowestPointIndex,
				Seg_loop[LowestPointIndex].Segment_line.MidPoint().x,
				Seg_loop[LowestPointIndex].Segment_line.MidPoint().y,
				Seg_loop[LowestPointIndex].Segment_line.MidPoint().z);

		// 从中线分界最低点，向两侧搜索
		const int loop_size = Seg_loop.size();
		const int _step = 1;
		const int Max_Count = loop_size > 40 ? 5 : 1;
		int _Count = 0;						// 连续计数器
		int LastPointID = LowestPointIndex; // 起始位置
		int LoopRightID = (loop_size + LowestPointIndex + _step) % loop_size;
		int dynamic_MaxCount = Max_Count;
		int while_stepLen = 0;
		while (IsCircle || LoopRightID < loop_size)
		{ // 转圈查找

			// 防止死循环
			if (while_stepLen >= loop_size) // 转了一圈
			{
				while_stepLen = 0;
				dynamic_MaxCount--;
				if (dynamic_MaxCount < 0) // 异常
				{
					LOGINFO("Error dynamic_MaxCount = %d", dynamic_MaxCount);
					break;
				}
			}

			Pointf3 _vec = Seg_loop[LoopRightID].Segment_line.MidPoint() - Seg_loop[LastPointID].Segment_line.MidPoint();
			double horline_val = _vec.x * OtherLine_vector.x + _vec.y * OtherLine_vector.y;
			double midline_val = _vec.x * midLine_vector.x + _vec.y * midLine_vector.y;
			if ((IsLoop == true && horline_val < 0 && midline_val > 0) || (IsLoop == false && horline_val > 0 && midline_val > 0))
			{
				if (_Count < dynamic_MaxCount)
					_Count++;
				else
					break;
			}
			else
				_Count = 0;
			// LOGINFO("IsLoop[%d]  LoopRightID[%d/%d]  horline[%f]midline[%f]  Count[%d]", IsLoop, LoopRightID, loop_size, horline_val, midline_val, _Count);
			//  向前查找
			LastPointID = LoopRightID;
			LoopRightID = (loop_size + LoopRightID + _step) % loop_size;
			while_stepLen += _step; // 循环的计数
		}
		// 补偿回退
		LoopRightID = (loop_size + LoopRightID - dynamic_MaxCount) % loop_size;

		_Count = 0;						// 连续计数器
		LastPointID = LowestPointIndex; // 起始位置
		int LoopLeftID = (loop_size + LowestPointIndex - _step) % loop_size;
		dynamic_MaxCount = Max_Count;
		while_stepLen = 0;
		while (IsCircle || LoopLeftID >= 0)
		{ // 转圈查找
			// 防止死循环
			if (while_stepLen >= loop_size) // 转了一圈
			{
				while_stepLen = 0;
				dynamic_MaxCount--;
				LOGINFO("New Circle dynamic_MaxCount = %d", dynamic_MaxCount);
				if (dynamic_MaxCount < 0) // 异常
				{
					LOGINFO("Error dynamic_MaxCount = %d", dynamic_MaxCount);
					break;
				}
			}

			Pointf3 _vec = Seg_loop[LoopLeftID].Segment_line.MidPoint() - Seg_loop[LastPointID].Segment_line.MidPoint();
			double horline_val = _vec.x * OtherLine_vector.x + _vec.y * OtherLine_vector.y;
			double midline_val = _vec.x * midLine_vector.x + _vec.y * midLine_vector.y;
			if ((IsLoop == true && horline_val > 0 && midline_val > 0) || (IsLoop == false && horline_val < 0 && midline_val > 0))
			{
				if (_Count < dynamic_MaxCount)
					_Count++;
				else
					break;
			}
			else
				_Count = 0;
			// LOGINFO("IsLoop[%d]  LoopLeftID[%d/%d]  horline[%f]midline[%f]  Count[%d]", IsLoop, LoopLeftID, loop_size, horline_val, midline_val, _Count);
			//  向后递进
			LastPointID = LoopLeftID;
			LoopLeftID = (loop_size + LoopLeftID - _step) % loop_size;
			while_stepLen += _step; // 循环的计数
		}
		// 补偿回退
		LoopLeftID = (loop_size + LoopLeftID + dynamic_MaxCount) % loop_size;
		LOGINFO("res IsLoop[%d]  LoopLeftID [%d] LoopRightID [%d]", IsLoop, LoopLeftID, LoopRightID);
		return std::pair<int, int>(LoopLeftID, LoopRightID);
	}

	std::pair<int, int> TriangleMesh::CutBoundLoop_by_LeftRight(BoundSegments_loop Seg_loop, Linef3 midLinef3)
	{
		if (Seg_loop.size() == 0)
			return std::pair<int, int>(-1, -1);
		const bool IsCircle = (Seg_loop.back().Segment_line.b == Seg_loop.front().Segment_line.a);
		// 计算loop中心起点 降维计算
		Pointf midLine_vector(midLinef3.b.x - midLinef3.a.x, midLinef3.b.y - midLinef3.a.y);
		midLine_vector.normalize(); // 中线方向
		Pointf OtherLine_vector = midLine_vector;
		OtherLine_vector.rotate(PI / 2); // 垂直于中线方向
		OtherLine_vector.normalize();
		// 中线最低点索引
		int LeftPointIndex = -1;
		double LeftPointVal = 99999;
		int RightPointIndex = -1;
		double RightPointVal = -99999;
		for (int i = 0; i < Seg_loop.size(); i++)
		{
			Linef3 segline = Seg_loop[i].Segment_line;
			Pointf3 midPoint = segline.MidPoint();
			// a,b 沿水平方向的分量
			double a_OtherVal = segline.a.x * OtherLine_vector.x + segline.a.y * OtherLine_vector.y;
			double b_OtherVal = segline.b.x * OtherLine_vector.x + segline.b.y * OtherLine_vector.y;
			// line mid 沿垂直方向分量
			double midPoint_MidVal = midPoint.x * midLine_vector.x + midPoint.y * midLine_vector.y;
			double midPoint_horVal = midPoint.x * OtherLine_vector.x + midPoint.y * OtherLine_vector.y;
			// 线段越过中线	且最低
			if (midPoint_horVal < LeftPointVal)
			{
				LeftPointVal = midPoint_horVal;
				LeftPointIndex = i;
			}
			if (midPoint_horVal > RightPointVal)
			{
				RightPointVal = midPoint_horVal;
				RightPointIndex = i;
			}
		}
		return std::pair<int, int>(LeftPointIndex, RightPointIndex);
	}
	// midLinef3 a--->b 方向为正
	BoundSegments_loops
	TriangleMesh::CutLoopByMajorLoopAndHole_oldversion(BoundSegments_loop Seg_loop, Linef3 midLinef3, bool IsLoop)
	{
		//
		BoundSegments_loops _loops;
		if (Seg_loop.size() == 0)
			return _loops;
		const bool IsCircle = (Seg_loop.back().Segment_line.b == Seg_loop.front().Segment_line.a);

		std::pair<int, int> cut_pair = this->CutBoundLoop_by_MidPoint(Seg_loop, midLinef3, IsLoop);
		int LoopLeftID = cut_pair.first;
		int LoopRightID = cut_pair.second;
		if (LoopLeftID == LoopRightID && LoopLeftID == 0)
		{
			_loops.push_back(Seg_loop);
			return _loops;
		}
		LOGINFO("CutLoopByMajorLoopAndHole Seg_loop.size() = %d [%d,%d]", Seg_loop.size(), LoopLeftID, LoopRightID);
		// 计算结果
		if (IsCircle) // 环形loop
		{
			if (IsLoop) // 外环  取两端  LoopRightID--》LoopLeftID
			{
				if (LoopRightID < LoopLeftID)
				{
					BoundSegments_loop segLoop;
					segLoop.insert(segLoop.end(), Seg_loop.begin() + LoopRightID, Seg_loop.begin() + LoopLeftID);
					_loops.push_back(segLoop);
				}
				else
				{
					BoundSegments_loop segLoop;
					segLoop.insert(segLoop.end(), Seg_loop.begin() + LoopRightID, Seg_loop.end());
					segLoop.insert(segLoop.end(), Seg_loop.begin(), Seg_loop.begin() + LoopLeftID);
					_loops.push_back(segLoop);
				}
			}
			else // 内环  取中间 LoopLeftID--》LoopRightID
			{
				if (LoopLeftID < LoopRightID)
				{
					BoundSegments_loop segLoop;
					segLoop.insert(segLoop.end(), Seg_loop.begin() + LoopLeftID, Seg_loop.begin() + LoopRightID);
					_loops.push_back(segLoop);
				}
				else
				{
					BoundSegments_loop segLoop;
					segLoop.insert(segLoop.end(), Seg_loop.begin() + LoopLeftID, Seg_loop.end());
					segLoop.insert(segLoop.end(), Seg_loop.begin(), Seg_loop.begin() + LoopRightID);
					_loops.push_back(segLoop);
				}
			}
		}
		else // 非环形loop
		{
			int minID = LoopLeftID < LoopRightID ? LoopLeftID : LoopRightID;
			int maxID = LoopLeftID < LoopRightID ? LoopRightID : LoopLeftID;
			if (IsLoop) // 外环  取两端
			{
				BoundSegments_loop segLoop_1;
				BoundSegments_loop segLoop_2;
				segLoop_1.insert(segLoop_1.end(), Seg_loop.begin(), Seg_loop.begin() + minID);
				segLoop_2.insert(segLoop_2.end(), Seg_loop.begin() + maxID, Seg_loop.end());
				_loops.push_back(segLoop_1);
				_loops.push_back(segLoop_2);
			}
			else // 内环  取中间
			{
				BoundSegments_loop segLoop_1;
				segLoop_1.insert(segLoop_1.end(), Seg_loop.begin() + minID, Seg_loop.begin() + maxID);
				_loops.push_back(segLoop_1);
			}
		}

		return _loops;
	}
	double
	TriangleMesh::Get_BoundLoops_Max_Y(BoundSegments_loops Seg_loops)
	{
		if (Seg_loops.size() == 0)
		{
			return 0.0;
		}
		double max_y = -DBL_MAX;
		double sum = 0.0;
		size_t num = 0;
		for (int j = 0; j < Seg_loops.size(); j++)
		{
			for (int i = 0; i < Seg_loops[j].size(); i++)
			{
				if (max_y < Seg_loops[j][i].Segment_line.MidPoint().y)
				{
					max_y = Seg_loops[j][i].Segment_line.MidPoint().y;
				}
			}
		}
		return max_y;
	}
	// midLinef3 a--->b 方向为正
	BoundSegments_loops
	TriangleMesh::CutLoopByMajorLoopAndHole(BoundSegments_loop Seg_loop, Linef3 midLinef3, bool IsLoop, bool IsFront)
	{
		//
		BoundSegments_loops _loops;
		if (Seg_loop.size() == 0)
			return _loops;
		const bool IsCircle = (Seg_loop.back().Segment_line.b == Seg_loop.front().Segment_line.a);

		std::pair<int, int> cut_pair = this->CutBoundLoop_by_MidPoint(Seg_loop, midLinef3, IsLoop);
		int LoopLeftID = cut_pair.first;
		int LoopRightID = cut_pair.second;
		if (LoopLeftID == LoopRightID && LoopLeftID == 0)
		{
			_loops.push_back(Seg_loop);
			return _loops;
		}
		LOGINFO("CutLoopByMajorLoopAndHole Seg_loop.size() = %d [%d,%d]", Seg_loop.size(), LoopLeftID, LoopRightID);
		// 计算结果
		if (IsCircle) // 环形loop
		{
			if (IsFront) // 外环  取两端  LoopRightID--》LoopLeftID
			{
				if (LoopRightID < LoopLeftID)
				{
					BoundSegments_loop segLoop;
					segLoop.insert(segLoop.end(), Seg_loop.begin() + LoopRightID, Seg_loop.begin() + LoopLeftID);
					_loops.push_back(segLoop);
				}
				else
				{
					BoundSegments_loop segLoop;
					segLoop.insert(segLoop.end(), Seg_loop.begin() + LoopRightID, Seg_loop.end());
					segLoop.insert(segLoop.end(), Seg_loop.begin(), Seg_loop.begin() + LoopLeftID);
					_loops.push_back(segLoop);
				}
			}
			else // 内环  取中间 LoopLeftID--》LoopRightID
			{
				if (LoopLeftID < LoopRightID)
				{
					BoundSegments_loop segLoop;
					segLoop.insert(segLoop.end(), Seg_loop.begin() + LoopLeftID, Seg_loop.begin() + LoopRightID);
					_loops.push_back(segLoop);
				}
				else
				{
					BoundSegments_loop segLoop;
					segLoop.insert(segLoop.end(), Seg_loop.begin() + LoopLeftID, Seg_loop.end());
					segLoop.insert(segLoop.end(), Seg_loop.begin(), Seg_loop.begin() + LoopRightID);
					_loops.push_back(segLoop);
				}
			}
		}
		else // 非环形loop
		{
			int minID = LoopLeftID < LoopRightID ? LoopLeftID : LoopRightID;
			int maxID = LoopLeftID < LoopRightID ? LoopRightID : LoopLeftID;
			if (IsFront) // 外环  取两端
			{
				BoundSegments_loop segLoop_1;
				BoundSegments_loop segLoop_2;
				segLoop_1.insert(segLoop_1.end(), Seg_loop.begin(), Seg_loop.begin() + minID);
				segLoop_2.insert(segLoop_2.end(), Seg_loop.begin() + maxID, Seg_loop.end());
				_loops.push_back(segLoop_1);
				_loops.push_back(segLoop_2);
			}
			else // 内环  取中间
			{
				BoundSegments_loop segLoop_1;
				segLoop_1.insert(segLoop_1.end(), Seg_loop.begin() + minID, Seg_loop.begin() + maxID);
				_loops.push_back(segLoop_1);
			}
		}

		return _loops;
	}

}

//检查两条线段是否相邻的函数byGuoXiao
bool TriangleMesh::isAdjacent(const Linef3& l1, const Linef3& l2)
{
   return l1.a.EslipEqual(l2.b);
}

//从给定的线段开始深度优先搜索以查找环byGuoXiao
bool TriangleMesh::dfs(BoundLines_vec &Orgin_Lines, int current, int start, std::unordered_set<int>& visited, std::stack<int>& path)
{
    visited.insert(current);
    path.push(current);

    for (int next = 0; next < Orgin_Lines.size(); ++next)
    {
        if (next != current && isAdjacent(Orgin_Lines[current], Orgin_Lines[next]))
        {
            if (visited.find(next) != visited.end())
            {
                if (next == start)
                {
                    // 找到了环
                    return true;
                }
            }
            else
             {
                if (dfs(Orgin_Lines, next, start, visited, path))
                {
                    return true;
                }
            }
        }
    }

    // 如果没有找到环，则回溯
    path.pop();
    return false;
}

//在线段集合中检测是否存在环byGuoXiao
bool TriangleMesh::hasCycle(BoundLines_vec &Orgin_Lines, std::vector<int>& cyclePath, int start)
{
  std::unordered_set<int> visited;
  std::stack<int> path;

  if (dfs(Orgin_Lines, start, start, visited, path))
  {
      while (!path.empty())
      {
          cyclePath.push_back(path.top());
          path.pop();
      }
      return true;
  }
  return false;
}

//从原始线段集合中移除环中的线段byGuoXiao
void TriangleMesh::removeCycleLines(BoundLines_vec &Orgin_Lines, const vector<int>& cyclePath)
{
  std::unordered_set<int> cycleLines(cyclePath.begin(), cyclePath.end());
  BoundLines_vec newLines;
  for (int i = 0; i < Orgin_Lines.size(); ++i)
  {
      if (cycleLines.find(i) == cycleLines.end())
      {
          newLines.push_back(Orgin_Lines[i]);
      }
  }
  Orgin_Lines = newLines;
}