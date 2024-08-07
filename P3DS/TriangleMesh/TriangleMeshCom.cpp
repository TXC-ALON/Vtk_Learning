#include "TriangleMesh.hpp"
#include <float.h>
#include "SVG.hpp"
#include <unordered_set>
#include <queue>
namespace Slic3r
{

	size_t
	TriangleMesh::get_tree_region_idx(size_t spted_volume_idx, size_t contor_idx, size_t hole_idx)
	{
		return (spted_volume_idx) * 256 * 256 * 256 * 256 * 256 + (contor_idx) * 256 * 256 * 256 + (hole_idx);
	};

	// 面片标记位操作
	bool TriangleMesh::fact_is_mark(char extrainfo, char Mark)
	{
		extrainfo = extrainfo & Mark;
		return (extrainfo == Mark);
	}

	void TriangleMesh::fact_set_mark(char &extrainfo, char Mark)
	{
		extrainfo = extrainfo | Mark;
	}

	void TriangleMesh::fact_remove_mark(char &extrainfo, char Mark)
	{
		Mark = ~Mark;
		extrainfo = extrainfo & Mark;
	}

	void TriangleMesh::fact_reset(char &extrainfo)
	{
		extrainfo = extrainfo & Norm_Fact;
	}

	bool TriangleMesh::is_face_rpd_mark(char extrainfo, char Mark)
	{
		return extrainfo == Mark;
	}

	void TriangleMesh::set_face_rpd_mark(char &extrainfo, char Mark)
	{
		extrainfo = Mark;
	}

	void TriangleMesh::reset_face_rpd_mark(char &extrainfo)
	{
		extrainfo = extrainfo & Norm_Fact;
	}

	void TriangleMesh::parse_face_mark(int mark, int &side, int &type, int &type_id)
	{
		if (mark <= 0)
		{
			side = 0;
			type = 0;
			type_id = 0;
			return;
		}

		int wan = mark / 10000;
		int qian = (mark % 10000) / 1000;
		int bai = (mark % 1000) / 100;
		int shi = (mark % 100) / 10;
		int ge = (mark % 10);

		type_id = wan * 10 + qian;
		type = bai * 10 + shi;
		side = ge;
	}

	int TriangleMesh::parse_mesh_type(bool upper)
	{
		int type = 0;

		int clasps = 0;
		int linar = 0;
		for (int i = 0; i < stl.stats.number_of_facets; i++)
		{
			int face_type = stl.facet_start[i].face_type;
			if (face_type == Clasp_Part || face_type == OcclusalRest_Part)
				clasps++;
			else if (face_type == LingualBar_Part)
				linar++;
		}

		if (upper)
		{
			if (clasps == 0)
				type = 2;
			else
				type = 1;
		}
		else
		{
			if (linar == 0)
				type = 4;
			else
				type = 3;
		}

		return type;
	}

	std::vector<int> TriangleMesh::MarkFacts_SelectedPoint(Pointf3 pickPos)
	{
		std::vector<int> modify_facets_index;
		int fact_id = this->GetFacetIdxByPickPoint(pickPos);
		if (this->FaceIndexValid(fact_id) == false)
		{
			LOGINFO("pickPos[%s] no fact_id %d", pickPos.dump_perl().c_str(), fact_id);
			return modify_facets_index;
		}

		stl_facet &facet = this->stl.facet_start[fact_id];
		// LOGINFO("pickPos[%s] fact_id %d normal[%f,%f,%f]", pickPos.dump_perl().c_str(), fact_id,
		//	facet.normal.x, facet.normal.y, facet.normal.z);
		if (facet.normal.z > 0.0)
			return modify_facets_index;
		modify_facets_index = this->ExpandFacetbyRing(fact_id, 3);
		//
		std::vector<int> _ret;
		for (int i = 0; i < modify_facets_index.size(); i++)
		{
			int fact_index = modify_facets_index[i];
			if (this->FaceIndexValid(fact_index) == false)
				continue;
			stl_facet &facet = this->stl.facet_start[fact_index];
			if (facet.normal.z <= 0.0)
			{
				TriangleMesh::fact_set_mark(facet.extra, Selected_Fact);
				_ret.push_back(fact_index);
			}
		}

		return _ret;
	}

	std::vector<int> TriangleMesh::HoleFaceMarkFacts_SelectedPoint(Pointf3 pickPos)
    {
        std::vector<int> modify_facets_index;
        int fact_id = this->GetFacetIdxByPickPoint(pickPos);
        if (this->FaceIndexValid(fact_id) == false)
        {
            LOGINFO("pickPos[%s] no fact_id %d", pickPos.dump_perl().c_str(), fact_id);
            return modify_facets_index;
        }

        stl_facet &facet = this->stl.facet_start[fact_id];
        // LOGINFO("pickPos[%s] fact_id %d normal[%f,%f,%f]", pickPos.dump_perl().c_str(), fact_id,
        //	facet.normal.x, facet.normal.y, facet.normal.z);
//    		if (facet.normal.z > 0.0)
//    			return modify_facets_index;
        modify_facets_index = this->ExpandFacetbyRing(fact_id, 3);
        //
        std::vector<int> _ret;
        for (int i = 0; i < modify_facets_index.size(); i++)
        {
            int fact_index = modify_facets_index[i];
            if (this->FaceIndexValid(fact_index) == false)
                continue;
            stl_facet &facet = this->stl.facet_start[fact_index];
//    			if (facet.normal.z <= 0.0)
//    			{
                TriangleMesh::fact_set_mark(facet.extra, Selected_Fact);
                _ret.push_back(fact_index);
            //}
        }

        return _ret;
    }

	std::vector<int> TriangleMesh::RemoveFacts_SelectedPoint(Pointf3 pickPos)
	{
		std::vector<int> modify_facets_index;
		int fact_id = this->GetFacetIdxByPickPoint(pickPos);
		if (fact_id < 0 || fact_id >= this->facets_count())
		{
			LOGINFO("pickPos[%s] no fact_id %d", pickPos.dump_perl().c_str(), fact_id);
			return modify_facets_index;
		}
		LOGINFO("pickPos[%s] fact_id %d", pickPos.dump_perl().c_str(), fact_id);
		stl_facet &facet = this->stl.facet_start[fact_id];
		if (facet.normal.z > 0.0)
			return modify_facets_index;
		modify_facets_index = this->ExpandFacetbyRing(fact_id, 3);
		//
		std::vector<int> _ret;
		for (int i = 0; i < modify_facets_index.size(); i++)
		{
			int fact_index = modify_facets_index[i];
			if (this->FaceIndexValid(fact_index) == false)
				continue;
			stl_facet &facet = this->stl.facet_start[fact_index];
			TriangleMesh::fact_remove_mark(facet.extra, Selected_Fact);
			_ret.push_back(fact_index);
		}

		return _ret;
	}
	std::vector<int> TriangleMesh::MarkHole_SelectedPoint(Pointf3 pickPos,const std::set<int>& LimitedFaces)
	{
		LOGINFO("LimitedFaces.size %d", LimitedFaces.size());
		std::vector<int> modify_facets_index;
		int fact_id = this->GetFacetIdxByPickPoint(pickPos);
		if (this->FaceIndexValid(fact_id) == false)
		{
			LOGINFO("pickPos[%s] no fact_id %d", pickPos.dump_perl().c_str(), fact_id);
			return modify_facets_index;
		}

		stl_facet& facet = this->stl.facet_start[fact_id];
		// LOGINFO("pickPos[%s] fact_id %d normal[%f,%f,%f]", pickPos.dump_perl().c_str(), fact_id,
		//	facet.normal.x, facet.normal.y, facet.normal.z);
		/*if (facet.normal.z > 0.0)
			return modify_facets_index;*/
		modify_facets_index = this->ExpandFacetbyRing(fact_id, 3);
		this->draw_Plane(fact_id,LimitedFaces);
		modify_facets_index.clear();
		std::copy(Selected_Triangles.begin(), Selected_Triangles.end(), std::back_inserter(modify_facets_index));
		//
		std::vector<int> _ret;
		for (int i = 0; i < modify_facets_index.size(); i++)
		{
			int fact_index = modify_facets_index[i];
			if (this->FaceIndexValid(fact_index) == false)
				continue;
			stl_facet& facet = this->stl.facet_start[fact_index];
			TriangleMesh::fact_set_mark(facet.extra, Selected_Fact);
			_ret.push_back(fact_index);
		}

		return _ret;
	}


	/*
	std::vector<int> TriangleMesh::MarkFacts_SelectedPoint(Pointf3 pickPos)
	{
		std::vector<int> modify_facets_index;
		double angel_throld = 75;
		int fact_id = this->GetFacetIdxByPickPoint(pickPos);
		if (fact_id < 0 || fact_id >= this->facets_count())
		{
			LOGINFO("pickPos[%s] no fact_id %d", pickPos.dump_perl().c_str(), fact_id);
			return modify_facets_index;
		}
		LOGINFO("pickPos[%s] fact_id %d", pickPos.dump_perl().c_str(), fact_id);
		stl_facet& facet = this->stl.facet_start[fact_id];
		if (acos(-facet.normal.z) * 180 <= (90 * PI))
		{
			TriangleMesh::fact_set_mark(facet.extra, Selected_Fact);
			modify_facets_index.push_back(fact_id);
			for (int i = 0; i < 3; i++)
			{
				int ner_id = this->stl.neighbors_start[fact_id].neighbor[i];
				if (ner_id == -1)
					continue;
				stl_facet& near_facet = this->stl.facet_start[ner_id];
				if (acos(-near_facet.normal.z) * 180 <= (90 * PI))
				{
					TriangleMesh::fact_set_mark(near_facet.extra, Selected_Fact);
					modify_facets_index.push_back(ner_id);
					for (int j = 0; j < 3; j++)
					{
						int nerner_id = this->stl.neighbors_start[ner_id].neighbor[j];
						if (nerner_id == -1)
							continue;
						stl_facet& nerner_facet = this->stl.facet_start[nerner_id];
						if (acos(-nerner_facet.normal.z) * 180 <= (angel_throld * PI))
						{
							TriangleMesh::fact_set_mark(nerner_facet.extra, Selected_Fact);
							modify_facets_index.push_back(nerner_id);
							for (int k = 0; k < 3; k++)
							{
								int nernerner_id = this->stl.neighbors_start[nerner_id].neighbor[k];
								if (nernerner_id == -1)
									continue;
								stl_facet& nernerner_facet = this->stl.facet_start[nernerner_id];
								if (acos(-nernerner_facet.normal.z) * 180 <= (angel_throld * PI))
								{
									TriangleMesh::fact_set_mark(nernerner_facet.extra, Selected_Fact);
									modify_facets_index.push_back(nernerner_id);
								}
							}
						}
					}
				}
			}
		}

		//// 反向退化
		// std::vector<int> facets_index;
		// facets_index = this->Get_facets(Selected_Fact, true);
		// this->reArea_FilterFacets(Selected_Fact, facets_index);
		//// 正向退化
		// facets_index = this->Get_facets(Selected_Fact, false);
		// this->Area_FilterFacets(Selected_Fact, facets_index);

		// 去重
		std::sort(modify_facets_index.begin(), modify_facets_index.end());
		auto index_it = std::unique(modify_facets_index.begin(), modify_facets_index.end());
		modify_facets_index.erase(index_it, modify_facets_index.end());

		return modify_facets_index;
	}
	*/

	/*
	std::vector<int> TriangleMesh::RemoveFacts_SelectedPoint(Pointf3 pickPos)
	{
		std::vector<int> modify_facets_index;
		int fact_id = this->GetFacetIdxByPickPoint(pickPos);
		if (fact_id < 0 || fact_id >= this->facets_count())
		{
			LOGINFO("pickPos[%s] no fact_id %d", pickPos.dump_perl().c_str(), fact_id);
			return modify_facets_index;
		}
		LOGINFO("pickPos[%s] fact_id %d", pickPos.dump_perl().c_str(), fact_id);
		stl_facet &facet = this->stl.facet_start[fact_id];
		TriangleMesh::fact_remove_mark(facet.extra, Selected_Fact);
		modify_facets_index.push_back(fact_id);
		for (int i = 0; i < 3; i++)
		{
			int ner_id = this->stl.neighbors_start[fact_id].neighbor[i];
			if (ner_id == -1)
				continue;
			TriangleMesh::fact_remove_mark(this->stl.facet_start[ner_id].extra, Selected_Fact);
			modify_facets_index.push_back(ner_id);
			for (int j = 0; j < 3; j++)
			{
				int nerner_id = this->stl.neighbors_start[ner_id].neighbor[j];
				if (nerner_id == -1)
					continue;
				TriangleMesh::fact_remove_mark(this->stl.facet_start[nerner_id].extra, Selected_Fact);
				modify_facets_index.push_back(nerner_id);
				for (int k = 0; k < 3; k++)
				{
					int nernerner_id = this->stl.neighbors_start[nerner_id].neighbor[k];
					if (nernerner_id == -1)
						continue;
					TriangleMesh::fact_remove_mark(this->stl.facet_start[nernerner_id].extra, Selected_Fact);
					modify_facets_index.push_back(nernerner_id);
				}
			}
		}

		// 去重
		std::sort(modify_facets_index.begin(), modify_facets_index.end());
		auto index_it = std::unique(modify_facets_index.begin(), modify_facets_index.end());
		modify_facets_index.erase(index_it, modify_facets_index.end());

		return modify_facets_index;
	}
	*/

	void TriangleMesh::Clear_SelectFacts()
	{
		for (unsigned int i = 0; i < this->facets_count(); i++)
		{
			stl_facet &facet = this->stl.facet_start[i];
			TriangleMesh::fact_remove_mark(facet.extra, Selected_Fact);
		}
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	///  计算碰撞点算法
	////////////////////////////////////////////////////////////////////////////////////////////////////////

	// 通过面片ID获取该面片的法向量
	Pointf3 TriangleMesh::GetFacetNormal(int Fact_Idx)
	{
		if (Fact_Idx > this->facets_count() - 1 || Fact_Idx < 0)
			return Pointf3(0, 0, -1);
		stl_normal facet_normal = this->stl.facet_start[Fact_Idx].normal;
		return Pointf3(facet_normal.x, facet_normal.y, facet_normal.z);
	}

	char TriangleMesh::GetFacetExtra(int Fact_Idx)
	{
		if (Fact_Idx > this->facets_count() - 1 || Fact_Idx < 0)
			return '0';

		return this->stl.facet_start[Fact_Idx].extra;
	}

#define T_DELTA (0.01)
	// Determine whether point P in triangle ABC
	bool TriangleMesh::Pre_PointinTriangle(int Fact_Idx, stl_vertex Zaixs, Ray_type ray_type)
	{
		if (Fact_Idx > this->facets_count() - 1 || Fact_Idx < 0)
			return false;

		// check X
		if (ray_type != axis_X)
		{
			if ((Zaixs.x - T_DELTA > this->stl.facet_start[Fact_Idx].vertex[0].x) &&
				(Zaixs.x - T_DELTA > this->stl.facet_start[Fact_Idx].vertex[1].x) &&
				(Zaixs.x - T_DELTA > this->stl.facet_start[Fact_Idx].vertex[2].x))
				return false;
			if ((Zaixs.x + T_DELTA < this->stl.facet_start[Fact_Idx].vertex[0].x) &&
				(Zaixs.x + T_DELTA < this->stl.facet_start[Fact_Idx].vertex[1].x) &&
				(Zaixs.x + T_DELTA < this->stl.facet_start[Fact_Idx].vertex[2].x))
				return false;
		}

		// check Y
		if (ray_type != axis_Y)
		{
			if ((Zaixs.y - T_DELTA > this->stl.facet_start[Fact_Idx].vertex[0].y) &&
				(Zaixs.y - T_DELTA > this->stl.facet_start[Fact_Idx].vertex[1].y) &&
				(Zaixs.y - T_DELTA > this->stl.facet_start[Fact_Idx].vertex[2].y))
				return false;
			if ((Zaixs.y + T_DELTA < this->stl.facet_start[Fact_Idx].vertex[0].y) &&
				(Zaixs.y + T_DELTA < this->stl.facet_start[Fact_Idx].vertex[1].y) &&
				(Zaixs.y + T_DELTA < this->stl.facet_start[Fact_Idx].vertex[2].y))
				return false;
		}

		// check Z
		if (ray_type != axis_Z)
		{
			if ((Zaixs.z - T_DELTA > this->stl.facet_start[Fact_Idx].vertex[0].z) &&
				(Zaixs.z - T_DELTA > this->stl.facet_start[Fact_Idx].vertex[1].z) &&
				(Zaixs.z - T_DELTA > this->stl.facet_start[Fact_Idx].vertex[2].z))
				return false;
			if ((Zaixs.z + T_DELTA < this->stl.facet_start[Fact_Idx].vertex[0].z) &&
				(Zaixs.z + T_DELTA < this->stl.facet_start[Fact_Idx].vertex[1].z) &&
				(Zaixs.z + T_DELTA < this->stl.facet_start[Fact_Idx].vertex[2].z))
				return false;
		}

		return true;
	}

	bool TriangleMesh::PointinTriangle(int Fact_Idx, stl_vertex P, Ray_type ray_type)
	{
		if (Fact_Idx > this->facets_count() - 1 || Fact_Idx < 0)
			return false;

		stl_vertex A = this->stl.facet_start[Fact_Idx].vertex[0];
		stl_vertex B = this->stl.facet_start[Fact_Idx].vertex[1];
		stl_vertex C = this->stl.facet_start[Fact_Idx].vertex[2];

		Vector3 v0(C.x - A.x, C.y - A.y, C.z - A.z);
		Vector3 v1(B.x - A.x, B.y - A.y, B.z - A.z);
		Vector3 v2(P.x - A.x, P.y - A.y, P.z - A.z);

		if (ray_type == axis_Z)
			v0.z = v1.z = v2.z = 0;
		else if (ray_type == axis_X)
			v0.x = v1.x = v2.x = 0;
		else if (ray_type == axis_Y)
			v0.y = v1.y = v2.y = 0;

		double dot00 = v0.Dot(v0);
		double dot01 = v0.Dot(v1);
		double dot02 = v0.Dot(v2);
		double dot11 = v1.Dot(v1);
		double dot12 = v1.Dot(v2);

		double inverDeno = 1 / (dot00 * dot11 - dot01 * dot01);

		double u = (dot11 * dot02 - dot01 * dot12) * inverDeno;
		if (u < 0 || u > 1) // if u out of range, return directly
			return false;

		double v = (dot00 * dot12 - dot01 * dot02) * inverDeno;
		if (v < 0 || v > 1) // if v out of range, return directly
			return false;

		// LOGINFO("Fact_Idx = %d, u = [%f], v = [%f], u+v = [%f]", Fact_Idx, u, v, u + v);
		return u + v <= 1.000000;
	}

	// 找到和直线相交的三角面片集合
	std::vector<int> TriangleMesh::GetFacetIdxByZaxis(double xpos, double ypos)
	{
		std::vector<int> reval;
		reval.clear();
		stl_vertex Zaixs;
		Zaixs.x = xpos;
		Zaixs.y = ypos;
		Zaixs.z = 0;

		for (int Fact_Idx = 0; Fact_Idx < this->facets_count(); Fact_Idx++)
		{
			if (this->Pre_PointinTriangle(Fact_Idx, Zaixs, Ray_type::axis_Z))
				if (this->PointinTriangle(Fact_Idx, Zaixs, Ray_type::axis_Z))
					reval.push_back(Fact_Idx);
		}

		return reval;
	}

	// 找到拾取点所在的三角面片
	int TriangleMesh::GetFacetIdxByPickPoint(Pointf3 pickPos)
	{
		stl_vertex Zaixs;
		Zaixs.x = pickPos.x;
		Zaixs.y = pickPos.y;
		Zaixs.z = pickPos.z;

		for (int Fact_Idx = 0; Fact_Idx < this->facets_count(); Fact_Idx++)
		{
			if (this->Pre_PointinTriangle(Fact_Idx, Zaixs, Ray_type::axis_None))
				if (this->PointinTriangle(Fact_Idx, Zaixs, Ray_type::axis_None))
					return Fact_Idx;
		}
		// 未找到合适的面片 找出距离点最近的面片
		std::vector<int> vec_ret = this->GetFacetIdxByZaxis(pickPos.x, pickPos.y);
		int retvel_idx = -1;
		double minZdistance = DBL_MAX;
		for (std::vector<int>::iterator Facet_It = vec_ret.begin(); Facet_It != vec_ret.end(); Facet_It++)
		{
			double Zdistance = this->GetPointZByZaxis(*Facet_It, pickPos.x, pickPos.y);
			Zdistance = std::abs(Zdistance - pickPos.z);
			if (Zdistance < minZdistance)
			{
				minZdistance = Zdistance;
				retvel_idx = *Facet_It;
			}
		}

		return retvel_idx;
	}

#define IS_DOUBLE_ZERO(d) (std::abs(d) < 0.00000001)

	// normal = [A,B,C];
	// A(x-x0)+B(y-y0)+C(z-z0)=0
	double TriangleMesh::GetPointZByZaxis(int Fact_Idx, double xpos, double ypos)
	{
		if (Fact_Idx > this->facets_count() - 1 || Fact_Idx < 0)
			return 0.0;

		stl_vertex pA = this->stl.facet_start[Fact_Idx].vertex[0];
		stl_vertex pB = this->stl.facet_start[Fact_Idx].vertex[1];
		stl_vertex pC = this->stl.facet_start[Fact_Idx].vertex[2];
		// LOGINFO("Fact_Idx = %d, A(%f,%f,%f) ,B(%f,%f,%f), C(%f,%f,%f)",
		//	Fact_Idx, pA.x, pA.y, pA.z, pB.x, pB.y, pB.z, pC.x, pC.y, pC.z);

		Vector3 v0(pC.x - pA.x, pC.y - pA.y, pC.z - pA.z);
		Vector3 v1(pB.x - pA.x, pB.y - pA.y, pB.z - pA.z);

		Vector3 normal = v0.Cross(v1);
		normal.normalize();

		double A = normal.x;
		double B = normal.y;
		double C = normal.z;

		if (IS_DOUBLE_ZERO(C))
		{
			LOGINFO("C is Zero !!! C = %f", C);
			float Zmin = std::min(pA.z, pB.z);
			Zmin = std::min(Zmin, pC.z);
			return Zmin;
		}

		double _rev = (A * (pA.x - xpos) / C + B * (pA.y - ypos) / C + pA.z);

		Vector3 t1 = Vector3(pA.x, pA.y, pA.z);
		Vector3 t2 = Vector3(pB.x, pB.y, pB.z);
		Vector3 t3 = Vector3(pC.x, pC.y, pC.z);
		TriangleFace3 temp_face = TriangleFace3(t1, t2, t3);
		Vector3 l1 = Vector3(xpos, ypos, -1000);
		Vector3 l2 = Vector3(xpos, ypos, 1000);
		Vector3 hitPoint = temp_face.cale_hitPoint(l1, l2);
		// LOGINFO("hitPos[%f, %f, %f] hitPos_other[%f, %f, %f]", xpos, ypos, _rev, hitPoint.x, hitPoint.y, hitPoint.z);
		return _rev;
	}

	// 计算经过点xpos，ypos的平行Z轴射线与Mesh面片相交的交点z值集合
	std::vector<stl_point_in_facet> TriangleMesh::GetZvecByZaxis(double xpos, double ypos)
	{
		std::vector<stl_point_in_facet> reval_vec;
		std::list<stl_point_in_facet> reval_list;
		reval_vec.clear();
		reval_list.clear();

		std::vector<int> idx_vec;
		idx_vec = this->GetFacetIdxByZaxis(xpos, ypos);
		for (int i = 0; i < idx_vec.size(); i++)
		{
			stl_point_in_facet onePoint;
			onePoint.facet_idx = idx_vec[i];
			onePoint.onepoint.x = xpos;
			onePoint.onepoint.y = ypos;
			onePoint.onepoint.z = this->GetPointZByZaxis(idx_vec[i], xpos, ypos);
			onePoint.normal = this->stl.facet_start[idx_vec[i]].normal;

			reval_list.push_back(onePoint);
		}
		reval_list.sort();
		std::copy(reval_list.begin(), reval_list.end(), std::back_inserter(reval_vec));

		return reval_vec;
	}

	//
	std::vector<SPT_pointf> *TriangleMesh::get_points(std::string _type)
	{
		if (_type == "grid")
		{
			return &(this->GirdPoints);
		}
		else if (_type == "boundary")
		{
			return &(this->BoundaryPoints);
		}
		else if (_type == "boundary_hexhole_wall")
		{
			return &(this->BoundaryPoints);
		}
		else if (_type == "stripe")
		{
			return &(this->tuqiPoints);
		}
		else if (_type == "tree" || _type == "fc_treebase")
		{
			return &(this->TreePoints);
		}
		else if (_type == "overhang")
		{
			return &(this->OverhangPoints);
		}
		else if (_type == "overhangline")
		{
			return &(this->OverhangLinesPoints);
		}
		else if (_type == "nummark" || _type == "charmark")
		{
			return &(this->numPoints);
		}
		else if (_type == "selected")
		{
			return &(this->SelectedPoints);
		}
		else if (_type == "custom_inner")
		{
			return &(this->custom_GridPoints);
		}
		else if (_type == "crossbar_tree")
		{
			return &(this->CrossBar_TreePoints);
		}
		else
		{
			LOGINFO("type error [%s]", _type.c_str());
		}

		return NULL;
	}

	void
	TriangleMesh::simplify_TreeBase()
	{
		std::vector<SPT_pointf>::iterator TreeBase_It = this->TreePoints.begin();
		while (TreeBase_It != this->TreePoints.end())
		{
			if (TreeBase_It->IsTreeBase())
			{
				TreeBase_It++;
			}
			else
			{
				TreeBase_It = this->TreePoints.erase(TreeBase_It);
			}
		}
	}

	void
	TriangleMesh::support_points_simplify_byCircle(std::string s_type, std::string d_type, double min_distance)
	{
		DWORD timecount = GetTickCount();

		if (IS_DOUBLE_ZERO(min_distance))
		{
			LOGINFO("min_distance is zero!");
			return;
		}

		std::vector<SPT_pointf> *p_sptVec_s = this->get_points(s_type);
		std::vector<SPT_pointf> *p_sptVec_d = this->get_points(d_type);
		if (p_sptVec_s == NULL || p_sptVec_s->size() < 1)
		{
			LOGINFO(" Get s_type [%s]spt vec error size < 1!", s_type.c_str());
			return;
		}
		if (p_sptVec_d == NULL || p_sptVec_d->size() < 1)
		{
			LOGINFO("Get d_type [%s]spt vec error size < 1!", d_type.c_str());
			return;
		}

		size_t s_size = p_sptVec_s->size();
		size_t d_size = p_sptVec_d->size();

		// 删除过近点
		for (int i = 0; i < p_sptVec_s->size(); i++)
		{
			std::vector<SPT_pointf>::iterator s_It = p_sptVec_s->begin() + i;
			for (int j = 0; j < p_sptVec_d->size(); j++)
			{
				std::vector<SPT_pointf>::iterator d_It = p_sptVec_d->begin() + j;
				if (p_sptVec_d == p_sptVec_s && i == j)
					continue;
				// 两个点之间距离小于阈值
				if (s_It->hitpoint.distance_to(d_It->hitpoint) < min_distance)
				{
					d_It->delete_BranchInfo();
					p_sptVec_d->erase(d_It);
					j--;
				}
			}
		}

		LOGINFO("differ [%s] (%d)<--> [%s] (%d) = %d  cost time[%d]",
				s_type.c_str(), s_size - p_sptVec_s->size(), d_type.c_str(), d_size - p_sptVec_d->size(), GetTickCount() - timecount);
	}

	void
	TriangleMesh::support_points_simplify_byBox(std::string s_type, std::string d_type, double X_distance, double Y_distance)
	{
		DWORD timecount = GetTickCount();

		if (IS_DOUBLE_ZERO(X_distance) || IS_DOUBLE_ZERO(Y_distance))
		{
			LOGINFO("X_distance or Y_distance is zero!");
			return;
		}

		std::vector<SPT_pointf> *p_sptVec_s = this->get_points(s_type);
		std::vector<SPT_pointf> *p_sptVec_d = this->get_points(d_type);
		if (p_sptVec_s == NULL || p_sptVec_s->size() < 1)
		{
			LOGINFO(" Get s_type [%s]spt vec error size < 1!", s_type.c_str());
			return;
		}
		if (p_sptVec_d == NULL || p_sptVec_d->size() < 1)
		{
			LOGINFO("Get d_type [%s]spt vec error size < 1!", d_type.c_str());
			return;
		}

		size_t s_size = p_sptVec_s->size();
		size_t d_size = p_sptVec_d->size();

		// 删除过近点
		for (int i = 0; i < p_sptVec_s->size(); i++)
		{
			std::vector<SPT_pointf>::iterator s_It = p_sptVec_s->begin() + i;
			for (int j = 0; j < p_sptVec_d->size(); j++)
			{
				std::vector<SPT_pointf>::iterator d_It = p_sptVec_d->begin() + j;
				if (p_sptVec_d == p_sptVec_s && i == j)
					continue;
				// 两个点之间距离小于阈值
				if (
					std::fabs(s_It->hitpoint.x - d_It->hitpoint.x) < X_distance && std::fabs(s_It->hitpoint.y - d_It->hitpoint.y) < Y_distance)
				{
					d_It->delete_BranchInfo();
					p_sptVec_d->erase(d_It);
					j--;
				}
			}
		}

		LOGINFO("differ [%s] (%d)<--> [%s] (%d) = %d  cost time[%d]",
				s_type.c_str(), s_size - p_sptVec_s->size(), d_type.c_str(), d_size - p_sptVec_d->size(), GetTickCount() - timecount);
	}

	bool
	TriangleMesh::support_points_simplify(
		double boundary_distance,
		double grid_distance,
		double tuqi_distance,
		double overhang_distance,
		int min_percent,
		int marknum)
	{
		if (min_percent <= 0)
			return true;

		double min_boundary_distance = boundary_distance * min_percent / 100;
		double min_grid_distance = grid_distance * min_percent / 100;
		double min_tuqi_distance = tuqi_distance * min_percent / 100;
		double min_overhang_distance = overhang_distance * min_percent / 100;
		double min_nummark_X_distance = 0.1;
		double min_nummark_Y_distance = 1.4;
		int _marknum = marknum; // 做一份拷贝
		while (_marknum > 0)
		{
			_marknum /= 10;
			min_nummark_X_distance += 0.7;
		}

		LOGINFO("首先分别检测各类型支撑间距!");
		LOGINFO("min_boundary_distance = [%f]", min_boundary_distance);
		LOGINFO("min_grid_distance = [%f]", min_grid_distance);
		LOGINFO("min_tuqi_distance = [%f]", min_tuqi_distance);
		LOGINFO("min_overhang_distance = [%f]", min_overhang_distance);
		// 首先分别检测各类型支撑间距
		// 边界点
		this->support_points_simplify_byCircle("boundary", "boundary", min_boundary_distance);
		// 网格
		this->support_points_simplify_byCircle("grid", "grid", min_grid_distance);
		// 凸起
		this->support_points_simplify_byCircle("stripe", "stripe", min_tuqi_distance);
		// 悬垂线
		this->support_points_simplify_byCircle("overhangline", "overhangline", min_overhang_distance);

		if (marknum >= 0) // 对于侧壁支撑来说 不需要化简
		{
			// 数字--》边界点
			// this->support_points_simplify_byBox("nummark", "boundary", min_nummark_X_distance, min_nummark_Y_distance);
			// 数字--》网格
			this->support_points_simplify_byBox("nummark", "grid", min_nummark_X_distance, min_nummark_Y_distance);
			// 数字--》花纹
			this->support_points_simplify_byBox("nummark", "stripe", min_nummark_X_distance, min_nummark_Y_distance);
			// 数字--》悬垂点
			this->support_points_simplify_byBox("nummark", "overhang", min_nummark_X_distance, min_nummark_Y_distance);
			// 数字--》悬垂线
			this->support_points_simplify_byBox("nummark", "overhangline", min_nummark_X_distance, min_nummark_Y_distance);
		}

		/*
		// 悬垂点--》悬垂线
		this->support_points_simplify_byCircle("overhang", "overhangline", min_overhang_distance);
		// 悬垂点--》边界
		this->support_points_simplify_byCircle("overhang", "boundary", min_overhang_distance);
		// 悬垂点--》网格
		this->support_points_simplify_byCircle("overhang", "grid", min_overhang_distance);
		// 悬垂点--》花纹
		this->support_points_simplify_byCircle("overhang", "stripe", min_overhang_distance);
		*/
		// 悬垂线--》边界
		this->support_points_simplify_byCircle("overhangline", "boundary", min_overhang_distance);
		// 悬垂线--》网格
		this->support_points_simplify_byCircle("overhangline", "grid", min_overhang_distance);
		// 悬垂线--》花纹
		this->support_points_simplify_byCircle("overhangline", "stripe", min_overhang_distance);
		// 花纹--》网格
		this->support_points_simplify_byCircle("stripe", "grid", min_grid_distance);

		// 化简树基
		this->simplify_TreeBase();

		return true;
	}

	// 对某个支撑点计算交线上的交点
	bool TriangleMesh::Cale_CollsionZ_byXY(const Pointf basePos, Pointf3 &hitPos, Pointf3 &hitNor)
	{
		SPT_pointf sptP(basePos, 0, 0);
		this->Cale_CollsionZ(sptP);
		if (sptP.CollsionZ_map.size() == 0)
			return false;

		hitPos.x = basePos.x;
		hitPos.y = basePos.y;
		hitPos.z = sptP.CollsionZ_map.begin()->first;

		hitNor = this->GetFacetNormal(sptP.CollsionZ_map.begin()->second);

		return true;
	}

	bool TriangleMesh::Cale_CollsionZ(unsigned int _index, std::string _type)
	{
		std::vector<SPT_pointf> *pPoints = this->get_points(_type);
		if (pPoints == NULL)
			return false;

		if (_index >= pPoints->size())
			return false;
		DWORD dwStart = GetTickCount();
		this->Cale_CollsionZ(pPoints->at(_index));
		LOGINFO("%s 计算支撑交线[%d/%d] 耗时【%u】", _type.c_str(), _index, pPoints->size(), GetTickCount() - dwStart);

		return true;
	}

	bool TriangleMesh::Cale_CollsionZ_bythreads(std::string _type, unsigned int _threads)
	{
		std::vector<SPT_pointf> *pPoints = this->get_points(_type);
		if (pPoints == NULL || pPoints->size() == 0)
		{
			LOGINFO("pPoints == NULL || pPoints->size() == 0 [%s]", _type.c_str());
			return false;
		}
		LOGINFO("pPoints->size() == %d [%s]", pPoints->size(), _type.c_str());

		LOGINFO("【实体】计算支撑交线线程数%d \n", _threads);
		DWORD dwStart = GetTickCount();
		boost::thread_group *_work_p;
		parallelize<size_t>(
			0,
			pPoints->size() - 1,
			boost::bind(&TriangleMesh::Cale_CollsionZ, this, _1, _type),
			_work_p,
			_threads);
		LOGINFO("【实体】计算支撑交线 总耗时【%u】", GetTickCount() - dwStart);

		/*
		// 做支撑过滤
		std::vector<SPT_pointf>::iterator vec_It = pPoints->begin();
		while (vec_It != pPoints->end())
		{
			int face_ID = vec_It->get_mincollsionFaceID();
			stl_facet &_facet = this->stl.facet_start[face_ID];
			// 支撑避开固位网
			if (_facet.face_type == RetentionMesh_Part
				|| _facet.face_type == Fulcrum_Part
				|| _facet.face_type == Nail_Part)
			{
				// 解除树型关系
				// vec_It->delete_BranchInfo();
				vec_It->IsValid = false;
			}
			vec_It++;
		}
		*/
//        std::cout<<"this->NoSupportFaces.size()"<<this->NoSupportFaces.size()<<std::endl;

		//种植桥架等种植孔里不需要加支撑
		std::vector<SPT_pointf>::iterator vec_It = pPoints->begin();
        while (vec_It != pPoints->end())
        {
            int face_ID = vec_It->get_mincollsionFaceID();
            stl_facet &_facet = this->stl.facet_start[face_ID];
//            std::cout<<"this->NoSupportFaces.size()"<<this->NoSupportFaces.size()<<std::endl;
            auto iterNoSupportFaces = this->NoSupportFaces.find(face_ID);
            if (iterNoSupportFaces != this->NoSupportFaces.end())
            {
                // 解除树型关系
                // vec_It->delete_BranchInfo();
                vec_It->IsValid = false;
            }
            vec_It++;
        }
	}

	bool TriangleMesh::Cale_CollsionZ(SPT_pointf &sptP)
	{
		// 计算碰撞的三角面片
		std::vector<int> facet_ids = this->GetFacetIdxByZaxis(sptP.hitpoint.x, sptP.hitpoint.y);
		if (facet_ids.size() == 0)
		{
			LOGINFO("this SPT_pointf does not has CollsionZ!!");
			return false;
		}
		// 记录碰撞信息
		sptP.CollsionZ_map.clear();
		for (int i = 0; i < facet_ids.size(); i++)
		{
			double collsion_z = this->GetPointZByZaxis(facet_ids[i], sptP.hitpoint.x, sptP.hitpoint.y);
			sptP.CollsionZ_map[collsion_z] = facet_ids[i];
		}

		return true;
	}

	int TriangleMesh::Get_PointsSize(std::string _type)
	{
		std::vector<SPT_pointf> *pPoints = this->get_points(_type);
		if (pPoints == NULL)
			return -1;
		return pPoints->size();
	}

	int TriangleMesh::Get_WallsSize(std::string _type)
	{
		if (_type == "boundary_single_wall" || _type == "boundary_wall" || _type == "boundary_hexhole_wall")
			return this->wallLoops_vec.size();
		else if (_type == "stripe_single_wall")
			return this->tuqiLoops_vec.size();
		else if (_type == "auto_crossbar")
			return this->Crossbar_Points.size();
		else if (_type == "auto_assist_crossbar")
			return this->Assit_Crossbar_Points.size();
		return 0;
	}

	int TriangleMesh::Get_WallPartSize(std::string _type, int wall_index)
	{
		if (_type == "boundary_single_wall" || _type == "boundary_wall" || _type == "boundary_hexhole_wall")
		{
			if (wall_index < this->wallLoops_vec.size() && wall_index > -1)
				return this->wallLoops_vec.at(wall_index).size();
		}
		else if (_type == "stripe_single_wall" && wall_index < this->tuqiLoops_vec.size() && wall_index > -1)
		{
			return this->tuqiLoops_vec.at(wall_index).size();
		}

		return 0;
	}

	Pointf3 TriangleMesh::Get_HitPos(std::string _type, unsigned int spt_index, unsigned int hit_index)
	{
		std::vector<SPT_pointf> *pPoints = this->get_points(_type);
		if (pPoints == NULL)
			return Pointf3(0, 0, 0);
		if (spt_index >= pPoints->size())
			return Pointf3(0, 0, 0);
		std::vector<SPT_pointf>::iterator p_It = pPoints->begin() + spt_index;
		MapCollsionInfo *pHitmap = &(p_It->CollsionZ_map);
		if (pHitmap == NULL)
			return Pointf3(0, 0, 0);
		if (hit_index >= pHitmap->size())
			return Pointf3(0, 0, 0);
		double HitZ = p_It->get_collsionZ(hit_index);

		return Pointf3(p_It->hitpoint.x, p_It->hitpoint.y, HitZ);
	}

	Pointf3 TriangleMesh::Get_HitNol(std::string _type, unsigned int spt_index, unsigned int hit_index)
	{
		std::vector<SPT_pointf> *pPoints = this->get_points(_type);
		if (pPoints == NULL)
			return Pointf3(0, 0, 0);
		if (spt_index >= pPoints->size())
			return Pointf3(0, 0, 0);
		std::vector<SPT_pointf>::iterator p_It = pPoints->begin() + spt_index;
		MapCollsionInfo *pHitmap = &(p_It->CollsionZ_map);
		if (pHitmap == NULL)
			return Pointf3(0, 0, 0);
		if (hit_index >= pHitmap->size())
			return Pointf3(0, 0, 0);
		double HitZ = p_It->get_collsionZ(hit_index);

		return this->GetFacetNormal(pHitmap->find(HitZ)->second);
	}

	Pointf3 TriangleMesh::Get_BasePos(std::string _type, unsigned int spt_index)
	{
		std::vector<SPT_pointf> *pPoints = this->get_points(_type);
		if (pPoints == NULL)
			return Pointf3(0, 0, 0);
		if (spt_index >= pPoints->size())
			return Pointf3(0, 0, 0);

		return Pointf3(pPoints->at(spt_index).basepoint.x, pPoints->at(spt_index).basepoint.y, pPoints->at(spt_index).basepoint_z);
	}

	double TriangleMesh::Get_Angle(std::string _type, unsigned int spt_index)
	{
		std::vector<SPT_pointf> *pPoints = this->get_points(_type);
		if (pPoints == NULL)
			return 0.0;
		if (spt_index >= pPoints->size())
			return 0.0;

		return (pPoints->at(spt_index).dirangle);
	}

	size_t TriangleMesh::Get_HitPointNum(std::string _type, unsigned int spt_index)
	{
		std::vector<SPT_pointf> *pPoints = this->get_points(_type);
		if (pPoints == NULL)
			return 0;
		if (spt_index >= pPoints->size())
			return 0;

		return pPoints->at(spt_index).CollsionZ_map.size();
	}

	bool TriangleMesh::Get_Valid(std::string _type, unsigned int spt_index)
	{
		std::vector<SPT_pointf> *pPoints = this->get_points(_type);
		if (pPoints == NULL)
			return false;
		if (spt_index >= pPoints->size())
			return false;

		return pPoints->at(spt_index).Is_Valid();
	}
	bool TriangleMesh::IsTreeLeaf(std::string _type, unsigned int spt_index)
	{
		std::vector<SPT_pointf> *pPoints = this->get_points(_type);
		if (pPoints == NULL)
			return false;
		if (spt_index >= pPoints->size())
			return false;
		return pPoints->at(spt_index).IsTreeBranch() && (!pPoints->at(spt_index).IsTreeBase());
	}
	bool TriangleMesh::IsTreeBaseOne(std::string _type, unsigned int spt_index)
	{
		std::vector<SPT_pointf> *pPoints = this->get_points(_type);
		if (pPoints == NULL)
			return false;
		if (spt_index >= pPoints->size())
			return false;

		return pPoints->at(spt_index).IsTreeBase();
	}
	size_t TriangleMesh::Get_TreeBaseBranchNum(unsigned int spt_index)
	{
		std::vector<SPT_pointf> *pPoints = this->get_points("tree");
		if (pPoints == NULL)
			return 0;
		if (spt_index >= pPoints->size())
			return 0;

		int ret = 0;
		for (int i = 0; i < pPoints->at(spt_index).branch_vecp.size(); i++)
		{
			Slic3r::SPT_pointf *_pf = pPoints->at(spt_index).branch_vecp[i];
			if (_pf->IsTreeBranch() && _pf->Is_Valid())
				ret++;
		}

		return ret;
	}

	//
	bool TriangleMesh::IsHitPoint_InThreshold(std::string _type, unsigned int spt_index, unsigned int hit_index)
	{
		std::vector<SPT_pointf> *pPoints = this->get_points(_type);
		if (pPoints == NULL)
			return false;
		if (spt_index >= pPoints->size())
			return false;
		std::vector<SPT_pointf>::iterator p_It = pPoints->begin() + spt_index;
		MapCollsionInfo *pHitmap = &(p_It->CollsionZ_map);
		if (pHitmap == NULL)
			return false;
		if (hit_index >= pHitmap->size())
			return false;
		double HitZ = p_It->get_collsionZ(hit_index);
		int face_id = pHitmap->find(HitZ)->second;
		char factMark = this->GetFacetExtra(face_id);

		return TriangleMesh::fact_is_mark(factMark, Area_Fact);
	}

	bool TriangleMesh::IsHitPoint_InRPDRetenion(std::string _type, unsigned int spt_index, unsigned int hit_index)
	{
		std::vector<SPT_pointf> *pPoints = this->get_points(_type);
		if (pPoints == NULL)
			return false;
		if (spt_index >= pPoints->size())
			return false;
		std::vector<SPT_pointf>::iterator p_It = pPoints->begin() + spt_index;
		MapCollsionInfo *pHitmap = &(p_It->CollsionZ_map);
		if (pHitmap == NULL)
			return false;
		if (hit_index >= pHitmap->size())
			return false;
		double HitZ = p_It->get_collsionZ(hit_index);
		int face_id = pHitmap->find(HitZ)->second;
		stl_facet &_facet = this->stl.facet_start[face_id];
		return (_facet.face_type == RetentionMesh_Part);
	}

	bool TriangleMesh::ClearBufferDate(std::string _type)
	{
		std::vector<SPT_pointf> *pPoints = this->get_points(_type);
		if (pPoints == NULL)
			return false;

		std::vector<SPT_pointf>().swap(*pPoints);

		if (_type == "boundary" || _type == "rpd_strengthen")
		{
			BoundLines_vec().swap(boundLines_vec);
			BoundLoops_vec().swap(boundLoops_vec);
			BoundLoops_vec().swap(wallLoops_vec);
			Polygons().swap(boundLoops);
			ExPolygons().swap(exboundLoops);
		}
		else if (_type == "stripe")
		{
			BoundLines_vec().swap(tuqiLines_vec);
			BoundLoops_vec().swap(tuqiLoops_vec);
			Polygons().swap(tuqiLoops);
			ExPolygons().swap(extuqiLoops);
			std::vector<int>().swap(tuqi_facets_index);
			std::vector<int>().swap(other_facets_index);
		}
		else if (_type == "overhangline")
		{
			BoundLines_vec().swap(OverhangLines);
			Polylines().swap(Overhang_polylines);
		}
		else if (_type == "lattice")
		{
			LatticeSPT_Pointf3s().swap(Lspt_Points);
		}
		else if (_type == "auto_crossbar")
		{
			CrossbarPointPairs().swap(Crossbar_Points);
		}
		else if (_type == "auto_assist_crossbar")
		{
			CrossbarPointPairs().swap(Assit_Crossbar_Points);
		}
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////
	// zhijia Adjust
	/////////////////////////////////////////////////////////////////////////////////////////////////

	// 法相的主成分分析
	bool TriangleMesh::Normal_PCA(std::map<double, Vectorf3> &ClaspNor_map, bool IsReverseTuqi)
	{
		unsigned int FactCount = 0;
		// 构造协方差矩阵
		Eigen::Matrix3d covariance;
		covariance.setZero();
		const double tuqi_area_threshold = 0.005;
		for (unsigned int i = 0; i < this->stl.stats.number_of_facets; ++i)
		{
			stl_facet &facet = this->stl.facet_start[i];
			Vectorf3 fact_ver = this->GetFacetNormal(i);
			if (IsReverseTuqi && TriangleMesh::Calculate_Facet_Area(facet) < tuqi_area_threshold) // 花纹部分面片 法相相反
				fact_ver = fact_ver.negative();
			Eigen::Vector3d ver(fact_ver.x, fact_ver.y, fact_ver.z);
			covariance += ver * ver.transpose();
			FactCount++;
		}
		LOGINFO("FactCount %d", FactCount);
		covariance /= (double)FactCount;
		Eigen::EigenSolver<Eigen::Matrix3d> solver(covariance);
		Eigen::Matrix3d eigenVectors = solver.eigenvectors().real();
		Eigen::Vector3d eigenValuses = solver.eigenvalues().real();

		Eigen::Vector3d eigenVector_ret;
		for (size_t i = 0; i < 3; i++)
		{
			eigenVector_ret = eigenVectors.col(i);
			Vectorf3 eigenVectorf3 = Vectorf3(eigenVector_ret(0), eigenVector_ret(1), eigenVector_ret(2));
			// 为特征值计算正负
			double _temp = 0.0;
			for (unsigned int i = 0; i < this->stl.stats.number_of_facets; ++i)
			{
				stl_facet &facet = this->stl.facet_start[i];
				Vectorf3 fact_ver = this->GetFacetNormal(i);
				if (IsReverseTuqi && TriangleMesh::Calculate_Facet_Area(facet) < tuqi_area_threshold) // 花纹部分面片 法相相反
					fact_ver = fact_ver.negative();
				_temp += fact_ver.x * eigenVectorf3.x;
				_temp += fact_ver.y * eigenVectorf3.y;
				_temp += fact_ver.z * eigenVectorf3.z;
			}
			if (_temp > 0.0)
			{
				ClaspNor_map[eigenValuses(i)] = eigenVectorf3;
			}
			else
			{
				ClaspNor_map[eigenValuses(i)] = eigenVectorf3.negative();
			}
		}

		std::ostringstream debug_str;
		debug_str << "协方差矩阵:" << covariance << std::endl
				  << std::endl;
		debug_str << "特征向量:" << solver.eigenvectors() << std::endl
				  << std::endl;
		debug_str << "特征值:" << solver.eigenvalues() << std::endl
				  << std::endl;
		LOGINFO(debug_str.str().c_str());

		return true;
	}

	ExPolygons TriangleMesh::Offset_ExboundPoints(ExPolygons exps, double offset_distance)
	{
		ExPolygons _ret;
		for (int i = 0; i < exps.size(); i++)
		{
			ExPolygons exs = offset_ex(exps, offset_distance);
			_ret.insert(_ret.end(), exs.begin(), exs.end());
		}
		return _ret;
	}

	// 从某一个边界多边形上提取边界采样点
	unsigned int TriangleMesh::make_PolylinePoints(std::string _type, Polyline polyline, double Points_distance, size_t Loop_Id)
	{
		if (polyline.points.size() == 0)
			return 0;
		// 间隔采样点
		Points points_temp;
		std::vector<coordf_t> DirectionAngles_temp;
		points_temp.clear();
		DirectionAngles_temp.clear();
		polyline.equally_spaced_points_dir(Points_distance, points_temp, DirectionAngles_temp);
		LOGINFO("Points_distance = %f, points_temp size = %d", unscale(Points_distance), points_temp.size());
		if (points_temp.size() == DirectionAngles_temp.size())
		{
			for (int i = 0; i < points_temp.size(); i++)
			{
				// 数据缩小处理
				Pointf tempPointf;
				tempPointf.x = unscale(points_temp[i].x);
				tempPointf.y = unscale(points_temp[i].y);
				/*
				if (this->PointIn_tuqiLoops(tempPointf) == -1) // 排除凸起区域
				{
					continue;
				}
				*/
				SPT_pointf spt_pf(tempPointf, DirectionAngles_temp[i], Loop_Id);
				if (this->get_points(_type))
					this->get_points(_type)->push_back(spt_pf);
			}
		}
		else
			CONFESS("equally_spaced_points_dir 出错，points_temp和DirectionAngles_temp个数不等！");

		return points_temp.size();
	}

	// 计算两个邻接面片法向量夹角
	double TriangleMesh::facet_angle(const stl_facet &facet, const stl_facet &facet_neighbor)
	{
		stl_normal normal = facet.normal;
		stl_normal normal_neighbor = facet_neighbor.normal;
		double delta = (normal.x * normal_neighbor.x + normal.y * normal_neighbor.y + normal.z * normal_neighbor.z) /
					   sqrt((normal.x * normal.x + normal.y * normal.y + normal.z * normal.z) * (normal_neighbor.x * normal_neighbor.x + normal_neighbor.y * normal_neighbor.y + normal_neighbor.z * normal_neighbor.z));
		double theta = acos(delta);
		return theta;
	}

	// 计算三角面片的面积
	double TriangleMesh::Calculate_Facet_Area(stl_facet facet)
	{
		return stl_get_area(&facet);
	}

	// 计算三角面片的周长
	double TriangleMesh::Calculate_Sides_Length(stl_facet facet)
	{
		double area = 0.0;

		double side[3]; // 存储三条边的长度;

		stl_vertex a = facet.vertex[0];
		stl_vertex b = facet.vertex[1];
		stl_vertex c = facet.vertex[2];

		side[0] = sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
		side[1] = sqrt(pow(a.x - c.x, 2) + pow(a.y - c.y, 2) + pow(a.z - c.z, 2));
		side[2] = sqrt(pow(c.x - b.x, 2) + pow(c.y - b.y, 2) + pow(c.z - b.z, 2));

		return (side[0] + side[1] + side[2]); // 周长;
	}

	// 计算三角面片的最长边和最短边比值
	double TriangleMesh::Calculate_MaxMinSide_Scale(stl_facet facet)
	{
		double area = 0.0;
		double Max_sideLen = -DBL_MAX;
		double Min_sideLen = -DBL_MIN;
		double side[3];
		stl_vertex a = facet.vertex[0];
		stl_vertex b = facet.vertex[1];
		stl_vertex c = facet.vertex[2];
		side[0] = sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
		side[1] = sqrt(pow(a.x - c.x, 2) + pow(a.y - c.y, 2) + pow(a.z - c.z, 2));
		side[2] = sqrt(pow(c.x - b.x, 2) + pow(c.y - b.y, 2) + pow(c.z - b.z, 2));

		for (int i = 0; i < 3; i++)
		{
			Max_sideLen = Max_sideLen > side[i] ? Max_sideLen : side[i];
			Min_sideLen = Min_sideLen < side[i] ? Min_sideLen : side[i];
		}

		return Max_sideLen / Min_sideLen;
	}

	std::vector<int> TriangleMesh::Get_facets(char facet_type, bool Iscomplementaryset)
	{
		std::vector<int> retval;
		retval.clear();
		for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
		{
			stl_facet &facet = this->stl.facet_start[i];
			if (TriangleMesh::fact_is_mark(facet.extra, facet_type) != Iscomplementaryset)
				retval.push_back(i);
		}

		return retval;
	}

	int TriangleMesh::Get_rpd_facet_mark_count(std::vector<int> facets_index, int facet_mark, int facet_side)
	{
		int count = 0;
		for (int i = 0; i < facets_index.size(); ++i)
		{
			stl_facet &facet = this->stl.facet_start[facets_index[i]];
			if (facet_mark == -1 && facet_side != -1)
			{
				if (facet.face_side == facet_side)
					count++;
			}
			else if (facet_mark != -1 && facet_side == -1)
			{
				if (facet.face_type == facet_mark)
					count++;
			}
			else if (facet_mark != -1 && facet_side != -1)
			{
				if (facet.face_type == facet_mark && facet.face_side == facet_side)
					count++;
			}
			else
				count++;
		}
		return count;
	}

	int
	TriangleMesh::Get_facet_mark_count(std::vector<int> facets_index, char facet_type)
	{
		int count = 0;
		for (int i = 0; i < facets_index.size(); ++i)
		{
			stl_facet &facet = this->stl.facet_start[facets_index[i]];
			if (TriangleMesh::fact_is_mark(facet.extra, facet_type))
				count++;
		}
		return count;
	}

	// 特定类型的面片进行角度过滤
	bool
	TriangleMesh::Angle_FilterFacets(char facet_type, std::vector<int> facets_index, double filter_angle)
	{
		if (facets_index.size() == 0)
		{
			return false;
		}

		for (int i = 0; i < facets_index.size(); ++i)
		{
			int fact_index = facets_index[i];
			stl_facet &facet = this->stl.facet_start[fact_index];
			double _facet_angle = stl_calculate_faceangle(&facet);
			if (TriangleMesh::fact_is_mark(facet.extra, facet_type) && _facet_angle >= filter_angle) // 在角度阈值之外的面片
			{
				TriangleMesh::fact_remove_mark(facet.extra, facet_type);
			}
		}

		return true;
	}

	// 对特定类型的三角面片进行退化处理
	bool TriangleMesh::Area_FilterFacets(char facet_type, std::vector<int> facets_index)
	{
		if (facets_index.size() == 0)
		{
			// LOGINFO("Area_FilterFacets 递归结束！");
			return true;
		}

		// LOGINFO("area_facets_index.size() = %d", facets_index.size());

		std::vector<int> NextLoop_facets_index;
		NextLoop_facets_index.clear();
		std::vector<int> facetNear_Index;
		facetNear_Index.clear();

		for (int i = 0; i < facets_index.size(); ++i)
		{
			int fact_index = facets_index[i];
			stl_facet &facet = this->stl.facet_start[fact_index];
			if (TriangleMesh::fact_is_mark(facet.extra, facet_type)) // 挑出所有命中的面片
			{
				facetNear_Index.clear();
				for (int j = 0; j <= 2; j++)
				{
					int near_facet_index = this->stl.neighbors_start[fact_index].neighbor[j];
					stl_facet &near_facet = this->stl.facet_start[near_facet_index];
					if (TriangleMesh::fact_is_mark(near_facet.extra, facet_type))
						facetNear_Index.push_back(near_facet_index);
				}

				if (facetNear_Index.size() < 2) // 对于命中邻接面片 少于2个的，剔除
				{
					// LOGINFO("退化处理，fact_index = %d, facetNear_Index.size() = %d", fact_index, facetNear_Index.size());
					TriangleMesh::fact_remove_mark(facet.extra, facet_type);
					NextLoop_facets_index.insert(NextLoop_facets_index.end(), facetNear_Index.begin(), facetNear_Index.end());
				}
			}
		}
		// LOGINFO("NextLoop_facets_index.size() = %d", NextLoop_facets_index.size());
		return this->Area_FilterFacets(facet_type, NextLoop_facets_index);
	}

	//  扩展面片
	std::vector<int> TriangleMesh::ExpandBaseFacets(std::vector<int> baseFace_IDs)
	{
		// 清空标记
		for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
		{
			stl_facet &facet = this->stl.facet_start[i];
			TriangleMesh::fact_remove_mark(facet.extra, Selected_Fact);
		}
		// 开始标记
		for (int i = 0; i < baseFace_IDs.size(); ++i)
		{
			int fact_index = baseFace_IDs[i];
			stl_facet &facet = this->stl.facet_start[fact_index];
			TriangleMesh::fact_set_mark(facet.extra, Selected_Fact);
			for (int j = 0; j <= 2; j++)
			{
				int near_facet_index = this->stl.neighbors_start[fact_index].neighbor[j];
				stl_facet &near_facet = this->stl.facet_start[near_facet_index];
				TriangleMesh::fact_set_mark(near_facet.extra, Selected_Fact);
			}
		}

		// 清空标记
		std::vector<int> Expand_IDs;
		Expand_IDs.clear();
		for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
		{
			stl_facet &facet = this->stl.facet_start[i];
			if (TriangleMesh::fact_is_mark(facet.extra, Selected_Fact))
			{
				Expand_IDs.push_back(i);
				TriangleMesh::fact_remove_mark(facet.extra, Selected_Fact);
			}
		}

		return Expand_IDs;
	}

	// 面片是否合法
	bool TriangleMesh::FaceIndexValid(int fact_index)
	{
		if (fact_index < 0 || fact_index > (this->stl.stats.number_of_facets - 1))
			return false;
		return true;
	}

	//
	std::vector<int> TriangleMesh::ExpandFacetbyEdge(int fact_index, int LoopNum)
	{
		std::vector<int> _ret;
		if (this->FaceIndexValid(fact_index) == false)
			return _ret;
		if (LoopNum <= 0)
			return _ret;
		for (int j = 0; j <= 2; j++)
		{
			int near_facet_index = this->stl.neighbors_start[fact_index].neighbor[j];
			if (this->FaceIndexValid(near_facet_index) == false)
				continue; // 不合法
			_ret.push_back(near_facet_index);
			std::vector<int> near_ret = this->ExpandFacetbyEdge(near_facet_index, LoopNum - 1);
			if (near_ret.size())
				_ret.insert(_ret.end(), near_ret.begin(), near_ret.end());
		}

		// 去重
		std::sort(_ret.begin(), _ret.end());
		auto index_it = std::unique(_ret.begin(), _ret.end());
		_ret.erase(index_it, _ret.end());

		return _ret;
	}

	// 将单个面片扩展成一圈
	std::vector<int> TriangleMesh::ExpandFacetbyRing(int fact_index, int LoopNum)
	{

		std::vector<int> temp_ret;
		if (this->FaceIndexValid(fact_index) == false)
			return temp_ret;
		if (LoopNum <= 0)
			return temp_ret;
		stl_facet &this_facet = this->stl.facet_start[fact_index];
		temp_ret = this->ExpandFacetbyEdge(fact_index, 3);

		// 过滤
		std::vector<int> _ret;
		for (int i = 0; i < temp_ret.size(); i++)
		{
			std::vector<int> near_ret;
			int temp_index = temp_ret[i];
			stl_facet &temp_facet = this->stl.facet_start[temp_index];
			if (temp_facet.concide_face(this_facet))
			{
				_ret.push_back(temp_index);
				near_ret = this->ExpandFacetbyRing(temp_index, LoopNum - 1);
				if (near_ret.size())
					_ret.insert(_ret.end(), near_ret.begin(), near_ret.end());
			}
		}
		// 去重
		std::sort(_ret.begin(), _ret.end());
		auto index_it = std::unique(_ret.begin(), _ret.end());
		_ret.erase(index_it, _ret.end());

		// LOGINFO("temp_index[%d]--->concide_face[%d]", temp_ret.size(), _ret.size());
		return _ret;
	}
	// 将单个面片扩展成一圈 非递归版本
	std::vector<int> TriangleMesh::ExpandFacetbyRing_BFS(int fact_index, int LoopNum)
	{

		std::vector<int> expanded_facets;
		if (this->FaceIndexValid(fact_index) == false || LoopNum <= 0)
			return expanded_facets;
		std::unordered_set<int> visited;	// 记录已访问的面片
		std::unordered_set<int> previsited; // 记录上一次访问的面片
		previsited.insert(fact_index);
		int record = 0;
		LoopNum *= 3;
		while (LoopNum > record)
		{
			visited.insert(previsited.begin(), previsited.end());
			// LOGINFO("times [%d] -- previsited [%d] visited [%d]", record, previsited.size(), visited.size());
			std::vector<int> previsited_Vec(previsited.begin(), previsited.end());
			previsited.clear();
			for (int i = 0; i < previsited_Vec.size(); i++)
			{
				int facet_index_ne = previsited_Vec[i];
				// LOGINFO("facet_index_ne [%d] ", facet_index_ne);
				if (this->FaceIndexValid(facet_index_ne) == false)
					continue;
				for (int j = 0; j <= 2; j++)
				{
					int near_facet_index = this->stl.neighbors_start[facet_index_ne].neighbor[j];
					// LOGINFO("near_facet_index [%d] ", near_facet_index);
					if (this->FaceIndexValid(near_facet_index) == false)
						continue; // 不合法
					previsited.insert(near_facet_index);
				}
			}
			// for (auto it : previsited)
			// {
			// 	LOGINFO("previsited is [%d] ", it);
			// }
			record++;
		}
		// LOGINFO("visited [%d]", visited.size());
		std::vector<int> res(visited.begin(), visited.end());
		expanded_facets = res;
		return expanded_facets;
	}

	//  将线扩成面片 递归版本
	std::vector<int> TriangleMesh::ExpandSegLine2Facets(BoundSegments_loop Seg_loop, int Expend_Num)
	{
		// 清空标记
		for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
		{
			stl_facet &facet = this->stl.facet_start[i];
			TriangleMesh::fact_remove_mark(facet.extra, Selected_Fact);
		}

		// 开始标记
		for (int i = 0; i < Seg_loop.size(); i++)
		{
			int fact_index = Seg_loop[i].FacetID.Mark_facetIndex;
			if (this->FaceIndexValid(fact_index) == false)
				continue;
			stl_facet &facet = this->stl.facet_start[fact_index];
			TriangleMesh::fact_set_mark(facet.extra, Selected_Fact);
		}

		std::vector<int> baseFace_IDs;
		baseFace_IDs = this->Get_facets(Selected_Fact, false);
		for (int i = 0; i < baseFace_IDs.size(); ++i)
		{
			int fact_index = baseFace_IDs[i];
			std::vector<int> onering = this->ExpandFacetbyRing(fact_index, Expend_Num);
			for (int j = 0; j < onering.size(); j++)
			{
				int fact_index = onering[i];
				if (this->FaceIndexValid(fact_index) == false)
					continue;
				stl_facet &facet = this->stl.facet_start[fact_index];
				if (facet.face_side != Upper_Face)
					continue;
				TriangleMesh::fact_set_mark(facet.extra, Selected_Fact);
			}
		}
		// 后处理
		std::vector<int> Expand_IDs;
		// 反向退化
		Expand_IDs = this->Get_facets(Selected_Fact, true);
		this->reArea_FilterFacets(Selected_Fact, Expand_IDs);
		// 正向退化
		Expand_IDs = this->Get_facets(Selected_Fact, false);
		this->Area_FilterFacets(Selected_Fact, Expand_IDs);
		// 考虑拓展面片的角度过滤
		Expand_IDs = this->Get_facets(Selected_Fact, false);
		this->Angle_FilterFacets(Selected_Fact, Expand_IDs, 90);
		// 输出结果
		Expand_IDs = this->Get_facets(Selected_Fact, false);
		for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
		{
			stl_facet &facet = this->stl.facet_start[i];
			TriangleMesh::fact_remove_mark(facet.extra, Selected_Fact);
		}
		return Expand_IDs;
	}

	//  将线扩成面片 非递归版本
	std::vector<int> TriangleMesh::ExpandSegLine2Facets_Region(BoundSegments_loop Seg_loop, int Expand_Num)
	{
		LOGINFO("ExpandSegLine2Facets [%d]", Expand_Num);
		// 清空标记
		for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
		{
			stl_facet &facet = this->stl.facet_start[i];
			TriangleMesh::fact_remove_mark(facet.extra, Selected_Fact);
		}
		int recordfacets = 0;
		// 开始标记
		for (int i = 0; i < Seg_loop.size(); i++)
		{
			int fact_index = Seg_loop[i].FacetID.Mark_facetIndex;
			if (this->FaceIndexValid(fact_index) == false)
				continue;
			stl_facet &facet = this->stl.facet_start[fact_index];
			TriangleMesh::fact_set_mark(facet.extra, Selected_Fact);
			recordfacets++;
		}
		LOGINFO("0818 recordfacets = %d", recordfacets);
		std::vector<int> baseFace_IDs;
		baseFace_IDs = this->Get_facets(Selected_Fact, false);
		LOGINFO("0818 baseFace_IDs = %d", baseFace_IDs.size());
		std::set<int> tempset;
		int selected_fact_num = 0;
		for (int i = 0; i < baseFace_IDs.size(); ++i)
		{
			int fact_index = baseFace_IDs[i];
			std::vector<int> onering = this->ExpandFacetbyRing_BFS(fact_index, Expand_Num);
			tempset.insert(onering.begin(), onering.end());
		}
		LOGINFO("0818 selected_fact_num = %d", selected_fact_num);

		int Invalid_Index = 0;
		int UpperFacet = 0;
		int LowerFacet = 0;
		int SideFacet = 0;
		int NoneFacet = 0;
		LOGINFO("0818 tempset size = %d", tempset.size());
		std::vector<int> tempvec(tempset.begin(), tempset.end());
		std::vector<int> UpperFacetVec;
		for (int i = 0; i < tempvec.size(); i++)
		{
			int fact_index = tempvec[i];
			if (this->FaceIndexValid(fact_index) == false)
				Invalid_Index++;
			stl_facet &facet = this->stl.facet_start[fact_index];
			if (facet.face_side == Upper_Face)
			{
				UpperFacetVec.push_back(fact_index);
			}
			if (facet.face_side == Lower_Face)
				LowerFacet++;
			if (facet.face_side == Side_Face)
				SideFacet++;
			if (facet.face_side == None_Face)
				NoneFacet++;
		}
		LOGINFO("0818 Invalid_Index  %d", Invalid_Index);
		LOGINFO("0818 UpperFacet  %d", UpperFacetVec.size());
		LOGINFO("0818 LowerFacet  %d", LowerFacet);
		LOGINFO("0818 SideFacet  %d", SideFacet);
		LOGINFO("0818 NoneFacet  %d", NoneFacet);

		// 后处理
		std::vector<int> Expand_IDs;
		Expand_IDs = UpperFacetVec;
		for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
		{
			stl_facet &facet = this->stl.facet_start[i];
			TriangleMesh::fact_remove_mark(facet.extra, Selected_Fact);
		}
		for (int i = 0; i < UpperFacetVec.size(); i++)
		{
			int fact_index = UpperFacetVec[i];
			if (this->FaceIndexValid(fact_index) == false)
				continue;
			stl_facet &facet = this->stl.facet_start[fact_index];
			TriangleMesh::fact_set_mark(facet.extra, Selected_Fact);
		}
		for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
		{
			stl_facet &facet = this->stl.facet_start[i];
			if (facet.face_type == RetentionMesh_Part)
			{
				TriangleMesh::fact_remove_mark(facet.extra, Selected_Fact);
			}
		}
		LOGINFO("0818 1 Expand_IDs = %d", Expand_IDs.size());
		Expand_IDs = this->Get_facets(Selected_Fact, true);
		LOGINFO("0818 2 Expand_IDs = %d", Expand_IDs.size());
		this->reArea_FilterFacets(Selected_Fact, Expand_IDs);
		Expand_IDs = this->Get_facets(Selected_Fact, false);
		LOGINFO("0818 3 Expand_IDs = %d", Expand_IDs.size());
		this->Area_FilterFacets(Selected_Fact, Expand_IDs);
		Expand_IDs = this->Get_facets(Selected_Fact, false);
		LOGINFO("0818 4 Expand_IDs = %d", Expand_IDs.size());

		// 反向退化
		Expand_IDs = this->Get_facets(Selected_Fact, true);
		LOGINFO("0818 1 Expand_IDs = %d", Expand_IDs.size());
		this->reArea_FilterFacets(Selected_Fact, Expand_IDs);
		// 正向退化
		Expand_IDs = this->Get_facets(Selected_Fact, false);
		LOGINFO("0818 2 Expand_IDs = %d", Expand_IDs.size());
		this->Area_FilterFacets(Selected_Fact, Expand_IDs);

		// 输出结果
		Expand_IDs = this->Get_facets(Selected_Fact, false);
		for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
		{
			stl_facet &facet = this->stl.facet_start[i];
			TriangleMesh::fact_remove_mark(facet.extra, Selected_Fact);
		}
		return Expand_IDs;
	}

	// 扩张面片集
	bool TriangleMesh::ExpandMarkFacets(char facet_type)
	{
		std::vector<int> MarkFace_IDs = this->Get_facets(facet_type, false);
		for (int i = 0; i < MarkFace_IDs.size(); ++i)
		{
			int fact_index = MarkFace_IDs[i];
			stl_facet &facet = this->stl.facet_start[fact_index];
			for (int j = 0; j <= 2; j++)
			{
				int near_facet_index = this->stl.neighbors_start[fact_index].neighbor[j];
				stl_facet &near_facet = this->stl.facet_start[near_facet_index];
				if (TriangleMesh::Calculate_MaxMinSide_Scale(near_facet) < 10 && near_facet.normal.z <= 0.0) // 不标记细长面片
					TriangleMesh::fact_set_mark(near_facet.extra, facet_type);
			}
		}
		return true;
	}

	bool TriangleMesh::ExpandMarkFacets_Anyway(char facet_type)
	{
		std::vector<int> MarkFace_IDs = this->Get_facets(facet_type, false);
		for (int i = 0; i < MarkFace_IDs.size(); ++i)
		{
			int fact_index = MarkFace_IDs[i];
			stl_facet &facet = this->stl.facet_start[fact_index];
			for (int j = 0; j <= 2; j++)
			{
				int near_facet_index = this->stl.neighbors_start[fact_index].neighbor[j];
				stl_facet &near_facet = this->stl.facet_start[near_facet_index];
				TriangleMesh::fact_set_mark(near_facet.extra, facet_type);
			}
		}
		return true;
	}

	// 扩张面片集
	bool TriangleMesh::ExpandMarkFacets_Special(char facet_type)
	{
		std::vector<int> MarkFace_IDs = this->Get_facets(facet_type, false);
		for (int i = 0; i < MarkFace_IDs.size(); ++i)
		{
			int fact_index = MarkFace_IDs[i];
			stl_facet &facet = this->stl.facet_start[fact_index];
			for (int j = 0; j <= 2; j++)
			{
				int near_facet_index = this->stl.neighbors_start[fact_index].neighbor[j];
				stl_facet &near_facet = this->stl.facet_start[near_facet_index];
				// 标记RPDWeak_Fact面片跳过
				if (TriangleMesh::fact_is_mark(near_facet.extra, RPDWeak_Fact))
					continue;
				// 不是上表面，跳过
				if (near_facet.face_side != Upper_Face)
					continue;

				// 只扩展部分部件
				if (near_facet.face_type == MajorConnector_Part || near_facet.face_type == StippledWax_Part || near_facet.face_type == LingualBar_Part || near_facet.face_type == MinorConnector_Part
					//|| near_facet.face_type == OcclusalRest_Part
				)
					TriangleMesh::fact_set_mark(near_facet.extra, facet_type);
			}
		}
		return true;
	}

	// 腐蚀面片集
	bool TriangleMesh::ContractMarkFacets(char facet_type)
	{
		std::vector<int> NoMarkFace_IDs = this->Get_facets(facet_type, true);
		for (int i = 0; i < NoMarkFace_IDs.size(); ++i)
		{
			int fact_index = NoMarkFace_IDs[i];
			stl_facet &facet = this->stl.facet_start[fact_index];
			for (int j = 0; j <= 2; j++)
			{
				int near_facet_index = this->stl.neighbors_start[fact_index].neighbor[j];
				stl_facet &near_facet = this->stl.facet_start[near_facet_index];
				TriangleMesh::fact_remove_mark(near_facet.extra, facet_type);
			}
		}
		return true;
	}

	// 对特定类型的三角面片进行反向退化处理
	bool TriangleMesh::reArea_FilterFacets(char facet_type, std::vector<int> facets_index)
	{
		if (facets_index.size() == 0)
		{
			// LOGINFO("reArea_FilterFacets 递归结束！");
			return true;
		}

		// LOGINFO("rearea_facets_index.size() = %d", facets_index.size());

		std::vector<int> NextLoop_facets_index;
		NextLoop_facets_index.clear();
		std::vector<int> facetNear_Index;
		facetNear_Index.clear();

		for (int i = 0; i < facets_index.size(); ++i)
		{
			int fact_index = facets_index[i];
			stl_facet &facet = this->stl.facet_start[fact_index];

			if (TriangleMesh::fact_is_mark(facet.extra, facet_type) == false)
			{
				facetNear_Index.clear();

				for (int j = 0; j <= 2; j++)
				{
					int near_facet_index = this->stl.neighbors_start[fact_index].neighbor[j];
					stl_facet &near_facet = this->stl.facet_start[near_facet_index];
					if (TriangleMesh::fact_is_mark(near_facet.extra, facet_type) == false)
						facetNear_Index.push_back(near_facet_index);
				}

				if (facetNear_Index.size() < 2) // 重新打上标记
				{
					// LOGINFO("退化处理，fact_index = %d, facetNear_Index.size() = %d", fact_index, facetNear_Index.size());
					TriangleMesh::fact_set_mark(facet.extra, facet_type);
					NextLoop_facets_index.insert(NextLoop_facets_index.end(), facetNear_Index.begin(), facetNear_Index.end());
				}
			}
		}
		// LOGINFO("NextLoop_facets_index.size() = %d", NextLoop_facets_index.size());
		return this->reArea_FilterFacets(facet_type, NextLoop_facets_index);
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////
	// 支撑点简化器
	/////////////////////////////////////////////////////////////////////////////////////////////////

	XoY_Index
	LatticeSPT_Simply::GetPointID(Pointf3 _p)
	{
		XoY_Index _ret;
		if (X_stepLen > 0.001 && Y_stepLen > 0.001)
		{
			_ret.X_id = floor(_p.x / X_stepLen);
			_ret.Y_id = floor(_p.y / Y_stepLen);
		}

		return _ret;
	}

	int
	LatticeSPT_Simply::GetPointZID(Pointf3 _p)
	{
		if (has_Z_step && Z_stepLen > 0.001)
			return floor(_p.z / Z_stepLen);
		return 0;
	}

	LatticeSPT_Pointf3
	LatticeSPT_Simply::GetOnePoint(std::vector<LatticeSPT_Pointf3> _vec)
	{

		int OnePoint_ID = 0;
		for (int i = 0; i < _vec.size(); i++)
		{
			if (_vec[i].Weight > _vec[OnePoint_ID].Weight)
			{
				OnePoint_ID = i;
			}
			else if (_vec[i].Weight == _vec[OnePoint_ID].Weight)
			{
				if (_vec[i].UpPoint.z < _vec[OnePoint_ID].UpPoint.z)
					OnePoint_ID = i;
			}
		}
		return _vec[OnePoint_ID];
	}

	LatticeSPT_Pointf3
	LatticeSPT_Simply::GetOnePoint_MidPf3(std::vector<LatticeSPT_Pointf3> _vec, Pointf3 MidPointf3)
	{
		int OnePoint_ID = 0;
		double dis = DBL_MAX;
		for (int i = 0; i < _vec.size(); i++)
		{
			double _dis = MidPointf3.distance_to(_vec[i].UpPoint);
			if (_dis < dis)
			{
				OnePoint_ID = i;
				dis = _dis;
			}
		}
		return _vec[OnePoint_ID];
	}

	LatticeSPT_Pointf3
	LatticeSPT_Simply::GetOnePoint_MidP(std::vector<LatticeSPT_Pointf3> _vec, Pointf MidPointf)
	{
		int OnePoint_ID = 0;
		double dis = DBL_MAX;
		for (int i = 0; i < _vec.size(); i++)
		{
			double _dis = MidPointf.distance_to(Pointf(_vec[i].UpPoint.x, _vec[i].UpPoint.y));
			if (_dis < dis)
			{
				OnePoint_ID = i;
				dis = _dis;
			}
		}
		return _vec[OnePoint_ID];
	}

	std::vector<LatticeSPT_Pointf3>
	LatticeSPT_Simply::GetSimplyVec(std::vector<LatticeSPT_Pointf3> points_vec, bool nearMid)
	{

		LOGINFO("------------GetSimplyVec begin--------------");
		// 构造hash抽屉
		std::vector<LatticeSPT_Pointf3> ret_vec;
		for (int i = 0; i < points_vec.size(); i++)
		{
			XoY_Index XY_ID = GetPointID(points_vec[i].UpPoint);
			Hash_Pointf3s[XY_ID].push_back(points_vec[i]);
		}

		std::map<XoY_Index, std::vector<LatticeSPT_Pointf3>>::iterator _Itmap = Hash_Pointf3s.begin();
		while (_Itmap != Hash_Pointf3s.end())
		{
			XoY_Index XY_ID = _Itmap->first;
			std::vector<LatticeSPT_Pointf3> XoY_vec = _Itmap->second;
			if (has_Z_step)
			{
				// 构建z分组
				std::map<int, std::vector<LatticeSPT_Pointf3>> Zpoints_map;
				for (int i = 0; i < XoY_vec.size(); i++)
				{
					int z_ID = GetPointZID(XoY_vec[i].UpPoint);
					Zpoints_map[z_ID].push_back(XoY_vec[i]);
				}
				//
				std::map<int, std::vector<LatticeSPT_Pointf3>>::iterator z_Itmap = Zpoints_map.begin();
				while (z_Itmap != Zpoints_map.end())
				{
					int z_ID = z_Itmap->first;
					std::vector<LatticeSPT_Pointf3> Z_vec = z_Itmap->second;
					Pointf3 Mid_Pointf3 = this->CellMidPoint(XY_ID.X_id, XY_ID.Y_id, z_ID);
					LatticeSPT_Pointf3 one_point = nearMid ? GetOnePoint_MidPf3(Z_vec, Mid_Pointf3) : GetOnePoint(Z_vec);
					ret_vec.push_back(one_point);
					z_Itmap++;
				}
			}
			else
			{
				Pointf Mid_Pointf = this->CellMidPoint(XY_ID.X_id, XY_ID.Y_id);
				LatticeSPT_Pointf3 one_point = nearMid ? GetOnePoint_MidP(XoY_vec, Mid_Pointf) : GetOnePoint(XoY_vec);
				ret_vec.push_back(one_point);
			}

			_Itmap++;
		}

		LOGINFO("------------GetSimplyVec end--------------");

		return ret_vec;
	}

	BoundLines_vec
	LatticeSPT_Simply::GetSimplyCellLines(std::vector<LatticeSPT_Pointf3> points_vec)
	{
		BoundLines_vec ret_vec;
		for (int i = 0; i < points_vec.size(); i++)
		{
			XoY_Index _ID = GetPointID(points_vec[i].UpPoint);
			Hash_Pointf3s[_ID].push_back(points_vec[i]);
		}

		std::map<XoY_Index, std::vector<LatticeSPT_Pointf3>>::iterator _Itmap = Hash_Pointf3s.begin();
		while (_Itmap != Hash_Pointf3s.end())
		{
			std::vector<LatticeSPT_Pointf3> XoY_vec = _Itmap->second;
			XoY_Index XY_Id = _Itmap->first;
			if (has_Z_step)
			{
				// 构建z分组
				std::map<int, std::vector<LatticeSPT_Pointf3>> Zpoints_map;
				for (int i = 0; i < XoY_vec.size(); i++)
				{
					int z_ID = GetPointZID(XoY_vec[i].UpPoint);
					Zpoints_map[z_ID].push_back(XoY_vec[i]);
				}
				//
				std::map<int, std::vector<LatticeSPT_Pointf3>>::iterator z_Itmap = Zpoints_map.begin();
				while (z_Itmap != Zpoints_map.end())
				{
					int Z_Index = z_Itmap->first;
					BoundLines_vec temp_vec = ShowCell(XY_Id.X_id, XY_Id.Y_id, Z_Index);
					ret_vec.insert(ret_vec.end(), temp_vec.begin(), temp_vec.end());
					z_Itmap++;
				}
			}
			else
			{
				BoundLines_vec temp_vec = ShowCell(XY_Id.X_id, XY_Id.Y_id);
				ret_vec.insert(ret_vec.end(), temp_vec.begin(), temp_vec.end());
			}

			_Itmap++;
		}

		return ret_vec;
	}

	Pointf3
	LatticeSPT_Simply::CellMidPoint(int Xcell_id, int Ycell_id, int Zcell_id)
	{
		Pointf3 Mid_Pointf3 = Pointf3(X_stepLen * Xcell_id, Y_stepLen * Ycell_id, Z_stepLen * Zcell_id);
		Pointf3 halfCell = Pointf3(X_stepLen * 0.5, Y_stepLen * 0.5, Z_stepLen * 0.5);
		return (Mid_Pointf3 + halfCell);
	}

	Pointf
	LatticeSPT_Simply::CellMidPoint(int Xcell_id, int Ycell_id)
	{
		Pointf Mid_Pointf = Pointf(X_stepLen * Xcell_id, Y_stepLen * Ycell_id);
		Pointf halfCell = Pointf(X_stepLen * 0.5, Y_stepLen * 0.5);
		return (Mid_Pointf + halfCell);
	}

	BoundLines_vec
	LatticeSPT_Simply::ShowCell(int Xcell_id, int Ycell_id, int Zcell_id)
	{
		BoundLines_vec ret_vec;
		Pointf3 O_p = Pointf3(X_stepLen * Xcell_id, Y_stepLen * Ycell_id, Z_stepLen * Zcell_id);
		Pointf3 OX_p = O_p + Pointf3(X_stepLen, 0, 0);
		Pointf3 OY_p = O_p + Pointf3(0, Y_stepLen, 0);
		Pointf3 OZ_p = O_p + Pointf3(0, 0, Z_stepLen);
		Pointf3 OXY_p = OX_p + Pointf3(0, Y_stepLen, 0);
		Pointf3 OZX_p = OZ_p + Pointf3(X_stepLen, 0, 0);
		Pointf3 OZY_p = OZ_p + Pointf3(0, Y_stepLen, 0);
		Pointf3 OZXY_p = OZX_p + Pointf3(0, Y_stepLen, 0);

		ret_vec.push_back(Linef3(O_p, OX_p));
		ret_vec.push_back(Linef3(O_p, OY_p));
		ret_vec.push_back(Linef3(OX_p, OXY_p));
		ret_vec.push_back(Linef3(OY_p, OXY_p));

		ret_vec.push_back(Linef3(OZ_p, OZX_p));
		ret_vec.push_back(Linef3(OZ_p, OZY_p));
		ret_vec.push_back(Linef3(OZX_p, OZXY_p));
		ret_vec.push_back(Linef3(OZY_p, OZXY_p));

		ret_vec.push_back(Linef3(O_p, O_p));
		ret_vec.push_back(Linef3(OX_p, OZX_p));
		ret_vec.push_back(Linef3(OY_p, OZY_p));
		ret_vec.push_back(Linef3(OXY_p, OZXY_p));

		return ret_vec;
	}

	BoundLines_vec
	LatticeSPT_Simply::ShowCell(int Xcell_id, int Ycell_id)
	{
		BoundLines_vec ret_vec;
		Pointf3 O_p = Pointf3(X_stepLen * Xcell_id, Y_stepLen * Ycell_id, 0);
		Pointf3 OX_p = O_p + Pointf3(X_stepLen, 0, 0);
		Pointf3 OY_p = O_p + Pointf3(0, Y_stepLen, 0);
		Pointf3 OZ_p = O_p + Pointf3(0, 0, 50);
		Pointf3 OXY_p = OX_p + Pointf3(0, Y_stepLen, 0);
		Pointf3 OZX_p = OZ_p + Pointf3(X_stepLen, 0, 0);
		Pointf3 OZY_p = OZ_p + Pointf3(0, Y_stepLen, 0);
		Pointf3 OZXY_p = OZX_p + Pointf3(0, Y_stepLen, 0);

		ret_vec.push_back(Linef3(O_p, OX_p));
		ret_vec.push_back(Linef3(O_p, OY_p));
		ret_vec.push_back(Linef3(OX_p, OXY_p));
		ret_vec.push_back(Linef3(OY_p, OXY_p));

		ret_vec.push_back(Linef3(OZ_p, OZX_p));
		ret_vec.push_back(Linef3(OZ_p, OZY_p));
		ret_vec.push_back(Linef3(OZX_p, OZXY_p));
		ret_vec.push_back(Linef3(OZY_p, OZXY_p));

		ret_vec.push_back(Linef3(O_p, O_p));
		ret_vec.push_back(Linef3(OX_p, OZX_p));
		ret_vec.push_back(Linef3(OY_p, OZY_p));
		ret_vec.push_back(Linef3(OXY_p, OZXY_p));

		return ret_vec;
	}

	class _area_comp
	{
	public:
		_area_comp(std::vector<double> *_aa) : abs_area(_aa){};
		bool operator()(const size_t &a, const size_t &b)
		{
			return (*this->abs_area)[a] > (*this->abs_area)[b];
		}

	private:
		std::vector<double> *abs_area;
	};

	void TriangleMesh::make_expolygons(const Polygons &loops, ExPolygons *slices, bool abandon_hole_loops)
	{
		LOGINFO("原始loops个数【%d】发现expolygons个数【%d】", loops.size(), slices->size());
		DWORD dwStart;
		dwStart = GetTickCount();

		std::vector<double> area;
		std::vector<double> abs_area;
		std::vector<size_t> sorted_area; // vector of indices
		for (Polygons::const_iterator loop = loops.begin(); loop != loops.end(); ++loop)
		{
			double a = loop->area();
			area.push_back(a);
			abs_area.push_back(std::fabs(a));
			sorted_area.push_back(loop - loops.begin());
		}
		LOGINFO("构建容器  耗时【%u】", GetTickCount() - dwStart);
		dwStart = GetTickCount();

		// 按照绝对面积大小来排序
		std::sort(sorted_area.begin(), sorted_area.end(), _area_comp(&abs_area)); // outer first
		LOGINFO("面积排序  耗时【%u】", GetTickCount() - dwStart);
		dwStart = GetTickCount();

		// 识别外轮廓和孔洞
		Polygons conter_loops;
		Polygons hole_loops;
		for (std::vector<size_t>::const_iterator loop_idx = sorted_area.begin(); loop_idx != sorted_area.end(); ++loop_idx)
		{
			Polygons::const_iterator loop = loops.begin() + *loop_idx;
			if (area[*loop_idx] > +EPSILON)
			{ // 面积为正，实体
				conter_loops.push_back(*loop);
			}
			else if (area[*loop_idx] < -EPSILON)
			{ // 面积为负，空洞
				hole_loops.push_back(*loop);
			}
		}
		LOGINFO("conter_loops = %d, hole_loops = %d", conter_loops.size(), hole_loops.size());

		// 初步组装外轮廓和孔洞
		ExPolygons ex_slices;
		// 遍历每个外轮廓
		for (int i = 0; i < conter_loops.size(); i++)
		{
			ExPolygon ex_slice;
			ex_slice.contour = conter_loops[i];
			// 遍历每个内轮廓
			for (std::vector<Polygon>::iterator j_it = hole_loops.begin(); j_it != hole_loops.end();)
			{
				if (ex_slice.contour.contains_b(*j_it)) // 被最外轮廓包含
				{
					bool IsHoleinHole = false;
					for (Polygon ex_hole : ex_slice.holes)
					{
						if (ex_hole.contains_b(*j_it))
						{
							LOGINFO("IsHoleinHole = True");
							IsHoleinHole = true;
							break;
						}
					}
					// 如果该洞被其他洞包含，则不属于这个轮廓
					if (IsHoleinHole)
					{
						j_it++;
					}
					else
					{
						ex_slice.holes.push_back(*j_it);
						j_it = hole_loops.erase(j_it);
					}
				}
				else
				{
					j_it++;
				}
			}
			if (false)
			{
				static int ex_count = 0;
				char _name[255];
				sprintf(_name, "ex_slice[%d].svg", ex_count++);
				SVG _svg(_name);
				_svg.draw(ex_slice, "red");
				_svg.Close();
			}

			ex_slices.push_back(ex_slice);
		}
		LOGINFO("形成多边形  耗时【%u】 Left hole size = %d", GetTickCount() - dwStart, hole_loops.size());
		// 不执行补救措施
		if (abandon_hole_loops)
		{
			slices->insert(slices->end(), ex_slices.begin(), ex_slices.end());
			return;
		}

		// 处理剩下的孔洞
		ExPolygons ex_slices_left;
		for (std::vector<Polygon>::iterator hole_it = hole_loops.begin(); hole_it != hole_loops.end(); hole_it++)
		{
			for (std::vector<ExPolygon>::iterator ex_it = ex_slices.begin(); ex_it != ex_slices.end();)
			{
				Polygons recale_polygons;
				// 图形相交
				if (ex_it->contour.part_contains_b(*hole_it))
				{
					ex_it->holes.push_back(*hole_it);
					recale_polygons.push_back(ex_it->contour);
					recale_polygons = diff(recale_polygons, ex_it->holes);
					ex_it = ex_slices.erase(ex_it);
					ExPolygons recale_expolygons = union_ex(recale_polygons);
					ex_slices_left.insert(ex_slices_left.end(), recale_expolygons.begin(), recale_expolygons.end());
				}
				else
				{
					ex_it++;
				}
			}
		}
		// append to the supplied collection
		slices->insert(slices->end(), ex_slices.begin(), ex_slices.end());
		slices->insert(slices->end(), ex_slices_left.begin(), ex_slices_left.end());
		// 还有孔洞残留
		if (hole_loops.size())
		{
			LOGINFO("abandon hole_loops size = %d", hole_loops.size());
			Polygons slice_pls;
			for (unsigned int i = 0; i < slices->size(); i++)
			{
				Polygons temp_pls = slices->at(i);
				slice_pls.insert(slice_pls.begin(), temp_pls.begin(), temp_pls.end());
			}
			slice_pls = diff(slice_pls, hole_loops);
			ExPolygons slice_exs = union_ex(slice_pls);
			slices->clear();
			slices->insert(slices->end(), slice_exs.begin(), slice_exs.end());
		}
	}



	void TriangleMesh::make_expolygons2(const Polygons& loops, ExPolygons* slices, double _offset)
	{
		LOGINFO("原始loops个数【%d】发现expolygons个数【%d】", loops.size(), slices->size());
		DWORD dwStart;
		dwStart = GetTickCount();

		std::vector<double> area;
		std::vector<double> abs_area;
		std::vector<size_t> sorted_area; // vector of indices
		for (Polygons::const_iterator loop = loops.begin(); loop != loops.end(); ++loop)
		{
			double a = loop->area();
			area.push_back(a);
			abs_area.push_back(std::fabs(a));
			sorted_area.push_back(loop - loops.begin());
		}
		LOGINFO("构建容器  耗时【%u】", GetTickCount() - dwStart);
		dwStart = GetTickCount();

		// 按照绝对面积大小来排序
		std::sort(sorted_area.begin(), sorted_area.end(), _area_comp(&abs_area)); // outer first
		Polygons copy_loops;
		for (int i : sorted_area)
		{
			copy_loops.push_back(loops[i]);
		}
		LOGINFO("面积排序  耗时【%u】", GetTickCount() - dwStart);
		dwStart = GetTickCount();
		// 组装
		while (copy_loops.size())
		{
			ExPolygon ex_slice;
			ex_slice.contour = copy_loops.front();
			copy_loops.erase(copy_loops.begin());
			// 遍历剩余
			for (std::vector<Polygon>::iterator j_it = copy_loops.begin(); j_it != copy_loops.end();)
			{
				Polygon _copy = *j_it;
				if (ex_slice.contour.contains_b(_copy)) // 被最外轮廓包含
				{

					j_it = copy_loops.erase(j_it);
					continue;
					/*
					// 判断是否偏移轮廓
					Polygons polys;
					polys.push_back(_copy);
					ExPolygons exs;
					exs.push_back(ex_slice);
					ExPolygons left_ex = diff_ex(
						exs,
						offset_ex(polys, scale_(_offset)),
						true);
					if (left_ex.empty()
						|| left_ex.front().is_valid() == false
						|| left_ex.front().area() < scale_(0.1)
						)
					{
						j_it = copy_loops.erase(j_it);
						continue;
					}
					
					bool IsHoleinHole = false;
					for (Polygon ex_hole : ex_slice.holes)
					{
						if (ex_hole.contains_b(_copy))
						{
							LOGINFO("IsHoleinHole = True");
							IsHoleinHole = true;
							break;
						}
					}
					// 如果该洞被其他洞包含，则不属于这个轮廓
					if (IsHoleinHole)
					{
						j_it++;
					}
					else
					{
						ex_slice.holes.push_back(_copy);
						j_it = copy_loops.erase(j_it);
					}
					*/
				}
				else
				{
					j_it++;
				}
			}
			slices->push_back(ex_slice);
		}
	}

}