#include "TriangleMesh.hpp"
#include <float.h>

namespace Slic3r
{

	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	///  树形支撑生成算法
	////////////////////////////////////////////////////////////////////////////////////////////////////////

	// perl入口函数
	// IsMuliti           多重支撑
	// branch_Num  分支最大个数
	// _offset_z         台柱偏移补偿
	// _min_angel     分支最大偏角
	// _min_z            最小树基高度
	bool TriangleMesh::Pre_GeneralTreePoint(bool IsMuliti, int branch_Num, double _offset_z, double _min_angel, double _min_z, double _max_dis)
	{
		std::vector<SPT_pointf>().swap(this->TreePoints);
		this->TreePoints.clear();
		this->offset_z = _offset_z;
		this->min_angel = _min_angel;
		this->min_z = _min_z;
		this->max_dis = _max_dis;

		LOGINFO("branch_Num = %d, offset_z = %f, min_angel = %f,", branch_Num, offset_z, min_angel);
		/*
		while (branch_Num >= 2)
		{
			this->Pre_GeneralTreePoint("all", branch_Num);
			if (IsMuliti)
				this->Pre_GeneralTreePoint("tree", branch_Num);
			branch_Num = branch_Num / 2;
		}
		*/

		this->Pre_GeneralTreePoint("no_boundary", branch_Num);
		this->Pre_GeneralTreePoint("boundary", branch_Num);
		if (IsMuliti)
			this->Pre_GeneralTreePoint("tree", branch_Num);

		return true;
	}

	// 将支撑按区域划分
	bool TriangleMesh::CheckRegion(std::string _type, std::map<unsigned int, std::vector<SPT_pointf *>> &map_pPoints)
	{
		if (_type == "all")
		{
			bool ret = this->CheckRegion("boundary", map_pPoints);
			ret = this->CheckRegion("grid", map_pPoints) || ret;
			ret = this->CheckRegion("stripe", map_pPoints) || ret;
			ret = this->CheckRegion("overhang", map_pPoints) || ret;
			ret = this->CheckRegion("overhangline", map_pPoints) || ret;
			// ret = this->CheckRegion("tree", map_pPoints) || ret;
			return ret;
		}
		else if (_type == "no_boundary")
		{
			bool ret = this->CheckRegion("grid", map_pPoints);
			ret = this->CheckRegion("stripe", map_pPoints) || ret;
			ret = this->CheckRegion("overhang", map_pPoints) || ret;
			ret = this->CheckRegion("overhangline", map_pPoints) || ret;
			ret = this->CheckRegion("custom_inner", map_pPoints) || ret;
			ret = this->CheckRegion("crossbar_tree", map_pPoints) || ret;
			// ret = this->CheckRegion("tree", map_pPoints) || ret;
			return ret;
		}

		std::vector<SPT_pointf> *pPoints = this->get_points(_type);
		if (pPoints == NULL || pPoints->size() == 0)
		{
			return false;
		}
		else
		{
			LOGINFO("%s spt size = [%d]", _type.c_str(), pPoints->size());
		}

		for (int i = 0; i < pPoints->size(); i++)
		{
			SPT_pointf *pPoint = &(pPoints->at(i));
			// 已经是枝丫  不再树成型
			if (pPoint->treebasep != NULL)
				continue;

			// 按照_region_id来分类
			unsigned int _region_id = pPoint->region_id;
			if (map_pPoints.find(_region_id) == map_pPoints.end())
			{
				std::vector<SPT_pointf *> new_regin;
				new_regin.push_back(pPoint);
				map_pPoints[_region_id] = new_regin;
			}
			else
			{
				map_pPoints[_region_id].push_back(pPoint);
			}
		}

		std::map<unsigned int, std::vector<SPT_pointf *>>::iterator p_It = map_pPoints.begin();
		while (p_It != map_pPoints.end())
		{
			LOGINFO("map_pPoints[%d].size = %d", p_It->first, p_It->second.size());
			p_It++;
		}

		return true;
	}

	// 计算点池的中心点
	Pointf3 TriangleMesh::Cale_treebase(std::vector<SPT_pointf *> &banch_points)
	{
		Pointf3 treebasePointf3(0, 0, 0);
		if (banch_points.size() == 0)
			return treebasePointf3;

		for (int i = 0; i < banch_points.size(); i++)
		{
			treebasePointf3.x += banch_points.at(i)->hitpoint.x;
			treebasePointf3.y += banch_points.at(i)->hitpoint.y;
			treebasePointf3.z += banch_points.at(i)->get_mincollsionZ();
		}
		treebasePointf3.x /= banch_points.size();
		treebasePointf3.y /= banch_points.size();
		treebasePointf3.z /= banch_points.size();

		return treebasePointf3;
	}

	// 从点池中寻找离treebasePoint最近的点
	SPT_pointf *TriangleMesh::find_onepoint_Nearest(Pointf3 treebasePoint, std::list<SPT_pointf *> &raw_points, double max_dis)
	{
		SPT_pointf *retval = NULL;
		if (raw_points.size() == 0) // 点池中不再有点
			return retval;

		double PPdistance = DBL_MAX;
		std::list<SPT_pointf *>::iterator rawIt = raw_points.begin();
		std::list<SPT_pointf *>::iterator neareast_It = raw_points.end();
		for (; rawIt != raw_points.end(); rawIt++)
		{
			Pointf3 temp((*rawIt)->hitpoint.x, (*rawIt)->hitpoint.y, (*rawIt)->get_mincollsionZ());
			double _distance = treebasePoint.distance_to(temp);
			if (PPdistance > _distance && _distance < max_dis)
			{
				PPdistance = _distance;
				neareast_It = rawIt;
			}
		}
		//
		if (neareast_It != raw_points.end())
		{
			retval = (*neareast_It);
			raw_points.erase(neareast_It);
		}

		return retval;
	}

	// 从点池中按照最近距离聚合一颗树
	bool TriangleMesh::find_onetree(int branch_Num,
									std::list<SPT_pointf *> &raw_points,
									std::vector<SPT_pointf *> &tree_points,
									Pointf &treebasePoint)
	{
		if (branch_Num < 2)
		{
			LOGINFO("branch_Num Error[%d]", branch_Num);
			return false;
		}
		if (raw_points.size() <= 0) // 点池中不再有点
		{
			LOGINFO("raw_points Error[%d]", raw_points.size());
			return false;
		}
		tree_points.clear();
		// 初始值
		Pointf3 treebasePointf3(raw_points.front()->hitpoint.x, raw_points.front()->hitpoint.y, raw_points.front()->get_mincollsionZ());
		SPT_pointf *onePoint = this->find_onepoint_Nearest(treebasePointf3, raw_points, this->max_dis);
		//
		while (onePoint != NULL)
		{
			tree_points.push_back(onePoint);
			treebasePointf3 = this->Cale_treebase(tree_points);
			if (tree_points.size() < branch_Num)
				onePoint = this->find_onepoint_Nearest(treebasePointf3, raw_points, this->max_dis);
			else
				break;
		}

		treebasePointf3 = this->Cale_treebase(tree_points);
		treebasePoint.x = treebasePointf3.x;
		treebasePoint.y = treebasePointf3.y;
		LOGINFO("treebasePoint [%s]", treebasePointf3.dump_perl().c_str());

		return true;
	}

	// 枝丫和主干划分
	bool TriangleMesh::Pre_GeneralTreePoint(std::string _type, int branch_Num)
	{
		// 支撑按区域划分
		std::map<unsigned int, std::vector<SPT_pointf *>> map_pPoints;
		map_pPoints.clear();
		this->CheckRegion(_type, map_pPoints);
		LOGINFO("[%s] map size %d", _type.c_str(), map_pPoints.size());

		// 每个区域独立处理
		std::map<unsigned int, std::vector<SPT_pointf *>>::iterator mapIt = map_pPoints.begin();
		for (; mapIt != map_pPoints.end(); mapIt++)
		{
			// 区域编号
			unsigned int region_id = mapIt->first;
			// 区域0的支撑不处理
			if (region_id == 0)
				continue;
			// 构造区域内的点索引集
			std::list<SPT_pointf *> Points_list;
			for (int i = 0; i < mapIt->second.size(); i++)
			{
				Points_list.push_back(mapIt->second.at(i));
			}
			LOGINFO("region_id = %d, size = %d", region_id, Points_list.size());

			// 树枝
			std::vector<SPT_pointf *> treePoints;
			// 树基台的位置
			Pointf treebasePoint(0, 0);
			while (this->find_onetree(branch_Num, Points_list, treePoints, treebasePoint))
			{
				LOGINFO("Points_list left size = %d, treePoints size = %d", Points_list.size(), treePoints.size());
				if (treePoints.size() < 2) // 树至少两颗枝丫
					continue;

				SPT_pointf new_treeBase = SPT_pointf(treebasePoint, 0.0, region_id);
				// 枝丫信息赋值
				new_treeBase.branch_vecp = treePoints;
				// 计算树枝高度
				if (this->Cale_treebaseZ(new_treeBase))
				{
					// 树基台支撑入库 枝丫指针赋值
					this->TreePoints.push_back(new_treeBase);
					for (int i = 0; i < treePoints.size(); i++)
					{
						SPT_pointf *pPoint = treePoints.at(i);
						pPoint->treebasep = &(this->TreePoints.back());
						LOGINFO("new_treeBase id = %d, branch id = %d", region_id, pPoint->region_id);
					}
				}
			}
		}

		return true;
	}

	// 计算主干的高度 形成树枝
	bool TriangleMesh::Cale_treebaseZ(SPT_pointf &sptP)
	{
		if (sptP.IsTreeBase() == false)
		{
			LOGINFO("It is not a tree base spt!");
			return false;
		}
		if (sptP.IsCollsion())
		{
			LOGINFO("this tree base has been caled!");
			return false;
		}
		// 树基台的最小高度
		double treeBaseZ = DBL_MAX;
		for (int i = 0; i < sptP.branch_vecp.size(); i++)
		{
			SPT_pointf *pPoint = sptP.branch_vecp.at(i);
			if (pPoint)
			{
				double BanchZ = this->Get_treebaseZ(sptP.hitpoint, pPoint->hitpoint, pPoint->get_mincollsionZ(), this->offset_z, this->min_angel);
				if (BanchZ < treeBaseZ)
					treeBaseZ = BanchZ;
			}
		}

		// 修改枝丫信息
		if (treeBaseZ == DBL_MAX)
		{
			return false;
		}
		else if (treeBaseZ < this->min_z)
		{
			LOGINFO("treeBaseZ error [%f], minZ = [%f]", treeBaseZ, this->min_z);
			// treeBaseZ = this->min_z;
			return false;
		}
		else
		{
			// 树基高度减小一半，树枝长度变长
			// treeBaseZ =treeBaseZ / 2;
		}
		// 为树根虚拟一个碰撞点信息
		sptP.CollsionZ_map[treeBaseZ] = -1;
		// 修改枝丫的脚点
		for (int i = 0; i < sptP.branch_vecp.size(); i++)
		{
			SPT_pointf *pPoint = sptP.branch_vecp.at(i);
			if (pPoint)
			{
				pPoint->basepoint = sptP.hitpoint;
				pPoint->basepoint_z = treeBaseZ;
				// 树基支撑点高度大于支撑点高度
				if (treeBaseZ >= pPoint->get_mincollsionZ())
				{
					LOGINFO("treeBaseZ = [%f], treeBase = %s, banch = %s, banchMinz = %f", treeBaseZ,
							sptP.hitpoint.dump_perl().c_str(),
							pPoint->hitpoint.dump_perl().c_str(),
							pPoint->get_mincollsionZ());
				}
			}
		}

		return true;
	}

	// 计算主干的z值
	double TriangleMesh::Get_treebaseZ(Pointf treeBase, Pointf BanchPoint, double BanchZ, double offset_z, double min_angel)
	{
		if (min_angel <= 0 || min_angel >= 90)
		{
			LOGINFO("min_angel error = %f", min_angel);
			return 0.0;
		}

		min_angel = min_angel * PI / 180;
		double Banch_distance = treeBase.distance_to(BanchPoint);
		double Banch_Heigt = (Banch_distance / tan(min_angel)) + offset_z;

		double retval = BanchZ - Banch_Heigt;
		if (retval < 0)
		{
			LOGINFO("Error retval = %f", retval);
			retval = 0;
		}

		return retval;
	}

}