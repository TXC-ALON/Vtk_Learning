#include "TriangleMesh.hpp"
#include <float.h>

namespace Slic3r
{
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	///  网格点生成算法
	////////////////////////////////////////////////////////////////////////////////////////////////////////

	// 构建网格点 并不保证一定会有交点
	bool TriangleMesh::Pre_GeneralGridPoint(
		size_t spted_volume_id,
		double angel,
		double step,
		int X_num,
		int Y_num,
		bool notInTuqi)
	{
		// 数据初始化
		this->GirdPoints.clear();

		double bound_diameter = sqrt(
			stl.stats.size.x * stl.stats.size.x +
			stl.stats.size.y * stl.stats.size.y);
		Pointf obj_center((stl.stats.min.x + stl.stats.max.x) / 2, (stl.stats.min.y + stl.stats.max.y) / 2);
		LOGINFO("Min (%f, %f), Max (%f, %f),size (%f, %f), bound_diameter->%f",
				stl.stats.min.x,
				stl.stats.min.y,
				stl.stats.max.x,
				stl.stats.max.y,
				stl.stats.size.x,
				stl.stats.size.y,
				bound_diameter);

		unsigned int i = 0;
		unsigned int j = 0;
		Pointf tempPoint, TransferPoint;
		tempPoint.x = (double)i * step;
		tempPoint.y = (double)j * step;
		LOGINFO("Step = %f", step);
		while (tempPoint.y <= bound_diameter * 2) // 换行
		{
			// 初始化行计数
			i = 0;
			tempPoint.x = (double)i * step;

			while (tempPoint.x <= bound_diameter * 2) // 某一行
			{
				bool BeSelected_line = false;
				bool BeSelected_colume = false;
				if ((j % (Y_num + 1) == 0) &&
					((i % (X_num + 1) != 0) || (X_num == 0))) // 行命中
				{
					BeSelected_line = true;
					// LOGINFO("BeSelected_line [%d, %d]", i, j);
				}
				if ((i % (X_num + 1) == 0) &&
					((j % (Y_num + 1) != 0) || (Y_num == 0))) // 列命中
				{
					BeSelected_colume = true;
					// LOGINFO("BeSelected_colume [%d, %d]", i, j);
				}

				if (BeSelected_line || BeSelected_colume)
				{
					TransferPoint = tempPoint;
					// LOGINFO("BeSelected_Orgin_Point [%f, %f]", tempPoint.x, tempPoint.y);
					TransferPoint.translate(-bound_diameter, -bound_diameter);
					TransferPoint.rotate(angel * PI / 180.0);
					TransferPoint.translate(obj_center.x, obj_center.y);
					if ((TransferPoint.x <= stl.stats.max.x) &&
						(TransferPoint.x >= stl.stats.min.x) &&
						(TransferPoint.y <= stl.stats.max.y) &&
						(TransferPoint.y >= stl.stats.min.y))
					{
						int loop_id = this->PointIn_exboundLoops(TransferPoint);
						if (loop_id >= 0) // 网格点必须在loop里
						{
							if ((notInTuqi && this->PointIn_tuqiLoops(TransferPoint) != -1) == false) // 排除凸起区域 该点在凸起里
							{
								// LOGINFO("BeSelected_Point [%f, %f]", TransferPoint.x, TransferPoint.y);
								if (BeSelected_line)
								{
									SPT_pointf spt_pf(TransferPoint, angel, this->get_tree_region_idx(spted_volume_id, loop_id));
									this->GirdPoints.push_back(spt_pf);
								}
								if ((BeSelected_colume) && !(X_num == 0 && Y_num == 0))
								{
									SPT_pointf spt_pf(TransferPoint, angel + 90, this->get_tree_region_idx(spted_volume_id, loop_id));
									this->GirdPoints.push_back(spt_pf);
								}
							}
						}
					}
				}
				tempPoint.x = (double)(++i) * step;
			}
			tempPoint.y = (double)(++j) * step;
		}

		return true;
	}
	// 构建自定义区域的网格点 并不保证一定会有交点
	bool TriangleMesh::Pre_CustomGridPoint(
		size_t spted_volume_id,
		double angel,
		double step,
		int X_num,
		int Y_num,
		bool notInTuqi)
	{
		LOGINFO("0724 Pre_CustomGridPoint start");
		// 数据初始化
		this->custom_GridPoints.clear();
		// BoundingBox bb;
		// for(ExPolygon::const_iterator ExPolygon_ct = custom_simply_boundary.begin();ExPolygon_ct != custom_simply_boundary.end();++ExPolygon_ct){
		// 	bb.merge(ExPolygon_ct->contour.bounding_box());
		// }
		double bound_diameter = sqrt(
			stl.stats.size.x * stl.stats.size.x +
			stl.stats.size.y * stl.stats.size.y);
		Pointf obj_center((stl.stats.min.x + stl.stats.max.x) / 2, (stl.stats.min.y + stl.stats.max.y) / 2);
		LOGINFO("Min (%f, %f), Max (%f, %f),size (%f, %f), bound_diameter->%f",
				stl.stats.min.x,
				stl.stats.min.y,
				stl.stats.max.x,
				stl.stats.max.y,
				stl.stats.size.x,
				stl.stats.size.y,
				bound_diameter);

		unsigned int i = 0;
		unsigned int j = 0;
		Pointf tempPoint, TransferPoint;
		tempPoint.x = (double)i * step;
		tempPoint.y = (double)j * step;
		LOGINFO("Step = %f", step);
		while (tempPoint.y <= bound_diameter * 2) // 换行
		{
			// 初始化行计数
			i = 0;
			tempPoint.x = (double)i * step;

			while (tempPoint.x <= bound_diameter * 2) // 某一行
			{
				bool BeSelected_line = false;
				bool BeSelected_colume = false;
				if ((j % (Y_num + 1) == 0) &&
					((i % (X_num + 1) != 0) || (X_num == 0))) // 行命中
				{
					BeSelected_line = true;
					// LOGINFO("BeSelected_line [%d, %d]", i, j);
				}
				if ((i % (X_num + 1) == 0) &&
					((j % (Y_num + 1) != 0) || (Y_num == 0))) // 列命中
				{
					BeSelected_colume = true;
					// LOGINFO("BeSelected_colume [%d, %d]", i, j);
				}

				if (BeSelected_line || BeSelected_colume)
				{
					TransferPoint = tempPoint;
					// LOGINFO("BeSelected_Orgin_Point [%f, %f]", tempPoint.x, tempPoint.y);
					TransferPoint.translate(-bound_diameter, -bound_diameter);
					TransferPoint.rotate(angel * PI / 180.0);
					TransferPoint.translate(obj_center.x, obj_center.y);
					if ((TransferPoint.x <= stl.stats.max.x) &&
						(TransferPoint.x >= stl.stats.min.x) &&
						(TransferPoint.y <= stl.stats.max.y) &&
						(TransferPoint.y >= stl.stats.min.y))
					{
						int loop_id = this->PointIn_customboundLoops(TransferPoint);
						if (loop_id >= 0) // 网格点必须在loop里
						{
							if ((notInTuqi && this->PointIn_tuqiLoops(TransferPoint) != -1) == false) // 排除凸起区域 该点在凸起里
							{
								// LOGINFO("BeSelected_Point [%f, %f]", TransferPoint.x, TransferPoint.y);
								if (BeSelected_line)
								{
									SPT_pointf spt_pf(TransferPoint, angel, this->get_tree_region_idx(spted_volume_id, loop_id));
									this->custom_GridPoints.push_back(spt_pf);
								}
								if ((BeSelected_colume) && !(X_num == 0 && Y_num == 0))
								{
									SPT_pointf spt_pf(TransferPoint, angel + 90, this->get_tree_region_idx(spted_volume_id, loop_id));
									this->custom_GridPoints.push_back(spt_pf);
								}
							}
						}
					}
				}
				tempPoint.x = (double)(++i) * step;
			}
			tempPoint.y = (double)(++j) * step;
		}
		LOGINFO("0724 CustomGridPoint num = %d", this->custom_GridPoints.size());
		return true;
	}

}