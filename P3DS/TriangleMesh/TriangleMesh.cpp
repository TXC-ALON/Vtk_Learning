#include "TriangleMesh.hpp"
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
#include "SVG.hpp"

namespace Slic3r
{

	TriangleMesh::TriangleMesh()
		: repaired(false), checkonly(false)
	{
		stl_initialize(&this->stl);
		stl_repair_init(&this->stl);
	}

	// 通过点和面的信息构建stl结构体
	// Pointf3和Point3的区别
	// Pointf3是一个点
	// Point3是一个面，x，y，z表示三个顶点在points里的索引号
	TriangleMesh::TriangleMesh(const Pointf3s &points, const std::vector<Point3> &facets)
		: repaired(false), checkonly(false)
	{
		stl_initialize(&this->stl);
		stl_file &stl = this->stl;
		stl.error = 0;
		stl.stats.type = inmemory;

		// count facets and allocate memory
		stl.stats.number_of_facets = facets.size();
		stl.stats.original_num_facets = stl.stats.number_of_facets;
		stl_allocate(&stl);

		for (int i = 0; i < stl.stats.number_of_facets; i++)
		{
			stl_facet facet;
			facet.normal.x = 0;
			facet.normal.y = 0;
			facet.normal.z = 0;

			const Pointf3 &ref_f1 = points[facets[i].x];
			facet.vertex[0].x = ref_f1.x;
			facet.vertex[0].y = ref_f1.y;
			facet.vertex[0].z = ref_f1.z;

			const Pointf3 &ref_f2 = points[facets[i].y];
			facet.vertex[1].x = ref_f2.x;
			facet.vertex[1].y = ref_f2.y;
			facet.vertex[1].z = ref_f2.z;

			const Pointf3 &ref_f3 = points[facets[i].z];
			facet.vertex[2].x = ref_f3.x;
			facet.vertex[2].y = ref_f3.y;
			facet.vertex[2].z = ref_f3.z;

			stl.facet_start[i] = facet;
		}
		stl_get_size(&stl);
	}

	TriangleMesh::TriangleMesh(const Pointf3s &points, const std::vector<Point3> &facets, const std::vector<int> &marks)
		: repaired(false), checkonly(false)
	{
		DWORD timecount = GetTickCount();
		stl_initialize(&this->stl);
		stl_file &stl = this->stl;
		stl.error = 0;
		stl.stats.type = inmemory;

		// count facets and allocate memory
		stl.stats.number_of_facets = facets.size();
		stl.stats.original_num_facets = stl.stats.number_of_facets;
		stl_allocate(&stl);

		for (int i = 0; i < stl.stats.number_of_facets; i++)
		{
			int mark = marks[i];

			stl_facet facet;
			facet.normal.x = 0;
			facet.normal.y = 0;
			facet.normal.z = 0;

			const Pointf3 &ref_f1 = points[facets[i].x];
			facet.vertex[0].x = ref_f1.x;
			facet.vertex[0].y = ref_f1.y;
			facet.vertex[0].z = ref_f1.z;

			const Pointf3 &ref_f2 = points[facets[i].y];
			facet.vertex[1].x = ref_f2.x;
			facet.vertex[1].y = ref_f2.y;
			facet.vertex[1].z = ref_f2.z;

			const Pointf3 &ref_f3 = points[facets[i].z];
			facet.vertex[2].x = ref_f3.x;
			facet.vertex[2].y = ref_f3.y;
			facet.vertex[2].z = ref_f3.z;

			TriangleMesh::parse_face_mark(mark, facet.face_side, facet.face_type, facet.face_type_id);
			// LOGINFO("facet.face_side = [%d], facet.face_type = [%d], facet.face_type_id = [%d]", facet.face_side, facet.face_type, facet.face_type_id);
			stl.facet_start[i] = facet;
		}

		stl_get_size(&stl);

		LOGINFO("TriangleMesh::TriangleMesh times =%d", GetTickCount() - timecount);
	}

	// 用已有的TriangleMesh构造本TriangleMesh结构
	TriangleMesh::TriangleMesh(const TriangleMesh &other)
		: stl(other.stl), repaired(other.repaired), checkonly(other.checkonly)
	{
		this->stl.heads = NULL;
		this->stl.tail = NULL;
		this->stl.error = other.stl.error;
		if (other.stl.facet_start != NULL)
		{
			this->stl.facet_start = (stl_facet *)calloc(other.stl.stats.facets_malloced, sizeof(stl_facet));
			std::copy(other.stl.facet_start, other.stl.facet_start + other.stl.stats.number_of_facets, this->stl.facet_start);
		}
		if (other.stl.neighbors_start != NULL)
		{
			this->stl.neighbors_start = (stl_neighbors *)calloc(other.stl.stats.facets_malloced, sizeof(stl_neighbors));
			std::copy(other.stl.neighbors_start, other.stl.neighbors_start + other.stl.stats.number_of_facets, this->stl.neighbors_start);
		}
		if (other.stl.v_indices != NULL)
		{
			this->stl.v_indices = (v_indices_struct *)calloc(other.stl.stats.number_of_facets, sizeof(v_indices_struct));
			std::copy(other.stl.v_indices, other.stl.v_indices + other.stl.stats.number_of_facets, this->stl.v_indices);
		}
		if (other.stl.v_shared != NULL)
		{
			this->stl.v_shared = (stl_vertex *)calloc(other.stl.stats.shared_vertices, sizeof(stl_vertex));
			std::copy(other.stl.v_shared, other.stl.v_shared + other.stl.stats.shared_vertices, this->stl.v_shared);
		}
		 this->BoundaryPoints = other.BoundaryPoints;
             this->GirdPoints = other.GirdPoints;
             this->tuqiPoints = other.tuqiPoints;
             this->TreePoints = other.TreePoints;
             this->OverhangPoints = other.OverhangPoints;
             this->OverhangLinesPoints = other.OverhangLinesPoints;
             this->numPoints = other.numPoints;
             this->SelectedPoints = other.SelectedPoints;
             this->custom_GridPoints = other.custom_GridPoints;
             this->SelectedCenter = other.SelectedCenter;
             this->SelectedNormal = other.SelectedNormal;
	}

	// 重载操作符=
	TriangleMesh &TriangleMesh::operator=(TriangleMesh other)
	{
		this->swap(other);
		return *this;
	}

	// 交换两个结构体的数值
	void
	TriangleMesh::swap(TriangleMesh &other)
	{
		 std::swap(this->stl, other.stl);
         std::swap(this->repaired, other.repaired);
         std::swap(this->checkonly, other.checkonly);
         std::swap(this->BoundaryPoints, other.BoundaryPoints);
         std::swap(this->GirdPoints, other.GirdPoints);
         std::swap(this->tuqiPoints, other.tuqiPoints);
         std::swap(this->TreePoints, other.TreePoints);
         std::swap(this->OverhangPoints, other.OverhangPoints);
         std::swap(this->OverhangLinesPoints, other.OverhangLinesPoints);
         std::swap(this->numPoints, other.numPoints);
         std::swap(this->SelectedPoints, other.SelectedPoints);
         std::swap(this->custom_GridPoints, other.custom_GridPoints);
         std::swap(this->SelectedCenter, other.SelectedCenter);
         std::swap(this->SelectedNormal, other.SelectedNormal);
	}

	TriangleMesh::~TriangleMesh()
	{
		this->ClearMesh();
	}

	void
	TriangleMesh::ClearMesh()
	{
         stl_close(&this->stl);
         stl_initialize(&this->stl);

         std::vector<SPT_pointf>().swap(GirdPoints);
         BoundLines_vec().swap(boundLines_vec);
         BoundLoops_vec().swap(boundLoops_vec);
         BoundLoops_vec().swap(wallLoops_vec);
         Polygons().swap(boundLoops);
         ExPolygons().swap(exboundLoops);
         ExPolygons().swap(custom_simply_boundary);
         std::vector<SPT_pointf>().swap(BoundaryPoints);
         BoundLines_vec().swap(tuqiLines_vec);
         BoundLoops_vec().swap(tuqiLoops_vec);
         Polygons().swap(tuqiLoops);
         ExPolygons().swap(extuqiLoops);
         std::vector<SPT_pointf>().swap(tuqiPoints);
         std::vector<int>().swap(tuqi_facets_index);
         std::vector<int>().swap(other_facets_index);
         std::vector<SPT_pointf>().swap(OverhangPoints);
         std::vector<SPT_pointf>().swap(OverhangLinesPoints);
         BoundLines_vec().swap(OverhangLines);
         Polylines().swap(Overhang_polylines);
         std::vector<SPT_pointf>().swap(TreePoints);
         std::vector<SPT_pointf>().swap(numPoints);
         std::vector<SPT_pointf>().swap(SelectedPoints);
         std::vector<SPT_pointf>().swap(custom_GridPoints);
         hash_facets.clear();
	}

	// 打开stl文件
	void
	TriangleMesh::ReadSTLFile(const std::string &input_file)
	{
		LOGINFO("input_file = %s", input_file.c_str());
		std::string _input_file = input_file;
		std::string input_file_hex = ConverToHexString(_input_file);
		LOGINFO("input_file_hex = %s", input_file_hex.c_str());

		std::wstring UTF8_input_file = Utf8ToUnicode(_input_file);
		OutputDebugStringW(UTF8_input_file.c_str());
		//std::wstring UTF8_input_file_hex = ConverToHexWString(UTF8_input_file);
		//OutputDebugStringW(UTF8_input_file_hex.c_str());

		stl_wopen(&stl, UTF8_input_file.c_str());
		if (this->stl.error != 0)
			LOGINFO("Failed to read STL file");
	}

	// 写ascii文件
	void
	TriangleMesh::write_ascii(const std::string &output_file)
	{
		stl_write_ascii(&this->stl, output_file.c_str(), "");
	}

	// 写二进制文件
	void
	TriangleMesh::write_binary(const std::string &output_file)
	{
		// stl_write_binary(&this->stl, output_file.c_str(), "");
		std::string _output_file = output_file;
		std::wstring UTF8_output_file = Utf8ToUnicode(_output_file);
		OutputDebugStringW(UTF8_output_file.c_str());

		stl_write_binary_w(&this->stl, UTF8_output_file.c_str(), "");
	}

	// 对stl文件进行修复
	void
	TriangleMesh::repair(bool auto_repair)
	{

		LOGINFO("start repairing...");
		if (this->repaired)
		{
			LOGINFO("repairing... 已修复。返回。");
			return; // 修复标记 不会进行重复修复
		}
		LOGINFO("0609-1");
		if (this->checkonly)
		{
			// LOGINFO("repairing... nonsolid，标记 repaired 为 true，返回。");
			this->repaired = true;
			return;
		}
		LOGINFO("0609-2");
		// admesh fails when repairing empty meshes
		if (this->stl.stats.number_of_facets == 0)
		{
			LOGINFO("repairing... 发现空 Mesh，返回。");
			this->stl.stats.unable_repair = true;
			this->repaired = true;
			return;
		}
		LOGINFO("0609-3");
		stl_repair_init(&stl);

		if (true)
		{
			int facetnum = this->stl.stats.number_of_facets;
			calculate_normals(&stl);
			float area = get_area(&stl);
			float volume = get_volume(&stl);
			if ((fabs(volume) < 0.000001) && (area < 0.1))
			{
				LOGINFO("repairing... 首次修复发现多壳体错误，此错误壳体面片数 %d，体积 %f，面积 %f。删除。", facetnum, volume, area);
				for (int i = 0; i < this->stl.stats.number_of_facets; i++)
				{
					stl_remove_degenerate(&stl, i);
				}
				this->stl.stats.unable_repair = true;
				this->repaired = true;
				return;
			}
		}
		LOGINFO("0609-4");
		if (auto_repair)
		{
			LOGINFO("repairing... 开始修复。");
		}

		// 检测三角面片的拓扑结构
		this->check_topology(auto_repair);

		// 原始模型有空洞为设计问题
		// 添加正常空洞的判定条件
		if (this->is_normal_hole() == false)
		{
			LOGINFO("The result of checking normal hole is false.");
		}

		stl_fix_facets_shared_one_edge(&stl, auto_repair);

		// remove_unconnected
		if (stl.stats.connected_facets_3_edge < stl.stats.number_of_facets && auto_repair)
		{
			stl_remove_unconnected_facets(&stl);
		}

		// fill_holes
		if (stl.stats.connected_facets_3_edge < stl.stats.number_of_facets && auto_repair)
		{
			stl_fill_holes(&stl);
			stl_clear_error(&stl);
		}
		LOGINFO("0609-5");
		// normal_directions 始终进行法向错误修复，确保 shell 数量准确
		// 兼顾非流形错误跳过法相修复，同时记录壳体信息
		if (stl.stats.no_need_fix_normal_direction)
		{
			LOGINFO("之前 Before no_need_fix_normal_direction stl.stats.number_of_parts %d", stl.stats.number_of_parts);
			LOGINFO("之前 Before no_need_fix_normal_direction stl.repair_stats.noise_shells %d", stl.repair_stats.noise_shells);

			stl_fix_normal_directions(&stl, false);

			LOGINFO("之后 After no_need_fix_normal_direction stl.stats.number_of_parts %d", stl.stats.number_of_parts);
			LOGINFO("之后 After no_need_fix_normal_direction stl.repair_stats.noise_shells %d", stl.repair_stats.noise_shells);
		}
		else
		{
			LOGINFO("之前 Before stl_fix_normal_directions stl.stats.number_of_parts %d", stl.stats.number_of_parts);
			LOGINFO("之前 Before stl_fix_normal_directions stl.repair_stats.noise_shells %d", stl.repair_stats.noise_shells);

			stl_fix_normal_directions(&stl, auto_repair);

			LOGINFO("之后 After stl_fix_normal_directions stl.stats.number_of_parts %d", stl.stats.number_of_parts);
			LOGINFO("之后 After stl_fix_normal_directions stl.repair_stats.noise_shells %d", stl.repair_stats.noise_shells);
		}

		// normal_values
		stl_fix_normal_values(&stl, auto_repair);

		if (auto_repair)
		{
			// always calculate the volume and reverse all normals if volume is negative
			(void)this->volume();

			// neighbors
			stl_verify_neighbors(&stl);

			this->repaired = true;
		}

		LOGINFO("repaired... 修复结束。");
	}

	// 计算stl中模型的体积
	float
	TriangleMesh::volume()
	{
		if (this->stl.stats.volume == -1)
			stl_calculate_volume(&this->stl);
		return this->stl.stats.volume;
	}

	// 计算stl中模型的表面积
	float
	TriangleMesh::surface_area()
	{
		float area = 0.0;
		for (unsigned int i = 0; i < this->facets_count(); i++)
		{
			stl_facet &facet = this->stl.facet_start[i];

			area += this->Calculate_Facet_Area(facet);
		}

		return area;
	}

	// 检查三角形面片的拓扑结构
	void
	TriangleMesh::check_topology(bool auto_repair)
	{
		if (this->facets_count() == 0 || this->stl.error != 0)
			return;

		if (auto_repair)
		{
			LOGINFO("repairing... 开始检查三角形面片的拓扑结构。");
		}
		else
		{
			LOGINFO("checking... 开始检查三角形面片的拓扑结构。");
		}
		// checking exact
		stl_check_facets_exact(&stl);																			 // 精确的检查模型中的三角形面片
		stl.stats.facets_w_1_bad_edge = (stl.stats.connected_facets_2_edge - stl.stats.connected_facets_3_edge); // 有一条孤立边的三角面片个数
		stl.stats.facets_w_2_bad_edge = (stl.stats.connected_facets_1_edge - stl.stats.connected_facets_2_edge); // 有两条孤立边的三角面片个数
		stl.stats.facets_w_3_bad_edge = (stl.stats.number_of_facets - stl.stats.connected_facets_1_edge);		 // 有三条孤立边的三角面片个数

		stl.repair_stats.bad_edges = stl.stats.facets_w_1_bad_edge + stl.stats.facets_w_2_bad_edge * 2 + stl.stats.facets_w_3_bad_edge * 3;

		// checking nearby
		// int last_edges_fixed = 0;
		float tolerance = stl.stats.shortest_edge;							// 过滤值为最小的边长
		float increment = stl.stats.bounding_diameter / 10000.0;			// 步长为包围盒的对角线长度
		int iterations = 2;													// 迭代次数
		if (stl.stats.connected_facets_3_edge < stl.stats.number_of_facets) // 迭代终止条件：无孤立边
		{
			for (int i = 0; i < iterations; i++)
			{
				if (stl.stats.connected_facets_3_edge < stl.stats.number_of_facets)
				{
					// printf("Checking nearby. Tolerance= %f Iteration=%d of %d...", tolerance, i + 1, iterations);
					stl_check_facets_nearby(&stl, tolerance);
					// printf("  Fixed %d edges.\n", stl.stats.edges_fixed - last_edges_fixed);
					// last_edges_fixed = stl.stats.edges_fixed;
					tolerance += increment;
				}
				else
				{
					break;
				}
			}
		}
	}

	// 检测stl模型不存在孤立边
	bool
	TriangleMesh::is_manifold() const
	{
		return this->stl.stats.connected_facets_3_edge == this->stl.stats.number_of_facets;
	}

	//== == == == == == == == = Results produced by ADMesh version 0.98.3 == == == == == == == == =
	// Input file : sphere.stl
	// File type : Binary STL file
	// Header : Processed by ADMesh version 0.98.3
	//	== == == == == == == Size == == == == == == ==
	//	Min X = -1.334557, Max X = 1.370952
	//	Min Y = -1.377953, Max Y = 1.377230
	//	Min Z = -1.373225, Max Z = 1.242838
	//	== == == == = Facet Status == == == == == Original == == == == == == Final == ==
	//	Number of facets : 3656                3656
	//	Facets with 1 disconnected edge : 18                   0
	//	Facets with 2 disconnected edges : 3                   0
	//	Facets with 3 disconnected edges : 0                   0
	//	Total disconnected facets : 21                   0
	//	== = Processing Statistics == = == == = Other Statistics == == =
	//	Number of parts : 1        Volume : 10.889216
	//	Degenerate facets : 0
	//	Edges fixed : 24
	//	Facets removed : 0
	//	Facets added : 0
	//	Facets reversed : 0
	//	Backwards edges : 0
	//	Normals fixed : 0
	//========================================================================

	// 重置所有状态参数
	void
	TriangleMesh::reset_repair_stats()
	{
		this->stl.stats.degenerate_facets = 0;
		this->stl.stats.edges_fixed = 0;
		this->stl.stats.facets_removed = 0;
		this->stl.stats.facets_added = 0;
		this->stl.stats.facets_reversed = 0;
		this->stl.stats.backwards_edges = 0;
		this->stl.stats.normals_fixed = 0;
	}

	// 判断模型是否需要修复
	bool
	TriangleMesh::needed_repair() const
	{
		return this->stl.stats.degenerate_facets > 0 || this->stl.stats.edges_fixed > 0 || this->stl.stats.facets_removed > 0 || this->stl.stats.facets_added > 0 || this->stl.stats.facets_reversed > 0 || this->stl.stats.backwards_edges > 0;
	}

	// 判断模型是否需要修复
	bool
	TriangleMesh::check_needed_repair() const
	{
		LOGINFO("Checking result of repairing... stl.repair_stats.inverted_normals			%d ", this->stl.repair_stats.inverted_normals);
		LOGINFO("Checking result of repairing... stl.repair_stats.bad_edges					%d ", this->stl.repair_stats.bad_edges);
		LOGINFO("Checking result of repairing... stl.repair_stats.bad_contours				%d ", this->stl.repair_stats.bad_contours);
		LOGINFO("Checking result of repairing... stl.repair_stats.near_bad_edges			%d ", this->stl.repair_stats.near_bad_edges);
		LOGINFO("Checking result of repairing... stl.repair_stats.planar_holes				%d ", this->stl.repair_stats.planar_holes);
		LOGINFO("Checking result of repairing... stl.repair_stats.noise_shells				%d ", this->stl.repair_stats.noise_shells);
		LOGINFO("Checking result of repairing... stl.repair_stats.overlapping_triangles		%d ", this->stl.repair_stats.overlapping_triangles);
		LOGINFO("Checking result of repairing... stl.repair_stats.intersecting_triangles	%d ", this->stl.repair_stats.intersecting_triangles);
		LOGINFO("Checking result of repairing... stl.repair_stats.non_manifold				%d ", this->stl.repair_stats.non_manifold);

		if (this->checkonly == true)
			return false;

		return this->stl.repair_stats.inverted_normals > 0 || this->stl.repair_stats.bad_edges > 0 || this->stl.repair_stats.bad_contours > 0 || this->stl.repair_stats.near_bad_edges > 0 || this->stl.repair_stats.planar_holes > 0 || this->stl.repair_stats.noise_shells > 0 || this->stl.repair_stats.overlapping_triangles > 0 || this->stl.repair_stats.intersecting_triangles > 0 || this->stl.repair_stats.non_manifold > 0;
	}

	// 返回模型所有面片数
	size_t
	TriangleMesh::facets_count() const
	{
		return this->stl.stats.number_of_facets;
	}

	// 返回模型的shell数
	size_t
	TriangleMesh::shells_count() const
	{
		return this->stl.stats.number_of_parts;
	}

	// 返回模型是否可以修复
	bool
	TriangleMesh::UnableRepair() const
	{
		return this->stl.stats.unable_repair;
	}

	// 判断模型空洞是否为正常空洞
	bool
	TriangleMesh::is_normal_hole() const
	{
		LOGINFO("Checking normal hole... stl.stats.number_of_facets			[%d]", this->stl.stats.number_of_facets);
		LOGINFO("Checking normal hole... stl.stats.connected_facets_1_edge	[%d]", this->stl.stats.connected_facets_1_edge);
		LOGINFO("Checking normal hole... stl.stats.connected_facets_2_edge	[%d]", this->stl.stats.connected_facets_2_edge);
		LOGINFO("Checking normal hole... stl.stats.connected_facets_3_edge	[%d]", this->stl.stats.connected_facets_3_edge);

		if (this->stl.stats.number_of_facets == this->stl.stats.connected_facets_1_edge && this->stl.stats.number_of_facets == this->stl.stats.connected_facets_2_edge && (this->stl.stats.number_of_facets - this->stl.stats.connected_facets_3_edge) % 3 == 0)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	// 输出obj文件
	void
	TriangleMesh::WriteOBJFile(const std::string &output_file)
	{
		stl_generate_shared_vertices(&stl);
		stl_write_obj(&stl, output_file.c_str());
	}

	// 进行glm矩阵模型变换
	void
	TriangleMesh::transform(float *trafo3x4)
	{
		if (trafo3x4 == NULL)
		{
			LOGINFO("trafo3x4 == NULL");
		}
		else
		{
			stl_transform(&(this->stl), trafo3x4);
		}
	}

	// 将模型按因子factor放缩
	void TriangleMesh::scale(float factor)
	{
		stl_scale(&(this->stl), factor);
		stl_invalidate_shared_vertices(&this->stl);
	}

	// 将模型按向量versor进行缩放
	void TriangleMesh::scale(const Pointf3 &versor)
	{
		float fversor[3];
		fversor[0] = versor.x;
		fversor[1] = versor.y;
		fversor[2] = versor.z;
		stl_scale_versor(&this->stl, fversor);
		stl_invalidate_shared_vertices(&this->stl);
	}

	// 平移
	void TriangleMesh::translate(float x, float y, float z)
	{
		stl_translate_relative(&(this->stl), x, y, z);
		stl_invalidate_shared_vertices(&this->stl);
	}

	TriangleMesh TriangleMesh::rotate(Eigen::Matrix3d rotation_matrix)
	{
		// LOGINFO("start rotate 1!");
		// DWORD timecount = GetTickCount();

		TriangleMesh mesh(*this);
		// LOGINFO("start rotate 2!");

		// 执行坐标旋转
		for (int i = 0; i < mesh.stl.stats.number_of_facets; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				Eigen::Vector3d ver(
					mesh.stl.facet_start[i].vertex[j].x,
					mesh.stl.facet_start[i].vertex[j].y,
					mesh.stl.facet_start[i].vertex[j].z);

				Eigen::Vector3d ver1 = rotation_matrix * ver;
				mesh.stl.facet_start[i].vertex[j].x = ver1(0);
				mesh.stl.facet_start[i].vertex[j].y = ver1(1);
				mesh.stl.facet_start[i].vertex[j].z = ver1(2);
			}
		}
		stl_get_size(&mesh.stl);
		calculate_normals(&mesh.stl);
		// LOGINFO("start rotate 3!");

		// LOGINFO("rotate mesh times =%d", GetTickCount() - timecount);
		return mesh;
	}

	// 计算整体法相，只针对局部面片生效
	Pointf3
	TriangleMesh::mesh_average_normal()
	{
		Pointf3 sum(0, 0, 0);

		for (int i = 0; i < this->facets_count(); i++)
		{
			stl_facet &facet = this->stl.facet_start[i];
			Pointf3 n = this->GetFacetNormal(i);
			double a = this->Calculate_Facet_Area(facet);
			sum = sum + a * n;
		}
		sum.normalize();
		return sum;
	}

	// 计算中心点，只针对局部面片生效
	Pointf3
	TriangleMesh::center_on_mesh()
	{
		// mesh旋转至法相向下
		Pointf3 normal = this->mesh_average_normal();
		Vector3 from(normal.x, normal.y, normal.z);
		Vector3 to(0.0, 0.0, -1.0);
		Pointf3 ptemp1(0, 0, 0);
		Pointf3 ptemp2(0, 0, 0);
		TriangleMesh mesh = this->rotate(from, to, ptemp1, ptemp2);
		// mesh.write_ascii("center_on_mesh_mesh.stl");
		// double zmin = mesh.bounding_box().min.z;
		// LOGINFO("zmin = [%f]", zmin);
		// if (zmin <= 0.0)
		//	mesh.translate(0, 0, 3.0 - zmin);

		// 取bounding_box中心点
		Pointf midPoint1(mesh.bounding_box().center().x, mesh.bounding_box().center().y);
		// 求中心点向上的第一个交点
		Pointf3 p(0.0, 0.0, 0.0);
		Pointf3 n(0.0, 0.0, 0.0);
		if (mesh.Cale_CollsionZ_byXY(Pointf(midPoint1.x, midPoint1.y), p, n) == false)
		{
			LOGINFO("bounding_box Cale_CollsionZ_byXY false!");
			// 取得投影的中心点
			ExPolygons proj = mesh.horizontal_projection_slm(90.0);
			Pointf midPoint2 = TriangleMesh::get_ExMidPoint(proj);
			LOGINFO("get_ExMidPoint midPoint2 = [%s]", midPoint2.dump_perl().c_str());
			if (mesh.Cale_CollsionZ_byXY(Pointf(midPoint2.x, midPoint2.y), p, n) == false)
			{
				LOGINFO("get_ExMidPoint Cale_CollsionZ_byXY false!");
			}
		}

		// 计算旋转矩阵
		Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
		Eigen::Vector3d v_to(from.x, from.y, from.z);
		Eigen::Vector3d v_from(to.x, to.y, to.z);
		rotation_matrix = Eigen::Quaterniond::FromTwoVectors(v_from, v_to).toRotationMatrix();

		Eigen::Vector3d p_1(p.x, p.y, p.z);
		Eigen::Vector3d p_11 = rotation_matrix * p_1;

		Pointf3 result_p(p_11(0), p_11(1), p_11(2));

		LOGINFO("p = [%s], result_p = [%s]", p.dump_perl().c_str(), result_p.dump_perl().c_str());

		return result_p;
	}

	bool TriangleMesh::Make_SelectedFacets_Center_and_Normal()
	{
		LOGINFO("Make_SelectedFacets_Center_and_Normal start!!!");
		// 对选中面片进行操作
		std::vector<int> facets_index;
		//// 反向退化
		facets_index = this->Get_facets(Selected_Fact, true);
		this->reArea_FilterFacets(Selected_Fact, facets_index);
		// 正向退化
		facets_index = this->Get_facets(Selected_Fact, false);
		this->Area_FilterFacets(Selected_Fact, facets_index);

		if (facets_index.empty())
		{
			LOGINFO("Make_SelectedFacets_Center_and_Normal empty!!!");
			// 重置所有面片的Selected_Fact信息
			for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
			{
				stl_facet &facet = this->stl.facet_start[i];
				TriangleMesh::fact_remove_mark(facet.extra, Selected_Fact);
			}

			return false;
		}

		TriangleMesh selectedMesh = this->GetSubMesh(facets_index);
		// selectedMesh.write_ascii("selectedMesh.stl");
		this->SelectedNormal = selectedMesh.mesh_average_normal();
		this->SelectedCenter = selectedMesh.center_on_mesh();
		LOGINFO("this->SelectedNormal = [%s], this->SelectedCenter = [%s]", this->SelectedNormal.dump_perl().c_str(), this->SelectedCenter.dump_perl().c_str());

		LOGINFO("Make_SelectedFacets_Center_and_Normal end!!!");

		// 重置所有面片的Selected_Fact信息
		for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
		{
			stl_facet &facet = this->stl.facet_start[i];
			TriangleMesh::fact_remove_mark(facet.extra, Selected_Fact);
		}

		return true;
	}

	// angle -> rad，原mesh不动，返回拷贝副本
	TriangleMesh TriangleMesh::rotate(Vector3 from, Vector3 to, Pointf3 &p1, Pointf3 &p2)
	{
		if (from.normalized() == false)
			from.normalize();
		if (to.normalized() == false)
			to.normalize();

		// 计算旋转矩阵
		Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
		Eigen::Vector3d v_from(from.x, from.y, from.z);
		Eigen::Vector3d v_to(to.x, to.y, to.z);
		rotation_matrix = Eigen::Quaterniond::FromTwoVectors(v_from, v_to).toRotationMatrix();

		Eigen::Vector3d p11(p1.x, p1.y, p1.z);
		Eigen::Vector3d p21(p2.x, p2.y, p2.z);

		Eigen::Vector3d p12 = rotation_matrix * p11;
		Eigen::Vector3d p22 = rotation_matrix * p21;

		// LOGINFO("before rotate p1 = [%s], p2 = [%s]", p1.dump_perl().c_str(), p2.dump_perl().c_str());
		p1.x = p12(0);
		p1.y = p12(1);
		p1.z = p12(2);
		p2.x = p22(0);
		p2.y = p22(1);
		p2.z = p22(2);

		// LOGINFO("after rotate p1 = [%s], p2 = [%s]", p1.dump_perl().c_str(), p2.dump_perl().c_str());

		return rotate(rotation_matrix);
	}

	// 旋转 按轴axis
	void TriangleMesh::rotate(float angle, const Axis &axis)
	{
		// admesh uses degrees
		angle = Slic3r::Geometry::rad2deg(angle);

		if (axis == X)
		{
			stl_rotate_x(&(this->stl), angle);
		}
		else if (axis == Y)
		{
			stl_rotate_y(&(this->stl), angle);
		}
		else if (axis == Z)
		{
			stl_rotate_z(&(this->stl), angle);
		}
		stl_invalidate_shared_vertices(&this->stl);
	}

	void TriangleMesh::rotate_x(float angle)
	{
		this->rotate(angle, X);
	}

	void TriangleMesh::rotate_y(float angle)
	{
		this->rotate(angle, Y);
	}

	void TriangleMesh::rotate_z(float angle)
	{
		this->rotate(angle, Z);
	}

	void TriangleMesh::rotate_self(Eigen::Matrix3d rotation_matrix)
	{
		// 执行坐标旋转
		for (int i = 0; i < this->stl.stats.number_of_facets; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				Eigen::Vector3d ver(
					this->stl.facet_start[i].vertex[j].x,
					this->stl.facet_start[i].vertex[j].y,
					this->stl.facet_start[i].vertex[j].z);

				Eigen::Vector3d ver1 = rotation_matrix * ver;
				this->stl.facet_start[i].vertex[j].x = ver1(0);
				this->stl.facet_start[i].vertex[j].y = ver1(1);
				this->stl.facet_start[i].vertex[j].z = ver1(2);
			}
		}

		stl_get_size(&this->stl);
		calculate_normals(&this->stl);
		stl_invalidate_shared_vertices(&this->stl);
	}

	// 镜像
	void TriangleMesh::mirror(const Axis &axis)
	{
		if (axis == X)
		{
			stl_mirror_yz(&this->stl);
		}
		else if (axis == Y)
		{
			stl_mirror_xz(&this->stl);
		}
		else if (axis == Z)
		{
			stl_mirror_xy(&this->stl);
		}
		stl_invalidate_shared_vertices(&this->stl);
	}

	void TriangleMesh::mirror_x()
	{
		this->mirror(X);
	}

	void TriangleMesh::mirror_y()
	{
		this->mirror(Y);
	}

	void TriangleMesh::mirror_z()
	{
		this->mirror(Z);
	}

	// 将模型最小点返回原点
	void TriangleMesh::align_to_origin()
	{
		this->translate(
			-(this->stl.stats.min.x),
			-(this->stl.stats.min.y),
			-(this->stl.stats.min.z));
	}

	// 将模型的中点返回原点
	void TriangleMesh::center_around_origin()
	{
		this->align_to_origin();
		this->translate(
			-(this->stl.stats.size.x / 2),
			-(this->stl.stats.size.y / 2),
			-(this->stl.stats.size.z / 2));
	}

	// 绕某个点旋转
	void TriangleMesh::rotate(double angle, Point *center)
	{
		this->translate(-center->x, -center->y, 0);

		stl_rotate_z(&(this->stl), (float)angle);
		this->translate(+center->x, +center->y, 0);
	}

	void TriangleMesh::rotate(double angle)
	{
		stl_rotate_z(&(this->stl), (float)angle);
	}

	TriangleMesh TriangleMesh::GetSubMesh(std::vector<int> facets)
	{
		TriangleMesh mesh;
		mesh.stl.stats.type = inmemory;
		mesh.stl.stats.number_of_facets = facets.size();
		mesh.stl.stats.original_num_facets = mesh.stl.stats.number_of_facets;
		stl_clear_error(&mesh.stl);
		stl_allocate(&mesh.stl);

		int first = 1;
		for (std::vector<int>::const_iterator facet = facets.begin(); facet != facets.end(); ++facet)
		{
			mesh.stl.facet_start[facet - facets.begin()] = this->stl.facet_start[*facet];
			stl_facet_stats(&mesh.stl, this->stl.facet_start[*facet], first);
			first = 0;
		}

		return mesh;
	}

	TriangleMesh TriangleMesh::GetSubMesh(char Mark)
	{
		std::vector<int> facets = this->Get_facets(Mark, false);
		return this->GetSubMesh(facets);
	}

	// 将stl中所有独立的模型分割成一个个独立的TriangleMesh对象
	TriangleMeshPtrs
	TriangleMesh::split() const
	{
		TriangleMeshPtrs meshes;
		std::set<int> seen_facets;

		// we need neighbors
		if (!this->repaired)
		{
			LOGINFO("split() requires repair()");
			return meshes;
		}

		// loop while we have remaining facets
		while (1)
		{
			// get the first facet
			std::queue<int> facet_queue;
			std::deque<int> facets;
			// 从stl中找出一片不在seen_facets中的面片，放入facet_queue中
			for (int facet_idx = 0; facet_idx < this->stl.stats.number_of_facets; facet_idx++)
			{
				if (seen_facets.find(facet_idx) == seen_facets.end())
				{
					// if facet was not seen put it into queue and start searching
					facet_queue.push(facet_idx);
					break;
				}
			}

			// while(1)的唯一出口 stl中所有的面片都包含在seen_facets中
			if (facet_queue.empty())
				break;

			// 找出所有与该面片接临的面片  放入facets中和seen_facets中
			while (!facet_queue.empty())
			{
				int facet_idx = facet_queue.front();
				facet_queue.pop();
				if (seen_facets.find(facet_idx) != seen_facets.end())
					continue;
				facets.push_back(facet_idx);
				for (int j = 0; j <= 2; j++)
				{
					if (this->stl.neighbors_start[facet_idx].neighbor[j] < this->stl.stats.number_of_facets)
					{
						facet_queue.push(this->stl.neighbors_start[facet_idx].neighbor[j]);
					}
					else
					{
						LOGINFO("facet_idx error [%d]  number_of_facets [%d]", facet_idx, this->stl.stats.number_of_facets);
					}
				}
				seen_facets.insert(facet_idx);
			}

			// 将facets中所有的面片构建成一个独立的mesh，放入meshes中
			TriangleMesh *mesh = new TriangleMesh;
			meshes.push_back(mesh);
			mesh->stl.stats.type = inmemory;
			mesh->stl.stats.number_of_facets = facets.size();
			mesh->stl.stats.original_num_facets = mesh->stl.stats.number_of_facets;
			stl_clear_error(&mesh->stl);
			stl_allocate(&mesh->stl);

			int first = 1;
			for (std::deque<int>::const_iterator facet = facets.begin(); facet != facets.end(); ++facet)
			{
				stl_facet one_facet = this->stl.facet_start[*facet];
				mesh->stl.facet_start[facet - facets.begin()] = one_facet;
				stl_facet_stats(&mesh->stl, one_facet, first);
				first = 0;
			}
		}

		return meshes;
	}

	bool TriangleMesh::MergeCorrespondingMesh(TriangleMesh *stl_meshreverse)
	{

		int after_stl_facets_num = this->stl.stats.number_of_facets;
		stl_check_facets_exact(&this->stl);
		// 将下面的stl_meshreverse 进行法向翻转。
		stl_reverse_all_facets(&(stl_meshreverse->stl)); //
		stl_check_facets_exact(&stl_meshreverse->stl);
		// 缝合,插入侧面面片
		int stl_neighbor_num = 0;
		for (int i = 0; i < after_stl_facets_num; i++)
		{
			stl_facet new_facet;
			for (int j = 0; j < 3; j++)
			{
				if ((this->stl.neighbors_start[i].neighbor[j] == -1) &&
					(this->stl.neighbors_start[i].neighbor[0] + this->stl.neighbors_start[i].neighbor[1] + this->stl.neighbors_start[i].neighbor[2] != -3)) // 判断是否为边缘面片
				{
					stl_neighbor_num++;
					stl_facet new_facet;
					if (j == 0)
					{
						// 不需要设置法向量，在stl_add_facet()里面会重置为0
						new_facet.vertex[0] = this->stl.facet_start[i].vertex[0];
						new_facet.vertex[1] = stl_meshreverse->stl.facet_start[i].vertex[1];
						new_facet.vertex[2] = this->stl.facet_start[i].vertex[1];
						stl_add_facet(&(this->stl), &new_facet);
						new_facet.vertex[0] = this->stl.facet_start[i].vertex[1];
						new_facet.vertex[1] = stl_meshreverse->stl.facet_start[i].vertex[1];
						new_facet.vertex[2] = stl_meshreverse->stl.facet_start[i].vertex[0];
						stl_add_facet(&(this->stl), &new_facet);
					}
					else if (j == 1)
					{
						new_facet.vertex[0] = this->stl.facet_start[i].vertex[1];
						new_facet.vertex[1] = stl_meshreverse->stl.facet_start[i].vertex[0];
						new_facet.vertex[2] = this->stl.facet_start[i].vertex[2];
						stl_add_facet(&(this->stl), &new_facet);
						new_facet.vertex[0] = this->stl.facet_start[i].vertex[2];
						new_facet.vertex[1] = stl_meshreverse->stl.facet_start[i].vertex[0];
						new_facet.vertex[2] = stl_meshreverse->stl.facet_start[i].vertex[2];
						stl_add_facet(&(this->stl), &new_facet);
					}
					else
					{
						new_facet.vertex[0] = this->stl.facet_start[i].vertex[2];
						new_facet.vertex[1] = stl_meshreverse->stl.facet_start[i].vertex[2];
						new_facet.vertex[2] = this->stl.facet_start[i].vertex[0];
						stl_add_facet(&(this->stl), &new_facet);
						new_facet.vertex[0] = this->stl.facet_start[i].vertex[0];
						new_facet.vertex[1] = stl_meshreverse->stl.facet_start[i].vertex[2];
						new_facet.vertex[2] = stl_meshreverse->stl.facet_start[i].vertex[1];
						stl_add_facet(&(this->stl), &new_facet);
					}
				}
			}
		}
		this->merge(*stl_meshreverse);
		// if (this->stl.stats.volume < 0)
		// {
		// 	this->stl.stats.volume = -1; // 为了让后续重新计算墙支撑的体积
		// 	this->volume();
		// 	stl_reverse_all_facets(&this->stl);
		// }
		this->checkonly == false;
		this->repaired = false;
		this->repair();
		LOGINFO("After the all the insert ,the stl_neighbor_num is %d", stl_neighbor_num);
		LOGINFO("After the all the insert ,the stl_num is %d", this->stl.stats.number_of_facets);
		return true;
	}
	// 按照网格切割
	TriangleMeshPtrs
	TriangleMesh::cut_by_grid(const Pointf &grid) const
	{
		TriangleMesh mesh = *this;
		const BoundingBoxf3 bb = mesh.bounding_box();
		const Sizef3 size = bb.size();
		const size_t x_parts = ceil((size.x - EPSILON) / grid.x);
		const size_t y_parts = ceil((size.y - EPSILON) / grid.y);

		TriangleMeshPtrs meshes;
		for (size_t i = 1; i <= x_parts; ++i)
		{
			TriangleMesh curr;
			if (i == x_parts)
			{
				curr = mesh;
			}
			else
			{
				TriangleMesh next;
				TriangleMeshSlicer<X>(&mesh).cut(bb.min.x + (grid.x * i), &next, &curr);
				curr.repair();
				next.repair();
				mesh = next;
			}

			for (size_t j = 1; j <= y_parts; ++j)
			{
				TriangleMesh *tile;
				if (j == y_parts)
				{
					tile = new TriangleMesh(curr);
				}
				else
				{
					TriangleMesh next;
					tile = new TriangleMesh;
					TriangleMeshSlicer<Y>(&curr).cut(bb.min.y + (grid.y * j), &next, tile);
					tile->repair();
					next.repair();
					curr = next;
				}

				meshes.push_back(tile);
			}
		}
		return meshes;
	}

	// 合并另一个mesh里面的内容到本mesh
	void
	TriangleMesh::merge(const TriangleMesh &mesh)
	{
		if (mesh.facets_count() == 0)
			return;

		// reset stats and metadata
		// 重置统计数据和元数据
		int number_of_facets = this->facets_count();
		stl_invalidate_shared_vertices(&this->stl);
		this->repaired = false;
		// update facet count and allocate more memory
		// 更新facet计数并分配更多内存
		this->stl.stats.number_of_facets = number_of_facets + mesh.stl.stats.number_of_facets;
		this->stl.stats.original_num_facets = this->stl.stats.number_of_facets;
		// 累加计数
		this->stl.stats.number_of_parts += mesh.stl.stats.number_of_parts;
		this->stl.stats.volume += mesh.stl.stats.volume;
		this->stl.stats.degenerate_facets += mesh.stl.stats.degenerate_facets;
		this->stl.stats.edges_fixed += mesh.stl.stats.edges_fixed;
		this->stl.stats.facets_removed += mesh.stl.stats.facets_removed;
		this->stl.stats.facets_added += mesh.stl.stats.facets_added;
		this->stl.stats.facets_reversed += mesh.stl.stats.facets_reversed;
		this->stl.stats.backwards_edges += mesh.stl.stats.backwards_edges;
		this->stl.stats.normals_fixed += mesh.stl.stats.normals_fixed;

		stl_reallocate(&this->stl);

		// copy facets
		// 拷贝面片
		std::copy(mesh.stl.facet_start, mesh.stl.facet_start + mesh.stl.stats.number_of_facets, this->stl.facet_start + number_of_facets);
		std::copy(mesh.stl.neighbors_start, mesh.stl.neighbors_start + mesh.stl.stats.number_of_facets, this->stl.neighbors_start + number_of_facets);

		// update size
		// 更新大小
		stl_get_size(&this->stl);

		// 合并支撑
		this->merge_SptPoints(mesh, "grid");
		this->merge_SptPoints(mesh, "boundary");
		this->merge_SptPoints(mesh, "stripe");
		this->merge_SptPoints(mesh, "tree");
		this->merge_SptPoints(mesh, "overhang");
		this->merge_SptPoints(mesh, "overhangline");
		this->merge_SptPoints(mesh, "nummark");
		this->merge_SptPoints(mesh, "selected");
	}

	Pointf3
	TriangleMesh::lowest_point()
	{
		coordf_t z = 1000000.0;
		const auto &stl = this->stl;
		Pointf3 res = Pointf3(stl.v_shared[0].x, stl.v_shared[0].y, stl.v_shared[0].z);
		for (size_t i = 0; i < stl.stats.shared_vertices; ++i)
		{
			if (stl.v_shared[i].z < z)
			{
				z = stl.v_shared[i].z;
				res = Pointf3(stl.v_shared[i].x, stl.v_shared[i].y, stl.v_shared[i].z);
			}
		}

		return res;
	}

	void TriangleMesh::merge_SptPoints(TriangleMesh mesh, std::string _type)
	{
		std::vector<SPT_pointf> *this_points_vex = this->get_points(_type);
		std::vector<SPT_pointf> *other_points_vex = mesh.get_points(_type);
		if (this_points_vex == NULL || other_points_vex == NULL)
		{
			LOGINFO("[%s]this_points_vex or other_points_vex  == NULL", _type.c_str());
			return;
		}

		for (int i = 0; i < other_points_vex->size(); i++)
		{
			SPT_pointf temp_spt = other_points_vex->at(i);
			MapCollsionInfo::iterator p_It = temp_spt.CollsionZ_map.begin();
			for (; p_It != temp_spt.CollsionZ_map.end(); p_It++)
			{
				p_It->second = p_It->second + (this->stl.stats.number_of_facets - mesh.stl.stats.number_of_parts);
			}
			// 临时修改
			temp_spt.treebasep = NULL;
			temp_spt.branch_vecp.clear();

			this_points_vex->push_back(temp_spt);
		}
	}

	// 获取模型的垂直投影面
	/* this will return scaled ExPolygons */
	ExPolygons
	TriangleMesh::horizontal_projection() const
	{
		Polygons pp;
		pp.reserve(this->stl.stats.number_of_facets);
		for (int i = 0; i < this->stl.stats.number_of_facets; i++)
		{
			stl_facet *facet = &this->stl.facet_start[i];
			if (acos(-facet->normal.z) * 180 < (90 * PI))
			{
				Polygon p;
				p.points.resize(3);
				p.points[0] = Point(facet->vertex[0].x / SCALING_FACTOR, facet->vertex[0].y / SCALING_FACTOR);
				p.points[1] = Point(facet->vertex[1].x / SCALING_FACTOR, facet->vertex[1].y / SCALING_FACTOR);
				p.points[2] = Point(facet->vertex[2].x / SCALING_FACTOR, facet->vertex[2].y / SCALING_FACTOR);
				p.make_counter_clockwise(); // do this after scaling, as winding order might change while doing that
				pp.push_back(p);
			}
		}

		// the offset factor was tuned using groovemount.stl
		// return union_ex(offset(pp, 0.01 / SCALING_FACTOR), true);
		return union_ex(pp, true);
	}

	ExPolygons
	TriangleMesh::Get_SelectFacets_Ex()
	{
		ExPolygons retval;
		Polygons pp;
		retval.clear();
		pp.clear();
		// 对选中面片进行操作
		std::vector<int> facets_index;
		// 反向退化
		facets_index = this->Get_facets(Selected_Fact, true);
		this->reArea_FilterFacets(Selected_Fact, facets_index);
		// 正向退化
		facets_index = this->Get_facets(Selected_Fact, false);
		this->Area_FilterFacets(Selected_Fact, facets_index);

		LOGINFO("Filter facets_index size %d", facets_index.size());
		BoundLines_vec Selected_boundLines_vec;
		this->General_BoundLoop_Line(Selected_Fact, Selected_boundLines_vec);
		// LOGINFO("Filter Selected_boundLines_vec size %d", Selected_boundLines_vec.size());
		//  成环处理
		while (Selected_boundLines_vec.size())
		{
			Polygon temp = this->find_oneLoop_polygon(Selected_boundLines_vec);
			if (temp.points.size() > 0)
			{
				pp.push_back(temp);
				LOGINFO("发现环 Polygon Points = [%d/%d]", temp.points.size(), Selected_boundLines_vec.size());
			}
		}
		// LOGINFO("pp.size = %d", pp.size());
		retval = union_ex(pp);

		return retval;
	}

	ExPolygons
	TriangleMesh::horizontal_projection_slm(double angle)
	{
		LOGINFO("TriangleMesh::horizontal_projection_slm start!");
		ExPolygons horizontal;
		Polygons pp;
		horizontal.clear();
		pp.clear();
		// 面片数大于一百万，直接取凸包
		if (/*this->stl.stats.number_of_facets > 1000000*/ false)
		{
			LOGINFO("this->stl.stats.number_of_facets > 1000000!");
			Polygon p = this->convex_hull();
			pp.push_back(p);
			LOGINFO("convex_hull succeed!");
		}
		else
		{
			this->Mark_FilterFacets(angle);
			BoundLines_vec boundLines_temp = this->boundLines_vec; // 边界线段集合拷贝副本
			LOGINFO("boundLines_temp.size = %d", boundLines_temp.size());
			if (boundLines_temp.size() == 0)
			{
				for (int i = 0; i < this->stl.stats.number_of_facets; ++i)
				{
					stl_facet &facet = this->stl.facet_start[i];
					if (TriangleMesh::fact_is_mark(facet.extra, Area_Fact))
					{
						Polygon p;
						p.points.resize(3);
						p.points[0] = Point(facet.vertex[0].x / SCALING_FACTOR, facet.vertex[0].y / SCALING_FACTOR);
						p.points[1] = Point(facet.vertex[1].x / SCALING_FACTOR, facet.vertex[1].y / SCALING_FACTOR);
						p.points[2] = Point(facet.vertex[2].x / SCALING_FACTOR, facet.vertex[2].y / SCALING_FACTOR);
						p.make_counter_clockwise(); // do this after scaling, as winding order might change while doing that
						pp.push_back(p);
					}
				}
			}
			else
			{
				// 成环处理
				while (boundLines_temp.size())
				{
					Polygon temp = this->find_oneLoop_polygon(boundLines_temp);
					if (temp.points.size() > 0)
					{
						pp.push_back(temp);
						// LOGINFO("发现环 Polygon Points = [%d/%d]", temp.points.size(), boundLines_temp.size());
					}
				}
			}
		}
		horizontal = union_ex(pp);
		// TriangleMesh::make_expolygons(pp, &horizontal, true);
		LOGINFO("pp.size = %d, horizontal.size = %d", pp.size(), horizontal.size());

		return horizontal;
	}

	BoundLoops_vec
	TriangleMesh::GetMeshBoundLoops(double angle)
	{
		BoundLoops_vec loops;
		loops.clear();
		this->Mark_FilterFacets(angle);
		BoundLines_vec boundLines_temp = this->boundLines_vec; // 边界线段集合拷贝副本
		LOGINFO("boundLines_temp.size = %d", boundLines_temp.size());
		if (boundLines_temp.size() != 0)
		{
			// 成环处理
			while (boundLines_temp.size())
			{
				BoundLoop temp_loop = this->find_oneLoop(boundLines_temp);
				if (temp_loop.size() > 0)
				{
					loops.push_back(temp_loop);
					LOGINFO("发现环 Polygon Points = [%d/%d]", temp_loop.size(), boundLines_temp.size());
				}
			}
		}
		LOGINFO("loops.size = %d", loops.size());

		return loops;
	}

	// 获取模型每个三角面在z轴上的投影点集形成的二维凸包集
	Polygon
	TriangleMesh::convex_hull()
	{
		if (this->facets_count() <= 3)
		{
			return Polygon();
		}
		this->require_shared_vertices();
		Points pp;
		if (this->stl.v_shared != NULL)
		{
			pp.reserve(this->stl.stats.shared_vertices);
			for (int i = 0; i < this->stl.stats.shared_vertices; i++)
			{
				stl_vertex *v = &this->stl.v_shared[i];
				pp.push_back(Point(v->x / SCALING_FACTOR, v->y / SCALING_FACTOR));
			}
		}
		else
		{
			pp.reserve(this->stl.stats.number_of_facets * 3);
			for (int i = 0; i < this->stl.stats.number_of_facets; i++)
			{
				stl_facet facet = this->stl.facet_start[i];
				for (int j = 0; j < 3; j++)
				{
					stl_vertex v = facet.vertex[j];
					pp.push_back(Point(v.x / SCALING_FACTOR, v.y / SCALING_FACTOR));
				}
			}
		}

		return Slic3r::Geometry::convex_hull(pp);
	}

	// 取包围盒
	BoundingBoxf3
	TriangleMesh::bounding_box() const
	{
		BoundingBoxf3 bb;
		bb.min.x = this->stl.stats.min.x;
		bb.min.y = this->stl.stats.min.y;
		bb.min.z = this->stl.stats.min.z;
		bb.max.x = this->stl.stats.max.x;
		bb.max.y = this->stl.stats.max.y;
		bb.max.z = this->stl.stats.max.z;
		/*
			LOGINFO("bounding_box  min [%f,%f,%f] max [%f,%f,%f]",
				this->stl.stats.min.x,
				this->stl.stats.min.y,
				this->stl.stats.min.z,
				this->stl.stats.max.x,
				this->stl.stats.max.y,
				this->stl.stats.max.z
				);
		*/
		return bb;
	}

	// 产生顶点副本
	void
	TriangleMesh::require_shared_vertices()
	{
		// LOGINFO("Begin");
		if (!this->repaired)
		{
			LOGINFO("require_shared_vertices  !this->repaired");

			this->repair();
		}
		if (this->stl.v_shared == NULL)
		{
			// LOGINFO("require_shared_vertices  this->stl.v_shared == NULL");
			stl_generate_shared_vertices(&(this->stl));
		}
		// LOGINFO("End");
	}

	// 将2.5d tin Mesh转成3d mesh
	void
	TriangleMesh::extrude_tin(float offset)
	{
		calculate_normals(&this->stl);

		const int number_of_facets = this->stl.stats.number_of_facets;
		if (number_of_facets == 0)
			throw std::runtime_error("Error: file is empty");

		const float z = this->stl.stats.min.z - offset;

		for (int i = 0; i < number_of_facets; ++i)
		{
			const stl_facet &facet = this->stl.facet_start[i];

			if (facet.normal.z < 0)
				throw std::runtime_error("Invalid 2.5D mesh: at least one facet points downwards.");

			for (int j = 0; j < 3; ++j)
			{
				// 没有邻接边的信息
				if (this->stl.neighbors_start[i].neighbor[j] == -1)
				{
					stl_facet new_facet;
					float normal[3];

					// first triangle
					new_facet.vertex[0] = new_facet.vertex[2] = facet.vertex[(j + 1) % 3];
					new_facet.vertex[1] = facet.vertex[j];
					new_facet.vertex[2].z = z;
					stl_calculate_normal(normal, &new_facet);
					stl_normalize_vector(normal);
					new_facet.normal.x = normal[0];
					new_facet.normal.y = normal[1];
					new_facet.normal.z = normal[2];
					stl_add_facet(&this->stl, &new_facet);

					// second triangle
					new_facet.vertex[0] = new_facet.vertex[1] = facet.vertex[j];
					new_facet.vertex[2] = facet.vertex[(j + 1) % 3];
					new_facet.vertex[1].z = new_facet.vertex[2].z = z;
					new_facet.normal.x = normal[0];
					new_facet.normal.y = normal[1];
					new_facet.normal.z = normal[2];
					stl_add_facet(&this->stl, &new_facet);
				}
			}
		}
		stl_get_size(&this->stl);

		this->repair();
	}

	std::set<XYZ_Index>
	TriangleMesh::build_mesh_voxels(double step)
	{
		std::set<XYZ_Index> mesh_voxels;
		for (int i = 0; i < this->facets_count(); ++i)
		{
			std::vector<XYZ_Index> grids = get_facet_grids(i, step);
			if (grids.empty() == false)
			{
				int count = grids.size();
				for (int j = 0; j < count; j++)
				{
					mesh_voxels.insert(grids[j]);
				}
			}
		}

		return mesh_voxels;
	}

	std::vector<int> TriangleMesh::get_Pointf3_gridfactes(Pointf3 _pointf3, size_t expand_ring)
	{
		std::vector<int> ret_vec;
		ret_vec.clear();
		int xmin = (int)std::floor(_pointf3.x / X_stepLen) - expand_ring;
		int xmax = (int)std::floor(_pointf3.x / X_stepLen) + expand_ring;
		int ymin = (int)std::floor(_pointf3.y / Y_stepLen) - expand_ring;
		int ymax = (int)std::floor(_pointf3.y / Y_stepLen) + expand_ring;
		int zmin = (int)std::floor(_pointf3.z / Z_stepLen) - expand_ring;
		int zmax = (int)std::floor(_pointf3.z / Z_stepLen) + expand_ring;
		// LOGINFO("1111 get_Pointf3_gridfactes  _pointf3[%s] expand_ring [%d] ", _pointf3.dump_perl().c_str(), expand_ring);
		// LOGINFO("2222 get_Pointf3_gridfactes  _pointf3[%s]  [%d, %d],[%d, %d],[%d, %d],",
		//	_pointf3.dump_perl().c_str(),
		//	xmin,
		//	xmax,
		//	ymin,
		//	ymax,
		//	zmin,
		//	zmax
		//);
		for (int i = xmin; i <= xmax; i++)
		{
			for (int j = ymin; j <= ymax; j++)
			{
				for (int k = zmin; k <= zmax; k++)
				{
					XYZ_Index pos;
					pos.X_id = i;
					pos.Y_id = j;
					pos.Z_id = k;
					if (this->hash_facets.find(pos) != this->hash_facets.end())
					{
						ret_vec.insert(ret_vec.end(), this->hash_facets[pos].begin(), this->hash_facets[pos].end());
					}
				}
			}
		}
		// LOGINFO("3333 get_Pointf3_gridfactes  _pointf3[%s]", _pointf3.dump_perl().c_str());
		// 去重
		sort(ret_vec.begin(), ret_vec.end());
		ret_vec.erase(unique(ret_vec.begin(), ret_vec.end()), ret_vec.end());
		// LOGINFO("4444 get_Pointf3_gridfactes  _pointf3[%s]  ret_vec.size = %d,", _pointf3.dump_perl().c_str(), ret_vec.size());
		return ret_vec;
	}

	std::vector<XYZ_Index> TriangleMesh::get_box_grids(BoundingBoxf3 box, double step)
	{
		std::vector<XYZ_Index> grids;

		Pointf3 min = box.min;
		Pointf3 max = box.max;

		double x_stepLen = X_stepLen;
		double y_stepLen = Y_stepLen;
		double z_stepLen = Z_stepLen;
		// LOGINFO("x_stepLen = [%f],y_stepLen = [%f],z_stepLen = [%f]", x_stepLen, y_stepLen, z_stepLen);

		if (step != 0)
		{
			x_stepLen = step;
			y_stepLen = step;
			z_stepLen = step;
			// LOGINFO("step != 0, step = [%f]", step);
		}

		int xmin = floor(min.x / x_stepLen);
		int xmax = ceil(max.x / x_stepLen);
		int ymin = floor(min.y / y_stepLen);
		int ymax = ceil(max.y / y_stepLen);
		int zmin = floor(min.z / z_stepLen);
		int zmax = ceil(max.z / z_stepLen);

		for (int i = xmin; i < xmax; i++)
		{
			for (int j = ymin; j < ymax; j++)
			{
				for (int k = zmin; k < zmax; k++)
				{
					XYZ_Index pos;
					pos.X_id = i;
					pos.Y_id = j;
					pos.Z_id = k;
					grids.push_back(pos);
				}
			}
		}

		return grids;
	}

	bool
	TriangleMesh::facet_is_in_XYZ_Index(TriangleFace3 facet, XYZ_Index index, double step)
	{
		if (X_stepLen != Y_stepLen || Y_stepLen != Z_stepLen || Z_stepLen != X_stepLen)
			return false;

		double len = X_stepLen;
		if (step != 0.0)
			len = step;

		// 将cube和facet的中心 移动到原点
		double x_translate = -((double)index.X_id + 0.5) * len;
		double y_translate = -((double)index.Y_id + 0.5) * len;
		double z_translate = -((double)index.Z_id + 0.5) * len;

		// LOGINFO("X_id = [%d],Y_id = [%d],Z_id = [%d]", index.X_id, index.Y_id, index.Z_id);
		// LOGINFO("x_translate = [%f],y_translate = [%f],z_translate = [%f]", x_translate, y_translate, z_translate);

		// LOGINFO("before facet.p1.x = [%f],facet.p1.y = [%f],facet.p1.z = [%f]", facet.p1.x, facet.p1.y, facet.p1.z);
		facet.translate(x_translate, y_translate, z_translate);
		// LOGINFO("after facet.p1.x = [%f],facet.p1.y = [%f],facet.p1.z = [%f]", facet.p1.x, facet.p1.y, facet.p1.z);
		bool result = facet.t_c_intersection(len) == 0;

		// if (result)
		//	LOGINFO("facet is in XYZ_Index");
		return result;
	}

	std::vector<XYZ_Index>
	TriangleMesh::get_facet_grids(int tid, double step)
	{
		const stl_facet &facet = this->stl.facet_start[tid];
		stl_vertex pA = facet.vertex[0];
		stl_vertex pB = facet.vertex[1];
		stl_vertex pC = facet.vertex[2];

		Pointf3s points;
		for (int j = 0; j < 3; ++j)
		{
			points.push_back(Pointf3(facet.vertex[j].x, facet.vertex[j].y, facet.vertex[j].z));
		}

		Vector3 t1 = Vector3(pA.x, pA.y, pA.z);
		Vector3 t2 = Vector3(pB.x, pB.y, pB.z);
		Vector3 t3 = Vector3(pC.x, pC.y, pC.z);
		TriangleFace3 temp_face = TriangleFace3(t1, t2, t3);

		BoundingBoxf3 box(points);
		std::vector<XYZ_Index> bgs = this->get_box_grids(box, step);

		std::vector<XYZ_Index> bgs_in;
		for (int i = 0; i < bgs.size(); i++)
		{
			XYZ_Index index = bgs.at(i);
			if (facet_is_in_XYZ_Index(temp_face, index, step))
				bgs_in.push_back(index);
		}

		// if(bgs.size() != bgs_in.size())
		//	LOGINFO("bgs = [%d], bgs_in = [%d]", bgs.size(), bgs_in.size());

		return step == 0.0 ? bgs : bgs_in; // bgs;
	}

	std::vector<XYZ_Index> TriangleMesh::get_line_grids(Linef3 line)
	{
		Pointf3s points;
		points.push_back(line.a);
		points.push_back(line.b);

		BoundingBoxf3 box(points);
		return get_box_grids(box);
	}

	std::vector<XYZ_Index> TriangleMesh::get_zline_grids(double x, double y)
	{
		double zmax = this->bounding_box().max.z;
		double zmin = this->bounding_box().min.z;

		return get_line_grids(Linef3(Pointf3(x, y, zmax), Pointf3(x, y, zmin)));
	}

	// 建立hash抽屉
	void
	TriangleMesh::init_hash_facets()
	{
		DWORD timecount = GetTickCount();
		this->hash_facets.clear();
		for (int i = 0; i < this->facets_count(); ++i)
		{
			std::vector<XYZ_Index> grids = get_facet_grids(i);
			if (grids.empty() == false)
			{
				int count = grids.size();
				for (int j = 0; j < count; j++)
				{
					hash_facets[grids[j]].push_back(i);
				}
			}
		}

		LOGINFO("init_hash_facets times =%d", GetTickCount() - timecount);
	}

	void
	TriangleMesh::translate_hash_facets(int x, int y, int z) // 平移hash抽屉
	{
		// DWORD timecount = GetTickCount();
		// LOGINFO("translate_hash_facets x =%d, y =%d, z =%d", x, y, z);

		std::map<XYZ_Index, std::vector<int>> temp = this->hash_facets;
		this->hash_facets.clear();

		if (this->hash_facets.empty())
			return;

		for (std::map<XYZ_Index, std::vector<int>>::const_iterator iterA = temp.begin(); iterA != temp.end(); iterA++)
		{
			XYZ_Index new_index;
			new_index.X_id = iterA->first.X_id + x;
			new_index.Y_id = iterA->first.Y_id + y;
			new_index.Z_id = iterA->first.Z_id + z;
			this->hash_facets[new_index] = iterA->second;
		}
		// LOGINFO("translate_hash_facets times =%d", GetTickCount() - timecount);
	}

	// 平移hash抽屉
	std::map<XYZ_Index, std::vector<int>>
	TriangleMesh::translate_hash_facets_new(int x, int y, int z)
	{
		// DWORD timecount = GetTickCount();
		// LOGINFO("translate_hash_facets x =%d, y =%d, z =%d", x,y,z);
		std::map<XYZ_Index, std::vector<int>> temp;
		temp.clear();

		if (this->hash_facets.empty())
			return temp;

		for (std::map<XYZ_Index, std::vector<int>>::const_iterator iterA = this->hash_facets.begin(); iterA != this->hash_facets.end(); iterA++)
		{
			XYZ_Index new_index;
			new_index.X_id = iterA->first.X_id + x;
			new_index.Y_id = iterA->first.Y_id + y;
			new_index.Z_id = iterA->first.Z_id + z;
			temp[new_index] = iterA->second;
		}
		// LOGINFO("translate_hash_facets times =%d", GetTickCount() - timecount);

		return temp;
	}

	std::vector<int> TriangleMesh::get_grids_facets(std::vector<XYZ_Index> grid_ids)
	{
		std::vector<int> facets;
		for (std::vector<XYZ_Index>::iterator it = grid_ids.begin(); it != grid_ids.end(); it++)
		{
			facets.insert(facets.end(), hash_facets[*it].begin(), hash_facets[*it].end());
		}

		// 去重
		sort(facets.begin(), facets.end());
		facets.erase(unique(facets.begin(), facets.end()), facets.end());

		return facets;
	}

	std::vector<int> TriangleMesh::GetFacetIdxByZaxisFast(double x, double y)
	{
		// LOGINFO("GetFacetIdxByZaxisFast, hash_facets.size() = [%d]", hash_facets.size());

		std::vector<XYZ_Index> grids = get_zline_grids(x, y);
		std::vector<int> facets = get_grids_facets(grids);
		// LOGINFO("TriangleMesh::GetFacetIdxByZaxisFast(double x, double y), facets = [%d]", facets.size());

		std::vector<int> reval;
		reval.clear();
		stl_vertex Zaixs;
		Zaixs.x = x;
		Zaixs.y = y;
		Zaixs.z = 0;

		for (std::vector<int>::iterator it = facets.begin(); it < facets.end(); it++)
		{
			if (this->Pre_PointinTriangle(*it, Zaixs, Ray_type::axis_Z))
				if (this->PointinTriangle(*it, Zaixs, Ray_type::axis_Z))
					reval.push_back(*it);
		}
		// LOGINFO("TriangleMesh::GetFacetIdxByZaxisFast(double x, double y), reval = [%d]", reval.size());

		return reval;
	}

	std::vector<int> TriangleMesh::GetFacetIdxByZaxisFast(Pointf3 p1, Pointf3 p2)
	{
		std::vector<XYZ_Index> grids = get_line_grids(Linef3(p1, p2));
		std::vector<int> facets = get_grids_facets(grids);
		// LOGINFO("TriangleMesh::GetFacetIdxByZaxisFast(Pointf3 p1, Pointf3 p2), facets = [%d]", facets.size());

		std::vector<int> reval;
		reval.clear();
		stl_vertex Zaixs;
		Zaixs.x = p1.x;
		Zaixs.y = p1.y;
		Zaixs.z = 0;

		for (std::vector<int>::iterator it = facets.begin(); it < facets.end(); it++)
		{
			if (this->Pre_PointinTriangle(*it, Zaixs, Ray_type::axis_Z))
				if (this->PointinTriangle(*it, Zaixs, Ray_type::axis_Z))
					reval.push_back(*it);
		}
		// LOGINFO("TriangleMesh::GetFacetIdxByZaxisFast(Pointf3 p1, Pointf3 p2), reval = [%d]", reval.size());

		return reval;
	}

	bool TriangleMesh::
		InteractionWithOtherTriangleMesh(TriangleMesh &otherMesh, double dis)
	{
		LOGINFO("start InteractionWithOtherTriangleMesh");
		DWORD timecount = GetTickCount();

		// 建立各自的哈希
		if (this->hash_facets.empty())
		{
			timecount = GetTickCount();
			this->set_step_length(dis, dis, dis);
			this->init_hash_facets();
			LOGINFO("thisMesh init_hash_facets times =%d", GetTickCount() - timecount);
			timecount = GetTickCount();
		}

		if (otherMesh.hash_facets.empty())
		{
			timecount = GetTickCount();
			otherMesh.set_step_length(dis, dis, dis);
			otherMesh.init_hash_facets();
			LOGINFO("otherMesh init_hash_facets times =%d", GetTickCount() - timecount);
			timecount = GetTickCount();
		}

		LOGINFO("thisMesh.size() = [%d]", this->hash_facets.size());
		LOGINFO("otherMesh.size() = [%d]", otherMesh.hash_facets.size());

		for (std::map<XYZ_Index, std::vector<int>>::const_iterator iterA = this->hash_facets.begin(); iterA != this->hash_facets.end(); iterA++)
		{
			std::map<XYZ_Index, std::vector<int>>::iterator itrB;
			itrB = otherMesh.hash_facets.find(iterA->first);
			if (itrB != otherMesh.hash_facets.end())
			{
				LOGINFO("get result times =%d", GetTickCount() - timecount);
				LOGINFO("end InteractionWithOtherTriangleMesh , find a Interaction!!!");
				return true;
			}
			/*
					for (std::map<XYZ_Index, std::vector<int>>::const_iterator iterB = otherMesh.hash_facets.begin(); iterB != otherMesh.hash_facets.end(); iterB++)
					{
						//LOGINFO("iterA->first = [%d], iterB->first = [%d]", iterA->first.union_id, iterB->first.union_id);
						if (iterA->first.X_id == iterB->first.X_id &&
							iterA->first.Y_id == iterB->first.Y_id &&
							iterA->first.Z_id == iterB->first.Z_id)
						{
							LOGINFO("find a iterA->first == iterB->first");
							LOGINFO("iterA->first.X_id = [%d], iterA->first.Y_id = [%d], iterA->first.Z_id = [%d]", iterA->first.X_id, iterA->first.Y_id, iterA->first.Z_id);
							LOGINFO("iterB->first.X_id = [%d], iterB->first.Y_id = [%d], iterB->first.Z_id = [%d]", iterB->first.X_id, iterB->first.Y_id, iterB->first.Z_id);
							LOGINFO("iterA->second.size = [%d], iterB->second.size = [%d]", iterA->second.size(), iterB->second.size());

							LOGINFO("get result times =%d", GetTickCount() - timecount);
							LOGINFO("end InteractionWithOtherTriangleMesh , find a Interaction!!!");

							return true;
						}
					}
			*/
		}

		LOGINFO("get result times =%d", GetTickCount() - timecount);
		LOGINFO("end InteractionWithOtherTriangleMesh , find a Interaction???");
		return false;
	}
	// 种植抽壳

	bool TriangleMesh::triFilterF(size_t tid, size_t ner_tid)
	{
		size_t min = (((tid) < (ner_tid)) ? (tid) : (ner_tid));
		size_t max = (((tid) > (ner_tid)) ? (tid) : (ner_tid));
		double ang = edge_normal[std::make_pair(min, max)];
		return ang <= 45.1;
	};
	void TriangleMesh::draw_Plane(size_t select_idx,const std::set<int>& draw_Plane)
	{
		LOGINFO("select_num ");
		size_t start_tid = select_idx;
		stl_file& stl = this->stl;
		edge_normal.clear();
		for (int tid = 0; tid < stl.stats.number_of_facets; tid++)
		{
			int* ner_tri = stl.neighbors_start[tid].neighbor;
			for (int ner_tid = 0; ner_tid < 3; ner_tid++)
			{
				size_t min = (((tid) < (ner_tri[ner_tid])) ? (tid) : (ner_tri[ner_tid]));
				size_t max = (((tid) > (ner_tri[ner_tid])) ? (tid) : (ner_tri[ner_tid]));
				Vector3 nora(stl.facet_start[min].normal.x, stl.facet_start[min].normal.y, stl.facet_start[min].normal.z);
				Vector3 norb(stl.facet_start[max].normal.x, stl.facet_start[max].normal.y, stl.facet_start[max].normal.z);
				double cos = nora.Dot(norb);
				if (std::abs(cos) > 1)
				{
					//cos /= std::abs(cos);
					//cos *= 0.97;
					cos = 1;
				}
				else if(std::abs(cos) < -1)
				{
					cos = -1;
				}
				double angle = std::acos(cos) / 3.1415926535 * 180;
				edge_normal[std::make_pair(min, max)] = angle;
			}
		}
		this->FloodFill(select_idx,draw_Plane);

		LOGINFO("select_num %d", Selected_Triangles.size());

	}
	void TriangleMesh::FloodFill(size_t tSeed,const std::set<int>& LimitedFaces)
	{
	    std::set<int>::iterator iterLimitedFaces;
		Selected_Triangles.clear();
		std::vector<size_t> fill_stack;
		fill_stack.push_back(tSeed);
		Selected_Triangles.insert(tSeed);
		while (!fill_stack.empty())
		{
			size_t tid = fill_stack.back();
			fill_stack.pop_back();
			int* ner = stl.neighbors_start[tid].neighbor;
            for (size_t nid = 0; nid < 3; nid++)
            {
                size_t nbr_tid = ner[nid];
                iterLimitedFaces = LimitedFaces.find(nbr_tid);
                if(iterLimitedFaces == LimitedFaces.end())
                {
                    if (IsSelected(nbr_tid))
                        continue;
                    if (!triFilterF(tid, nbr_tid))
                        continue;
                    Selected_Triangles.insert(nbr_tid);
                    fill_stack.push_back(nbr_tid);
                }
            }
		}
	}
	void TriangleMesh::convex_Hull()
	{}
}