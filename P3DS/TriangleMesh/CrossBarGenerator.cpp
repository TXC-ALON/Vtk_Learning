#include "TriangleMesh.hpp"
#include <float.h>
#include <assert.h>
#include "SVG.hpp"

namespace Slic3r {

	Special_Cylinder::Special_Cylinder(std::string _filepath)
	{
		if (_access(_filepath.c_str(), 0) == -1)
		{
			LOGINFO("[%s] 文件不存在", _filepath.c_str());
			return;
		}
		cy_mesh.ClearMesh();
		// 从STL文件中读取TriangleMesh
		cy_mesh.ReadSTLFile(_filepath);
		// 检查拓扑结构
		cy_mesh.check_topology();
		if (cy_mesh.facets_count() == 0)
		{
			LOGINFO("[%s] 文件读stl失败", _filepath.c_str());
			return;
		}
		cy_mesh.repair();
		if (cy_mesh.repaired == false)
		{
			LOGINFO("[%s] 文件stl修复失败", _filepath.c_str());
			return;
		}
		FilePath = _filepath;
		BoundingBoxf3 _bb3 = cy_mesh.bounding_box();
		Pointf3 _size = _bb3.size();
		length = _size.z;
		length_round = round(length);
		raduis = (_size.x + _size.y) / 4;
		raduis_round = round(raduis);
	}

	void Special_Cylinder::Print_Info()
	{
		LOGINFO("Special_Cylinder FilePath [%s], Mesh Faces[%d], raduis[%f], length[%f]",
			FilePath.c_str(), cy_mesh.facets_count(), raduis, length);
	}

	CrossBarGenerator::CrossBarGenerator(Pointf3 start_point, double radius, double contact_width, double contact_length, bool Is_Special)
	{
		this->start_point = start_point;
		this->radius = radius;
		this->start_ball_radius = contact_width / 2;
		this->contact_width = contact_width;
		this->contact_length = contact_length;
		this->Is_Special = Is_Special;
	}

	// 特殊结构
	TriangleMesh 
	CrossBarGenerator::Generate_SpecialBarMesh(Pointf3 start_p, Pointf3 end_p, double raduis)
	{
		TriangleMesh bar_mesh;
		// 
		if (start_p.z > end_p.z)
		{
			Pointf3 temp = start_p;
			start_p = end_p;
			end_p = temp;
		}
		// 获取高度信息
		double Bar_Height = end_p.z - start_p.z;
		if (Bar_Height <= 0.05)
		{
			LOGINFO("Bar_Height[%f] too small, start_p[%s],end_p[%s]", 
				Bar_Height, start_p.dump_perl().c_str(), end_p.dump_perl().c_str());
			return bar_mesh;
		}
		int Bar_Height_Int = round(Bar_Height);
		if (Bar_Height_Int < 1)
			Bar_Height_Int = 1;
		else if (Bar_Height_Int > 100)
			Bar_Height_Int = 100;
		else if (Bar_Height_Int > 10)
		{
			Bar_Height_Int = round(Bar_Height / 10);
			Bar_Height_Int *= 10;
		}
		
		LOGINFO("start_p[%s], end_p[%s], raduis[%f], Bar_Height[%f], Bar_HeightCeil[%d]",
			start_p.dump_perl().c_str(), 
			end_p.dump_perl().c_str(), 
			raduis,
			Bar_Height,
			Bar_Height_Int
		);

		// 找到最匹配的mesh
		std::map<int, Special_Cylinder*>::iterator map_It = SpCy_Meshs.find(Bar_Height_Int);
		if (map_It == SpCy_Meshs.end())
		{
			LOGINFO("Can not find Bar_HeightCeil[%d] in the SpCy_Meshs size[%d]", Bar_Height_Int, SpCy_Meshs.size());
			return bar_mesh;
		}
		Special_Cylinder* Match_Mesh_p = map_It->second;
		if (Match_Mesh_p == nullptr)
		{
			LOGINFO("Bar_HeightCeil[%d] in the SpCy_Meshs[%d] is  nullptr", Bar_Height_Int, SpCy_Meshs.size());
			return bar_mesh;
		}
		Match_Mesh_p->Print_Info();
		// 变形Mesh
		bar_mesh = Match_Mesh_p->cy_mesh;
		Pointf3 scale_point(
			raduis / Match_Mesh_p->raduis, 
			raduis / Match_Mesh_p->raduis, 
			Bar_Height / Match_Mesh_p->length);
		bar_mesh.scale(scale_point);
		bar_mesh.translate(start_p.x, start_p.y, start_p.z);
		return bar_mesh;
	}

	// 实心圆柱
	TriangleMesh
	CrossBarGenerator::Generate_CylinderBarMesh(Pointf3 start_p, Pointf3 end_p, double raduis)
	{
		//LOGINFO("start_p[%s],end_p[%s] raduis[%f]",
		//	 start_p.dump_perl().c_str(), end_p.dump_perl().c_str(), raduis);
		SingleSPT_BaseMesh* cross_bar = nullptr;
		if (this->contact_length * 2 + 0.1 >= start_p.distance_to(end_p))
		{
			cross_bar = new Cylinder_Mesh(start_p, end_p, 0, this->contact_width / 2, 18);
		}
		else
		{
			cross_bar = new Cylinder_Mesh(start_p, end_p, 0, this->radius, 18);
			cross_bar->Add_Contact(true, this->contact_width, this->contact_width, this->contact_length);
			cross_bar->Add_Contact(false, this->contact_width, this->contact_width, this->contact_length);
		}
		TriangleMesh bar_mesh = cross_bar->GeneralMesh();
		if (cross_bar != nullptr)
		{
			delete cross_bar;
			cross_bar = nullptr;
		}
		return bar_mesh;
	}

	// 星型面片
	TriangleMesh 
	CrossBarGenerator::Generate_CrossfaceBarMesh(Pointf3 start_p, Pointf3 end_p, double raduis)
	{
		TriangleMesh BarMesh;
		const int Facets_count = 4;
		for (int i = 0; i < Facets_count; i++) {
			SingleSPT_BaseMesh* cross_bar = nullptr;
			cross_bar = new SliceSingle_Mesh(start_p, end_p, PI / Facets_count * i, radius * 2, SliceType::sTypeMiddle);
			BarMesh.merge(cross_bar->GeneralMesh());
			delete cross_bar;
		}
		return BarMesh;
	}

	// 生成接口
	TriangleMesh CrossBarGenerator::Generate(Pointf3 end_point)
	{
		// 横杆的真实首尾端点
		this->end_point = end_point;

		Pointf3 p = this->end_point - this->start_point;

		Vector3 from(p.x, p.y, p.z);
		Vector3 to(0, 0, 1);

		//计算旋转矩阵
		Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
		Eigen::Vector3d v_from(from.x, from.y, from.z);
		Eigen::Vector3d v_to(to.x, to.y, to.z);
		rotation_matrix = Eigen::Quaterniond::FromTwoVectors(v_from, v_to).toRotationMatrix();

		Eigen::Vector3d p11(this->start_point.x, this->start_point.y, this->start_point.z);
		Eigen::Vector3d p21(this->end_point.x, this->end_point.y, this->end_point.z);

		Eigen::Vector3d p12 = rotation_matrix * p11;
		Eigen::Vector3d p22 = rotation_matrix * p21;
		// 竖直化的横杆首尾端点
		Pointf3 start_p(p12(0), p12(1), p12(2));
		Pointf3 end_p(p22(0), p22(1), p22(2));

		// 生成横杆主体
		TriangleMesh bar_mesh;
		if(this->Is_Special)		
			bar_mesh = Generate_SpecialBarMesh(start_p, end_p, radius).rotate(rotation_matrix.inverse());
		else
			bar_mesh = Generate_CylinderBarMesh(start_p, end_p, radius).rotate(rotation_matrix.inverse());
		// 生成横杆两端的小球
		TriangleMesh sphere_mesh = TriangleMeshGeneral::make_sphere(this->start_ball_radius);
		sphere_mesh.translate(this->start_point.x, this->start_point.y, this->start_point.z);
		bar_mesh.merge(sphere_mesh);
		TriangleMesh end_sphere_mesh = TriangleMeshGeneral::make_sphere(this->start_ball_radius);
		end_sphere_mesh.translate(this->end_point.x, this->end_point.y, this->end_point.z);
		bar_mesh.merge(end_sphere_mesh);

		bar_mesh.repair();
		if (bar_mesh.UnableRepair()) {
			LOGINFO("CrossBarGenerator::Generate Unable Repair");
			return TriangleMesh();
		}
		return bar_mesh;
	}

	// 小球
	TriangleMesh CrossBarGenerator::GenerateStartBall()
	{
		TriangleMesh sphere_mesh = TriangleMeshGeneral::make_sphere(this->start_ball_radius);
		sphere_mesh.translate(this->start_point.x, this->start_point.y, this->start_point.z);
		return sphere_mesh;
	}

	// 初始化静态成员
	bool CrossBarGenerator::Init_Special_CyMesh()
	{			
		std::string MeshDataPath = GetRunPath()+ "\\MeshDate\\";
		std::string SearchPath = MeshDataPath + "*.stl";
		LOGINFO("SearchPath = %s", SearchPath.c_str());
		//
		using namespace std;
		HANDLE hFind;
		WIN32_FIND_DATAA findData;
		hFind = FindFirstFileA(SearchPath.c_str(), &findData);
		if (hFind == INVALID_HANDLE_VALUE)
		{
			LOGINFO("Failed to find first file!\n");
			return false;
		}
		do
		{
			// 忽略"."和".."两个结果 
			if (strcmp(findData.cFileName, ".") == 0 || strcmp(findData.cFileName, "..") == 0)
				continue;
			if (findData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)    // 是否是目录 
			{
				LOGINFO("Is dir--%s", findData.cFileName);
			}
			else
			{
				std::string _filepath = findData.cFileName;
				_filepath = MeshDataPath + _filepath;
				Special_Cylinder* sc_p = new Special_Cylinder(_filepath);
				if (sc_p->length > 0.0) // 读取成功
				{
					LOGINFO("sc_p->FilePath == %s 读取成功! H[%f], R[%f], face count[%d]", 
						sc_p->FilePath.c_str(), sc_p->length, sc_p->raduis, sc_p->cy_mesh.facets_count()
					);
					Slic3r::SpCy_Meshs[sc_p->length_round] = sc_p;
				}
			}
		} while (FindNextFileA(hFind, &findData));
		LOGINFO("Slic3r::SpCy_Meshs.size() = %d", Slic3r::SpCy_Meshs.size());
		return Slic3r::SpCy_Meshs.size() > 0;
	}

}