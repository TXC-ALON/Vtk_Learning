#include "Model.hpp"
#include "IO.hpp"
#include <iostream>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>
#include <boost/typeof/typeof.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/locale.hpp>
#include <math.h>
#include <cmath>
#include "SVG.hpp"
#include "ClipperUtils.hpp"
#include "Print.hpp"
#include "AFFWrite.hpp"
#include <float.h>
#include <locale>
#include <codecvt>
#include <fstream>
#include <string>

namespace Slic3r
{

	// 约束种植孔上下面片的id
	std::set<int> ModelObject::LimitedFaces = {};

	////////////////////////////////////////////////////////////////////////////////////
	//  Model 类实现
	///////////////////////////////////////////////////////////////////////////////////

	Model::Model():object_count(0)
	{
		// LOGINFO("Model::Model()");
		// CrossBarGenerator::Init_Special_CyMesh();
	}

	// 构造函数
	Model::Model(const Model &other)
	{
		// copy materials
		for (ModelMaterialMap::const_iterator i = other.materials.begin(); i != other.materials.end(); ++i)
			this->add_material(i->first, *i->second);

		// copy objects
		this->objects.reserve(other.objects.size());
		for (ModelObjectPtrs::const_iterator i = other.objects.begin(); i != other.objects.end(); ++i)
			this->add_object(**i, true);

		object_count = other.object_count;
	}

	// 重载 =函数
	Model &Model::operator=(Model other)
	{
		this->swap(other);
		return *this;
	}

	// 交换两个Model值
	void
	Model::swap(Model &other)
	{
		std::swap(this->materials, other.materials);
		std::swap(this->objects, other.objects);
		std::swap(this->object_count, other.object_count);
	}

	// 析构函数
	Model::~Model()
	{
		this->clear_objects();
		this->clear_materials();
	}

	// perl中调用接口 读取文件stl文件并生成model类
	Model
	Model::read_from_file(std::string input_file, std::string file_name)
	{
		LOGINFO("input_file is %s, file_name is %s", input_file.c_str(), file_name.c_str());
		Model model;
		// stl obj amf amf.xml
		// 在model中添加一个modelobject类，在modelobject类中添加一个volume，将读取到trianglemesh添加到volume中
		if (boost::algorithm::iends_with(input_file, ".stl"))
		{
			LOGINFO("input_file is stl!");
			IO::STL::read(input_file, &model);
			model.objects[0]->volumes[0]->mesh.write_ascii("C:\\test.stl");
			model.objects[0]->volumes[0]->mesh.WriteOBJFile("C:\\test.obj");
		}
		else if (boost::algorithm::iends_with(input_file, ".obj"))
		{
			IO::OBJ::read(input_file, &model);
		}
		else if (boost::algorithm::iends_with(input_file, ".ipd"))
		{
			IO::OBJ::read(input_file, &model);
		}
		else if (boost::algorithm::iends_with(input_file, ".amf") || boost::algorithm::iends_with(input_file, ".amf.xml"))
		{
			IO::AMF::read(input_file, &model);
		}
		else if (boost::algorithm::iends_with(input_file, ".ptl"))
		{
			LOGINFO("input_file is ptl!");
			IO::PTL::read(input_file, &model);
		}
		else if (boost::algorithm::iends_with(input_file, ".aff"))
		{
			LOGINFO("input_file is aff!");
			AFFReader _aff(input_file);
			std::string header = _aff.GetXmlHeaderStr();
			_aff.ReadXMLHeader(header);
			_aff.ReadLayers();
			_aff.OutputContour("C:\\Users\\Administrator\\Desktop\\oqton\\svg");
		}
		else
		{
			throw std::runtime_error("Unknown file format");
		}

		if (model.objects.empty())
			throw std::runtime_error("The supplied file couldn't be read because it's empty");
		// 为ModelObject 记录 名称和路径
		for (ModelObjectPtrs::const_iterator o = model.objects.begin(); o != model.objects.end(); ++o)
		{
			(*o)->input_file = input_file;
			(*o)->name = file_name;
		}

		return model;
	}

	// 在Model中添加一个Object
	ModelObject *
	Model::add_object()
	{
		ModelObject *new_object = new ModelObject(this);
		this->objects.push_back(new_object);

		//add by gaohui 20240722
		this->object_count++;
		new_object->object_num = this->object_count;
		
		return new_object;
	}

	// 同上
	ModelObject *
	Model::add_object(const ModelObject &other, bool copy_volumes)
	{
		ModelObject *new_object = new ModelObject(this, other, copy_volumes);
		this->objects.push_back(new_object);

		//add by gaohui 20240722
		this->object_count++;
		new_object->object_num = this->object_count;

		return new_object;
	}

	// 从Model里面删除某个object
	void
	Model::delete_object(size_t idx)
	{
		if (this->objects.empty() || idx >= this->objects.size())
			return;

		ModelObjectPtrs::iterator i = this->objects.begin() + idx;
		if(*i != nullptr)
		{
			delete *i;	
			*i = nullptr;
		}
		this->objects.erase(i);
	}

	// 清空object
	void
	Model::clear_objects()
	{
		// int instead of size_t because it can be -1 when vector is empty
		for (int i = this->objects.size() - 1; i >= 0; --i)
			this->delete_object(i);
	}
	void
	Model::clear_minorsupport_mark()
	{
		if (this->objects.empty())
			return;
		for (auto it : this->objects)
		{
			it->Set_Minorsupport(0);
		}
	}
	bool // 检查导入模型是否有重复
	Model::Is_Repeated(std::string name)
	{
		for (ModelObjectPtrs::iterator it = this->objects.begin(); it != this->objects.end(); it++)
		{
			if ((*it)->input_file == name)
			{
				return true;
			}
		}
		return false;
	}
	// 删除material
	void
	Model::delete_material(t_model_material_id material_id)
	{
		ModelMaterialMap::iterator i = this->materials.find(material_id);
		if (i != this->materials.end())
		{
			if(i->second != nullptr)
			{
				delete i->second;	
				i->second = nullptr;
			}
			this->materials.erase(i);
		}
	}

	// 清空materials
	void
	Model::clear_materials()
	{
		while (!this->materials.empty())
			this->delete_material(this->materials.begin()->first);
	}

	// 添加一种material  序号为material_id
	ModelMaterial *
	Model::add_material(t_model_material_id material_id)
	{
		ModelMaterial *material = this->get_material(material_id);
		if (material == NULL)
		{
			material = this->materials[material_id] = new ModelMaterial(this);
		}
		return material;
	}

	// 将other添加为一种material  序号为material_id
	ModelMaterial *
	Model::add_material(t_model_material_id material_id, const ModelMaterial &other)
	{
		// delete existing material if any
		ModelMaterial *material = this->get_material(material_id);
		if (material != NULL)
		{
			delete material;
			material = nullptr;
		}

		// set new material
		material = new ModelMaterial(this, other);
		this->materials[material_id] = material;
		return material;
	}

	// 通过ID获得material
	ModelMaterial *
	Model::get_material(t_model_material_id material_id)
	{
		ModelMaterialMap::iterator i = this->materials.find(material_id);
		if (i == this->materials.end())
		{
			return NULL;
		}
		else
		{
			return i->second;
		}
	}

	// 如果有object没有instance  返回true
	bool
	Model::has_objects_with_no_instances() const
	{
		for (ModelObjectPtrs::const_iterator i = this->objects.begin();
			 i != this->objects.end(); ++i)
		{
			if ((*i)->instances.empty())
			{
				return true;
			}
		}

		return false;
	}

	// 把没有instance的object加上默认的instance
	// makes sure all objects have at least one instance
	bool
	Model::add_default_instances()
	{
		// apply a default position to all objects not having one
		for (ModelObjectPtrs::const_iterator o = this->objects.begin(); o != this->objects.end(); ++o)
		{
			if ((*o)->instances.empty())
			{
				(*o)->add_instance();
			}
		}
		return true;
	}

	// 合并所有objects的bounding_box为一个
	// this returns the bounding box of the *transformed* instances
	BoundingBoxf3
	Model::bounding_box() const
	{
		BoundingBoxf3 bb;
		for (ModelObjectPtrs::const_iterator o = this->objects.begin(); o != this->objects.end(); ++o)
		{
			bb.merge((*o)->bounding_box());
		}
		return bb;
	}

	// 修复所有objects
	void
	Model::repair()
	{
		for (ModelObjectPtrs::const_iterator o = this->objects.begin(); o != this->objects.end(); ++o)
			(*o)->repair();
	}

	// 将所有物体的中心点放置在point点上
	void
	Model::center_instances_around_point(const Pointf &point)
	{
		BoundingBoxf3 bb = this->bounding_box();

		Sizef3 size = bb.size();
		coordf_t shift_x = -bb.min.x + point.x - size.x / 2;
		coordf_t shift_y = -bb.min.y + point.y - size.y / 2;
		for (ModelObjectPtrs::const_iterator o = this->objects.begin(); o != this->objects.end(); ++o)
		{
			for (ModelInstancePtrs::const_iterator i = (*o)->instances.begin(); i != (*o)->instances.end(); ++i)
			{
				(*i)->offset.translate(shift_x, shift_y);
			}
			(*o)->invalidate_bounding_box();
		}
	}

	bool
	Model::can_merge()
	{
		if (this->objects.size() <= 1)
			return false;

		int NC = 0;
		int LC = 0;
		int RC = 0;
		for (ModelObjectPtrs::const_iterator o = this->objects.begin(); o != this->objects.end(); ++o)
		{
			if ((*o)->bed_type == "Normal")
				NC++;
			else if ((*o)->bed_type == "Left")
				LC++;
			else if ((*o)->bed_type == "Right")
				RC++;
		}

		if (NC <= 1 && LC == 0 && RC == 0)
			return false;

		if (NC == 0 && LC <= 1 && RC <= 1)
			return false;

		return true;
	}

	// 将物体移动到原点
	void
	Model::align_instances_to_origin()
	{
		BoundingBoxf3 bb = this->bounding_box();

		Pointf new_center = (Pointf)bb.size();
		new_center.translate(-new_center.x / 2, -new_center.y / 2);
		this->center_instances_around_point(new_center);
	}

	// 将物体平移(x,y,z)
	void
	Model::translate(coordf_t x, coordf_t y, coordf_t z)
	{
		for (ModelObjectPtrs::const_iterator o = this->objects.begin(); o != this->objects.end(); ++o)
		{
			(*o)->translate(x, y, z);
		}
	}

	TriangleMesh Model::support_mesh()
	{
		if (this->objects.size() == 1)
			return this->objects[0]->support_mesh();
		TriangleMesh _mesh;
		for (ModelObjectPtrs::const_iterator o = this->objects.begin(); o != this->objects.end(); ++o)
		{
			_mesh.merge((*o)->support_mesh());
		}
		_mesh.repair();

		return _mesh;
	}

	// 合并所有的objects带各自instance到一个TriangleMesh
	// flattens everything to a single mesh
	TriangleMesh
	Model::mesh()
	{
		if (this->objects.size() == 1)
			return this->objects[0]->mesh();

		TriangleMesh _mesh;
		for (ModelObjectPtrs::const_iterator o = this->objects.begin(); o != this->objects.end(); ++o)
		{
			_mesh.merge((*o)->mesh());
		}
		_mesh.repair();

		return _mesh;
	}

	// 合并所有的objects合并到一个TriangleMesh
	// flattens everything to a single mesh
	TriangleMesh
	Model::raw_mesh()
	{
		if (this->objects.size() == 1)
			return this->objects[0]->raw_mesh();

		TriangleMesh _raw_mesh;
		for (ModelObjectPtrs::const_iterator o = this->objects.begin(); o != this->objects.end(); ++o)
		{
			_raw_mesh.merge((*o)->raw_mesh());
		}
		_raw_mesh.repair();

		return _raw_mesh;
	}

	bool
	Model::_arrange(const Pointfs &sizes, coordf_t dist, const BoundingBoxf *bb, Pointfs &out) const
	{
		// we supply unscaled data to arrange()
		bool result = Slic3r::Geometry::arrange(
			sizes.size(),			 // number of parts
			BoundingBoxf(sizes).max, // width and height of a single cell
			dist,					 // distance between cells
			bb,						 // bounding box of the area to fill
			out						 // output positions
		);

		if (!result && bb != NULL)
		{
			// Try to arrange again ignoring bb
			result = Slic3r::Geometry::arrange(
				sizes.size(),			 // number of parts
				BoundingBoxf(sizes).max, // width and height of a single cell
				dist,					 // distance between cells
				NULL,					 // bounding box of the area to fill
				out						 // output positions
			);
		}

		return result;
	}

	/*  arrange objects preserving their instance count
		but altering their instance positions */
	bool
	Model::arrange_objects(coordf_t dist, const BoundingBoxf *bb)
	{
		// get the (transformed) size of each instance so that we take
		// into account their different transformations when packing
		Pointfs instance_sizes;
		for (ModelObjectPtrs::const_iterator o = this->objects.begin(); o != this->objects.end(); ++o)
		{
			for (size_t i = 0; i < (*o)->instances.size(); ++i)
			{
				instance_sizes.push_back((*o)->instance_bounding_box(i).size());
			}
		}

		Pointfs positions;
		if (!this->_arrange(instance_sizes, dist, bb, positions))
			return false;

		for (ModelObjectPtrs::const_iterator o = this->objects.begin(); o != this->objects.end(); ++o)
		{
			for (ModelInstancePtrs::const_iterator i = (*o)->instances.begin(); i != (*o)->instances.end(); ++i)
			{
				(*i)->offset = positions.back();
				positions.pop_back();
			}
			(*o)->invalidate_bounding_box();
		}
		return true;
	}

	// 调整支架姿态
	bool
	ModelObject::Pre_adjust_object_zhijia(int facets_threshold, double tuqi_facets_percent)
	{
		Vectorf3 nor_1, nor_2, nor_3;
		double var_1, var_2, var_3;
		std::map<double, Vectorf3> map_nor;
		TriangleMesh objectMesh = this->mesh_output("Obj", false, false);
		LOGINFO("[%s] facets_count [%d/%d]",
				this->name.c_str(),
				facets_threshold * 10000,
				objectMesh.facets_count());
		// 面片小于10w的物体不做主成分分析
		if (objectMesh.facets_count() < facets_threshold * 10000)
			return false;

		unsigned int TuqiFactCount = 0;
		const double tuqi_area_threshold = 0.005;
		for (unsigned int i = 0; i < objectMesh.stl.stats.number_of_facets; ++i)
		{
			stl_facet facet = objectMesh.stl.facet_start[i];
			if (TriangleMesh::Calculate_Facet_Area(facet) < tuqi_area_threshold) // 花纹部分面片 法相相反
				TuqiFactCount++;
		}
		bool IsReverseTuqi = ((double)TuqiFactCount / objectMesh.stl.stats.number_of_facets > tuqi_facets_percent);
		LOGINFO("[%s] TuqiFactCount [%d/%d]",
				this->name.c_str(),
				TuqiFactCount,
				objectMesh.facets_count());

		// 主成分分析
		if (objectMesh.Normal_PCA(map_nor, IsReverseTuqi))
		{
			std::map<double, Vectorf3>::iterator map_nor_It = map_nor.begin();
			var_1 = map_nor_It->first;
			nor_1 = (map_nor_It++)->second;
			var_2 = map_nor_It->first;
			nor_2 = (map_nor_It++)->second;
			var_3 = map_nor_It->first;
			nor_3 = (map_nor_It++)->second;
		}
		LOGINFO("[%s] before nor1 %s [%f],  nor2 %s [%f], nor3 %s [%f]",
				this->name.c_str(),
				nor_1.dump_perl().c_str(),
				var_1,
				nor_2.dump_perl().c_str(),
				var_2,
				nor_3.dump_perl().c_str(),
				var_3);

		// 翻转模型
		if (nor_3.z < -0.00001)
		{
			this->rotate(PI, X);
			nor_1 = nor_1.rotate_x(PI);
			nor_2 = nor_2.rotate_x(PI);
			nor_3 = nor_3.rotate_x(PI);
			objectMesh.rotate_x(PI);
		}
		// nor_3分量对其z轴
		double z_angle = nor_3.Get_XOY_rotate2Y();
		double nor_xyz = nor_3.length();
		double x_angle = acos(nor_3.z / nor_xyz);

		// 将nor_3转到Y轴平面
		this->rotate(z_angle, Z);
		nor_1 = nor_1.rotate_z(z_angle);
		nor_2 = nor_2.rotate_z(z_angle);
		nor_3 = nor_3.rotate_z(z_angle);
		objectMesh.rotate_z(z_angle);

		// 将nor_3变成Z轴
		this->rotate(x_angle, X);
		nor_1 = nor_1.rotate_x(x_angle);
		nor_2 = nor_2.rotate_x(x_angle);
		nor_3 = nor_3.rotate_x(x_angle);
		objectMesh.rotate_x(x_angle);

		// 将nor_1变成Y轴
		double z_angle_toY = nor_1.Get_XOY_rotate2Y();
		this->rotate(z_angle_toY, Z);
		nor_1 = nor_1.rotate_z(z_angle_toY);
		nor_2 = nor_2.rotate_z(z_angle_toY);
		nor_3 = nor_3.rotate_z(z_angle_toY);
		objectMesh.rotate_z(z_angle_toY);

		// 再沿x轴翻转
		this->rotate(PI, Z);
		nor_1 = nor_1.rotate_z(PI);
		nor_2 = nor_2.rotate_z(PI);
		nor_3 = nor_3.rotate_z(PI);
		objectMesh.rotate_z(PI);

		LOGINFO("[%s] rotate z[%f], rotate x[%f], z_angle_toY z[%f]",
				this->name.c_str(),
				(z_angle * 180 / PI),
				(x_angle * 180 / PI),
				(z_angle_toY * 180 / PI));
		LOGINFO("[%s] after nor1 %s [%f], nor2 %s [%f], nor3 %s [%f]",
				this->name.c_str(),
				nor_1.dump_perl().c_str(),
				var_1,
				nor_2.dump_perl().c_str(),
				var_2,
				nor_3.dump_perl().c_str(),
				var_3);

		return true;
	}

	bool
	ModelObject::Pro_adjust_object_zhijia(int angle_threshold_min, int angle_threshold_max, double z_min, double z_max)
	{
		if (this->volumes.size() == 0)
			return false;
		TriangleMesh objectMesh = this->mesh_output("Obj", false, false);
		double temp_z = objectMesh.bounding_box().size().z;
		int rotate_angle = 0;
		if (temp_z >= z_max)
			rotate_angle = angle_threshold_min;
		else if (temp_z <= z_min)
			rotate_angle = angle_threshold_max;
		else
		{
			rotate_angle = angle_threshold_max - (angle_threshold_max - angle_threshold_min) * (temp_z - z_min) / (z_max - z_min);
		}
		LOGINFO("%s Pro_adjust_object_zhijia, z_heigh %f, x_angle %d", this->name.c_str(), temp_z, rotate_angle);
		this->rotate(PI * (rotate_angle) / 180, X);

		return true;
	}

	// 调整object姿态，使其包围盒z范围最小
	void
	ModelObject::adjust_object(int angle, int step)
	{
		angle = angle > 0 ? angle : 15;
		step = step > 0 ? step : 1;
		LOGINFO("begin adjust_object!!!");
		int x_angle = 0;
		int y_angle = 0;

		TriangleMesh objectMesh = this->mesh_output("Obj", false, false);

		// X轴微调
		objectMesh.rotate(-PI * angle / 180.0, X);
		double x_min_z = objectMesh.bounding_box().size().z;

		for (int x = 0; x <= 2 * angle; x += step)
		{
			double temp_z = objectMesh.bounding_box().size().z;
			if (temp_z < x_min_z)
			{
				x_min_z = temp_z;
				x_angle = x;
			}
			objectMesh.rotate(step * PI / 180.0, X);
		}
		objectMesh.rotate(-PI * (2 * angle + step - x_angle) / 180.0, X);

		// Y轴微调
		objectMesh.rotate(-PI * angle / 180.0, Y);
		double y_min_z = objectMesh.bounding_box().size().z;

		for (int y = 0; y <= 2 * angle; y += step)
		{
			double temp_z = objectMesh.bounding_box().size().z;
			if (temp_z < y_min_z)
			{
				y_min_z = temp_z;
				y_angle = y;
			}
			objectMesh.rotate(step * PI / 180.0, Y);
		}
		// 	raw_mesh_copy.rotate(-PI * (2 * angle + 1 - y_angle) / 180.0, Y);

		this->rotate(-PI * (angle - x_angle) / 180.0, X);
		this->rotate(-PI * (angle - y_angle) / 180.0, Y);
		// 	this->apply_matrix_to_mesh();
	}

	void
	ModelObject::reverse_object()
	{
		this->rotate(PI, X);
		// 	this->apply_matrix_to_mesh();
	}

	void
	ModelObject::rotate_around_X(double angle)
	{
		this->rotate(angle * PI / 180.0, X);
	}

	Polygon ModelObject::get_boundingbox_thumbnail()
	{
		BoundingBoxf3 bb = this->bounding_box();
		coord_t x_min = scale_(bb.min.x);
		coord_t y_min = scale_(bb.min.y);
		coord_t x_max = scale_(bb.max.x);
		coord_t y_max = scale_(bb.max.y);

		Polygon thumbnail;
		thumbnail.append(Point(x_min, y_min));
		thumbnail.append(Point(x_max, y_min));
		thumbnail.append(Point(x_max, y_max));
		thumbnail.append(Point(x_min, y_max));

		return thumbnail;
	}

	int
	ModelObject::get_IpdRpdType()
	{
		if (this->volumes.size() == 0)
			return -1;

		return this->volumes.at(0)->IpdRpdType;
	}

	Polygon
	ModelObject::get_thumbnail(double tolerance)
	{
		return this->_get_thumbnail(tolerance);
	}

	Polygon
	ModelObject::_get_thumbnail(double tolerance)
	{
		Polygon thumb;
		TriangleMesh rawMesh_obj = this->raw_mesh();
		if (rawMesh_obj.shells_count() > 1)
			thumb = rawMesh_obj.convex_hull();
		else
		{
			this->update_thumbnails();
			LOGINFO("ex size = %d", this->thumbnails.size());
			Polygons thumbs;
			for (int i = 0; i < this->thumbnails.size(); i++)
			{
				thumbs.push_back(this->thumbnails.at(i).contour);
				LOGINFO("i = %d, thumbnails.size() = [%d]", i, thumbs.size());
			}
			if (thumbs.size() > 1)
			{
				thumbs = union_(thumbs);
				std::vector<size_t> retval_vec = Slic3r::Geometry::sort_thumbnails(thumbs);
				thumb = thumbs.at(retval_vec.front());
			}
			else if (thumbs.size() == 1)
			{
				thumb = thumbs.front();
				LOGINFO("thumbs.size() == 1!!");
			}
			else
			{
				LOGINFO("thumbs is empty!!");
			}

			if (thumb.points.size() == 0)
			{
				LOGINFO("thumbnail.points.size() == 0, thumbnail.simplify is empty!!");
				thumb = rawMesh_obj.convex_hull();
			}

			if (false)
			{
				SVG svg1("thumbnail.svg");
				svg1.draw(thumbs, "blue");
				svg1.draw(thumb, "red");
				svg1.Close();
			}
		}

		// 考虑横杆
		std::vector<TriangleMesh> cross_bar_meshs = this->crossbar_meshs();
		Polygons thumbs2;
		thumbs2.push_back(thumb);
		for (int i = 0; i < cross_bar_meshs.size(); i++)
		{
			thumbs2.push_back(cross_bar_meshs[i].convex_hull());
		}
		if (thumbs2.size() > 1)
		{
			thumbs2 = union_(thumbs2);
			std::vector<size_t> retval_vec = Slic3r::Geometry::sort_thumbnails(thumbs2);
			thumb = thumbs2.at(retval_vec.front());
		}
		// 化简图形  否则排版会卡
		thumbs2 = thumb.simplify(tolerance);
		if (thumbs2.size() > 1)
		{
			std::vector<size_t> retval_vec = Slic3r::Geometry::sort_thumbnails(thumbs2);
			thumb = thumbs2.at(retval_vec.front());
		}
		else if (thumbs2.size() == 1)
		{
			thumb = thumbs2.front();
			LOGINFO("thumbs2.size() == 1!!");
		}
		else
		{
			LOGINFO("thumbs2.simplify is empty!!");
			thumb = rawMesh_obj.convex_hull();
		}

		//
		ModelInstance *p_major = this->instances.at(0);
		if (p_major)
		{
			thumb.rotate(p_major->rotation * PI / 180);
			thumb.translate(scale_(p_major->offset.x), scale_(p_major->offset.y));
		}
		else
		{
			CONFESS("this object do not has object instance");
		}

		LOGINFO("thumbnail Points size = %d area = %f", thumb.points.size(), thumb.area() / 1e12);
		LOGINFO("ModelObject::_get_thumbnail() end.. [%s]", this->name.c_str());

		return thumb;
	}

	static bool comp_polygon_area(const Polygon &a, const Polygon &b)
	{
		return a.area() > b.area();
	}

	Polygons
	ModelObject::get_next_bed(const Polygons bed, const coordf_t object_dist, const coordf_t area, const bool crown, const double tolerance)
	{
		LOGINFO("start get next bed!!");
		coord_t dist = scale_(object_dist > 2 * tolerance ? object_dist : 2 * tolerance);

		Polygon thumbnail = this->get_thumbnail(tolerance);
		LOGINFO("get the thumbnail!!");

		if (false)
		{
			SVG svg1("thumbnail.svg");
			svg1.draw(bed, "green");
			svg1.draw(thumbnail, "red");
			svg1.Close();
		}

		Polygons next_beds = diff(bed, offset(thumbnail, dist));
		next_beds = offset2(next_beds, -scale_(2.0), scale_(2.0));
		// next_beds = offset2(next_beds, -dist * 2, dist * 2);
		LOGINFO("get next beds!!");

		Polygons next_beds_simplify;
		next_beds_simplify.clear();
		for (Polygons::const_iterator it = next_beds.begin(); it != next_beds.end(); ++it)
			next_beds_simplify += it->simplify(scale_(tolerance));
		LOGINFO("get next_beds_simplify!!");

		if (crown)
		{
			Polygons::iterator pol_It = next_beds_simplify.begin();
			for (; pol_It != next_beds_simplify.end();)
			{
				if (pol_It->area() < area)
				{
					pol_It = next_beds_simplify.erase(pol_It);
					LOGINFO("delete a small bed!!!");
				}
				else
				{
					pol_It++;
				}
			}
		}
		LOGINFO("end get next bed!!");

		return next_beds_simplify;
	}

	bool
	ModelObject::arrange_object_slm(const Polygons bed, const coord_t nr1, const coord_t nr2, const std::string direction, const coordf_t area, const bool crown, const double tolerance)
	{

		LOGINFO("[%s] strat arrange_objects_slm!!!", this->name.c_str());
		DWORD timecount = GetTickCount();

		// 排序标志位置false
		this->arranged = false;

		if (bed.empty())
			return false;

		// 计算object的投影
		Polygon proj = this->get_thumbnail(tolerance);

		std::string temp_direction;

		if (crown && (std::abs(proj.area()) < area))
		{
			temp_direction = Slic3r::Geometry::get_reverse_direction(direction);
			LOGINFO("get_reverse_direction!!!");
		}
		else
			temp_direction = direction;

		LOGINFO("bed.siez() = %d, proj.area = %f, area = %f, temo_direction is %s", bed.size(), proj.area() / 1e12, area / 1e12, temp_direction.c_str());

		Point offset = Slic3r::Geometry::find_farest_point(bed, temp_direction);

		double rotation = 0.0;
		bool result = Slic3r::Geometry::get_position(bed, &proj, nr1, nr2, temp_direction, offset, rotation);
		if (result)
		{
			LOGINFO("get a right position!!!!!!!!!!!!!!");
			LOGINFO("this->get_thumbnail()1.centroid().x = %d, this->get_thumbnail().centroid()1.y = %d",
					this->get_thumbnail(tolerance).centroid().x,
					this->get_thumbnail(tolerance).centroid().y);
			ModelInstance *p_major = this->instances.at(0);
			if (p_major)
			{
				// 顺序不能动！！！
				p_major->rotation += rotation * 180 / PI;
				offset = offset - this->get_thumbnail(tolerance).centroid();
				p_major->offset.translate(unscale((coordf_t)offset.x), unscale((coordf_t)offset.y));
				LOGINFO("arrange_object_slm p_major->offset = [%s]", p_major->offset.dump_perl().c_str());
				this->invalidate_bounding_box();
			}
			else
			{
				CONFESS("this object do not has object instance");
			}
		}
		else
			LOGINFO("cannot get a right position??????????????");

		this->arranged = result;
		LOGINFO("[%s] arrange_object_slm time = %d", this->name.c_str(), GetTickCount() - timecount);

		return result;
	}

	void
	ModelObject::pre_adjust_zhijia_for_arrange(int angle, int step)
	{
		angle = angle > 0 ? angle : 90;
		step = step > 0 ? step : 1;
		LOGINFO("begin pre_adjust_zhijia_for_arrange!!!");
		int z_angle = -angle;

		TriangleMesh objectMesh = this->mesh_output("Obj", false, false);

		// Z轴微调
		objectMesh.rotate(-PI * angle / 180.0, Z);
		double max_x = objectMesh.bounding_box().size().x;

		for (int z = 0; z <= 2 * angle; z += step)
		{
			double temp_x = objectMesh.bounding_box().size().x;
			if (temp_x > max_x)
			{
				max_x = temp_x;
				z_angle = z;
			}
			objectMesh.rotate(step * PI / 180.0, Z);
		}

		this->rotate(-PI * (angle - z_angle) / 180.0, Z);
		// 	this->apply_matrix_to_mesh();
	}

	int
	ModelObject::arrange_object_slm_compact_new_complex(const Polygon &origin_bed, double step, double x_length, int last_idx, std::string direction, bool fast, bool crossbar, bool support)
	{
		LOGINFO("arrange_object_slm_compact_new_complex start");
		this->arranged = false;

		coord_t Step = scale_(step);
		DWORD timecount_start = GetTickCount();
		DWORD timecount = GetTickCount();

		BoundingBox bed_bb = origin_bed.bounding_box();
		coord_t bed_width = bed_bb.size().x;
		coord_t bed_max_y = bed_bb.max.y;
		coord_t bed_min_y = bed_bb.min.y;
		coord_t bed_max_x = bed_bb.max.x;
		coord_t bed_min_x = bed_bb.min.x;

		TriangleMesh currentMesh = this->meshWithSupport(crossbar, support);
		BoundingBoxf3 obj_bb = currentMesh.bounding_box();
		coord_t obj_width = scale_(obj_bb.size().x);
		coord_t obj_height = scale_(obj_bb.size().y);
		Pointf3 oc = obj_bb.center();
		Point obj_origin_center = Point(scale_(oc.x), scale_(oc.y));
		LOGINFO("last_idx = [%d], direction = [%s], obj_origin_center = [%s]", last_idx, direction.c_str(), obj_origin_center.dump_perl().c_str());

		Polygon thumbnail = this->get_thumbnail();
		LOGINFO("get_thumbnail times =%d", GetTickCount() - timecount);
		timecount = GetTickCount();

		coord_t start_x;
		// 确定起始的x坐标
		if (direction == "left")
		{
			start_x = bed_min_x + obj_width / 2 + 20;
		}
		else if (direction == "right")
		{
			start_x = bed_max_x - obj_width / 2 - 20;
		}

		LOGINFO("start_x = %d, direction = %s", start_x, direction.c_str());

		Point arrange_point;
		if (last_idx == -1) // 上方没有物体，直接定格排列
		{
			LOGINFO("last_idx == -1");

			arrange_point.x = start_x;
			arrange_point.y = bed_max_y - obj_height / 2;

			timecount = GetTickCount();
			Point trans = arrange_point - obj_origin_center;
			thumbnail.translate(trans);
			currentMesh.translate(unscale(trans.x), unscale(trans.y), 0.0);
			currentMesh.set_step_length(step, step, step);
			currentMesh.init_hash_facets();
			LOGINFO("currentMesh init_hash_facets times =%d", GetTickCount() - timecount);
			timecount = GetTickCount();

			int max_y = 0;
			// 找到第一个在盘内的位置
			while (Slic3r::Geometry::A_Is_In_B(thumbnail, origin_bed) == false)
			{
				thumbnail.translate(0, -Step);
				arrange_point.y -= Step;
				max_y++;
				// LOGINFO("last_idx = [%d], arrange_point = [%s], max_y = [%d]", last_idx, arrange_point.dump_perl().c_str(), max_y);

				if (arrange_point.y < bed_min_y + obj_height / 2) // 已经超出盘外，表明排不下
				{
					LOGINFO("arrange_point = [%s], bed_min_y = [%d], obj_height = [%d],arrange_point.y < bed_min_y + obj_height / 2",
							arrange_point.dump_perl().c_str(), bed_min_y, obj_height);
					LOGINFO("arranged false, model_object name = [%s]", this->name.c_str());
					LOGINFO("arrange_object_slm_compact_new end, times =%d", GetTickCount() - timecount_start);
					return 0;
				}
			}
			LOGINFO("last_idx == -1, get right place, times =%d", GetTickCount() - timecount);
			timecount = GetTickCount();

			// 第一个在盘内的位置和其他支架有相交，继续向下移动
			std::map<XYZ_Index, std::vector<int>> temp_hash_facets = currentMesh.translate_hash_facets_new(0, -max_y, 0);
			while (this->get_model()->hash_facets_for_arrange_contain_value(temp_hash_facets))
			{
				arrange_point.y -= Step;
				max_y++;
				temp_hash_facets = currentMesh.translate_hash_facets_new(0, -max_y, 0);
			}

			// 再次检测是否超盘
			if (arrange_point.y < bed_min_y + obj_height / 2) // 已经超出盘外，表明排不下
			{
				LOGINFO("arrange_point = [%s], bed_min_y = [%d], obj_height = [%d],arrange_point.y < bed_min_y + obj_height / 2",
						arrange_point.dump_perl().c_str(), bed_min_y, obj_height);
				LOGINFO("arranged false, model_object name = [%s]", this->name.c_str());
				LOGINFO("arrange_object_slm_compact_new end, times =%d", GetTickCount() - timecount_start);
				return 0;
			}

			this->get_model()->update_hash_facets_for_arrange(temp_hash_facets);
		}
		else
		{
			LOGINFO("last_idx != -1");
			TriangleMesh lastMesh = this->get_meshWithSupport_from_model(last_idx, crossbar, support);
			Pointf3 lm3 = lastMesh.bounding_box().center();
			coordf_t lm_height = lastMesh.bounding_box().size().y;
			Pointf lastCenter = Pointf(lm3.x, lm3.y);
			LOGINFO("lastMesh lastCenter = [%s]", lastCenter.dump_perl().c_str());

			arrange_point.x = start_x;
			arrange_point.y = scale_(lm3.y);

			LOGINFO("get_meshWithSupport_from_model times =%d", GetTickCount() - timecount);
			timecount = GetTickCount();

			Point trans = arrange_point - obj_origin_center;
			thumbnail.translate(trans);
			currentMesh.translate(unscale(trans.x), unscale(trans.y), 0.0);
			currentMesh.set_step_length(step, step, step);
			currentMesh.init_hash_facets();
			coordf_t cm_height = currentMesh.bounding_box().size().y;
			LOGINFO("currentMesh init_hash_facets times =%d", GetTickCount() - timecount);
			timecount = GetTickCount();

			int k = direction == "right" ? -1 : 1;

			int max_y = -100000;
			int max_y_x = 0;

			int max_i = floor(x_length / step);
			int max_j = ceil((cm_height + lm_height) / step);

			// 左右移动，y找到最高的位置
			for (int i = 0; i < max_i; i++)
			{
				if (fast) // 二分查找法，快速定位位置
				{
					std::map<XYZ_Index, std::vector<int>> temp_hash_facets = currentMesh.translate_hash_facets_new(i * k, -max_j, 0);
					if (this->get_model()->hash_facets_for_arrange_contain_value(temp_hash_facets)) // 最大值如果也发生碰撞，跳过
						continue;

					int j_start = 0;
					int j_end = max_j;
					if (max_j > abs(max_y))
						max_j = abs(max_y);
					LOGINFO("j_start = [%d], j_end = [%d]", j_start, j_end);

					int final_j = j_end;
					while (j_start < j_end)
					{
						// 从0到j折半查找，直到找到符合条件的最小的j值
						int j_mid = (j_start + j_end) / 2;
						LOGINFO("j_mid = [%d], final_j = [%d]", j_mid, final_j);

						std::map<XYZ_Index, std::vector<int>> temp_hash_facets = currentMesh.translate_hash_facets_new(i * k, -j_mid, 0);
						if (this->get_model()->hash_facets_for_arrange_contain_value(temp_hash_facets)) // 如果发生碰撞
						{
							j_start = j_mid + 1;
						}
						else // 没有发生碰撞
						{
							final_j = j_mid;
							j_end = j_mid - 1;
						}
					}

					coord_t arrange_point_y = arrange_point.y - final_j * Step;
					if (arrange_point_y < bed_min_y + obj_height / 2) // 已经超出盘外，表明排不下
					{
						LOGINFO("已经超出盘外, arrange_point_y = [%d], bed_min_y = [%d], obj_height = [%d],arrange_point.y < bed_min_y + obj_height / 2", arrange_point_y, bed_min_y, obj_height);
						continue;
					}

					Polygon thumbnail_temp(thumbnail.points);
					thumbnail_temp.translate(i * k * Step, -final_j * Step);
					if (Slic3r::Geometry::A_Is_In_B(thumbnail_temp, origin_bed) == false)
					{
						LOGINFO("this->A_Is_In_B(thumbnail_temp, origin_bed) == false, i = [%d], final_j = [%d]", i, final_j);
						if (false)
						{
							char svg_name[255];
							sprintf(svg_name, "half_thumbnail_temp[%s][%d][%d].svg", this->name.c_str(), i, final_j);
							SVG svg(svg_name);
							svg.draw(thumbnail_temp, "red");
							svg.draw(origin_bed, "green");
							svg.Close();
						}

						continue;
					}

					if (max_y < -final_j)
					{
						max_y = -final_j;
						max_y_x = i * k;
					}
				}
				else // 从上往下查找位置
				{
					DWORD timecount_for = GetTickCount();
					int j = 0;
					Polygon thumbnail_temp(thumbnail.points);
					thumbnail_temp.translate(i * k * Step, 0);

					std::map<XYZ_Index, std::vector<int>> temp_hash_facets = currentMesh.translate_hash_facets_new(i * k, j, 0);
					coord_t arrange_point_y = arrange_point.y;

					LOGINFO("translate_hash_facets_new, times =%d", GetTickCount() - timecount_for);

					bool inBed = true;
					// DWORD timecount_while = GetTickCount();

					// 延y轴向下，直到物体与已排版物体不再相交
					while (this->get_model()->hash_facets_for_arrange_contain_value(temp_hash_facets))
					{
						j--;
						LOGINFO("i = [%d], j = [%d]", i * k, j);

						if (j <= max_y)
						{
							LOGINFO("j <= max_y");
							inBed = false;
							break;
						}

						thumbnail_temp.translate(0, -Step);
						arrange_point_y -= Step;

						if (arrange_point_y < bed_min_y + obj_height / 2) // 已经超出盘外，表明排不下
						{
							LOGINFO("arrange_point_y = [%d], bed_min_y = [%d], obj_height = [%d],arrange_point.y < bed_min_y + obj_height / 2",
									arrange_point_y, bed_min_y, obj_height);
							// LOGINFO("arrange_object_slm_compact_new_complex end");
							inBed = false;
							break;
						}

						if (Slic3r::Geometry::A_Is_In_B(thumbnail_temp, origin_bed) == false)
						{
							LOGINFO("this->A_Is_In_B(thumbnail_temp, origin_bed) == false, i = [%d], final_j = [%d]", i, j);
							if (false)
							{
								char svg_name[255];
								sprintf(svg_name, "ud_thumbnail_temp[%s][%d][%d].svg", this->name.c_str(), i, j);
								SVG svg(svg_name);
								svg.draw(thumbnail_temp, "red");
								svg.draw(origin_bed, "green");
								svg.Close();
							}

							inBed = false;
							break;
						}

						temp_hash_facets = currentMesh.translate_hash_facets_new(i * k, j, 0);
						// LOGINFO("translate_hash_facets times =%d", GetTickCount() - timecount_while);
						// timecount_while = GetTickCount();
						// LOGINFO("last_idx = [%d], arrange_point = [%s]", last_idx, arrange_point.dump_perl().c_str());
					}
					LOGINFO("max_y = [%d], j = [%d]", max_y, j);

					LOGINFO("for i = [%d] over, times =%d", i, GetTickCount() - timecount_for);
					timecount_for = GetTickCount();

					if (inBed == false)
					{
						continue;
					}

					if (max_y < j)
					{
						max_y = j;
						max_y_x = i * k;
					}
				}
			}

			// 未找到合适的位置
			if (max_y == -100000)
			{
				LOGINFO("max_y = [%d], Can't arrang this object", max_y);
				LOGINFO("arrange_object_slm_compact_new end, times =%d", GetTickCount() - timecount_start);
				return 0;
			}

			LOGINFO("for stop times =%d", GetTickCount() - timecount);
			timecount = GetTickCount();

			LOGINFO("max_y = [%d], max_y_x = [%d]", max_y, max_y_x);
			arrange_point.translate(max_y_x * Step, max_y * Step);

			// 向hash_facets_for_arrange中添加已排版物体
			std::map<XYZ_Index, std::vector<int>> final_hash_facets = currentMesh.translate_hash_facets_new(max_y_x, max_y, 0);
			this->get_model()->update_hash_facets_for_arrange(final_hash_facets);

			LOGINFO("update_hash_facets_for_arrange =%d", GetTickCount() - timecount);
			timecount = GetTickCount();
		}

		// arrange_point就是物体的最终排版位置
		ModelInstance *p_major = this->instances.at(0);
		if (p_major)
		{
			Point trans = arrange_point - obj_origin_center;
			LOGINFO("trans = [%s]", trans.dump_perl().c_str());
			p_major->offset.translate(unscale(trans.x), unscale(trans.y));
			LOGINFO("arrange_object_slm p_major->offset = [%s]", p_major->offset.dump_perl().c_str());

			this->invalidate_bounding_box();
		}
		else
		{
			CONFESS("this object do not has object instance");
		}
		LOGINFO("ModelInstance translate, times =%d", GetTickCount() - timecount);

		this->arranged = true;
		LOGINFO("arrange_object_slm_compact_new end, times =%d", GetTickCount() - timecount_start);

		return 1;
	}

	int
	ModelObject::arrange_object_slm_compact_new_complex_multithread(const Polygon &origin_bed, double step, double x_length, int last_idx, std::string direction, bool fast, bool crossbar, bool support)
	{
		LOGINFO("arrange_object_slm_compact_new_complex_multithread start");
		this->arranged = false;

		coord_t Step = scale_(step);
		DWORD timecount_start = GetTickCount();
		DWORD timecount = GetTickCount();

		BoundingBox bed_bb = origin_bed.bounding_box();
		coord_t bed_width = bed_bb.size().x;
		coord_t bed_max_y = bed_bb.max.y;
		coord_t bed_min_y = bed_bb.min.y;
		coord_t bed_max_x = bed_bb.max.x;
		coord_t bed_min_x = bed_bb.min.x;

		TriangleMesh currentMesh = this->meshWithSupport(crossbar, support);
		BoundingBoxf3 obj_bb = currentMesh.bounding_box();
		coord_t obj_width = scale_(obj_bb.size().x);
		coord_t obj_height = scale_(obj_bb.size().y);
		Pointf3 oc = obj_bb.center();
		Point obj_origin_center = Point(scale_(oc.x), scale_(oc.y));
		LOGINFO("last_idx = [%d], direction = [%s], obj_origin_center = [%s]", last_idx, direction.c_str(), obj_origin_center.dump_perl().c_str());

		Polygon thumbnail = this->get_thumbnail();
		LOGINFO("get_thumbnail times =%d", GetTickCount() - timecount);
		timecount = GetTickCount();

		coord_t start_x;
		// 确定起始的x坐标
		if (direction == "left")
		{
			start_x = bed_min_x + obj_width / 2 + 20;
		}
		else if (direction == "right")
		{
			start_x = bed_max_x - obj_width / 2 - 20;
		}

		LOGINFO("start_x = %d, direction = %s", start_x, direction.c_str());

		Point arrange_point;
		if (last_idx == -1) // 上方没有物体，直接定格排列
		{
			LOGINFO("last_idx == -1");

			arrange_point.x = start_x;
			arrange_point.y = bed_max_y - obj_height / 2;

			timecount = GetTickCount();
			Point trans = arrange_point - obj_origin_center;
			thumbnail.translate(trans);
			currentMesh.translate(unscale(trans.x), unscale(trans.y), 0.0);
			currentMesh.set_step_length(step, step, step);
			currentMesh.init_hash_facets();
			LOGINFO("currentMesh init_hash_facets times =%d", GetTickCount() - timecount);
			timecount = GetTickCount();

			int max_y = 0;
			// 找到第一个在盘内的位置
			while (Slic3r::Geometry::A_Is_In_B(thumbnail, origin_bed) == false)
			{
				thumbnail.translate(0, -Step);
				arrange_point.y -= Step;
				max_y++;
				// LOGINFO("last_idx = [%d], arrange_point = [%s], max_y = [%d]", last_idx, arrange_point.dump_perl().c_str(), max_y);

				if (arrange_point.y < bed_min_y + obj_height / 2) // 已经超出盘外，表明排不下
				{
					LOGINFO("arrange_point = [%s], bed_min_y = [%d], obj_height = [%d],arrange_point.y < bed_min_y + obj_height / 2",
							arrange_point.dump_perl().c_str(), bed_min_y, obj_height);
					LOGINFO("arranged false, model_object name = [%s]", this->name.c_str());
					LOGINFO("arrange_object_slm_compact_new end, times =%d", GetTickCount() - timecount_start);
					return 0;
				}
			}
			LOGINFO("last_idx == -1, get right place, times =%d", GetTickCount() - timecount);
			timecount = GetTickCount();

			// 第一个在盘内的位置和其他支架有相交，继续向下移动
			std::map<XYZ_Index, std::vector<int>> temp_hash_facets = currentMesh.translate_hash_facets_new(0, -max_y, 0);
			while (this->get_model()->hash_facets_for_arrange_contain_value(temp_hash_facets))
			{
				arrange_point.y -= Step;
				max_y++;
				temp_hash_facets = currentMesh.translate_hash_facets_new(0, -max_y, 0);
			}

			// 再次检测是否超盘
			if (arrange_point.y < bed_min_y + obj_height / 2) // 已经超出盘外，表明排不下
			{
				LOGINFO("arrange_point = [%s], bed_min_y = [%d], obj_height = [%d],arrange_point.y < bed_min_y + obj_height / 2",
						arrange_point.dump_perl().c_str(), bed_min_y, obj_height);
				LOGINFO("arranged false, model_object name = [%s]", this->name.c_str());
				LOGINFO("arrange_object_slm_compact_new end, times =%d", GetTickCount() - timecount_start);
				return 0;
			}

			this->get_model()->update_hash_facets_for_arrange(temp_hash_facets);
		}
		else
		{
			// last_idx != -1排版采用多线程方式进行
			LOGINFO("last_idx != -1");
			TriangleMesh lastMesh = this->get_meshWithSupport_from_model(last_idx, crossbar, support);
			Pointf3 lm3 = lastMesh.bounding_box().center();
			coordf_t lm_height = lastMesh.bounding_box().size().y;
			Pointf lastCenter = Pointf(lm3.x, lm3.y);
			LOGINFO("lastMesh lastCenter = [%s]", lastCenter.dump_perl().c_str());

			arrange_point.x = start_x;
			arrange_point.y = scale_(lm3.y);

			LOGINFO("get_meshWithSupport_from_model times =%d", GetTickCount() - timecount);
			timecount = GetTickCount();

			Point trans = arrange_point - obj_origin_center;
			thumbnail.translate(trans);
			currentMesh.translate(unscale(trans.x), unscale(trans.y), 0.0);
			currentMesh.set_step_length(step, step, step);
			currentMesh.init_hash_facets();
			coordf_t cm_height = currentMesh.bounding_box().size().y;
			LOGINFO("currentMesh init_hash_facets times =%d", GetTickCount() - timecount);
			timecount = GetTickCount();

			int k = direction == "right" ? -1 : 1;
			int max_i = floor(x_length / step);
			// int max_j = ceil((cm_height + lm_height) / step);

			Point offset(0, 0);
			calc_compact_arrange_threads ccat(max_i, k, Step, this->get_model(), &currentMesh, thumbnail, arrange_point.y);
			bool result = ccat.get_point(origin_bed, fast, offset);

			// 未找到合适的位置
			if (result == false)
			{
				LOGINFO("Can't arrange this object");
				LOGINFO("arrange_object_slm_compact_new end, times =%d", GetTickCount() - timecount_start);
				return 0;
			}
			arrange_point.translate(offset.x * Step, offset.y * Step);

			// 向hash_facets_for_arrange中添加已排版物体
			std::map<XYZ_Index, std::vector<int>> final_hash_facets = currentMesh.translate_hash_facets_new(offset.x, offset.y, 0);
			this->get_model()->update_hash_facets_for_arrange(final_hash_facets);

			LOGINFO("update_hash_facets_for_arrange =%d", GetTickCount() - timecount);
			timecount = GetTickCount();
		}

		// arrange_point就是物体的最终排版位置
		ModelInstance *p_major = this->instances.at(0);
		if (p_major)
		{
			Point trans = arrange_point - obj_origin_center;
			LOGINFO("trans = [%s]", trans.dump_perl().c_str());
			p_major->offset.translate(unscale(trans.x), unscale(trans.y));
			LOGINFO("arrange_object_slm p_major->offset = [%s]", p_major->offset.dump_perl().c_str());

			this->invalidate_bounding_box();
		}
		else
		{
			CONFESS("this object do not has object instance");
		}
		LOGINFO("ModelInstance translate, times =%d", GetTickCount() - timecount);

		this->arranged = true;
		LOGINFO("arrange_object_slm_compact_new_complex_multithread end, times =%d", GetTickCount() - timecount_start);

		return 1;
	}

	int
	ModelObject::arrange_object_slm_compact_new_complex_align_x(const Polygon &origin_bed, double step, double x_length, int last_idx, std::string direction_x, std::string direction_y, bool fast, bool crossbar, bool support)
	{
		LOGINFO("arrange_object_slm_compact_new_complex_align_x start");
		this->arranged = false;

		coord_t Step = scale_(step);
		DWORD timecount_start = GetTickCount();
		DWORD timecount = GetTickCount();

		BoundingBox bed_bb = origin_bed.bounding_box();
		coord_t bed_width = bed_bb.size().x;
		coord_t bed_max_y = bed_bb.max.y;
		coord_t bed_min_y = bed_bb.min.y;
		coord_t bed_max_x = bed_bb.max.x;
		coord_t bed_min_x = bed_bb.min.x;
		LOGINFO("bed_max_x = [%d], bed_min_x = [%d], bed_max_y = [%d], bed_min_y = [%d]", bed_max_x, bed_min_x, bed_max_y, bed_min_y);

		TriangleMesh currentMesh = this->meshWithSupport(crossbar, support);
		BoundingBoxf3 obj_bb = currentMesh.bounding_box();
		coord_t obj_width = scale_(obj_bb.size().x);
		coord_t obj_height = scale_(obj_bb.size().y);
		LOGINFO("obj_width = [%d], obj_height = [%d]", obj_width, obj_height);

		Pointf3 oc = obj_bb.center();
		Point obj_origin_center = Point(scale_(oc.x), scale_(oc.y));
		LOGINFO("last_idx = [%d], direction_x = [%s], direction_y = [%s], obj_origin_center = [%s]", last_idx, direction_x.c_str(), direction_y.c_str(), obj_origin_center.dump_perl().c_str());

		Polygon thumbnail = this->get_thumbnail(SCALED_RESOLUTION_SPT);
		LOGINFO("get_thumbnail times =%d", GetTickCount() - timecount);
		timecount = GetTickCount();

		coord_t start_y;
		// 确定起始的x坐标
		if (direction_y == "up")
		{
			start_y = bed_max_y - obj_height / 2 - 10000;
		}
		else if (direction_y == "center")
		{
			start_y = 0;
		}
		else if (direction_y == "down")
		{
			start_y = bed_min_y + obj_height / 2 + 10000;
		}

		LOGINFO("start_y = %d", start_y);
		int kx = direction_x == "right" ? -1 : 1;

		Point arrange_point;
		if (last_idx == -1) // 上方没有物体，直接定格排列
		{
			LOGINFO("last_idx == -1");

			arrange_point.y = start_y;
			if (direction_x == "left")
			{
				arrange_point.x = bed_min_x + kx * obj_width / 2;
			}
			else if (direction_x == "right")
			{
				arrange_point.x = bed_max_x + kx * obj_width / 2;
			}

			timecount = GetTickCount();
			Point trans = arrange_point - obj_origin_center;
			thumbnail.translate(trans);
			currentMesh.translate(unscale(trans.x), unscale(trans.y), 0.0);
			currentMesh.set_step_length(step, step, step);
			currentMesh.init_hash_facets();
			LOGINFO("currentMesh init_hash_facets times =%d", GetTickCount() - timecount);
			timecount = GetTickCount();

			int max_y = 0;
			// 找到第一个在盘内的位置
			while (Slic3r::Geometry::A_Is_In_B(thumbnail, origin_bed) == false)
			{
				thumbnail.translate(Step * kx, 0);
				arrange_point.x += Step * kx;
				max_y++;

				if (true)
				{
					char svg_name[255];
					sprintf(svg_name, "thumbnail_origin_bed1[%d].svg", max_y);
					SVG svg1(svg_name);
					svg1.draw(thumbnail, "blue");
					svg1.draw(origin_bed, "red");
					svg1.Close();
				}
				LOGINFO("last_idx = [%d], arrange_point = [%s], max_y = [%d], kx = [%d], Step = [%d]", last_idx, arrange_point.dump_perl().c_str(), max_y, kx, Step);

				if ((direction_x == "left" && arrange_point.x > bed_max_x - obj_width / 2) ||
					(direction_x == "right" && arrange_point.x < bed_min_x + obj_width / 2)) // 已经超出盘外，表明排不下
				{
					LOGINFO("arranged false, model_object name = [%s], direction_x == [%s], arrange_point = [%s], obj_width = [%d], bed_max_x = [%d], bed_min_x = [%d]", this->name.c_str(), direction_x.c_str(), arrange_point.dump_perl().c_str(), obj_width, bed_max_x, bed_min_x);
					LOGINFO("arrange_object_slm_compact_new 1 end, times =%d", GetTickCount() - timecount_start);
					return 0;
				}
			}
			LOGINFO("last_idx == -1, get right place, times =%d", GetTickCount() - timecount);
			timecount = GetTickCount();

			// 第一个在盘内的位置和其他支架有相交，继续向下移动
			std::map<XYZ_Index, std::vector<int>> temp_hash_facets = currentMesh.translate_hash_facets_new(max_y * kx, 0, 0);
			while (this->get_model()->hash_facets_for_arrange_contain_value(temp_hash_facets))
			{
				arrange_point.x += Step * kx;
				max_y++;
				temp_hash_facets = currentMesh.translate_hash_facets_new(max_y * kx, 0, 0);
			}

			// 再次检测是否超盘
			if ((direction_x == "left" && arrange_point.x > bed_max_x - obj_width / 2) ||
				((direction_x == "right" && arrange_point.x < bed_min_x + obj_width / 2))) // 已经超出盘外，表明排不下
			{
				LOGINFO("arranged false, model_object name = [%s], direction_x == [%s], arrange_point = [%s], obj_width = [%d], bed_max_x = [%d], bed_min_x = [%d]", this->name.c_str(), direction_x.c_str(), arrange_point.dump_perl().c_str(), obj_width, bed_max_x, bed_min_x);
				LOGINFO("arrange_object_slm_compact_new 2 end, times =%d", GetTickCount() - timecount_start);
				return 0;
			}

			this->get_model()->update_hash_facets_for_arrange(temp_hash_facets);
		}
		else
		{
			LOGINFO("last_idx != -1");
			TriangleMesh lastMesh = this->get_meshWithSupport_from_model(last_idx, crossbar, support);
			Pointf3 lm3 = lastMesh.bounding_box().center();
			Pointf lastCenter = Pointf(lm3.x, lm3.y);
			LOGINFO("lastMesh lastCenter = [%s]", lastCenter.dump_perl().c_str());

			arrange_point.y = start_y;
			arrange_point.x = scale_(lm3.x);

			LOGINFO("get_meshWithSupport_from_model times =%d", GetTickCount() - timecount);
			timecount = GetTickCount();

			Point trans = arrange_point - obj_origin_center;
			thumbnail.translate(trans);
			currentMesh.translate(unscale(trans.x), unscale(trans.y), 0.0);
			currentMesh.set_step_length(step, step, step);
			currentMesh.init_hash_facets();
			// coordf_t cm_height = currentMesh.bounding_box().size().y;
			LOGINFO("currentMesh init_hash_facets times =%d", GetTickCount() - timecount);

			// 从查找下一个支架的合适位置
			DWORD timecount_for = GetTickCount();
			int j = 0;

			std::map<XYZ_Index, std::vector<int>> temp_hash_facets = currentMesh.translate_hash_facets_new(j * kx, 0, 0);
			LOGINFO("translate_hash_facets_new, times =%d", GetTickCount() - timecount_for);

			// 延y轴向下，直到物体与已排版物体不再相交
			while (this->get_model()->hash_facets_for_arrange_contain_value(temp_hash_facets))
			{
				j++;

				thumbnail.translate(kx * Step, 0);
				arrange_point.x += kx * Step;

				if ((direction_x == "left" && arrange_point.x > bed_max_x - obj_width / 2) ||
					((direction_x == "right" && arrange_point.x < bed_min_x + obj_width / 2))) // 已经超出盘外，表明排不下
				{
					LOGINFO("x value valid, arrange_object_slm_compact_new_complex end");
					return 0;
				}

				temp_hash_facets = currentMesh.translate_hash_facets_new(kx * j, 0, 0);
				// LOGINFO("last_idx = [%d], arrange_point = [%s]", last_idx, arrange_point.dump_perl().c_str());
			}

			if (Slic3r::Geometry::A_Is_In_B(thumbnail, origin_bed) == false)
			{
				LOGINFO("not in thumbnail, arrange_object_slm_compact_new_complex end");
				if (true)
				{
					char svg_name[255];
					sprintf(svg_name, "thumbnail_origin_bed2[%d].svg", j);
					SVG svg1(svg_name);
					svg1.draw(thumbnail, "blue");
					svg1.draw(origin_bed, "red");
					svg1.Close();
				}
				return 0;
			}

			// 向hash_facets_for_arrange中添加已排版物体
			this->get_model()->update_hash_facets_for_arrange(temp_hash_facets);
		}

		// arrange_point就是物体的最终排版位置
		ModelInstance *p_major = this->instances.at(0);
		if (p_major)
		{
			Point trans = arrange_point - obj_origin_center;
			LOGINFO("trans = [%s]", trans.dump_perl().c_str());
			p_major->offset.translate(unscale(trans.x), unscale(trans.y));
			LOGINFO("arrange_object_slm p_major->offset = [%s]", p_major->offset.dump_perl().c_str());

			this->invalidate_bounding_box();
		}
		else
		{
			CONFESS("this object do not has object instance");
		}

		this->arranged = true;
		LOGINFO("arrange_object_slm_compact_new end, times =%d", GetTickCount() - timecount_start);

		return 1;
	}

	void
	ModelObject::build_object_voxels(double step, bool crossbar, bool support)
	{
		if (step != this->_step)
		{
			object_voxels.clear();
			this->_step = step;
		}

		if (object_voxels.empty() == false)
		{
			LOGINFO("ModelObject::build_object_voxels, object_voxels.empty() == false");
			return;
		}

		TriangleMesh currentMesh = this->meshWithSupport(crossbar, support);
		object_voxels = currentMesh.build_mesh_voxels(step);

		LOGINFO("ModelObject::build_object_voxels, object_voxels.size() = [%d]", object_voxels.size());
	}

	void
	ModelObject::clear_object_voxels()
	{
		LOGINFO("ModelObject::clear_object_voxels");
		object_voxels.clear();
		isCollision = false;
	}
	void
	ModelObject::clear_object_undo_vecs()
	{
		LOGINFO("~ModelObject::clear_object_undo_vecs-----------start");

		for (int i = 0; i < this->Undo_Vec.size(); i++)
		{
			this->temp_Delete_vec = this->Undo_Vec.at(i);
			for (int i = 0; i < this->temp_Delete_vec.size(); i++)
			{
				ModelVolume *this_vol = this->temp_Delete_vec.at(i);
				if (this_vol != nullptr)
				{
					delete this_vol;
					this_vol = nullptr;
				}
			}
			this->temp_Delete_vec.clear();
		}
		this->Undo_Vec.clear();
		LOGINFO("~ModelObject::clear  undo_vecs");
		for (int i = 0; i < this->Redo_Vec.size(); i++)
		{
			this->temp_Delete_vec = this->Redo_Vec.at(i);
			for (int i = 0; i < this->temp_Delete_vec.size(); i++)
			{
				ModelVolume *this_vol = this->temp_Delete_vec.at(i);
				if (this_vol != nullptr)
				{
					delete this_vol;
					this_vol = nullptr;
				}
			}
			this->temp_Delete_vec.clear();
		}

		this->Redo_Vec.clear();
		LOGINFO("~ModelObject::clear  redo_vecs");
		for (int i = 0; i < this->temp_Delete_vec.size(); i++)
		{
			ModelVolume *this_vol = this->temp_Delete_vec.at(i);
			if (this_vol != nullptr)
			{
				delete this_vol;
				this_vol = nullptr;
			}
		}
		this->temp_Delete_vec.clear();
		LOGINFO("~ModelObject::clear_object_undo_vecs-----------over");
	}
	size_t ModelObject::get_idx_in_model()
	{
		return this->model->get_object_idx(this);
	}

	void
	ModelObject::check_collision()
	{
		if (object_voxels.empty())
			return;

		// 不在盘上的不作检查
		if (this->IsAutoXXX == false)
			return;
		if (this->Get_bed_type() == "None")
			return;
		if (this->get_object_voxels().empty())
			return;

		size_t idx = this->get_idx_in_model();
		std::set<XYZ_Index> other_voxels = this->model->get_model_voxels_for_collision(idx);
		std::set<XYZ_Index> p_obj_voxels = this->get_object_voxels();
		std::set<XYZ_Index>::iterator it_c;
		bool result = false;
		LOGINFO("Model::check_collision, p_obj->get_object_voxels().size() = [%d]", this->get_object_voxels().size());

		for (it_c = p_obj_voxels.begin(); it_c != p_obj_voxels.end(); it_c++)
		{
			std::set<XYZ_Index>::iterator itr;
			itr = other_voxels.find(*it_c);

			if (itr != other_voxels.end())
			{
				result = true;
				LOGINFO("ModelObject::check_collision, find a collision voxels!!!!!!!!!!!!!!!");
				break;
			}
		}

		this->isCollision = result;
	}

	std::vector<size_t>
	Model::sort_arrange_for_complex_for_zhijia()
	{
		LOGINFO("start sort_arrange_by_boundingbox_for_zhijia");
		DWORD timecount = GetTickCount();

		// 挑出支架类型，上颌先排，下颌后排，其他最后
		std::vector<size_t> ipdjaw_idxs;
		std::vector<BoundingBoxf3> ipdjaw_polygons;

		std::vector<size_t> other_idxs;
		std::vector<BoundingBoxf3> other_polygons;

		int i = 0;
		for (ModelObjectPtrs::iterator it = objects.begin(); it != objects.end(); ++it)
		{
			ModelObject *p_obj = *it;
			if (p_obj->has_LatticeMerge_volume())
				continue;
			if (p_obj->instances.size() == 0)
				continue;

			LOGINFO("p_obj->bounding_box().size().x = [%f]",
					p_obj->instance_bounding_box(0).size().x);

			if (p_obj->IsIpdModel == false)
			{
				other_polygons.push_back(p_obj->bounding_box());
				other_idxs.push_back(it - objects.begin());
				continue;
			}

			if (p_obj->get_IpdRpdType() == 1 || p_obj->get_IpdRpdType() == 2 ||
				p_obj->get_IpdRpdType() == 3 || p_obj->get_IpdRpdType() == 4)
			{
				ipdjaw_polygons.push_back(p_obj->bounding_box());
				ipdjaw_idxs.push_back(it - objects.begin());
			}
			else
			{
				other_polygons.push_back(p_obj->bounding_box());
				other_idxs.push_back(it - objects.begin());
			}
		}
		LOGINFO("ipdjaw_polygons.size = [%d], other_polygons.size = [%d]",
				ipdjaw_polygons.size(), other_polygons.size());
		LOGINFO("ipdjaw_idxs.size = [%d], other_idxs.size = [%d]",
				ipdjaw_idxs.size(), other_idxs.size());

		std::vector<size_t> result;

		if (ipdjaw_polygons.empty() == false)
		{
			std::vector<size_t> ipd_vec = Slic3r::Geometry::sort_thumbnails_by_boundingbox_x(ipdjaw_polygons);
			for (int i = 0; i < ipd_vec.size(); i++)
			{
				result.push_back(ipdjaw_idxs.at(ipd_vec.at(i)));
			}
		}

		if (other_polygons.empty() == false)
		{
			std::vector<size_t> other_vec = Slic3r::Geometry::sort_thumbnails_by_boundingbox_x(other_polygons);
			for (int i = 0; i < other_vec.size(); i++)
			{
				result.push_back(other_idxs.at(other_vec.at(i)));
			}
		}

		LOGINFO("sort_arrange_by_boundingbox_for_zhijia times =%d", GetTickCount() - timecount);

		// 删除不参于自动排序的物体
		std::vector<size_t>::iterator retval_It = result.begin();
		while (retval_It != result.end())
		{
			if (this->objects.at(*retval_It)->IsAutoXXX)
				retval_It++;
			else
				retval_It = result.erase(retval_It);
		}
		return result;
	}

	std::vector<size_t>
	Model::sort_arrange_for_simplify_for_zhijia(double x_size)
	{
		LOGINFO("start sort_arrange_by_boundingbox_for_zhijia");
		DWORD timecount = GetTickCount();

		// 挑出支架类型，上颌先排，下颌后排，其他最后
		std::vector<size_t> upperjaw_idxs;
		Polygons upperjaw_polygons;
		std::vector<size_t> lowerjaw_idxs;
		Polygons lowerjaw_polygons;
		std::vector<size_t> other_idxs;
		Polygons other_polygons;
		std::vector<size_t> large_idxs;
		Polygons large_polygons;

		int i = 0;
		for (ModelObjectPtrs::iterator it = objects.begin(); it != objects.end(); ++it)
		{
			ModelObject *p_obj = *it;
			if (p_obj->has_LatticeMerge_volume())
				continue;
			if (p_obj->instances.size() == 0)
				continue;

			LOGINFO("p_obj->bounding_box().size().x = [%f], x_size = [%f]",
					p_obj->instance_bounding_box(0).size().x, x_size);

			if (p_obj->instance_bounding_box(0).size().x > x_size)
			{
				LOGINFO("p_obj->bounding_box().size().x > x_size");
				large_polygons.push_back(p_obj->get_boundingbox_thumbnail());
				large_idxs.push_back(it - objects.begin());
				continue;
			}

			if (p_obj->IsIpdModel == false)
			{
				other_polygons.push_back(p_obj->get_boundingbox_thumbnail());
				other_idxs.push_back(it - objects.begin());
				continue;
			}

			if (p_obj->get_IpdRpdType() == 1 || p_obj->get_IpdRpdType() == 2)
			{
				upperjaw_polygons.push_back(p_obj->get_boundingbox_thumbnail());
				upperjaw_idxs.push_back(it - objects.begin());
			}
			else if (p_obj->get_IpdRpdType() == 3 || p_obj->get_IpdRpdType() == 4)
			{
				lowerjaw_polygons.push_back(p_obj->get_boundingbox_thumbnail());
				lowerjaw_idxs.push_back(it - objects.begin());
			}
			else
			{
				other_polygons.push_back(p_obj->get_boundingbox_thumbnail());
				other_idxs.push_back(it - objects.begin());
			}
		}
		LOGINFO("upperjaw_polygons.size = [%d], lowerjaw_polygons.size = [%d], other_polygons.size = [%d], large_polygons.size = [%d]",
				upperjaw_polygons.size(), lowerjaw_polygons.size(), other_polygons.size(), large_polygons.size());
		LOGINFO("upperjaw_idxs.size = [%d], lowerjaw_idxs.size = [%d], other_idxs.size = [%d], large_idxs.size = [%d]",
				upperjaw_idxs.size(), lowerjaw_idxs.size(), other_idxs.size(), large_idxs.size());

		std::vector<size_t> result;

		if (upperjaw_polygons.empty() == false)
		{
			std::vector<size_t> upp_vec = Slic3r::Geometry::sort_thumbnails(upperjaw_polygons);
			for (int i = 0; i < upp_vec.size(); i++)
			{
				result.push_back(upperjaw_idxs.at(upp_vec.at(i)));
			}
		}

		if (lowerjaw_polygons.empty() == false)
		{
			std::vector<size_t> low_vec = Slic3r::Geometry::sort_thumbnails(lowerjaw_polygons);
			for (int i = 0; i < low_vec.size(); i++)
			{
				result.push_back(lowerjaw_idxs.at(low_vec.at(i)));
			}
		}

		if (other_polygons.empty() == false)
		{
			std::vector<size_t> other_vec = Slic3r::Geometry::sort_thumbnails(other_polygons);
			for (int i = 0; i < other_vec.size(); i++)
			{
				result.push_back(other_idxs.at(other_vec.at(i)));
			}
		}

		if (large_polygons.empty() == false)
		{
			// x方向长度大的从小到大排列
			std::vector<size_t> large_vec = Slic3r::Geometry::sort_thumbnails(large_polygons);
			for (int i = large_vec.size() - 1; i >= 0; i--)
			{
				result.push_back(large_idxs.at(large_vec.at(i)));
			}
		}
		LOGINFO("sort_arrange_by_boundingbox_for_zhijia times =%d", GetTickCount() - timecount);
		// 删除不参于自动排序的物体
		std::vector<size_t>::iterator retval_It = result.begin();
		while (retval_It != result.end())
		{
			if (this->objects.at(*retval_It)->IsAutoXXX)
				retval_It++;
			else
				retval_It = result.erase(retval_It);
		}
		return result;
	}

	std::vector<size_t>
	Model::sort_arrange_polygons()
	{
		LOGINFO("start sort_arrange_polygons");
		DWORD timecount = GetTickCount();

		Polygons objects_proj;
		objects_proj.clear();
		for (ModelObjectPtrs::iterator it = objects.begin(); it != objects.end(); ++it)
		{
			ModelObject *p_obj = *it;
			objects_proj.push_back(p_obj->get_boundingbox_thumbnail());
		}
		//
		std::vector<size_t> retval_vec = Slic3r::Geometry::sort_thumbnails(objects_proj);

		// 删除不参于自动排序的物体
		std::vector<size_t>::iterator retval_It = retval_vec.begin();
		while (retval_It != retval_vec.end())
		{
			if (this->objects.at(*retval_It)->IsAutoXXX)
				retval_It++;
			else
				retval_It = retval_vec.erase(retval_It);
		}

		LOGINFO("sort_arrange_polygons times =%d", GetTickCount() - timecount);

		return retval_vec;
	}
	struct arrange_z
	{
		size_t idx;
		coordf_t z;
	};
	bool _cmp_arrange_by_height(arrange_z z1, arrange_z z2) { return z1.z > z2.z; }

	std::vector<size_t>
	Model::sort_arrange_by_height()
	{
		LOGINFO("start sort_arrange_polygons by height");
		DWORD timecount = GetTickCount();

		std::vector<coordf_t> arrange_heights;
		arrange_heights.clear();
		for (ModelObjectPtrs::iterator it = objects.begin(); it != objects.end(); ++it)
		{
			ModelObject *p_obj = *it;
			arrange_heights.push_back(p_obj->bounding_box().max.z);
		}

		std::vector<size_t> retval_vec;
		retval_vec.clear();
		std::vector<arrange_z> objects_copy;
		objects_copy.clear();
		for (size_t i = 0; i < arrange_heights.size(); i++)
		{
			arrange_z temp;
			temp.idx = i;
			temp.z = arrange_heights.at(i);
			objects_copy.push_back(temp);
		}
		std::sort(objects_copy.begin(), objects_copy.end(), _cmp_arrange_by_height);

		for (auto temp : objects_copy)
		{
			retval_vec.push_back(temp.idx);
			LOGINFO("sorted %d vol's z is %lf\n", temp.idx, temp.z);
		}
		LOGINFO("sort_by_height get result times =%d", GetTickCount() - timecount);

		// 删除不参于自动排序的物体
		std::vector<size_t>::iterator retval_It = retval_vec.begin();
		while (retval_It != retval_vec.end())
		{
			if (this->objects.at(*retval_It)->IsAutoXXX)
				retval_It++;
			else
				retval_It = retval_vec.erase(retval_It);
		}

		LOGINFO("sort_arrange_by_height times =%d", GetTickCount() - timecount);

		return retval_vec;
	}
	/*  duplicate the entire model preserving instance relative positions */
	void
	Model::duplicate(size_t copies_num, coordf_t dist, const BoundingBoxf *bb)
	{
		Pointfs model_sizes(copies_num - 1, this->bounding_box().size());
		Pointfs positions;
		if (!this->_arrange(model_sizes, dist, bb, positions))
			CONFESS("Cannot duplicate part as the resulting objects would not fit on the print bed.\n");

		// note that this will leave the object count unaltered

		for (ModelObjectPtrs::const_iterator o = this->objects.begin(); o != this->objects.end(); ++o)
		{
			// make a copy of the pointers in order to avoid recursion when appending their copies
			ModelInstancePtrs instances = (*o)->instances;
			for (ModelInstancePtrs::const_iterator i = instances.begin(); i != instances.end(); ++i)
			{
				for (Pointfs::const_iterator pos = positions.begin(); pos != positions.end(); ++pos)
				{
					ModelInstance *instance = (*o)->add_instance(**i);
					instance->offset.translate(*pos);
				}
			}
			(*o)->invalidate_bounding_box();
		}
	}

	/*  this will append more instances to each object
		and then automatically rearrange everything */
	void
	Model::duplicate_objects(size_t copies_num, coordf_t dist, const BoundingBoxf *bb)
	{
		for (ModelObjectPtrs::const_iterator o = this->objects.begin(); o != this->objects.end(); ++o)
		{
			// make a copy of the pointers in order to avoid recursion when appending their copies
			ModelInstancePtrs instances = (*o)->instances;
			for (ModelInstancePtrs::const_iterator i = instances.begin(); i != instances.end(); ++i)
			{
				for (size_t k = 2; k <= copies_num; ++k)
					(*o)->add_instance(**i);
			}
		}

		this->arrange_objects(dist, bb);
	}

	void
	Model::duplicate_objects_grid(size_t x, size_t y, coordf_t dist)
	{
		if (this->objects.size() > 1)
			throw "Grid duplication is not supported with multiple objects";
		if (this->objects.empty())
			throw "No objects!";

		ModelObject *object = this->objects.front();
		object->clear_instances();

		Sizef3 size = object->bounding_box().size();

		for (size_t x_copy = 1; x_copy <= x; ++x_copy)
		{
			for (size_t y_copy = 1; y_copy <= y; ++y_copy)
			{
				ModelInstance *instance = object->add_instance();
				instance->offset.x = (size.x + dist) * (x_copy - 1);
				instance->offset.y = (size.y + dist) * (y_copy - 1);
			}
		}
	}

	// 打印Model中所有的ModelObject的信息
	void
	Model::print_info()
	{
		for (ModelObjectPtrs::const_iterator o = this->objects.begin(); o != this->objects.end(); ++o)
			(*o)->print_info();
	}

	void
	Model::copy_obj(int obj_idx, int row, double row_space, int column, double column_space, bool keep_original, Model *model)
	{
		if (obj_idx < 0 || obj_idx >= this->objects.size())
			return;

		if (row == 1 && column == 1)
			return;

		ModelObject *p_obj = this->objects.at(obj_idx);
		ModelInstance *p_obj_instance = p_obj->instances.at(0); // 默认1个object只有1个instance

		for (int j = 0; j < column; j++)
		{
			for (int i = 0; i < row; i++)
			{
				if (i == 0 && j == 0)
					continue;

				ModelObject *new_object = model->add_object(*p_obj);

				// 重命名
				std::ostringstream oss;
				std::string old_name = new_object->name;

				if (old_name.substr(old_name.size() - 4) == ".stl")
				{
					old_name.erase(old_name.end() - 4, old_name.end());
					oss << old_name << "_c(" << j * row + i << ").stl"; // 将文件类型后缀名挪到最后
				}
				else if (old_name.substr(old_name.size() - 4) == ".obj")
				{
					old_name.erase(old_name.end() - 4, old_name.end());
					oss << old_name << "_c(" << j * row + i << ").obj"; // 将文件类型后缀名挪到最后
				}
				else if (old_name.substr(old_name.size() - 4) == ".ipd")
				{
					old_name.erase(old_name.end() - 4, old_name.end());
					oss << old_name << "_c(" << j * row + i << ").ipd"; // 将文件类型后缀名挪到最后
				}
				else
					oss << old_name << "_c(" << j * row + i << ").stl"; // 将文件类型后缀名挪到最后

				new_object->name = oss.str();

				double y_offset = i * row_space;
				double x_offset = j * column_space;

				LOGINFO("copy_obj, x_offset = [%f], y_offset = [%f]", x_offset, y_offset);
				// 删除instance，在perl中load_model_object中添加
				if (new_object->instances.empty())
				{
					// ModelInstance* new_instance = new_object->add_instance(*p_obj_instance);
				}
				else
				{
					x_offset += new_object->instances[0]->offset.x;
					y_offset += new_object->instances[0]->offset.y;
					new_object->instances.clear();
				}

				new_object->translate(x_offset, y_offset, 0.0);
				new_object->apply_matrix_to_mesh();

				// ModelInstance* instance = new_object->instances.at(0);
				// instance->offset.translate(x_offset, y_offset);
			}
		}
	}

	void
	Model::clear_model_voxels()
	{
		for (ModelObjectPtrs::iterator it = objects.begin(); it != objects.end(); ++it)
		{
			ModelObject *p_obj = *it;
			p_obj->clear_object_voxels();
		}
	}

	void
	Model::build_model_voxels(double step, bool crossbar, bool support)
	{
		DWORD start = GetTickCount();
		if (objects.empty())
			return;

		for (ModelObjectPtrs::iterator it = objects.begin(); it != objects.end(); ++it)
		{
			ModelObject *p_obj = *it;
			p_obj->build_object_voxels(step, crossbar, support);
		}

		LOGINFO("Model::build_model_voxels times =%d", GetTickCount() - start);
	}

	std::set<XYZ_Index>
	Model::get_model_voxels_for_collision(size_t ignore_index)
	{
		DWORD start = GetTickCount();

		std::set<XYZ_Index> voxels_for_collision;
		voxels_for_collision.clear();
		for (ModelObjectPtrs::iterator it = objects.begin(); it != objects.end(); ++it)
		{
			if (it - objects.begin() == ignore_index)
			{
				LOGINFO("ignore_index = [%d], current_index = [%d], continue", ignore_index, it - objects.begin());
				continue;
			}

			ModelObject *p_obj = *it;
			if (p_obj->IsAutoXXX == false)
				continue;
			// LOGINFO("1");

			if (p_obj->Get_bed_type() == "None")
				continue;
			// LOGINFO("2");

			if (p_obj->get_object_voxels().empty())
				continue;
			// LOGINFO("3");
			std::set<XYZ_Index> p_obj_voxels = p_obj->get_object_voxels();
			// LOGINFO("p_obj_voxels.size() = [%d]", p_obj_voxels.size());
			voxels_for_collision.insert(p_obj_voxels.begin(), p_obj_voxels.end());
			// std::set<XYZ_Index>::iterator it_c;
			// for (it_c = p_obj->get_object_voxels().begin(); it_c != p_obj->get_object_voxels().end(); it_c++) {
			//	voxels_for_collision.insert(*it_c);
			// }
			// LOGINFO("4");
		}
		LOGINFO("Model::get_model_voxels_for_collision, voxels_for_collision.size() = [%d]", voxels_for_collision.size());
		LOGINFO("Model::get_model_voxels_for_collision times =%d, ignore_index = [%d]", GetTickCount() - start, ignore_index);

		return voxels_for_collision;
	}

	size_t
	Model::get_object_idx(ModelObject *obj)
	{
		if (objects.empty())
			return -1;

		ModelObjectPtrs::iterator iter = std::find(objects.begin(), objects.end(), obj); // 返回的是一个迭代器指针
		if (iter == objects.end())
		{
			return -1;
		}
		else
		{
			return std::distance(objects.begin(), iter);
		}
	}

	void
	Model::check_collision()
	{
		if (objects.empty())
			return;
		DWORD start = GetTickCount();

		for (ModelObjectPtrs::iterator it = objects.begin(); it != objects.end(); ++it)
		{
			DWORD start_1 = GetTickCount();

			size_t idx = it - objects.begin();
			ModelObject *p_obj = *it;

			// 不在盘上的不作检查
			if (p_obj->IsAutoXXX == false)
				continue;
			if (p_obj->Get_bed_type() == "None")
				continue;
			if (p_obj->get_object_voxels().empty())
				continue;

			std::set<XYZ_Index> other_voxels = this->get_model_voxels_for_collision(idx);
			std::set<XYZ_Index> p_obj_voxels = p_obj->get_object_voxels();
			std::set<XYZ_Index>::iterator it_c;
			bool result = false;
			LOGINFO("Model::check_collision, p_obj->get_object_voxels().size() = [%d]", p_obj->get_object_voxels().size());

			for (it_c = p_obj_voxels.begin(); it_c != p_obj_voxels.end(); it_c++)
			{
				std::set<XYZ_Index>::iterator itr;
				itr = other_voxels.find(*it_c);

				if (itr != other_voxels.end())
				{
					result = true;
					LOGINFO("Model::check_collision, find a collision voxels!!!!!!!!!!!!!!!");
					break;
				}
			}

			p_obj->isCollision = result;

			if (p_obj->isCollision == false)
			{
				LOGINFO("Model::check_collision, not collision?????????????????????????");
			}
			LOGINFO("Model::check_collision times =%d, idx = [%d]", GetTickCount() - start_1, idx);
		}

		LOGINFO("Model::check_collision times =%d", GetTickCount() - start);
	}

	bool
	Model::hash_facets_for_arrange_contain_value(XYZ_Index index)
	{
		if (hash_facets_for_arrange.empty())
			return false;

		// DWORD timecount = GetTickCount();
		std::set<XYZ_Index>::iterator itr;
		itr = hash_facets_for_arrange.find(index);
		// LOGINFO("hash_facets_for_arrange.size() = [%d],hash_facets_for_arrange_contain_value, times =%d", hash_facets_for_arrange.size(), GetTickCount() - timecount);

		return itr != hash_facets_for_arrange.end();

		// return hash_facets_for_arrange.count(index) != 0;
	}

	bool
	Model::hash_facets_for_arrange_contain_value(std::map<XYZ_Index, std::vector<int>> new_hash_facets)
	{
		// LOGINFO("hash_facets_for_arrange size = [%d], new_hash_facets size = [%d]", hash_facets_for_arrange.size(), new_hash_facets.size());
		if (hash_facets_for_arrange.empty())
			return false;

		if (new_hash_facets.empty())
			return false;

		std::map<XYZ_Index, std::vector<int>>::iterator iterA;
		for (iterA = new_hash_facets.begin(); iterA != new_hash_facets.end(); iterA++)
		{
			if (hash_facets_for_arrange_contain_value(iterA->first))
				return true;
		}

		return false;
	}

	void
	Model::update_hash_facets_for_arrange(std::set<XYZ_Index> new_hash_facets)
	{
		if (new_hash_facets.empty())
			return;

		std::set<XYZ_Index>::iterator iterA;
		for (iterA = new_hash_facets.begin(); iterA != new_hash_facets.end(); iterA++)
		{
			if (hash_facets_for_arrange_contain_value(*iterA) == false)
			{
				hash_facets_for_arrange.insert(*iterA);
			}
		}
	}

	void
	Model::update_hash_facets_for_arrange(std::map<XYZ_Index, std::vector<int>> new_hash_facets)
	{
		if (new_hash_facets.empty())
			return;

		for (std::map<XYZ_Index, std::vector<int>>::const_iterator iterA = new_hash_facets.begin(); iterA != new_hash_facets.end(); iterA++)
		{
			if (iterA->second.empty())
				continue;

			if (hash_facets_for_arrange_contain_value(iterA->first) == false)
			{
				hash_facets_for_arrange.insert(iterA->first);
			}
		}
	}

	void
	Model::add_group_instance()
	{
		if (this->objects.empty())
			return;

		BoundingBoxf3 bb = this->bounding_box();
		double z_min = bb.min.z;
		// 把boundingbox最小点对其零点
		Vectorf3 ToCenter_Vector(-bb.min.x, -bb.min.y, -bb.min.z);
		// 把boundingbox中心点对其零点
		Sizef3 size = bb.size();
		ToCenter_Vector.x -= size.x / 2;
		ToCenter_Vector.y -= size.y / 2;

		for (ModelObjectPtrs::const_iterator o = this->objects.begin(); o != this->objects.end(); ++o)
		{
			if ((*o)->instances.empty() == false)
				continue;

			// 移动mesh模型
			(*o)->translate(ToCenter_Vector);
			(*o)->origin_translation.translate(ToCenter_Vector);
			// LOGINFO("origin_translation %s ToCenter_Vector %s",
			//	origin_translation.dump_perl().c_str(),
			//	ToCenter_Vector.dump_perl().c_str());
			//  重载模型
			(*o)->apply_matrix_to_mesh();
			(*o)->reload_mesh_to_array();

			// 添加instance
			(*o)->add_instance();
		}
	}

	////////////////////////////////////////////////////////////////////////////////////
	//  ModelMaterial 类实现
	///////////////////////////////////////////////////////////////////////////////////

	ModelMaterial::ModelMaterial(Model *model) : model(model) {}
	ModelMaterial::ModelMaterial(Model *model, const ModelMaterial &other)
		: attributes(other.attributes), config(other.config), model(model)
	{
	}

	void
	ModelMaterial::apply(const t_model_material_attributes &attributes)
	{
		this->attributes.insert(attributes.begin(), attributes.end());
	}

	////////////////////////////////////////////////////////////////////////////////////
	//  ModelObject 类实现
	///////////////////////////////////////////////////////////////////////////////////

	// ModelObject的构造函数 父类指针赋值
	ModelObject::ModelObject(Model *model)
		: _bounding_box_valid(false), _raw_bounding_box_valid(false), model(model),
		  arranged(false), IsIpdModel(false), IsAutoXXX(true), 
		AddMergeLattice(false), _raw_thumbnails_valid(false), 
		group_id(0),object_num(-1)
	{
		verarray_p = new ModelVertexArray(this);
		map_sptvol.clear();
	}

	// 构造函数  用ModelObject的参数来构造
	ModelObject::ModelObject(Model *model, const ModelObject &other, bool copy_volumes)
		: name(other.name),
		  Medical_ID(other.Medical_ID),
		  input_file(other.input_file),
		  instances(),
		  volumes(),
		  config(other.config),
		  layer_height_ranges(other.layer_height_ranges),
		  origin_translation(other.origin_translation),
		  _bounding_box(other._bounding_box),
		  _bounding_box_valid(other._bounding_box_valid),
		  _raw_bounding_box(other._bounding_box),
		  _raw_bounding_box_valid(other._bounding_box_valid),
		  model(model),
		  arranged(other.arranged),
		  IsAutoXXX(other.IsAutoXXX),
		  IsIpdModel(other.IsIpdModel),
		  AddMergeLattice(other.AddMergeLattice),
		  _raw_thumbnails_valid(other._raw_thumbnails_valid),
		  can_split(other.can_split),
		  thumbnails(other.thumbnails),
		  num_positions(other.num_positions),
		  group_id(other.group_id),
		  bed_type(other.bed_type),
		  char_mark(other.char_mark),
		  object_num(-1)
	{
		// 新建ModelVertexArray
		verarray_p = new ModelVertexArray(this);
		map_sptvol.clear();

		// 拷贝volumes流程
		if (copy_volumes)
		{
			this->volumes.reserve(other.volumes.size());
			for (int i = 0; i < other.volumes.size(); i++)
			{
				ModelVolume *other_obj_volp = other.volumes.at(i);
				if (other_obj_volp->surpporter)
					continue; // 支撑体跳过 由被支撑体统一添加
				ModelVolume *new_obj_volp = this->add_volume(*other_obj_volp);
				if (other.map_sptvol.find(other_obj_volp) != other.map_sptvol.end())
				{
					ModelVolumePtrs obj_spts = other.map_sptvol.find(other_obj_volp)->second;
					for (int j = 0; j < obj_spts.size(); j++)
					{
						ModelVolume *other_spt_volp = obj_spts.at(j);
						ModelVolume *new_spt_volp = this->get_pvolume_spt(new_obj_volp, *other_spt_volp);
						this->add_pvolume_spt(new_spt_volp);
					}
				}
				else
				{
					LOGINFO("the obj volume has no spts!");
				}
			}
		}

		this->instances.reserve(other.instances.size());
		for (ModelInstancePtrs::const_iterator i = other.instances.begin(); i != other.instances.end(); ++i)
			this->add_instance(**i);
	}

	// 重载赋值函数
	ModelObject &ModelObject::operator=(ModelObject other)
	{
		this->swap(other);
		return *this;
	}

	//  拷贝所有参数
	void
	ModelObject::swap(ModelObject &other)
	{
		std::swap(this->input_file, other.input_file);
		std::swap(this->Medical_ID, other.Medical_ID);
		std::swap(this->instances, other.instances);
		std::swap(this->volumes, other.volumes);
		std::swap(this->config, other.config);
		std::swap(this->layer_height_ranges, other.layer_height_ranges);
		std::swap(this->origin_translation, other.origin_translation);
		std::swap(this->_bounding_box, other._bounding_box);
		std::swap(this->_bounding_box_valid, other._bounding_box_valid);
		std::swap(this->_raw_bounding_box, other._raw_bounding_box);
		std::swap(this->_raw_bounding_box_valid, other._raw_bounding_box_valid);
		std::swap(this->verarray_p, other.verarray_p);
		std::swap(this->map_sptvol, other.map_sptvol);
		std::swap(this->arranged, other.arranged);
		std::swap(this->IsAutoXXX, other.IsAutoXXX);
		std::swap(this->IsIpdModel, other.IsIpdModel);
		std::swap(this->AddMergeLattice, other.AddMergeLattice);
		std::swap(this->thumbnails, other.thumbnails);
		std::swap(this->num_positions, other.num_positions);
		std::swap(this->_raw_thumbnails_valid, other._raw_thumbnails_valid);
		std::swap(this->can_split, other.can_split);
		std::swap(this->bed_type, other.bed_type);
		this->object_num = -1;
	}

	// 析构函数
	ModelObject::~ModelObject()
	{
		this->clear_volumes();
		this->clear_instances();
		this->map_sptvol.clear();
		this->clear_object_voxels();
		this->clear_object_undo_vecs();

		if (verarray_p != NULL)
		{
			delete verarray_p;
			LOGINFO("Delete ModelVertexArray*");
			verarray_p = NULL;
		}
	}

	// 添加支撑接口
	ModelVolume *
	ModelObject::get_pvolume_spt(ModelVolume *_p_spted_vol, const ModelVolume &volume)
	{
		if (_p_spted_vol == NULL)
		{
			LOGINFO("get_pvolume_spt error _p_spted_vol == NULL");
			return NULL;
		}
		if (_p_spted_vol->surpporter)
		{
			LOGINFO("get_pvolume_spt error _p_spted_vol->surpporter == true");
			return NULL;
		}

		ModelVolume *v = new ModelVolume(this, volume);
		v->surpporter = true;
		v->p_spted_vol = _p_spted_vol;
		return v;
	}

	// 记录关系容器
	bool
	ModelObject::add_pvolume_map(ModelVolume *p_spt)
	{
		if (p_spt == NULL)
		{
			LOGINFO("p_spt == NULL");
			return false;
		}
		if (p_spt->p_spted_vol == NULL)
		{
			LOGINFO("p_spt->p_spted_vol == NULL");
			return false;
		}
		if (p_spt->surpporter == false)
		{
			LOGINFO("p_spt->surpporter == false");
			return false;
		}

		if (this->map_sptvol.find(p_spt->p_spted_vol) == this->map_sptvol.end())
		{
			ModelVolumePtrs objvol_spts;
			objvol_spts.push_back(p_spt);
			this->map_sptvol[p_spt->p_spted_vol] = objvol_spts;
			LOGINFO("Create volume spts, obj->[%s] support->[%s]",
					p_spt->p_spted_vol->name.c_str(),
					p_spt->name.c_str());
		}
		else
		{
			this->map_sptvol[p_spt->p_spted_vol].push_back(p_spt);
			LOGINFO("Add volume spts, obj->[%s] support->[%s] obj spts size->[%d]",
					p_spt->p_spted_vol->name.c_str(),
					p_spt->name.c_str(),
					this->map_sptvol[p_spt->p_spted_vol].size());
		}
		return true;
	}

	// 删除关系容器
	bool
	ModelObject::del_pvolume_map(ModelVolume *p_spt)
	{
		if (p_spt == NULL)
		{
			LOGINFO("p_spt == NULL");
			return false;
		}

		if (p_spt->surpporter) // 支撑
		{
			if (p_spt->p_spted_vol == NULL)
			{
				LOGINFO("p_spt->p_spted_vol == NULL");
				return false;
			}
			if (this->map_sptvol.find(p_spt->p_spted_vol) == this->map_sptvol.end())
			{
				LOGINFO("p_spt->p_spted_vol  not find in map");
				return false;
			}
			ModelVolumePtrs *p_vols = &(this->map_sptvol[p_spt->p_spted_vol]);
			for (int i = 0; i < p_vols->size(); i++)
			{
				if (p_vols->at(i) == p_spt)
				{
					ModelVolumePtrs::iterator volume_it = p_vols->begin() + i;
					p_vols->erase(volume_it);
					LOGINFO("Del one volume spt, obj->[%s] support->[%s] obj spts left size->[%d]",
							p_spt->p_spted_vol->name.c_str(),
							p_spt->name.c_str(),
							this->map_sptvol[p_spt->p_spted_vol].size());
					break;
				}
			}
		}
		else // 实体
		{
			if (this->map_sptvol.find(p_spt) == this->map_sptvol.end())
			{
				LOGINFO("p_spt->p_spted_vol  not find in map");
				return false;
			}
			this->map_sptvol.erase(p_spt);
			LOGINFO("Del volume spts, obj->[%s]",
					p_spt->name.c_str());
		}

		return true;
	}

	TriangleMesh
	ModelObject::get_raw_mesh_from_model(int i)
	{
		return this->model->objects.at(i)->raw_mesh();
	}

	TriangleMesh
	ModelObject::get_mesh_from_model(int i)
	{
		return this->model->objects.at(i)->mesh();
	}

	TriangleMesh
	ModelObject::get_meshWithCrossbar_from_model(int i)
	{
		return this->model->objects.at(i)->meshWithCrossbar();
	}

	TriangleMesh ModelObject::get_meshWithSupport_from_model(int i, bool crossbar, bool support)
	{
		return this->model->objects.at(i)->meshWithSupport(crossbar, support);
	}

	void
	ModelObject::invalid_thumbnails()
	{
		this->_raw_thumbnails_valid = false;
		this->thumbnails.clear();
		LOGINFO("invalid_thumbnails()");
	}

	void
	ModelObject::update_thumbnails()
	{
		LOGINFO("ModelObject::update_thumbnails() Begin.. [%s]", this->name.c_str());
		if (this->_raw_thumbnails_valid && this->thumbnails.size() != 0)
		{
			LOGINFO("no need to ModelObject::update_thumbnails()!");
			return;
		}
		this->thumbnails.clear();
		// 遍历所有的volumes
		for (int i = 0; i < this->volumes.size(); i++)
		{
			ModelVolume *p_vol = this->volumes.at(i);
			if (p_vol->modifier)
				continue;
			if (p_vol->surpporter)
				continue;

			p_vol->update_volume_thumbnails();
			this->thumbnails.insert(this->thumbnails.end(), p_vol->thumbnails.begin(), p_vol->thumbnails.end());
		}
		LOGINFO("ModelObject::update_thumbnails() this->thumbnails.size()=[%d]", this->thumbnails.size());

		this->thumbnails = union_ex(this->thumbnails);
		this->_raw_thumbnails_valid = true;
		LOGINFO("ModelObject::update_thumbnails() End.. [%s]", this->name.c_str());
	}

	void
	ModelObject::update_num_positions()
	{
		this->num_positions.clear();
		ExPolygons thumbnails_instance = this->get_model_thumbnails();
		for (auto temp : thumbnails_instance)
		{
			this->num_positions.push_back(temp.find_centroid_inside());
		}
		LOGINFO("ModelObject::update_num_positions()!");
	}

	int ModelObject::get_nospt_model_volume_num()
	{
		int num = 0;
		for (int i = 0; i < this->volumes.size(); i++)
		{
			ModelVolume *p_vol = this->volumes.at(i);
			if (p_vol->modifier)
				continue;
			if (p_vol->surpporter)
				continue;
			num++;
		}

		return num;
	}

	int ModelObject::get_first_object_volume_idx()
	{
		for (int i = 0; i < this->volumes.size(); i++)
		{
			ModelVolume *p_vol = this->volumes.at(i);
			if (p_vol->modifier)
				continue;
			if (p_vol->surpporter)
				continue;
			return i;
		}

		return -1;
	}

	Pointf3 ModelObject::get_lowest_point_by_volume_idx(int idx)
	{
		TriangleMesh m(this->volumes.at(idx)->mesh);
		ModelInstance *instance = this->instances.at(0);
		instance->transform_mesh(&m);

		return m.lowest_point();
	}

	int ModelObject::get_ipd_model_volume_num()
	{
		int num = 0;
		for (int i = 0; i < this->volumes.size(); i++)
		{
			ModelVolume *p_vol = this->volumes.at(i);
			if (p_vol->modifier)
				continue;
			if (p_vol->surpporter)
				continue;

			if (p_vol->IsIpdVolume)
			{
				LOGINFO("get_ipd_model_volume_num, find a ipd Volume!");
				num++;
			}
			else
				LOGINFO("get_ipd_model_volume_num, find a not ipd volume");
		}
		LOGINFO("get_ipd_model_volume_num = [%d]", num);
		return num;
	}

	// 2DPlater使用的投影算法
	ExPolygons
	ModelObject::get_model_thumbnails(bool is_raw)
	{
		LOGINFO("ModelObject::get_model_thumbnails begin！");

		this->update_thumbnails();
		ExPolygons _retval;
		// 取出主要的Instance
		ModelInstance *p_major = this->instances.at(0);
		if (p_major == NULL)
		{
			CONFESS("this object do not has object instance");
		}

		for (int i = 0; i < this->thumbnails.size(); i++)
		{
			ExPolygon _temp = this->thumbnails.at(i);
			if (is_raw == false)
			{
				_temp.rotate(p_major->rotation * PI / 180);
				_temp.translate(scale_(p_major->offset.x), scale_(p_major->offset.y));
			}
			_retval.push_back(_temp);
		}

		if (false)
		{
			SVG svg1("get_model_thumbnails.svg");
			svg1.draw(this->thumbnails, "red");
			svg1.draw(_retval, "green");
			svg1.Close();
		}

		return _retval;
	}

	Points
	ModelObject::get_num_positions()
	{
		LOGINFO("get_num_positions begin！");

		this->update_thumbnails();
		this->update_num_positions();

		if (this->num_positions.size() != this->thumbnails.size())
			CONFESS("模型投影数量和标记点数量不一致！");

		return this->num_positions;
	}

	// 添加支撑接口
	ModelVolume *
	ModelObject::get_pvolume_spt(ModelVolume *_p_spted_vol)
	{
		if (_p_spted_vol == NULL)
		{
			LOGINFO("get_pvolume_spt error _p_spted_vol == NULL");
			return NULL;
		}
		// b_cross_bar需要转接
		if (_p_spted_vol->b_cross_bar)
		{
			_p_spted_vol = _p_spted_vol->p_spted_vol;
		}
		if (_p_spted_vol->surpporter)
		{
			LOGINFO("get_pvolume_spt error _p_spted_vol->surpporter == true");
			return NULL;
		}

		TriangleMesh _mesh;
		ModelVolume *v = new ModelVolume(this, _mesh);
		v->surpporter = true;
		v->p_spted_vol = _p_spted_vol;
		return v;
	}

	bool
	ModelObject::add_pvolume_spt(ModelVolume *p_spt)
	{
		// 记录关系容器
		if (this->add_pvolume_map(p_spt) == false)
		{
			LOGINFO("add_pvolume_spt-->add_pvolume_map false");
			return false;
		}
		// 加入容器中
		this->volumes.push_back(p_spt);
		if (verarray_p)
			verarray_p->add_array(p_spt);
		this->invalidate_bounding_box();
		this->invalid_thumbnails();
		return true;
	}

	// 在ModelObject中添加volume 来源是TriangleMesh
	ModelVolume *
	ModelObject::add_volume(const TriangleMesh &mesh)
	{
		ModelVolume *new_v = new ModelVolume(this, mesh);
		this->add_pvolume_map(new_v);
		this->volumes.push_back(new_v);
		if (verarray_p)
			verarray_p->add_array(new_v);
		this->invalidate_bounding_box();
		this->invalid_thumbnails();
		return new_v;
	}

	// 在ModelObject中添加volume 来源是ModelVolume
	ModelVolume *
	ModelObject::add_volume(const ModelVolume &other)
	{
		ModelVolume *new_v = new ModelVolume(this, other);
		this->add_pvolume_map(new_v);
		this->volumes.push_back(new_v);
		if (verarray_p)
			verarray_p->add_array(new_v);
		this->invalidate_bounding_box();
		this->invalid_thumbnails();

		return new_v;
	}

	// 删除某个volume
	void
	ModelObject::delete_volume(size_t idx)
	{
		if (idx > this->volumes.size())
		{
			LOGINFO("idx > volumes.size()");
			return;
		}

		ModelVolumePtrs::iterator volume_it = this->volumes.begin() + idx;
		ModelVolume *p_volume = *volume_it;

		if (p_volume->SurpportType == "lattice")
			this->can_split = true;

		this->del_pvolume_map(p_volume);
		this->verarray_p->delete_array(p_volume);
		if (p_volume != nullptr)
		{
			delete p_volume;
			p_volume = nullptr;
		}
		this->volumes.erase(volume_it);
		this->invalidate_bounding_box();
		this->invalid_thumbnails();
	}

	bool
	ModelObject::delete_volume_by_volume(ModelVolume *p_volume)
	{
		if (p_volume == NULL)
		{
			LOGINFO("p_volume == NULL");
			return false;
		}
		for (int idx = 0; idx < this->volumes.size(); idx++)
		{
			if (this->volumes.at(idx) == p_volume)
			{
				this->delete_volume(idx);
				return true;
			}
		}
		LOGINFO("p_volume do not find");
		return false;
	}
	bool
	ModelObject::re_delete_volume(ModelVolume *p_volume)
	{
		for (int idx = 0; idx < this->volumes.size(); idx++)
		{
			if ((this->volumes.at(idx))->Isequal(p_volume))
			{
				this->delete_volume(idx);
				return true;
			}
		}
		LOGINFO("volume do not in the volumes");
		return false;
	}
	void
	ModelObject::delete_volume_sence_id(size_t sence_idx)
	{
		ModelVolume *p_volume = this->get_volume_by_selectid(sence_idx);
		this->delete_volume_by_volume(p_volume);
	}

	// 清空所有的volume
	void
	ModelObject::clear_volumes()
	{
		this->map_sptvol.clear();
		// 重置 ModelVertexArray
		if (verarray_p != NULL)
		{
			delete verarray_p;
			LOGINFO("Delete ModelVertexArray*");
			verarray_p = nullptr;
		}
		verarray_p = new ModelVertexArray(this);

		// int instead of size_t because it can be -1 when vector is empty
		// for (int i = this->volumes.size()-1; i >= 0; --i)
		//    this->delete_volume(i);
		for (int i = 0; i < this->volumes.size(); i++)
		{
			ModelVolume *this_vol = this->volumes.at(i);
			if (this_vol != nullptr)
			{
				delete this_vol;
				this_vol = nullptr;
			}		
		}
		this->volumes.clear();
	}

	void
	ModelObject::clear_spt_volumes(bool delete_crossbar)
	{
		this->map_sptvol.clear();
		if (verarray_p)
			verarray_p->clear_array_spt();

		ModelVolumePtrs::iterator volume_It = this->volumes.begin();
		while (volume_It != this->volumes.end())
		{
			ModelVolume *this_vol = *volume_It;
			if (this_vol->surpporter)
			{
				if (delete_crossbar == false && this_vol->b_cross_bar)
				{ // 横杆  重建索引
					this->add_pvolume_map(this_vol);
					if (verarray_p)
						verarray_p->add_array(this_vol);
					volume_It++;
				}
				else
				{
					volume_It = this->volumes.erase(volume_It);
					if(this_vol != nullptr)
					{
						delete this_vol;
						this_vol = nullptr;
					}			
				}
			}
			else
				volume_It++;
		}
		this->can_split = true;
		this->invalidate_bounding_box();
		LOGINFO("clear_spt_volumes end");
	}

	int ModelObject::get_sence_id(size_t idx)
	{
		if (idx < 0 || idx > this->volumes.size() - 1 || this->verarray_p == NULL)
			return -1;
		ModelVolume* p_volume = this->volumes.at(idx);
		return this->verarray_p->GetSelect_idx(p_volume);
		//return -1;
	}

	ModelVolume *ModelObject::get_volume_by_selectid(size_t select_idx)
	{
		if (this->verarray_p == NULL)
			return NULL;
		return this->verarray_p->GetVolume(select_idx);
		//return NULL;
	}

	std::vector<int> ModelObject::get_treeBranch_sence_ids(size_t select_idx)
	{
		std::vector<int> _ret;
		ModelVolume *treebase_volume_p = get_volume_by_selectid(select_idx);
		if (treebase_volume_p == NULL || treebase_volume_p->SurpportType != "treebase")
			return std::vector<int>();
		// 获取树支撑基点
		Pointf base_Point;
		base_Point.x = treebase_volume_p->upPickPos.x;
		base_Point.y = treebase_volume_p->upPickPos.y;
		//
		for (int i = 0; i < this->volumes.size(); i++)
		{
			ModelVolume *p_vol = this->volumes[i];
			if (p_vol == NULL)
				continue;
			Pointf dw_Point;
			dw_Point.x = p_vol->dwPickPos.x;
			dw_Point.y = p_vol->dwPickPos.y;
			if (dw_Point.distance_to(base_Point) < 0.001)
			{
				size_t branch_idx = this->get_sence_id(i);
				if (branch_idx != select_idx)
					_ret.push_back(branch_idx);
			}
		}
		return _ret;
	}

	int ModelObject::sence_volumes_count()
	{
		if (this->verarray_p == NULL)
			return -1;
		//delete by gaohui 20240725
		//return this->verarray_p->sence_volumes.size();
		return -1;
	}

	bool ModelObject::has_crossbar_volume()
	{
		if (this->verarray_p == NULL)
		{
			LOGINFO("ModelObject::has_crossbar_volume verarray_p == NULL");
			return false;
		}
		for (int i = 0; i < this->volumes.size(); i++)
		{
			if (this->volumes.at(i)->b_cross_bar)
			{
				return true;
			}
		}
		return false;
	}
	bool ModelObject::has_charmark_volume()
	{
		if (this->verarray_p == NULL)
		{
			LOGINFO("ModelObject::has_charmark_volume verarray_p == NULL");
			return false;
		}
		for (int i = 0; i < this->volumes.size(); i++)
		{
			if (this->volumes.at(i)->mark == true || this->volumes.at(i)->name.find("char_mark") != std::string::npos)
			{
				return true;
			}
		}
		return false;
	}
	bool ModelObject::has_lattice_volume()
	{
		if (this->verarray_p == NULL)
		{
			LOGINFO("ModelObject::has_lattice_volume verarray_p == NULL");
			return false;
		}
		for (int i = 0; i < this->volumes.size(); i++)
		{
			if (this->volumes.at(i)->lattice_support_ptr != NULL)
			{
				return true;
			}
		}
		return false;
	}

	bool ModelObject::has_LatticeMerge_volume()
	{
		if (this->verarray_p == NULL)
		{
			LOGINFO("ModelObject::has_LatticeMerge_volume verarray_p == NULL");
			return false;
		}
		for (int i = 0; i < this->volumes.size(); i++)
		{
			if (this->volumes.at(i)->Lattice_merged == true)
			{
				return true;
			}
		}
		return false;
	}

	bool ModelObject::has_AddMergeLattice() // 是否加了合并的晶格支撑
	{
		return AddMergeLattice;
	}

	bool ModelObject::has_spt_volume(bool exclude_crossbar, bool exclude_lattice)
	{
		if (this->verarray_p == NULL)
		{
			LOGINFO("ModelObject::has_spt_volume verarray_p == NULL");
			return false;
		}
		for (int i = 0; i < this->volumes.size(); i++)
		{
			if (this->volumes.at(i)->surpporter)
			{
				if (exclude_crossbar && // 排除横杆
					(this->volumes.at(i)->b_cross_bar || this->volumes.at(i)->b_crossbar_ball))
				{
					continue;
				}
				else if (exclude_lattice && // 排除晶格
						 this->volumes.at(i)->solider == false &&
						 this->volumes.at(i)->lattice_support_ptr != NULL)
				{
					continue;
				}
				else
				{
					return true;
				}
			}
		}
		return false;
	}

	// 添加instance
	ModelInstance *
	ModelObject::add_instance()
	{
		ModelInstance *i = new ModelInstance(this);
		this->instances.push_back(i);
		this->invalidate_bounding_box();
		this->invalid_thumbnails();

		return i;
	}

	// 添加instance
	ModelInstance *
	ModelObject::add_instance(const ModelInstance &other)
	{
		ModelInstance *i = new ModelInstance(this, other);
		this->instances.push_back(i);
		this->invalidate_bounding_box();
		this->invalid_thumbnails();

		return i;
	}

	// 删除某个instance
	void
	ModelObject::delete_instance(size_t idx)
	{
		ModelInstancePtrs::iterator i = this->instances.begin() + idx;
		if(*i != nullptr)
		{
			delete *i;
			*i = nullptr;
		}

		this->instances.erase(i);
		this->invalidate_bounding_box();
		this->invalid_thumbnails();
	}

	// 删除最后一个intance
	void
	ModelObject::delete_last_instance()
	{
		this->delete_instance(this->instances.size() - 1);
	}

	// 清空所有的instance
	void
	ModelObject::clear_instances()
	{
		for (size_t i = 0; i < this->instances.size(); ++i)
			this->delete_instance(i);
	}

	// 增加undo delete by zhangzheng

	void ModelObject::add_items_to_tempvec(size_t idx)
	{
		ModelVolume *tempVolume = new ModelVolume(this, *(this->get_volume_by_selectid(idx)));
		temp_Delete_vec.push_back(tempVolume);
		LOGINFO("add elements to tempvec\n");
		LOGINFO("temp_Delete_vec.size = %d", this->temp_Delete_vec.size());
	}
	void ModelObject::clear_vecs()
	{
		LOGINFO("ModelObject::clear_object_undo_vecs-----------start");
		for (int i = 0; i < this->Undo_Vec.size(); i++)
		{
			this->temp_Delete_vec = this->Undo_Vec.at(i);
			for (int i = 0; i < this->temp_Delete_vec.size(); i++)
			{
				ModelVolume *this_vol = this->temp_Delete_vec.at(i);
				if (this_vol != nullptr)
				{
					delete this_vol;
					this_vol = nullptr;
				}				
			}
			this->temp_Delete_vec.clear();
		}
		this->Undo_Vec.clear();
		LOGINFO("ModelObject::clear  undo_vecs");
		for (int i = 0; i < this->Redo_Vec.size(); i++)
		{
			this->temp_Delete_vec = this->Redo_Vec.at(i);
			for (int i = 0; i < this->temp_Delete_vec.size(); i++)
			{
				ModelVolume *this_vol = this->temp_Delete_vec.at(i);
				if (this_vol != nullptr)
				{
					delete this_vol;
					this_vol = nullptr;
				}					
			}
			this->temp_Delete_vec.clear();
		}

		this->Redo_Vec.clear();
		LOGINFO("ModelObject::clear redo_vecs");
		for (int i = 0; i < this->temp_Delete_vec.size(); i++)
		{
			ModelVolume *this_vol = this->temp_Delete_vec.at(i);
			if (this_vol != nullptr)
			{
				delete this_vol;
				this_vol = nullptr;
			}			
		}
		this->temp_Delete_vec.clear();
		LOGINFO("ModelObject::clear_object_undo_vecs-----------over");
	}
	void ModelObject::push_delete_items()
	{
		Redo_Vec.clear();
		Undo_Vec.push_back(temp_Delete_vec);
		LOGINFO("add tempvec to undovec\n");
		LOGINFO("Undo_Vec.size = %d\n", this->Undo_Vec.size());
		LOGINFO("Redo_Vec.size = %d\n", this->Redo_Vec.size());
		temp_Delete_vec.clear();
	}
	void ModelObject::pop_delete_items()
	{
		temp_Delete_vec.clear();
		temp_Delete_vec = Undo_Vec.back();
		Undo_Vec.pop_back();
	}
	void ModelObject::undo_delete()
	{
		LOGINFO("start undo\n");
		int dif = Undo_Vec.size() - this->init_length;
		LOGINFO("dif is  = %d\n", dif);
		if (dif > 0)
		{
			LOGINFO("start resize undo vec\n");
			Undo_Vec.erase(Undo_Vec.begin(), (Undo_Vec.begin() + dif));
			LOGINFO("resize undo vec ok\n");
		}

		if (Undo_Vec.size() > 0)
		{
			this->pop_delete_items();
			Redo_Vec.push_back(temp_Delete_vec);
			for (auto it : temp_Delete_vec)
			{
				this->add_volume(*it);
			}
			temp_Delete_vec.clear();
			LOGINFO("this.undovec.size = %d\n", this->Undo_Vec.size());
			LOGINFO("this.redovec.size = %d\n", this->Redo_Vec.size());
		}

		else
		{
			LOGINFO("the undo list is empty");
			return;
		}
	}
	void ModelObject::redo_delete()
	{
		LOGINFO("start redo \n");
		int dif = Redo_Vec.size() - this->init_length;
		if (dif > 0)
		{
			Redo_Vec.erase(Redo_Vec.begin(), (Redo_Vec.begin() + dif));
			LOGINFO("resize redo vec ok\n");
		}
		if (Redo_Vec.size() > 0)
		{
			temp_Delete_vec = Redo_Vec.back();
			Redo_Vec.pop_back();
			Undo_Vec.push_back(temp_Delete_vec);
			for (auto it : temp_Delete_vec)
			{
				this->re_delete_volume(it);
			}
			temp_Delete_vec.clear();
			LOGINFO("---this.undovec.size = %d\n", this->Undo_Vec.size());
			LOGINFO("---this.redovec.size = %d\n", this->Redo_Vec.size());
		}
		else
		{
			LOGINFO("the redo list is empty");
			return;
		}
	}

	// 返回BoundingBox
	// this returns the bounding box of the *transformed* instances
	BoundingBoxf3
	ModelObject::bounding_box()
	{
		if (!this->_bounding_box_valid)
			this->update_bounding_box();
		return this->_bounding_box;
	}

	// 将当前的boundbox状态置为无效
	void
	ModelObject::invalidate_bounding_box()
	{
		this->_raw_bounding_box_valid = false;
		this->_bounding_box_valid = false;
		// this->_raw_thumbnails_valid = false;
		// this->arranged = false;
		//// 遍历所有的volumes
		// for (int i = 0; i < this->volumes.size(); i++)
		//{
		//	ModelVolume* p_vol = this->volumes.at(i);
		//	if (p_vol->modifier) continue;
		//	if (p_vol->surpporter) continue;
		//	p_vol->_raw_thumbnails_valid = false;
		// }
		this->clear_object_voxels();
	}

	// 采用第一种变幻计算boundingbox
	BoundingBoxf3
	ModelObject::raw_bounding_box()
	{
		if (!this->_raw_bounding_box_valid)
			this->update_bounding_box();
		return this->_raw_bounding_box;
	}

	BoundingBoxf3
	ModelObject::raw_bounding_box_volume(size_t select_idx)
	{
		ModelVolume *p_volume = this->get_volume_by_selectid(select_idx);
		BoundingBoxf3 raw_bbox;
		if (p_volume)
			raw_bbox.merge(p_volume->mesh.bounding_box());

		return raw_bbox;
	}

	// 采用某种变幻计算boundingbox
	// this returns the bounding box of the *transformed* given instance
	BoundingBoxf3
	ModelObject::instance_bounding_box(size_t instance_idx) const
	{
		BoundingBoxf3 bb;
		for (ModelVolumePtrs::const_iterator v = this->volumes.begin(); v != this->volumes.end(); ++v)
		{
			if ((*v)->modifier)
				continue;
			bb.merge(this->instances[instance_idx]->transform_mesh_bounding_box(&(*v)->mesh, true));
		}
		return bb;
	}

	// 计算当前ModelOject的boundingbox
	void
	ModelObject::update_bounding_box()
	{
		LOGINFO("---update_bounding_box");
		//    this->_bounding_box = this->mesh().bounding_box();
		if (_raw_bounding_box_valid == false)
		{
			BoundingBoxf3 raw_bbox;
			for (ModelVolumePtrs::const_iterator v = this->volumes.begin(); v != this->volumes.end(); ++v)
			{
				if ((*v)->modifier)
					continue;
				if ((*v)->mesh.stl.stats.number_of_facets == 0)
				{
					continue;
				}
				raw_bbox.merge((*v)->mesh.bounding_box());
			}

			this->_raw_bounding_box = raw_bbox;
			this->_raw_bounding_box_valid = true;
		}

		if (_bounding_box_valid == false)
		{
			LOGINFO("_bounding_box_valid == false");
			BoundingBoxf3 bb;
			for (ModelInstancePtrs::const_iterator i = this->instances.begin(); i != this->instances.end(); ++i)
				bb.merge((*i)->transform_bounding_box(this->_raw_bounding_box));
			this->_bounding_box = bb;
			this->_bounding_box_valid = true;
		}
		LOGINFO("---this.ModelObject.bb = [%f,%f,%f]\n", this->_bounding_box.size().x, this->_bounding_box.size().y, this->_bounding_box.size().z);
	}

	BoundingBoxf3
	ModelObject::bounding_box_volume(size_t select_idx)
	{
		ModelVolume *p_volume = this->get_volume_by_selectid(select_idx);
		BoundingBoxf3 raw_bbox;
		if (p_volume)
			raw_bbox.merge(p_volume->mesh.bounding_box());

		BoundingBoxf3 bb;
		for (ModelInstancePtrs::const_iterator i = this->instances.begin(); i != this->instances.end(); ++i)
			bb.merge((*i)->transform_bounding_box(raw_bbox));

		return bb;
	}

	// 对包含的所有mesh进行修复
	void
	ModelObject::repair(bool auto_repair)
	{
		for (ModelVolumePtrs::const_iterator v = this->volumes.begin(); v != this->volumes.end(); ++v)
		{
			if ((*v)->solider)
				(*v)->mesh.repair(auto_repair);
		}
	}

	// 是否选择修复
	bool
	ModelObject::check_needed_repair() const
	{
		LOGINFO("this.volumes.size = %d", this->volumes.size());
		for (ModelVolumePtrs::const_iterator v = this->volumes.begin(); v != this->volumes.end(); ++v)
		{
			if ((*v)->modifier)
				continue;
			if ((*v)->mesh.checkonly)
			{
				LOGINFO("mesh.checkonly = true, volume name is %s", (*v)->name.c_str());
				continue;
			}
			else
			{
				LOGINFO("mesh.checkonly = false, volume name is %s", (*v)->name.c_str());
			}

			if ((*v)->mesh.check_needed_repair())
			{
				LOGINFO("volume name is %s", (*v)->name.c_str());
				return true;
			}
		}
		return false;
	}

	// 是否可以修复
	bool
	ModelObject::UnableRepair() const
	{
		for (ModelVolumePtrs::const_iterator v = this->volumes.begin(); v != this->volumes.end(); ++v)
		{
			if ((*v)->mesh.UnableRepair())
			{
				return true;
			}
		}

		return false;
	}

	// 将合并后的Mesh经过不同的变化再合并成一个
	// flattens all volumes and instances into a single mesh
	TriangleMesh
	ModelObject::mesh(bool _repair, bool _shared_vertices)
	{
		TriangleMesh _mesh;
		TriangleMesh _mesh_merge;
		_mesh = this->raw_mesh(_repair);
		for (ModelInstancePtrs::const_iterator i = this->instances.begin(); i != this->instances.end(); ++i)
		{
			TriangleMesh m(_mesh);
			(*i)->transform_mesh(&m);
			_mesh_merge.merge(m);
		}
		if (_repair)
			_mesh_merge.repair();

		if (_shared_vertices)
			_mesh_merge.require_shared_vertices();

		return _mesh_merge;
	}

	TriangleMesh
	ModelObject::meshWithSupport(bool crossbar, bool support, bool _repair)
	{
		TriangleMesh _mesh;
		TriangleMesh _mesh_merge;
		_mesh = this->raw_mesh(_repair);

		for (ModelVolumePtrs::const_iterator v = this->volumes.begin(); v != this->volumes.end(); ++v)
		{
			if ((*v)->surpporter)
			{
				if ((*v)->b_cross_bar)
				{
					if (crossbar)
					{
						_mesh.merge((*v)->mesh);
					}
				}
				else
				{
					if (support)
					{
						_mesh.merge((*v)->mesh);
					}
				}
			}
		}

		for (ModelInstancePtrs::const_iterator i = this->instances.begin(); i != this->instances.end(); ++i)
		{
			TriangleMesh m(_mesh);
			(*i)->transform_mesh(&m);
			_mesh_merge.merge(m);
		}
		if (_repair)
			_mesh_merge.repair();

		return _mesh_merge;
	}

	double
	ModelObject::get_surface_area(bool iSpt)
	{
		double area = 0.0;
		for (int i = 0; i < this->volumes.size(); i++)
		{
			ModelVolume *p_vol = this->volumes.at(i);

			if (p_vol->surpporter != iSpt)
			{
				continue;
			}

			double area_temp = p_vol->mesh.surface_area();
			if (area_temp > 0.0)
				area += area_temp;
		}

		return area;
	}

	TriangleMesh
	ModelObject::meshWithCrossbar(bool _repair)
	{
		TriangleMesh _mesh;
		TriangleMesh _mesh_merge;
		_mesh = this->raw_mesh(_repair);

		for (ModelVolumePtrs::const_iterator v = this->volumes.begin(); v != this->volumes.end(); ++v)
		{
			if ((*v)->b_cross_bar && (*v)->solider)
				_mesh.merge((*v)->mesh);
		}

		for (ModelInstancePtrs::const_iterator i = this->instances.begin(); i != this->instances.end(); ++i)
		{
			TriangleMesh m(_mesh);
			(*i)->transform_mesh(&m);
			_mesh_merge.merge(m);
		}
		if (_repair)
			_mesh_merge.repair();

		return _mesh_merge;
	}

	TriangleMesh
	ModelObject::support_mesh(bool _repair, bool _shared_vertices)
	{
		TriangleMesh _mesh;
		TriangleMesh _raw_mesh;
		for (ModelVolumePtrs::const_iterator v = this->volumes.begin(); v != this->volumes.end(); ++v)
		{
			if ((*v)->surpporter)
				_raw_mesh.merge((*v)->mesh);
			if ((*v)->b_cross_bar)
				_raw_mesh.merge((*v)->mesh);
		}

		for (ModelInstancePtrs::const_iterator i = this->instances.begin(); i != this->instances.end(); ++i)
		{
			TriangleMesh m(_raw_mesh);
			(*i)->transform_mesh(&m);
			_mesh.merge(m);
		}

		if (_repair)
			_mesh.repair();

		if (_shared_vertices)
			_mesh.require_shared_vertices();
		return _mesh;
	}

	// 将所有的Mesh合并成一个
	TriangleMesh
	ModelObject::raw_mesh(bool _repair)
	{
		TriangleMesh _raw_mesh;
		for (ModelVolumePtrs::const_iterator v = this->volumes.begin(); v != this->volumes.end(); ++v)
		{
			if ((*v)->modifier)
				continue;
			if ((*v)->surpporter)
				continue;
			_raw_mesh.merge((*v)->mesh);
		}
		if (_repair)
			_raw_mesh.repair();

		return _raw_mesh;
	}

	std::vector<TriangleMesh>
	ModelObject::crossbar_meshs()
	{
		std::vector<TriangleMesh> _ret;
		for (ModelVolumePtrs::const_iterator v = this->volumes.begin(); v != this->volumes.end(); ++v)
		{
			if ((*v)->b_cross_bar && (*v)->solider)
				_ret.push_back((*v)->mesh);
		}
		return _ret;
	}

	// 将所有的Mesh合并成一个
	TriangleMesh
	ModelObject::mesh_output(std::string _type, bool tra_instance, bool _repair)
	{
		TriangleMesh _mesh_output;
		for (ModelVolumePtrs::const_iterator v = this->volumes.begin(); v != this->volumes.end(); ++v)
		{
			if ((*v)->modifier)
				continue;
			if (_type == "All")
			{
				_mesh_output.merge((*v)->mesh);
			}
			else if (_type == "Obj" && (*v)->surpporter == false)
			{
				_mesh_output.merge((*v)->mesh);
			}
			else if (_type == "Spt")
			{
				if ((*v)->mark == true)
				{
					continue;
				}
				if ((*v)->surpporter == true && (*v)->solider == true && (*v)->b_cross_bar == false && (*v)->b_crossbar_ball == false)
				{
					_mesh_output.merge((*v)->mesh);
				}
			}
			else if (_type == "Fpt" && (*v)->surpporter == true && (*v)->solider == false)
			{
				_mesh_output.merge((*v)->mesh);
			}
		}
		if (tra_instance)
		{
			for (ModelInstancePtrs::const_iterator i = this->instances.begin(); i != this->instances.end(); ++i)
			{
				(*i)->transform_mesh(&_mesh_output);
			}
		}

		if (_repair)
		{
			_mesh_output.checkonly = true;
			_mesh_output.repair();
		}

		return _mesh_output;
	}

	bool
	ModelObject::write_object_stl(bool binary, std::string dir)
	{
		TriangleMesh Obj = this->mesh_output("Obj", false, true);
		std::string filename = "Obj.stl";
		boost::filesystem::path fullPath(dir);
		std::string fullFilePath = (fullPath / filename).string();//<boost::filesystem::utf8>
		IO::STL::write(Obj, fullFilePath, binary);
		return true;
	}

	bool
	ModelObject::write_spt_stl(bool binary, std::string dir)
	{
		TriangleMesh Spt = this->mesh_output("Spt", false, true);
		std::string filename = "Spt.stl";
		boost::filesystem::path fullPath(dir);
		std::string fullFilePath = (fullPath / filename).string();//<boost::filesystem::utf8>
		IO::STL::write(Spt, fullFilePath, binary);
		return true;
	}

	bool
	ModelObject::write_fpt_stl(bool binary, std::string dir)
	{
		TriangleMesh Fpt = this->mesh_output("Fpt", false, true);

		std::string filename = "Fpt.stl";
		boost::filesystem::path fullPath(dir);
		std::string fullFilePath = (fullPath / filename).string();//<boost::filesystem::utf8>
		IO::STL::write(Fpt, fullFilePath, binary);

		return true;
	}

	bool
	ModelObject::write_instance_xml(std::string dir)
	{
		ModelInstance *instance = this->instances.at(0);
		// xml节点
		boost::property_tree::ptree root;
		boost::property_tree::ptree MedicalIDs;
		boost::property_tree::ptree instanceNode;
		LOGINFO("that Medical_ID = [%s]", this->Medical_ID.c_str());
		MedicalIDs.add("MedicalID", this->Medical_ID);
		if (this->Medical_ID.empty() == true)
		{
			LOGINFO("fail to read the Medicalid");
		}
		instanceNode.add("Rotation", instance->rotation);
		instanceNode.add("OffsetX", instance->offset.x);
		instanceNode.add("OffsetY", instance->offset.y);
		instanceNode.add("ScaleFactor", instance->scaling_factor);

		root.add_child("MedicalIDs", MedicalIDs);
		root.add_child("Instance", instanceNode);

		boost::property_tree::xml_writer_settings<std::string> settings('\t', 1, "GB2312");
		std::string filename = "instance.xml";
		boost::filesystem::path fullPath(dir);
		std::string fullFilePath = (fullPath / filename).string();
		boost::property_tree::write_xml(fullFilePath, root, std::locale(), settings);
		return true;
	}

	bool ModelObject::write_cpt_xml(std::string dir)
	{
		if (this->has_crossbar_volume() == false)
			return false;

		// xml节点
		boost::property_tree::ptree root;
		boost::property_tree::ptree Crossbar;
		for (ModelVolumePtrs::const_iterator v = this->volumes.begin(); v != this->volumes.end(); ++v)
		{
			if ((*v)->modifier)
				continue;
			if ((*v)->surpporter == true && (*v)->solider == true && (*v)->b_cross_bar == true)
			{
				boost::property_tree::ptree cbNode;

				cbNode.add("SurpportType", (*v)->SurpportType);

				boost::property_tree::ptree upPickPos;
				Pointf3 up = (*v)->upPickPos;
				upPickPos.add("x", up.x);
				upPickPos.add("y", up.y);
				upPickPos.add("z", up.z);
				cbNode.add_child("upPickPos", upPickPos);

				boost::property_tree::ptree downPickPos;
				Pointf3 down = (*v)->dwPickPos;
				downPickPos.add("x", down.x);
				downPickPos.add("y", down.y);
				downPickPos.add("z", down.z);
				cbNode.add_child("downPickPos", downPickPos);

				boost::property_tree::ptree params;
				std::map<std::string, double> sp = (*v)->SurpportParams;
				params.add("cross_bar_radius", sp["cross_bar_radius"]);
				params.add("cross_bar_contact_width", sp["cross_bar_contact_width"]);
				params.add("cross_bar_contact_length", sp["cross_bar_contact_length"]);
				params.add("cross_bar_is_special", sp["cross_bar_is_special"]);
				params.add("assist_cross_bar_radius", sp["assist_cross_bar_radius"]);
				params.add("assist_cross_bar_contact_width", sp["assist_cross_bar_contact_width"]);
				params.add("assist_cross_bar_contact_length", sp["assist_cross_bar_contact_length"]);
				params.add("assist_cross_bar_is_special", sp["assist_cross_bar_is_special"]);
				cbNode.add_child("params", params);

				Crossbar.add_child("Cpt", cbNode);
			}
		}
		root.add_child("Crossbar", Crossbar);
		boost::property_tree::xml_writer_settings<std::string> settings('\t', 1, "GB2312");

		std::string filename = "cpt.xml";
		boost::filesystem::path fullPath(dir);
		std::string fullFilePath = (fullPath / filename).string();//<boost::filesystem::utf8>
		std::locale::global(std::locale(""));
		boost::property_tree::write_xml(fullFilePath, root, std::locale(), settings);

		return true;
	}

	bool
	ModelObject::read_cpt_xml(std::string cpt_xml)
	{
		LOGINFO("start read_cpt_xml");
		// 判断文件是否存在
		//  boost::filesystem::path path_file(instance_xml);
		//  if (!boost::filesystem::exists(path_file) || !boost::filesystem::is_regular_file(path_file))
		//	return false;

		std::string _input_file = cpt_xml;
		// std::string input_file_hex = ConverToHexString(_input_file);
		// LOGINFO("input_file_hex = %s", input_file_hex.c_str());

		std::wstring UTF8_input_file = Utf8ToUnicode(_input_file);
		OutputDebugStringW(UTF8_input_file.c_str());
		// std::wstring UTF8_input_file_hex = ConverToHexWString(UTF8_input_file);
		// OutputDebugStringW(UTF8_input_file_hex.c_str());

		std::string str;
		int len = WideCharToMultiByte(CP_ACP, 0, UTF8_input_file.c_str(), UTF8_input_file.size(), NULL, 0, NULL, NULL);
		char *buffer = new char[len + 1];
		WideCharToMultiByte(CP_ACP, 0, UTF8_input_file.c_str(), UTF8_input_file.size(), buffer, len, NULL, NULL);
		buffer[len] = '\0';
		str.append(buffer);
		delete[] buffer;

		LOGINFO("str = %s", str.c_str());
		if (boost::filesystem::exists(str) == true)
		{
			boost::property_tree::ptree pt;
			boost::property_tree::read_xml(str, pt);
			// 读取数据
			boost::property_tree::ptree crossbar = pt.get_child("Crossbar"); // 取一个子节点
			int i = 1;
			for (BOOST_AUTO(cpt, crossbar.begin()); cpt != crossbar.end(); ++cpt) // boost中的auto
			{
				std::string support_type = cpt->second.get<std::string>("SurpportType", "");
				double up_x = cpt->second.get<double>("upPickPos.x", 0.0);
				double up_y = cpt->second.get<double>("upPickPos.y", 0.0);
				double up_z = cpt->second.get<double>("upPickPos.z", 0.0);
				Pointf3 upPickPos = Pointf3(up_x, up_y, up_z);
				LOGINFO("read_cpt_xml, upPickPos = %s", upPickPos.dump_perl().c_str());

				double dw_x = cpt->second.get<double>("downPickPos.x", 0.0);
				double dw_y = cpt->second.get<double>("downPickPos.y", 0.0);
				double dw_z = cpt->second.get<double>("downPickPos.z", 0.0);
				Pointf3 downPickPos = Pointf3(dw_x, dw_y, dw_z);
				LOGINFO("read_cpt_xml, downPickPos = %s", downPickPos.dump_perl().c_str());

				double cross_bar_radius = cpt->second.get<double>("params.cross_bar_radius", 0.0);
				double cross_bar_contact_width = cpt->second.get<double>("params.cross_bar_contact_width", 0.0);
				double cross_bar_contact_length = cpt->second.get<double>("params.cross_bar_contact_length", 0.0);
				double cross_bar_is_special = cpt->second.get<double>("params.cross_bar_is_special", 0.0);
				double assist_cross_bar_radius = cpt->second.get<double>("params.assist_cross_bar_radius", 0.0);
				double assist_cross_bar_contact_width = cpt->second.get<double>("params.assist_cross_bar_contact_width", 0.0);
				double assist_cross_bar_contact_length = cpt->second.get<double>("params.assist_cross_bar_contact_length", 0.0);
				double assist_cross_bar_is_special = cpt->second.get<double>("params.assist_cross_bar_is_special", 0.0);
				LOGINFO("read_cpt_xml, cross_bar_radius = %f", cross_bar_radius);
				LOGINFO("read_cpt_xml, cross_bar_contact_width = %f", cross_bar_contact_width);
				LOGINFO("read_cpt_xml, cross_bar_contact_length = %f", cross_bar_contact_length);
				LOGINFO("read_cpt_xml, cross_bar_is_special = %f", cross_bar_is_special);
				LOGINFO("read_cpt_xml, assist_cross_bar_radius = %f", assist_cross_bar_radius);
				LOGINFO("read_cpt_xml, assist_cross_bar_contact_width = %f", assist_cross_bar_contact_width);
				LOGINFO("read_cpt_xml, assist_cross_bar_contact_length = %f", assist_cross_bar_contact_length);
				LOGINFO("read_cpt_xml, assist_cross_bar_is_special = %f", assist_cross_bar_is_special);

				double radius = 0.0;
				double contact_width = 0.0;
				double contact_length = 0.0;
				bool is_special = false;
				if (support_type == "crossbar" || support_type == "auto_crossbar")
				{
					radius = cross_bar_radius;
					contact_width = cross_bar_contact_width;
					contact_length = cross_bar_contact_length;
					is_special = cross_bar_is_special;
				}
				else if (support_type == "assist_crossbar" || support_type == "auto_assist_crossbar")
				{
					radius = assist_cross_bar_radius;
					contact_width = assist_cross_bar_contact_width;
					contact_length = assist_cross_bar_contact_length;
					is_special = assist_cross_bar_is_special;
				}
				CrossBarGenerator *crossbarMesh = new CrossBarGenerator(
					upPickPos,
					radius,
					contact_width,
					contact_length,
					is_special);
				TriangleMesh mesh_cb = crossbarMesh->Generate(downPickPos);
				mesh_cb.checkonly = false;

				ModelVolume *obj = this->volumes.at(0); // 默认第一个volume是物体volume
				ModelVolume *cb = this->get_pvolume_spt(obj);
				std::string volumename = "crossbar(" + boost::lexical_cast<std::string>(i) + ")";
				cb->name = volumename;
				cb->mesh = mesh_cb;
				cb->upPickPos = upPickPos;
				cb->dwPickPos = downPickPos;
				cb->solider = true;
				cb->surpporter = true;
				cb->b_cross_bar = true;
				cb->adder = !(is_special);
				cb->SurpportType = support_type;
				cb->SurpportParams["cross_bar_radius"] = cross_bar_radius;
				cb->SurpportParams["cross_bar_contact_width"] = cross_bar_contact_width;
				cb->SurpportParams["cross_bar_contact_length"] = cross_bar_contact_length;
				cb->SurpportParams["cross_bar_is_special"] = cross_bar_is_special;
				cb->SurpportParams["assist_cross_bar_radius"] = assist_cross_bar_radius;
				cb->SurpportParams["assist_cross_bar_contact_width"] = assist_cross_bar_contact_width;
				cb->SurpportParams["assist_cross_bar_contact_length"] = assist_cross_bar_contact_length;
				cb->SurpportParams["assist_cross_bar_is_special"] = assist_cross_bar_is_special;
				this->add_pvolume_spt(cb);
				i++;
			}
		}
		else
			LOGINFO("read_cpt_xml false, donnot exist cpt_file");

		return true;
	}

	bool ModelObject::write_mpt_xml(std::string dir)
	{
		if (this->has_charmark_volume() == false)
			return false;

		// xml节点
		boost::property_tree::ptree root;
		boost::property_tree::ptree CharMark;
		for (ModelVolumePtrs::const_iterator v = this->volumes.begin(); v != this->volumes.end(); ++v)
		{
			if (0)
			{
				std::map<std::string, double> sp = (*v)->SurpportParams;
				LOGINFO("write_mpt_xml , (*v)->name = [%s]", (*v)->name.c_str());
				LOGINFO("write_mpt_xml, (*v)->surpporter = %d", (*v)->surpporter);
				LOGINFO("write_mpt_xml, (*v)->solider = %d", (*v)->solider);
				LOGINFO("write_mpt_xml, (*v)->mark = %d", (*v)->mark);
				LOGINFO("write_mpt_xml , (*v)->SurpportType = [%s]", (*v)->SurpportType.c_str());
				LOGINFO("write_mpt_xml, (*v)->midPickPos = [%s]", (*v)->midPickPos.dump_perl().c_str());
				LOGINFO("write_mpt_xml, (*v)->MidNarmol = [%s]", (*v)->MidNarmol.dump_perl().c_str());
				LOGINFO("write_mpt_xml, auto_mark = %d", sp["automark"]);
				LOGINFO("write_mpt_xml, char_depth = %f", sp["char_depth"]);
				LOGINFO("write_mpt_xml, char_height = %f", sp["char_height"]);
			}
			if ((*v)->modifier)
				continue;
			if ((*v)->surpporter == true && (*v)->solider == true && (*v)->mark == true)
			{

				boost::property_tree::ptree mbNode;
				if ((*v)->SurpportType.find("char_mark") == std::string::npos)
				{
					LOGINFO("write_mpt_xml false, (*v)->SurpportType = [%s]", (*v)->SurpportType.c_str());
					continue;
				}

				mbNode.add("SurpportType", (*v)->SurpportType);
				boost::property_tree::ptree midPickPos;
				Pointf3 mid = (*v)->midPickPos;
				midPickPos.add("x", mid.x);
				midPickPos.add("y", mid.y);
				midPickPos.add("z", mid.z);
				mbNode.add_child("midPickPos", midPickPos);

				boost::property_tree::ptree midNormal;
				Pointf3 n = (*v)->MidNarmol;
				midNormal.add("x", n.x);
				midNormal.add("y", n.y);
				midNormal.add("z", n.z);
				mbNode.add_child("midNormal", midNormal);

				boost::property_tree::ptree params;
				std::map<std::string, double> sp = (*v)->SurpportParams;
				double automark = (*v)->automark ? 1.0 : 0.0;
				params.add("auto_mark", automark);
				params.add("char_mark", (*v)->CharMark);
				params.add("is_concave", sp["is_concave"]);
				params.add("char_size", sp["char_size"]);
				params.add("char_depth", sp["char_depth"]);
				params.add("char_height", sp["char_height"]);
				params.add("char_gap", sp["char_gap"]);
				params.add("Modify_Char_size", (*v)->CharSize);
				mbNode.add_child("params", params);

				CharMark.add_child("Mpt", mbNode);
			}
		}
		root.add_child("CharMark", CharMark);
		boost::property_tree::xml_writer_settings<std::string> settings('\t', 1, "GB2312");

		// 使用 / 运算符拼接路径
		std::string filename = "mpt.xml";
		boost::filesystem::path fullPath(dir);
		std::string fullFilePath = (fullPath / filename).string();//<boost::filesystem::utf8>
		std::locale::global(std::locale("")); 
		boost::property_tree::write_xml(fullFilePath, root, std::locale(), settings);

		return true;
	}

	bool
	ModelObject::read_mpt_xml(std::string mpt_xml)
	{
		LOGINFO("start read_mpt_xml");

		std::string _input_file = mpt_xml;
		std::wstring UTF8_input_file = Utf8ToUnicode(_input_file);
		OutputDebugStringW(UTF8_input_file.c_str());

		std::string str;
		int len = WideCharToMultiByte(CP_ACP, 0, UTF8_input_file.c_str(), UTF8_input_file.size(), NULL, 0, NULL, NULL);
		char *buffer = new char[len + 1];
		WideCharToMultiByte(CP_ACP, 0, UTF8_input_file.c_str(), UTF8_input_file.size(), buffer, len, NULL, NULL);
		buffer[len] = '\0';
		str.append(buffer);
		delete[] buffer;

		LOGINFO("str = %s", str.c_str());
		if (boost::filesystem::exists(str) == true)
		{
			boost::property_tree::ptree pt;
			boost::property_tree::read_xml(str, pt);
			// 读取数据
			boost::property_tree::ptree charmark = pt.get_child("CharMark"); // 取一个子节点
			int i = 1;
			for (BOOST_AUTO(cpt, charmark.begin()); cpt != charmark.end(); ++cpt) // boost中的auto
			{
				std::string support_type = cpt->second.get<std::string>("SurpportType", "");
				if (support_type.find("char_mark") == std::string::npos)
				{
					continue;
				}
				double mid_x = cpt->second.get<double>("midPickPos.x", 0.0);
				double mid_y = cpt->second.get<double>("midPickPos.y", 0.0);
				double mid_z = cpt->second.get<double>("midPickPos.z", 0.0);
				Pointf3 midPickPos = Pointf3(mid_x, mid_y, mid_z);
				LOGINFO("read_mpt_xml, midPickPos = %s", midPickPos.dump_perl().c_str());

				double n_x = cpt->second.get<double>("midNormal.x", 0.0);
				double n_y = cpt->second.get<double>("midNormal.y", 0.0);
				double n_z = cpt->second.get<double>("midNormal.z", 0.0);
				Pointf3 midNormal = Pointf3(n_x, n_y, n_z);
				LOGINFO("read_mpt_xml, midNormal = %s", midNormal.dump_perl().c_str());

				std::string str = cpt->second.get<std::string>("params.char_mark", "");
				bool automark = cpt->second.get<double>("params.auto_mark", 1.0) != 0.0;
				double is_concave = 0.0;
				double char_size = cpt->second.get<double>("params.char_size", 1.0);
				double char_depth = cpt->second.get<double>("params.char_depth", 0.0);
				double char_height = cpt->second.get<double>("params.char_height", 0.0);
				double char_gap = cpt->second.get<double>("params.char_gap", 0.0);
				double CharSize = cpt->second.get<double>("params.Modify_Char_size", 0.0);
				LOGINFO("read_mpt_xml, char_mark = %s", str.c_str());
				LOGINFO("read_mpt_xml, is_concave = %f", is_concave);
				LOGINFO("read_mpt_xml, char_size = %f", char_size);
				LOGINFO("read_mpt_xml, char_depth = %f", char_depth);
				LOGINFO("read_mpt_xml, char_height = %f", char_height);
				LOGINFO("read_mpt_xml, char_gap = %f", char_gap);
				LOGINFO("read_mpt_xml, Char_size = %f", CharSize);

				ModelVolume *obj = this->volumes.at(0); // 默认第一个volume是物体volume
				char_depth = 0;
				double size = (char_size - 1) / 5.0 + 0.5;
				Side_STL_CharMark_Mesh *pMark;
				if (automark)
				{
					pMark = new Side_STL_CharMark_Mesh(
						obj->mesh,
						str,
						char_height, // mark_thickness
						char_depth,	 // insert_thickness
						3.0,
						1.2,
						size,
						char_gap);
					LOGINFO("read_mpt_xml, auto generate!!!");
				}
				else
				{
					if (CharSize != 1.0)
					{
						pMark = new Side_STL_CharMark_Mesh(
							obj->mesh,
							midPickPos,
							midNormal,
							str,
							char_height, // mark_thickness
							char_depth,	 // insert_thickness
							3.0,
							1.2,
							CharSize / 3.5,
							char_gap);
					}
					else
					{
						pMark = new Side_STL_CharMark_Mesh(
							obj->mesh,
							midPickPos,
							midNormal,
							str,
							char_height, // mark_thickness
							char_depth,	 // insert_thickness
							3.0,
							1.2,
							size,
							char_gap);
					}
					LOGINFO("read_mpt_xml, not auto generate!!!");
				}

				TriangleMesh mesh_mb = pMark->GeneralMesh();
				mesh_mb.checkonly = true;
				std::string volumename = "char_mark(" + boost::lexical_cast<std::string>(i) + ")";
				LOGINFO("0615 read_mpt_xml, volumename is %s\n", volumename.c_str());
				ModelVolume *mb = this->get_pvolume_spt(obj);
				mb->mark = true;
				mb->name = volumename;
				mb->mesh = mesh_mb;
				mb->midPickPos = midPickPos;
				mb->MidNarmol = midNormal;
				mb->automark = automark;
				mb->solider = true;
				mb->surpporter = true;
				mb->concave = false;
				mb->adder = true;
				mb->SurpportType = support_type;
				mb->CharMark = str;
				mb->CharSize = CharSize;
				mb->SurpportParams["char_size"] = char_size;
				mb->SurpportParams["is_concave"] = false;
				mb->SurpportParams["char_depth"] = char_depth;
				mb->SurpportParams["char_height"] = char_height;
				mb->SurpportParams["char_gap"] = char_gap;
				this->add_pvolume_spt(mb);
				i++;
			}
		}
		else
			LOGINFO("read_mpt_xml false, donnot exist mpt_file");

		return true;
	}

	bool
	ModelObject::read_instance_xml(std::string instance_xml)
	{
		LOGINFO("start read_instance_xml");
		// 判断文件是否存在
		//  boost::filesystem::path path_file(instance_xml);
		//  if (!boost::filesystem::exists(path_file) || !boost::filesystem::is_regular_file(path_file))
		//	return false;

		std::string _input_file = instance_xml;
		// std::string input_file_hex = ConverToHexString(_input_file);
		// LOGINFO("input_file_hex = %s", input_file_hex.c_str());

		std::wstring UTF8_input_file = Utf8ToUnicode(_input_file);
		OutputDebugStringW(UTF8_input_file.c_str());
		// std::wstring UTF8_input_file_hex = ConverToHexWString(UTF8_input_file);
		// OutputDebugStringW(UTF8_input_file_hex.c_str());

		std::string str;
		int len = WideCharToMultiByte(CP_ACP, 0, UTF8_input_file.c_str(), UTF8_input_file.size(), NULL, 0, NULL, NULL);
		char *buffer = new char[len + 1];
		WideCharToMultiByte(CP_ACP, 0, UTF8_input_file.c_str(), UTF8_input_file.size(), buffer, len, NULL, NULL);
		buffer[len] = '\0';
		str.append(buffer);
		delete[] buffer;

		LOGINFO("str = %s", str.c_str());

		boost::property_tree::ptree pt;
		boost::property_tree::read_xml(str, pt);
		this->Medical_ID = pt.get<std::string>("MedicalIDs.MedicalID", "");
		LOGINFO("***The MedicalID = %s", this->Medical_ID.c_str());
		if (this->instances.size() > 0)
		{
			ModelInstance *p_major = this->instances.at(0);
			p_major->rotation = pt.get<double>("Instance.Rotation", 0.0);
			p_major->offset.x = pt.get<coordf_t>("Instance.OffsetX", 0.0);
			p_major->offset.y = pt.get<coordf_t>("Instance.OffsetY", 0.0);
			p_major->scaling_factor = pt.get<double>("Instance.ScaleFactor", 1.0);
		}
		else
		{
			ModelInstance *mi = this->add_instance();
			mi->offset.x = pt.get<coordf_t>("Instance.OffsetX", 0.0);
			mi->offset.y = pt.get<coordf_t>("Instance.OffsetY", 0.0);
			mi->rotation = pt.get<double>("Instance.Rotation", 0.0);
			mi->scaling_factor = pt.get<double>("Instance.ScaleFactor", 1.0);
		}

		LOGINFO("MedicalID is %s, rotation is %f, offset.x is %f, offset.y is %f, scaling_factor is %f!!!",
				this->Medical_ID.c_str(),
				this->instances.at(0)->rotation,
				this->instances.at(0)->offset.x,
				this->instances.at(0)->offset.y,
				this->instances.at(0)->scaling_factor);

		if (this->instances.at(0)->offset.x > 300)
		{
			this->instances.at(0)->offset.x = 300;
		}

		if (this->instances.at(0)->offset.x < -300)
		{
			this->instances.at(0)->offset.x = -300;
		}

		if (this->instances.at(0)->offset.y > 300)
		{
			this->instances.at(0)->offset.y = 300;
		}

		if (this->instances.at(0)->offset.y < -300)
		{
			this->instances.at(0)->offset.y = 300;
		}

		LOGINFO("After fix, MedicalID is %s, rotation is %f, offset.x is %f, offset.y is %f, scaling_factor is %f!!!",
				this->Medical_ID.c_str(),
				this->instances.at(0)->rotation,
				this->instances.at(0)->offset.x,
				this->instances.at(0)->offset.y,
				this->instances.at(0)->scaling_factor);

		return true;
	}

	// 将modelobject放置在盘上面
	void
	ModelObject::put_on_plater(bool only_bed_down)
	{
		if (this->has_spt_volume())
		{
			LOGINFO("ModelObject[%s] has_spt_volume, no put on plater", this->name.c_str());
			return;
		}
		if (this->has_AddMergeLattice())
		{
			LOGINFO("ModelObject[%s] has_AddMergeLattice, no put on plater", this->name.c_str());
			return;
		}

		// 计算出ModelObject的BoundBox
		BoundingBoxf3 bb;
		for (ModelVolumePtrs::const_iterator v = this->volumes.begin(); v != this->volumes.end(); ++v)
		{
			if ((*v)->modifier)
				continue;
			bb.merge((*v)->mesh.bounding_box());
		}

		LOGINFO("BoundingBox min = [%.3f, %.3f, %.3f]", bb.min.x, bb.min.y, bb.min.z);

		if (only_bed_down && bb.min.z >= 0)
			return;

		this->translate(0, 0, -bb.min.z);
		this->apply_matrix_to_mesh();
		this->update_bounding_box();
	}

	// 将modelobject几何中心移置原点
	Vectorf3
	ModelObject::center_around_origin(bool translate_instance, bool keep_z, bool reload)
	{
		// this->mesh().write_ascii("mesh0.stl");
		// this->raw_mesh().write_ascii("raw_mesh0.stl");

		this->apply_matrix_to_mesh();
		this->update_bounding_box();
		// this->mesh().write_ascii("mesh1.stl");
		// this->raw_mesh().write_ascii("raw_mesh1.stl");

		// 计算出ModelObject的BoundBox
		BoundingBoxf3 bb;
		for (ModelVolumePtrs::const_iterator v = this->volumes.begin(); v != this->volumes.end(); ++v)
		{
			if ((*v)->modifier)
				continue;
			bb.merge((*v)->mesh.bounding_box());
		}

		LOGINFO("bb min %s max %s", bb.min.dump_perl().c_str(), bb.max.dump_perl().c_str());

		// 把boundingbox最小点对其零点
		Vectorf3 ToCenter_Vector(-bb.min.x, -bb.min.y, -bb.min.z);
		// 把boundingbox中心点对其零点
		Sizef3 size = bb.size();
		ToCenter_Vector.x -= size.x / 2;
		ToCenter_Vector.y -= size.y / 2;
		// 移动mesh模型
		this->translate(ToCenter_Vector);
		this->origin_translation.translate(ToCenter_Vector);
		LOGINFO("origin_translation %s ToCenter_Vector %s",
				origin_translation.dump_perl().c_str(),
				ToCenter_Vector.dump_perl().c_str());
		// 重载模型
		this->apply_matrix_to_mesh();
		// this->mesh().write_ascii("mesh2.stl");
		// this->raw_mesh().write_ascii("raw_mesh2.stl");

		if (reload)
			this->reload_mesh_to_array();

		if (translate_instance)
		{
			for (ModelInstancePtrs::const_iterator instance_it = this->instances.begin(); instance_it != this->instances.end(); ++instance_it)
			{
				Vectorf3 ToCenter_Vector_ne = ToCenter_Vector.negative();
				LOGINFO("before Major instance offset = %s, ToCenter_Vector_ne = %s",
						(*instance_it)->offset.dump_perl().c_str(),
						ToCenter_Vector_ne.dump_perl().c_str());
				// ToCenter_Vector_ne.rotate((*instance_it)->rotation, (*instance_it)->offset);
				// ToCenter_Vector_ne.scale((*instance_it)->scaling_factor);
				(*instance_it)->offset.translate(ToCenter_Vector_ne.x, ToCenter_Vector_ne.y);
			}
		}
		this->update_bounding_box();

		if (keep_z)
		{
			this->translate(0, 0, bb.min.z);
			this->apply_matrix_to_mesh();
			// this->mesh().write_ascii("mesh3.stl");
			// this->raw_mesh().write_ascii("raw_mesh3.stl");

			ToCenter_Vector.z = 0.0;
		}

		return ToCenter_Vector;
	}

	// 平移
	void
	ModelObject::translate(const Vectorf3 &vector)
	{
		this->translate(vector.x, vector.y, vector.z);
	}

	// 平移所有mesh
	void
	ModelObject::translate(coordf_t x, coordf_t y, coordf_t z)
	{
		if (this->has_spt_volume())
		{
			LOGINFO("has SPT, can not translate by Z");
			z = 0;
		}

		LOGINFO("ModelObject::translate [x,y,z] = [%.3f, %.3f, %.3f]", x, y, z);

		if (this->_bounding_box_valid)
			this->_bounding_box.translate(x, y, z);
		if (this->verarray_p)
		{
			this->verarray_p->translate(x, y, z);
			LOGINFO("ModelObject::translate verarray_p changed");
		}
	}

	// 缩放
	void
	ModelObject::scale(float factor)
	{
		this->scale(Pointf3(factor, factor, factor));
	}

	// 缩放所有网格
	void
	ModelObject::scale(const Pointf3 &versor)
	{
		if (versor.x == 1 && versor.y == 1 && versor.z == 1)
			return;
		//for (ModelVolumePtrs::const_iterator v = this->volumes.begin(); v != this->volumes.end(); ++v) {
		//	(*v)->mesh.scale(versor);
		//}

		// reset origin translation since it doesn't make sense anymore
		this->origin_translation = Pointf3(0, 0, 0);
		this->invalidate_bounding_box();
		this->invalid_thumbnails();

		if (this->verarray_p)
			this->verarray_p->scale(versor.x, versor.y, versor.z);
	}

	// 将物体缩放到size大小 （保持比例）
	void
	ModelObject::scale_to_fit(const Sizef3 &size)
	{
		Sizef3 orig_size = this->bounding_box().size();
		float factor = fminf(
			size.x / orig_size.x,
			fminf(
				size.y / orig_size.y,
				size.z / orig_size.z));
		this->scale(factor);
	}

	// 围绕axis轴旋转angle角度
	void
	ModelObject::rotate(float angle, const Axis &axis)
	{
		DWORD dwStart1 = GetTickCount();
		if (angle == 0)
			return;
		if (axis != Axis::Z && this->has_spt_volume())
		{
			LOGINFO("has SPT, can not rotate by x or y axis");
			return;
		}
		// for (ModelVolumePtrs::const_iterator v = this->volumes.begin(); v != this->volumes.end(); ++v) {
		//     (*v)->mesh.rotate(angle, axis);
		// }
		this->origin_translation = Pointf3(0, 0, 0);
		this->invalidate_bounding_box();
		this->invalid_thumbnails();

		DWORD dwStart2 = GetTickCount();

		if (this->verarray_p)
			this->verarray_p->rotate(angle, axis);
		DWORD dwStart3 = GetTickCount();

		int a = (int)axis;
		LOGINFO("angle--[%f], axis--[%d], rotate mesh--[%d], matrix--[%d]", angle, a, dwStart2 - dwStart1, dwStart3 - dwStart2);
	}

	// 通过axis轴做镜像
	void
	ModelObject::mirror(const Axis &axis)
	{
		for (ModelVolumePtrs::const_iterator v = this->volumes.begin(); v != this->volumes.end(); ++v)
		{
			(*v)->mesh.mirror(axis);
		}
		this->origin_translation = Pointf3(0, 0, 0);
		this->invalidate_bounding_box();
		this->invalid_thumbnails();
	}

	void
	ModelObject::transbyinstance_fixpos(const ModelInstance &instance)
	{
		this->rotate(instance.rotation * PI / 180, Axis::Z);
		this->scale(instance.scaling_factor);
		this->translate(instance.offset.x, instance.offset.y, 0);

		for (ModelInstancePtrs::iterator i = this->instances.begin(); i != this->instances.end(); ++i)
		{
			(*i)->rotation -= instance.rotation;
			(*i)->scaling_factor /= instance.scaling_factor;
			(*i)->offset.translate(-instance.offset.x, -instance.offset.y);
		}
		this->origin_translation = Pointf3(0, 0, 0);
		this->invalidate_bounding_box();
	}

	bool
	ModelObject::apply_matrix_to_mesh()
	{
		this->invalidate_bounding_box();
		this->invalid_thumbnails();

		if (this->verarray_p == NULL)
		{
			LOGINFO("this->verarray_p == NULL");
			return false;
		}
		glm::mat4 dis_matrix = this->verarray_p->get_transfer_Matrix();

		//  单位矩阵不用同步
		if (ModelVertexArray::IsMatrixUnit(dis_matrix))
			return false;
		ModelVertexArray::Log_Matrix(dis_matrix);

		dis_matrix = glm::transpose(dis_matrix); // mesh的translate需要转置
		float *dis_matrix_p = (float *)glm::value_ptr(dis_matrix);
		DWORD dwStart1 = GetTickCount();
		for (ModelVolumePtrs::const_iterator v = this->volumes.begin(); v != this->volumes.end(); ++v)
		{
			ModelVolume *p_vol = *v;
			if (p_vol)
			{
				p_vol->mesh.transform(dis_matrix_p);
				p_vol->dwPickPos.translate(dis_matrix_p);
				p_vol->upPickPos.translate(dis_matrix_p);
				p_vol->invalid_volume_thumbnails();
			}
		}
		DWORD dwStart2 = GetTickCount();
		LOGINFO("mesh transform watse time--[%d]", dwStart2 - dwStart1);

		return true;
	}

	bool
	ModelObject::reload_mesh_to_array()
	{
		if (this->verarray_p == NULL)
			return false;

		DWORD dwStart2 = GetTickCount();
		this->verarray_p->reload_array();
		this->verarray_p->clear_matrix();
		DWORD dwStart3 = GetTickCount();
		LOGINFO("roload array watse time--[%d]",
			dwStart3 - dwStart2);

		return true;
	}

	// 恢复上一步操作
	bool
	ModelObject::recover_last_operate()
	{
		if (this->verarray_p == NULL)
			return false;
		this->verarray_p->recover_last_matrix();

		return true;
	}

	// 计算materials的种类
	size_t
	ModelObject::materials_count() const
	{
		std::set<t_model_material_id> material_ids;
		for (ModelVolumePtrs::const_iterator v = this->volumes.begin(); v != this->volumes.end(); ++v)
		{
			material_ids.insert((*v)->material_id());
		}
		return material_ids.size();
	}

	// 计算所有面片的数量
	size_t
	ModelObject::facets_count() const
	{
		size_t num = 0;
		for (ModelVolumePtrs::const_iterator v = this->volumes.begin(); v != this->volumes.end(); ++v)
		{
			if ((*v)->modifier)
				continue;
			num += (*v)->mesh.stl.stats.number_of_facets;
		}
		return num;
	}

	// 检查模型是否需要修复
	bool
	ModelObject::needed_repair() const
	{
		for (ModelVolumePtrs::const_iterator v = this->volumes.begin(); v != this->volumes.end(); ++v)
		{
			if ((*v)->modifier)
				continue;
			if ((*v)->mesh.needed_repair())
				return true;
		}
		return false;
	}

	// 按某个轴的某个平面来切割
	void
	ModelObject::cut(Axis axis, coordf_t z, Model *model) const
	{
		// clone this one to duplicate instances, materials etc.
		ModelObject *upper = model->add_object(*this);
		ModelObject *lower = model->add_object(*this);
		upper->clear_volumes();
		lower->clear_volumes();
		upper->input_file = "";
		lower->input_file = "";

		for (ModelVolumePtrs::const_iterator v = this->volumes.begin(); v != this->volumes.end(); ++v)
		{
			ModelVolume *volume = *v;
			if (volume->modifier)
			{
				// don't cut modifiers
				upper->add_volume(*volume);
				lower->add_volume(*volume);
			}
			else
			{
				TriangleMesh upper_mesh, lower_mesh;

				if (axis == X)
				{
					TriangleMeshSlicer<X>(&volume->mesh).cut(z, &upper_mesh, &lower_mesh);
				}
				else if (axis == Y)
				{
					TriangleMeshSlicer<Y>(&volume->mesh).cut(z, &upper_mesh, &lower_mesh);
				}
				else if (axis == Z)
				{
					TriangleMeshSlicer<Z>(&volume->mesh).cut(z, &upper_mesh, &lower_mesh);
				}

				upper_mesh.repair();
				lower_mesh.repair();
				upper_mesh.reset_repair_stats();
				lower_mesh.reset_repair_stats();

				if (upper_mesh.facets_count() > 0)
				{
					ModelVolume *vol = upper->add_volume(upper_mesh);
					vol->name = volume->name;
					vol->config = volume->config;
					vol->set_material(volume->material_id(), *volume->material());
				}
				if (lower_mesh.facets_count() > 0)
				{
					ModelVolume *vol = lower->add_volume(lower_mesh);
					vol->name = volume->name;
					vol->config = volume->config;
					vol->set_material(volume->material_id(), *volume->material());
				}
			}
		}
	}

	// 分割每个独立的个体
	void
	ModelObject::split_volumes()
	{
		if (this->has_spt_volume())
		{
			LOGINFO("split_volumes Error, has spt volumes, return");
			return;
		}

		// 获取分离后的meshs
		LOGINFO("before split volumes size = %d", this->volumes.size());
		DWORD dwstart = GetTickCount();
		// 每次取第一个进行拆分后删除，拆分成的物体加入末尾
		// 循环orgin_size次
		int volume_idx = 1; // volume 计数
		int orgin_size = this->volumes.size();
		ModelVolumePtrs delete_volumes;
		for (int idx = 0; idx < orgin_size; idx++)
		{
			ModelVolume *volume = this->volumes.at(idx);
			if (volume == NULL)
				continue;
			std::string name = volume->name;
			if (name.empty())
				name = this->name;

			// 只对obj进行拆分
			if (volume->surpporter == false)
			{
				LOGINFO("volume->mesh.shells_count() = [%d]!!", volume->mesh.shells_count());

				// volume mesh只有一个壳体，不拆分
				if (volume->mesh.shells_count() == 1)
					continue;

				delete_volumes.push_back(volume);
				TriangleMeshPtrs meshptrs = volume->mesh.split();

				for (int mesh_Idx = 0; mesh_Idx < meshptrs.size(); mesh_Idx++)
				{
					TriangleMesh *p_mesh = meshptrs.at(mesh_Idx);
					p_mesh->repair();
					if (p_mesh->UnableRepair())
					{
						LOGINFO("mesh_Idx[%d] UnableRepair()!!", mesh_Idx);
						continue;
					}
					LOGINFO("mesh_Idx[%d] EnableRepair()!!  facets_count[%d]", mesh_Idx, p_mesh->facets_count());
					ModelVolume *new_volume = this->add_volume(*p_mesh);
					if(p_mesh != nullptr)
					{
						delete p_mesh;
						p_mesh = nullptr;
					}
				
					new_volume->IsIpdVolume = volume->IsIpdVolume;
					new_volume->IpdRpdType = volume->IpdRpdType;
					new_volume->dentalCode = volume->dentalCode;
					new_volume->Lattice_merged = volume->Lattice_merged;
					new_volume->Rpd_Rotate_Angle_X = volume->Rpd_Rotate_Angle_X;
					new_volume->FrameX = volume->FrameX;
					new_volume->FrameY = volume->FrameY;
					new_volume->FrameZ = volume->FrameZ;

					std::stringstream new_name;
					std::string old_name = name;

					if (old_name.substr(old_name.size() - 4) == ".stl")
					{
						old_name.erase(old_name.end() - 4, old_name.end());
						new_name << old_name << "_v" << volume_idx << ".stl"; // 将文件类型后缀名挪到最后
						LOGINFO("split_volumes, volume[%d], name = [%s]", volume_idx, new_name.str().c_str());
					}
					else if (old_name.substr(old_name.size() - 4) == ".obj")
					{
						old_name.erase(old_name.end() - 4, old_name.end());
						new_name << old_name << "_v" << volume_idx << ".obj"; // 将文件类型后缀名挪到最后
						LOGINFO("split_volumes, volume[%d], name = [%s]", volume_idx, new_name.str().c_str());
					}
					else if (old_name.substr(old_name.size() - 4) == ".ipd")
					{
						old_name.erase(old_name.end() - 4, old_name.end());
						new_name << old_name << "_v" << volume_idx << ".ipd"; // 将文件类型后缀名挪到最后
						LOGINFO("split_volumes, volume[%d], name = [%s]", volume_idx, new_name.str().c_str());
					}
					else
						new_name << old_name << "_v" << volume_idx << ".stl"; // 将文件类型后缀名挪到最后

					volume_idx += 1; // 计数加1
					new_volume->name = new_name.str();
					new_volume->config = volume->config;
					new_volume->modifier = volume->modifier;
					new_volume->material_id(volume->material_id());

					LOGINFO("mesh_Idx[%d, %d], this->volumes.size() = %d",
							mesh_Idx, meshptrs.size(), this->volumes.size());
				}
			}
		}
		LOGINFO("volume->mesh.split() --> Cost Time [%d]", GetTickCount() - dwstart);
		LOGINFO("after split volumes size = %d", this->volumes.size());
		for (int i = 0; i < delete_volumes.size(); i++)
		{
			this->delete_volume_by_volume(delete_volumes[i]);
		}
		LOGINFO("after delete split volumes size = %d", this->volumes.size());
		return;
	}

	//  将model里面的modelobjects合并成为一个modelobject
	ModelObject *
	Model::Lattice_merge_obj(Model &other_model, std::string bed_type, double spt_Height)
	{
		// 创建新的ModelObject
		ModelObject *new_object = other_model.add_object();
		new_object->input_file = bed_type + "_LatticeMerged_object.stl";
		new_object->name = bed_type + "_LatticeMerged_object.stl";
		new_object->add_instance();			// 初始化new instance
		new_object->Set_bed_type(bed_type); // add by gao, 设置基台类型
		// 每个ModelObject
		for (size_t obj_idx = 0; obj_idx < this->objects.size(); obj_idx++)
		{
			ModelObject *p_obj = this->objects.at(obj_idx);
			ModelInstance *p_major_instance = p_obj->instances.at(0);

			// add by gao, 不是该基台类型的不合并
			if (p_obj->Get_bed_type() != bed_type)
			{
				LOGINFO("[%s] [%s]is not in the plater [%s]", p_obj->name.c_str(), p_obj->Get_bed_type().c_str(), bed_type.c_str());
				continue;
			}

			// 已带有晶格的物体不参与合并
			if (p_obj->has_lattice_volume())
			{
				LOGINFO("[%s] is has_lattice_volume,not lattice_merge", p_obj->name.c_str());
				continue;
			}

			// 没开启自动添加
			if (p_obj->IsAutoXXX == false)
			{
				LOGINFO("[%s] IsAutoXXX == false,not lattice_merge", p_obj->name.c_str());
				continue;
			}

			// 保存当前ModelInstance
			ModelInstance *p_major_instance_ne = p_major_instance->clone();
			p_major_instance_ne->reverse_intance();
			// 晶格合并前 对于没有支撑的物体需要抬高
			if (p_obj->has_spt_volume() == false && spt_Height > 0.001)
			{
				p_obj->translate(0, 0, spt_Height);
				p_obj->apply_matrix_to_mesh();
				// 重置打印状态和bbox
				p_obj->_print->add_model_object(p_obj, obj_idx);
				// p_obj->invalidate_bounding_box();
			}
			// 同步变换数据
			p_obj->transbyinstance_fixpos(*p_major_instance);
			p_obj->apply_matrix_to_mesh();
			// 每个ModelVolume
			for (size_t volume_idx = 0; volume_idx < p_obj->volumes.size(); volume_idx++)
			{
				ModelVolume *p_volume = p_obj->volumes.at(volume_idx);
				if (p_volume->surpporter || p_volume->modifier || p_volume->Lattice_merged)
					continue;												   // 跳过
				ModelVolume *p_volume_new = new_object->add_volume(*p_volume); // 加入实体volume
				p_volume_new->Lattice_merged = true;						   // 加上特殊标记，不会参与切片流程
				if (p_obj->map_sptvol.find(p_volume) != p_obj->map_sptvol.end())
				{
					LOGINFO("has spt obj!");
					ModelVolumePtrs obj_spts = p_obj->map_sptvol.find(p_volume)->second;
					for (size_t obj_spts_idx = 0; obj_spts_idx < obj_spts.size(); obj_spts_idx++)
					{
						ModelVolume *p_volume_spt = obj_spts.at(obj_spts_idx);
						ModelVolume *p_volume_spt_new = new_object->get_pvolume_spt(p_volume_new, *p_volume_spt);
						p_volume_spt_new->Lattice_merged = true; // 加上特殊标记，不会参与切片流程
						new_object->add_pvolume_spt(p_volume_spt_new);
					}
				}
				else
				{
					LOGINFO("has no spt obj!");
				}
			}
			// 恢复数据
			p_obj->transbyinstance_fixpos(*p_major_instance_ne);
			p_obj->apply_matrix_to_mesh();
			if (p_major_instance_ne != nullptr)
			{
				delete p_major_instance_ne;
				p_major_instance_ne = nullptr;
			}
				
			p_obj->AddMergeLattice = true; // 标记该物体已经添加合并晶格支撑
		}

		if (new_object->volumes.size() == 0)
		{
			LOGINFO("this model has no merge object");
			return NULL;
		}

		return new_object;
	}

	bool
	Model::has_LatticeMerge_object()
	{
		for (size_t obj_idx = 0; obj_idx < this->objects.size(); obj_idx++)
		{
			if (this->objects.at(obj_idx)->has_LatticeMerge_volume())
				return true;
		}
		return false;
	}

	//  将model里面的modelobjects合并成为一个modelobject
	ModelObject *
	Model::merge_obj(Model &other_model, std::string bed_type)
	{
		if (this->objects.size() == 1)
		{
			LOGINFO("this model has only one obj! can not merge");
			return NULL;
		}

		// 创建新的ModelObject
		ModelObject *new_object = other_model.add_object();
		new_object->input_file = bed_type + "_Merged_object.stl";
		new_object->name = bed_type + "_Merged_object.stl";
		new_object->add_instance();			// 初始化new instance
		new_object->Set_bed_type(bed_type); // add by gao, 设置基台类型
		// 每个ModelObject
		for (size_t obj_idx = 0; obj_idx < this->objects.size(); obj_idx++)
		{
			ModelObject *p_obj = this->objects.at(obj_idx);
			ModelInstance *p_major_instance = p_obj->instances.at(0);

			// add by gao, 不是该基台类型的不合并
			if (p_obj->Get_bed_type() != bed_type)
			{
				LOGINFO("[%s] [%s]is not in the plater [%s]", p_obj->name.c_str(), p_obj->Get_bed_type().c_str(), bed_type.c_str());
				continue;
			}

			// 保存当前ModelInstance
			ModelInstance *p_major_instance_ne = p_major_instance->clone();
			p_major_instance_ne->reverse_intance();
			// 同步变换数据
			p_obj->transbyinstance_fixpos(*p_major_instance);
			LOGINFO("Model::merge_obj apply_matrix_to_mesh 1");
			p_obj->apply_matrix_to_mesh();
			// 每个ModelVolume
			for (size_t volume_idx = 0; volume_idx < p_obj->volumes.size(); volume_idx++)
			{
				ModelVolume *p_volume = p_obj->volumes.at(volume_idx);
				if (p_volume->surpporter || p_volume->modifier || p_volume->Lattice_merged)
					continue;												   // 跳过
				ModelVolume *p_volume_new = new_object->add_volume(*p_volume); // 加入实体volume
				if (p_obj->map_sptvol.find(p_volume) != p_obj->map_sptvol.end())
				{
					LOGINFO("has spt obj!");
					ModelVolumePtrs obj_spts = p_obj->map_sptvol.find(p_volume)->second;
					for (size_t obj_spts_idx = 0; obj_spts_idx < obj_spts.size(); obj_spts_idx++)
					{
						ModelVolume *p_volume_spt = obj_spts.at(obj_spts_idx);
						ModelVolume *p_volume_spt_new = new_object->get_pvolume_spt(p_volume_new, *p_volume_spt);
						new_object->add_pvolume_spt(p_volume_spt_new);
					}
				}
				else
				{
					LOGINFO("has no spt obj!");
				}
			}
			// 恢复数据
			p_obj->transbyinstance_fixpos(*p_major_instance_ne);
			LOGINFO("Model::merge_obj apply_matrix_to_mesh 2");
			p_obj->apply_matrix_to_mesh();
			if (p_major_instance_ne != nullptr)
			{
				delete p_major_instance_ne;
				p_major_instance_ne = nullptr;
			}	
		}

		if (new_object->volumes.size() == 0)
		{
			LOGINFO("this model has no merge object");
			return NULL;
		}

		return new_object;
	}

	bool
	ModelObject::Is_In_plater(const Polygon &bed_shape, bool Is_boundary)
	{
		DWORD dwstart = GetTickCount();
		ExPolygons Model_shapes_ex = this->get_model_thumbnails();
		Polygons Model_shapes_contour;
		Model_shapes_contour.clear();
		for (int i = 0; i < Model_shapes_ex.size(); i++)
		{
			Model_shapes_contour.push_back(Model_shapes_ex.at(i).contour);
		}

		int Points_num = 0;
		for (int i = 0; i < Model_shapes_contour.size(); i++)
		{
			Points_num += Model_shapes_contour.at(i).points.size();
		}
		if (Points_num == 0)
		{
			LOGINFO("[%s] get_thumbnails is empty", this->name.c_str());
			return false;
		}
		BoundingBox bb = bed_shape.bounding_box();
		LOGINFO("bed_shape xmin = [%d], xmax = [%d], ymin = [%d], ymax = [%d]", bb.min.x, bb.max.x, bb.min.y, bb.max.y);

		Polygons Bed_shapes;
		Bed_shapes.push_back(bed_shape);

		Polygons interParts;
		bool retval;
		interParts.clear();
		if (Is_boundary)
		{
			interParts = intersection(Model_shapes_contour, Bed_shapes);
			retval = !(interParts.empty()); // 两个图案有相交部分
		}
		else
		{
			interParts = diff(Model_shapes_contour, Bed_shapes);
			retval = interParts.empty();
		}

		if (false)
		{
			char svg_name[255];
			sprintf(svg_name, "Is_In_plater[%s].svg", this->name.c_str());
			SVG svg1(svg_name);
			svg1.draw(Bed_shapes, "green");
			svg1.draw(Model_shapes_contour, "red");
			svg1.draw(interParts, "black");
			svg1.Close();
		}

		return retval;
	}

	void
	ModelObject::reload_obj_facet_color()
	{
		if (this->verarray_p == NULL)
			return;
		this->verarray_p->reload_obj_facet_color();
	}

	void
	ModelObject::reload_rpd_facet_color()
	{
		if (this->verarray_p == NULL)
			return;
		this->verarray_p->reload_rpd_facet_color();
	}

	// 将本ModelObject中的volume拆成一个个独立的个体
	bool
	ModelObject::split_obj(Model &other_model, ModelObjectPtrs *new_objects)
	{
		if (this->can_split == false) // 整体加上晶格支撑后，不能再拆分
			return false;

		// this->split_volumes();

		// volume为1不用拆分
		if (this->volumes.size() == 1)
		{
			LOGINFO("this ModelObject has only one volume! can not split");
			return false;
		}

		// 同步变换数据 将主instance中的rotation同步到mesh中
		ModelInstance *p_major_instance = this->instances.at(0);
		this->transbyinstance_fixpos(*p_major_instance);
		this->apply_matrix_to_mesh();

		LOGINFO("this->map_sptvol size = %d", this->map_sptvol.size());
		for (size_t i = 0; i < this->volumes.size(); i++)
		{
			ModelVolume *p_volume = this->volumes.at(i);

			if (p_volume->IsIpdVolume)
				LOGINFO("ModelObject::split_obj p_volume->IsIpdVolume is true");

			if (p_volume->surpporter)
				continue; // 支撑体跳过
			// 创建新的ModelObject
			ModelObject *new_object = other_model.add_object(*this, false);
			std::ostringstream oss;
			std::string old_inputfile = new_object->input_file;
			std::string old_name = p_volume->name;

			if (old_name.substr(old_name.size() - 4) == ".stl")
			{
				old_name.erase(old_name.end() - 4, old_name.end());
				oss << old_name << "_s" << i << ".stl"; // 将文件类型后缀名挪到最后
				LOGINFO("split_obj, volume[%d], name = [%s]", i, oss.str().c_str());
			}
			else if (old_name.substr(old_name.size() - 4) == ".obj")
			{
				old_name.erase(old_name.end() - 4, old_name.end());
				oss << old_name << "_s" << i << ".obj"; // 将文件类型后缀名挪到最后
				LOGINFO("split_obj, volume[%d], name = [%s]", i, oss.str().c_str());
			}
			else if (old_name.substr(old_name.size() - 4) == ".ipd")
			{
				old_name.erase(old_name.end() - 4, old_name.end());
				oss << old_name << "_s" << i << ".ipd"; // 将文件类型后缀名挪到最后
				LOGINFO("split_obj, volume[%d], name = [%s]", i, oss.str().c_str());
			}
			else
				oss << old_name << "_s" << i << ".stl"; // 将文件类型后缀名挪到最后

			new_object->input_file = old_inputfile;
			new_object->name = oss.str();
			new_object->Set_bed_type(this->Get_bed_type()); // add by gao, 设置基台类型
			new_object->IsIpdModel = p_volume->IsIpdVolume;

			// 加入实体volume
			ModelVolume *p_volume_new = new_object->add_volume(*p_volume);
			if (this->map_sptvol.find(p_volume) != this->map_sptvol.end())
			{
				LOGINFO("has spt obj!");
				ModelVolumePtrs obj_spts = this->map_sptvol.find(p_volume)->second;
				for (size_t j = 0; j < obj_spts.size(); j++)
				{
					ModelVolume *p_volume_spt = obj_spts.at(j);
					ModelVolume *p_volume_spt_new = new_object->get_pvolume_spt(p_volume_new, *p_volume_spt);
					new_object->add_pvolume_spt(p_volume_spt_new);
				}
			}
			else
			{
				LOGINFO("has no spt obj!");
			}

			new_object->center_around_origin(true);
			new_objects->push_back(new_object);
		}

		return true;
	}

	// 获取modelobject的编号
	size_t
	ModelObject::Get_modelobject_Index()
	{
		if (this->model == NULL)
			return 0;

		for (size_t i = 0; i < this->model->objects.size(); i++)
		{
			if (this->model->objects.at(i) == this)
				return i + 1;
		}

		return 0;
	}

	// 打印模型信息
	void
	ModelObject::print_info()
	{
		using namespace std;
		cout << fixed;
		cout << "[" << this->name << "]" << endl;

		TriangleMesh mesh = this->raw_mesh();
		mesh.check_topology();
		BoundingBoxf3 bb = mesh.bounding_box();
		Sizef3 size = bb.size();
		cout << "size_x = " << size.x << endl;
		cout << "size_y = " << size.y << endl;
		cout << "size_z = " << size.z << endl;
		cout << "min_x = " << bb.min.x << endl;
		cout << "min_y = " << bb.min.y << endl;
		cout << "min_z = " << bb.min.z << endl;
		cout << "max_x = " << bb.max.x << endl;
		cout << "max_y = " << bb.max.y << endl;
		cout << "max_z = " << bb.max.z << endl;
		cout << "number_of_facets = " << mesh.stl.stats.number_of_facets << endl;
		cout << "manifold = " << (mesh.is_manifold() ? "yes" : "no") << endl;

		mesh.repair(); // this calculates number_of_parts
		if (mesh.needed_repair())
		{
			mesh.repair();
			if (mesh.stl.stats.degenerate_facets > 0)
				cout << "degenerate_facets = " << mesh.stl.stats.degenerate_facets << endl;
			if (mesh.stl.stats.edges_fixed > 0)
				cout << "edges_fixed = " << mesh.stl.stats.edges_fixed << endl;
			if (mesh.stl.stats.facets_removed > 0)
				cout << "facets_removed = " << mesh.stl.stats.facets_removed << endl;
			if (mesh.stl.stats.facets_added > 0)
				cout << "facets_added = " << mesh.stl.stats.facets_added << endl;
			if (mesh.stl.stats.facets_reversed > 0)
				cout << "facets_reversed = " << mesh.stl.stats.facets_reversed << endl;
			if (mesh.stl.stats.backwards_edges > 0)
				cout << "backwards_edges = " << mesh.stl.stats.backwards_edges << endl;
		}
		cout << "number_of_parts =  " << mesh.stl.stats.number_of_parts << endl;
		cout << "volume = " << mesh.volume() << endl;
	}

	void ModelObject::RefreshFaceColorByPos(size_t select_idx, Pointf3 pickPos, bool IsRemove)
	{
		ModelVolume *p_vol = this->get_volume_by_selectid(select_idx);
		if (p_vol)
		{
			std::vector<int> fact_vec;
			if (IsRemove)
				fact_vec = p_vol->mesh.RemoveFacts_SelectedPoint(pickPos);
			else
				fact_vec = p_vol->mesh.MarkFacts_SelectedPoint(pickPos);

			//////////////////////////////////////////////////////////////////////////
			// delete by gaohui 20240725
			//for (int i = 0; i < fact_vec.size(); i++)
			//{
			//	int fact_id = fact_vec[i];
			//	this->verarray_p->vercol_obj.modify_volume_factet_color(select_idx, fact_id, p_vol->mesh.GetFacetExtra(fact_id));
			//}
		}
	}

	/*激活种植孔内不加支撑功能时，选择种植孔上下面约束面片*/
	void ModelObject::GetNoSupportHoleLimitFaceColorByPos(size_t select_idx, Pointf3 pickPos, bool IsRemove)
	{
		ModelVolume *p_vol = this->get_volume_by_selectid(select_idx);
		if (p_vol)
		{
			std::vector<int> fact_vec;
			if (IsRemove)
			{
				// fact_vec = p_vol->mesh.RemoveFacts_SelectedPoint(pickPos);
			}
			else
			{
				fact_vec = p_vol->mesh.HoleFaceMarkFacts_SelectedPoint(pickPos);
			}

			for (int i = 0; i < fact_vec.size(); i++)
			{
				int fact_id = fact_vec[i];
				//////////////////////////////////////////////////////////////////////////
				// delete by gaohui 20240725
				//this->verarray_p->vercol_obj.modify_volume_factet_color_confiningfiled(select_idx, fact_id, p_vol->mesh.GetFacetExtra(fact_id));
				p_vol->mesh.NoSupportLimitFaces.insert(fact_id);
			}
		}
	}

	/*根据涂色选择种植孔内部不加支撑的区域*/
	void ModelObject::RefreshNoSupportFaceColorByPos(size_t select_idx, Pointf3 pickPos, bool IsRemove)
	{
		ModelVolume *p_vol = this->get_volume_by_selectid(select_idx);
		if (p_vol)
		{
			std::vector<int> fact_vec;
			if (IsRemove)
			{
				// fact_vec = p_vol->mesh.RemoveHole_SelectedPoint(pickPos);
			}
			else
			{
				fact_vec = p_vol->mesh.MarkHole_SelectedPoint(pickPos, p_vol->mesh.NoSupportLimitFaces);
			}

			for (int i = 0; i < fact_vec.size(); i++)
			{
				int fact_id = fact_vec[i];
				//////////////////////////////////////////////////////////////////////////
				// delete by gaohui 20240725
				//this->verarray_p->vercol_obj.modify_volume_factet_color(select_idx, fact_id, p_vol->mesh.GetFacetExtra(fact_id));
				p_vol->mesh.NoSupportFaces.insert(fact_id);
			}
			std::cout << "p_vol->mesh.NoSupportFaces.size()" << p_vol->mesh.NoSupportFaces.size() << std::endl;
		}
	}

	void ModelObject::CancelNoSupportArea()
	{
		for (int i = 0; i < this->volumes.size(); i++)
		{
			ModelVolume *this_vol = this->volumes.at(i);
			if (this_vol)
			{
				this_vol->mesh.NoSupportFaces.clear();
				this_vol->mesh.NoSupportLimitFaces.clear();
			}
		}
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//   ModelVertexArray类的定义
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	ModelVertexArray::ModelVertexArray(ModelObject *object) : object(object)
	{
		//this->sence_volumes.clear();
		//this->vercol_obj.ArrayClear();
		//this->vercol_spt.ArrayClear();
		this->clear_matrix();
	}

	// 添加顶点数组
	bool ModelVertexArray::add_array(ModelVolume *volume_p)
	{
		//delete by gaohui 20240725
		//if (volume_p == NULL)
		//	return false;

		//// 获取volume_idx 保存volume_p指针
		//this->sence_volumes.push_back(volume_p);
		//size_t sence_volume_idx = this->sence_volumes.size() - 1;

		//// 合入OpenGL数据
		//if (volume_p->surpporter)
		//{
		//	LOGINFO("vercol_spt-->appendMore volume_idx = [%d], sence_volumes.size = [%d]",
		//			sence_volume_idx, this->sence_volumes.size());
		//	vercol_spt.appendMore(volume_p->Get_VerArray(), sence_volume_idx);
		//}
		//else
		//{
		//	LOGINFO("vercol_obj-->appendMore volume_idx = [%d], sence_volumes.size = [%d]",
		//			sence_volume_idx, this->sence_volumes.size());
		//	vercol_obj.appendMore(volume_p->Get_VerArray(), sence_volume_idx);
		//}

		return true;
	}

	// 删除顶点数组
	bool ModelVertexArray::delete_array(const size_t select_idx)
	{		
		//delete by gaohui 20240725
		//// 将指针置空
		//if (select_idx < this->sence_volumes.size())
		//{
		//	// 删除OpenGL数据
		//	if (this->sence_volumes[select_idx]->surpporter)
		//		this->vercol_spt.remove_volume(select_idx);
		//	else
		//		this->vercol_obj.remove_volume(select_idx);

		//	this->sence_volumes[select_idx] = NULL;

		//	LOGINFO("select_idx[%d] == Null, sence_volumes.size() = %d", select_idx, this->sence_volumes.size());
		//}

		return true;
	}

	bool ModelVertexArray::delete_array(ModelVolume *volume_p)
	{
		//delete by gaohui 20240725

		//// 将指针置空
		//if (volume_p == NULL)
		//	return false;

		//int volume_idx = this->GetSelect_idx(volume_p);
		//if (volume_idx == -1)
		//{
		//	LOGINFO("ModelVolume not found; volume == [%d]", volume_idx);
		//	return false;
		//}
		//this->delete_array(volume_idx);

		return true;
	}

	// 清空顶点数组
	void ModelVertexArray::clear_array()
	{
		//delete by gaohui 20240725

		//this->sence_volumes.clear();
		//this->vercol_spt.ArrayClear();
		//this->vercol_obj.ArrayClear();
	}

	void ModelVertexArray::clear_array_spt()
	{
		//delete by gaohui 20240725

		//this->vercol_spt.ArrayClear();
		//// 处理sence_volumes
		//for (size_t i = 0; i < this->sence_volumes.size(); i++)
		//{
		//	ModelVolume *p_vol = this->sence_volumes.at(i);
		//	if (p_vol && p_vol->surpporter)
		//		this->sence_volumes[i] = NULL;
		//}
		//LOGINFO("2");
	}

	void ModelVertexArray::clear_matrix()
	{
		this->translate_Matrix = glm::mat4(1.0f);
		this->rotate_Matrix = glm::mat4(1.0f);
		this->scale_Matrix = glm::mat4(1.0f);
		this->last_translate = glm::mat4(1.0f);
		this->last_rotate = glm::mat4(1.0f);
		this->last_scale = glm::mat4(1.0f);
	}

	void ModelVertexArray::recover_last_matrix()
	{
		this->translate_Matrix = this->last_translate;
		this->rotate_Matrix = this->last_rotate;
		this->scale_Matrix = this->last_scale;
	}

	// 重载顶点数组
	void ModelVertexArray::reload_array()
	{
		//delete by gaohui 20240725

		//if (this->object == NULL)
		//	return;
		//this->clear_array();

		//for (int i = 0; i < object->volumes.size(); i++)
		//{
		//	ModelVolume *p_v = object->volumes.at(i);
		//	if (p_v)
		//		this->add_array(p_v);
		//}
	}

	void ModelVertexArray::reload_obj_facet_color()
	{
		//delete by gaohui 20240725

		//if (this->object == NULL)
		//	return;

		//this->vercol_obj.clear_facet_Info();
		//std::map<int, GLVertex_SizePos>::iterator scence_map_It = this->vercol_obj.volumes_sizepos.begin();
		//for (; scence_map_It != this->vercol_obj.volumes_sizepos.end(); scence_map_It++)
		//{
		//	int sence_idx = scence_map_It->first;
		//	ModelVolume *p_vol = this->object->get_volume_by_selectid(sence_idx);
		//	this->vercol_obj.push_facet_Info(p_vol->mesh);
		//}
	}

	void ModelVertexArray::reload_rpd_facet_color()
	{
		//delete by gaohui 20240725

		//if (this->object == NULL)
		//	return;

		//this->vercol_obj.clear_facet_Info();
		//std::map<int, GLVertex_SizePos>::iterator scence_map_It = this->vercol_obj.volumes_sizepos.begin();
		//for (; scence_map_It != this->vercol_obj.volumes_sizepos.end(); scence_map_It++)
		//{
		//	int sence_idx = scence_map_It->first;
		//	ModelVolume *p_vol = this->object->get_volume_by_selectid(sence_idx);
		//	this->vercol_obj.push_facet_rpd_mark_Info(p_vol->mesh);
		//}
	}

	// 通过idx 获取 ModelVolume*
	ModelVolume *ModelVertexArray::GetVolume(const size_t select_idx)
	{
		//delete by gaohui 20240725

		//if (select_idx >= this->sence_volumes.size())
		//	return NULL;

		//return this->sence_volumes.at(select_idx);
		return NULL;
	}

	// 通过 ModelVolume*获取idx
	int ModelVertexArray::GetSelect_idx(ModelVolume *volume_p)
	{

		//if (volume_p == NULL)
		//	return -1;
		//int select_idx = -1;

		//delete by gaohui 20240725
		//for (int i = 0; i < this->sence_volumes.size(); i++)
		//{
		//	if (volume_p == this->sence_volumes.at(i))
		//	{
		//		select_idx = i;
		//		break;
		//	}
		//}

		//return select_idx;

		return -1;
	}

	GLVertexArrayCollection *ModelVertexArray::get_Aarray_ptr(bool Is_spt)
	{
		GLVertexArrayCollection *retval = NULL;

		//delete by gaohui 20240725

		//if (Is_spt)
		//{
		//	retval = &vercol_spt;
		//}
		//else
		//{
		//	retval = &vercol_obj;
		//}
		return retval;
	}

	// 组合模型矩阵
	void ModelVertexArray::Combin_Matrix()
	{
		this->t_Matrix = this->translate_Matrix * this->rotate_Matrix * this->scale_Matrix;
	}

	void *ModelVertexArray::get_Matrix_ptr(bool no_Z)
	{
		this->Combin_Matrix();
		float *retval = glm::value_ptr(this->t_Matrix);
		// 将矩阵z位置设置为0
		if (no_Z)
			retval[14] = 0.0;
		return retval;
	};

	void *ModelVertexArray::get_transfered_Matrix_ptr(const coordf_t _x, const coordf_t _y, const coordf_t _z, bool no_Z)
	{
		LOGINFO("(_x, _y, _z): [ %.3f, %.3f,%.3f ]", _x, _y, _z);

		this->Combin_Matrix();
		temp_Matrix_for_copy = this->t_Matrix;
		Log_Matrix(temp_Matrix_for_copy);

		temp_Matrix_for_copy = glm::translate(temp_Matrix_for_copy, glm::vec3(_x, _y, _z));
		Log_Matrix(temp_Matrix_for_copy);

		float *retval = glm::value_ptr(temp_Matrix_for_copy);
		// 将矩阵z位置设置为0
		if (no_Z)
			retval[14] = 0.0;
		return retval;
	}

	glm::mat4 ModelVertexArray::get_Matrix()
	{
		this->Combin_Matrix();
		return this->t_Matrix;
	}

	glm::mat4 ModelVertexArray::get_transfer_Matrix()
	{
		this->Combin_Matrix();
		glm::mat4 dis_matrix = this->t_Matrix * glm::inverse(this->last_translate * this->last_rotate * this->last_scale); // 计算两次变换的矩阵差
		// 记录上一次同步的矩阵
		this->last_translate = this->translate_Matrix;
		this->last_rotate = this->rotate_Matrix;
		this->last_scale = this->scale_Matrix;
		return dis_matrix;
	};

	void ModelVertexArray::Log_Matrix(const float *pSource)
	{
		if (pSource)
		{
			LOGINFO("---------------------------------------------------");
			LOGINFO("Line1: [ %.3f, %.3f,%.3f,%.3f ]", pSource[0], pSource[1], pSource[2], pSource[3]);
			LOGINFO("Line2: [ %.3f, %.3f,%.3f,%.3f ]", pSource[4], pSource[5], pSource[6], pSource[7]);
			LOGINFO("Line3: [ %.3f, %.3f,%.3f,%.3f ]", pSource[8], pSource[9], pSource[10], pSource[11]);
			LOGINFO("Line4: [ %.3f, %.3f,%.3f,%.3f ]", pSource[12], pSource[13], pSource[14], pSource[15]);
		}
	}

	// 调试接口
	void ModelVertexArray::Log_Matrix(glm::mat4 _Matrix)
	{
		_Matrix = glm::transpose(_Matrix);
		const float *pSource = (const float *)(glm::value_ptr(_Matrix));
		ModelVertexArray::Log_Matrix(pSource);
	}

	bool ModelVertexArray::IsMatrixUnit(glm::mat4 _Matrix)
	{
		const float *pSource = (const float *)(glm::value_ptr(_Matrix));
		for (int i = 0; i < 16; i++)
		{
			if (i == 0 || i == 5 || i == 10 || i == 15)
			{
				if (pSource[i] > 1.0 + 0.00001 || pSource[i] < 1.0 - 0.00001)
				{
					LOGINFO("pSource[%d] = %f", i, pSource[i]);
					return false;
				}
			}
			else if (pSource[i] > 0.00001 || pSource[i] < -0.00001)
			{
				LOGINFO("pSource[%d] = %f", i, pSource[i]);
				return false;
			}
		}

		return true;
	}

	// 矩阵操作-->平移
	void ModelVertexArray::translate(const coordf_t _x, const coordf_t _y, const coordf_t _z)
	{
		this->translate_Matrix = glm::translate(this->translate_Matrix, glm::vec3(_x, _y, _z));
	}

	// 矩阵操作-->缩放
	void ModelVertexArray::scale(const coordf_t _x, const coordf_t _y, const coordf_t _z)
	{
		this->scale_Matrix = glm::scale(this->scale_Matrix, glm::vec3(_x, _y, _z));
	}

	// 获取世界矩阵轴
	glm::vec3 ModelVertexArray::Get_WorldAxis(const Axis &axis)
	{
		float axis_x = 0.0;
		float axis_y = 0.0;
		float axis_z = 0.0;

		switch (axis)
		{
		case Axis::X:
			axis_x = 1.0;
			break;
		case Axis::Y:
			axis_y = 1.0;
			break;
		case Axis::Z:
			axis_z = 1.0;
			break;
		default:
			LOGINFO("Axis Type Error! %d", (int)axis);
			break;
		}

		glm::mat4 rotate_rev = glm::inverse(this->rotate_Matrix);
		rotate_rev = glm::transpose(rotate_rev);
		float *trafo3x4 = (float *)glm::value_ptr(rotate_rev);
		float axis_x_world = trafo3x4[0] * axis_x + trafo3x4[1] * axis_y + trafo3x4[2] * axis_z + trafo3x4[3];
		float axis_y_world = trafo3x4[4] * axis_x + trafo3x4[5] * axis_y + trafo3x4[6] * axis_z + trafo3x4[7];
		float axis_z_world = trafo3x4[8] * axis_x + trafo3x4[9] * axis_y + trafo3x4[10] * axis_z + trafo3x4[11];

		return glm::vec3(axis_x_world, axis_y_world, axis_z_world);
	}

	// 矩阵操作-->旋转
	void ModelVertexArray::rotate(float angle, const Axis &axis)
	{
		this->rotate_Matrix = glm::rotate(this->rotate_Matrix, angle, this->Get_WorldAxis(axis));

		LOGINFO("rotate_Matrix");
		this->Log_Matrix(this->rotate_Matrix);
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// ModelVolume类的定义
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	ModelVolume::ModelVolume(ModelObject *object, const TriangleMesh &mesh)
		: mesh(mesh), modifier(false), object(object), surpporter(false), solider(true), IsIpdVolume(false), adder(false), merged(false), Lattice_merged(false), SurpportType(""), p_spted_vol(NULL)
	{
		IpdRpdType = 0;
		Rpd_Rotate_Angle_X = 0.0;
		FrameX = Pointf3(1, 0, 0);
		FrameY = Pointf3(0, 1, 0);
		FrameZ = Pointf3(0, 0, 1);
	}

	ModelVolume::ModelVolume(ModelObject *object, const ModelVolume &other)
		: name(other.name), mesh(other.mesh), config(other.config),
		  modifier(other.modifier), object(object), surpporter(other.surpporter), p_spted_vol(other.p_spted_vol),
		  solider(other.solider), adder(other.adder), automark(other.automark), mark(other.mark),
		  merged(other.merged),
		  CharMark(other.CharMark),
		  Lattice_merged(other.Lattice_merged),
		  SurpportType(other.SurpportType),
		  IsIpdVolume(other.IsIpdVolume), IpdRpdType(other.IpdRpdType), Rpd_Rotate_Angle_X(other.Rpd_Rotate_Angle_X),
		  FrameX(other.FrameX), FrameY(other.FrameY), FrameZ(other.FrameZ),
		  TopNarmol(other.TopNarmol), ButtomNarmol(other.ButtomNarmol), MidNarmol(other.MidNarmol),
		  upPickPos(other.upPickPos), dwPickPos(other.dwPickPos), midPickPos(other.midPickPos),
		  b_exist_cross_bar_not_generate(other.b_exist_cross_bar_not_generate), b_cross_bar(other.b_cross_bar), b_crossbar_ball(other.b_crossbar_ball), _raw_thumbnails_valid(other._raw_thumbnails_valid)
	{
		this->material_id(other.material_id());
		this->SurpportParams.insert(other.SurpportParams.begin(), other.SurpportParams.end());
		this->thumbnails.clear();
		this->thumbnails.insert(this->thumbnails.end(), other.thumbnails.begin(), other.thumbnails.end());
	}

	// 重载赋值=操作符
	ModelVolume &ModelVolume::operator=(ModelVolume other)
	{
		this->swap(other);
		return *this;
	}

	ModelVolume::~ModelVolume()
	{
		std::map<std::string, double>().swap(SurpportParams);
		SurpportParams.clear();
		this->mesh.ClearMesh();
		CrossbarPointPair().swap(this->crossbar_pointpair);
		if (lattice_support_ptr != nullptr)
		{
			delete lattice_support_ptr;
			lattice_support_ptr = nullptr;
		}
	}

	// 交换两个ModelVolume的内容
	void
	ModelVolume::swap(ModelVolume &other)
	{
		std::swap(this->name, other.name);
		std::swap(this->mesh, other.mesh);
		std::swap(this->config, other.config);
		std::swap(this->modifier, other.modifier);
		std::swap(this->mark, other.mark);
		std::swap(this->Lattice_merged, other.Lattice_merged);
		std::swap(this->surpporter, other.surpporter);
		std::swap(this->adder, other.adder);
		std::swap(this->automark, other.automark);
		std::swap(this->p_spted_vol, other.p_spted_vol);
		std::swap(this->solider, other.solider);
		std::swap(this->IsIpdVolume, other.IsIpdVolume);
		std::swap(this->IpdRpdType, other.IpdRpdType);
		std::swap(this->Rpd_Rotate_Angle_X, other.Rpd_Rotate_Angle_X);
		std::swap(this->merged, other.merged);
		std::swap(this->SurpportType, other.SurpportType);
		std::swap(this->TopNarmol, other.TopNarmol);
		std::swap(this->ButtomNarmol, other.ButtomNarmol);
		std::swap(this->MidNarmol, other.MidNarmol);
		std::swap(this->dwPickPos, other.dwPickPos);
		std::swap(this->upPickPos, other.upPickPos);
		std::swap(this->midPickPos, other.midPickPos);
		std::swap(this->SurpportParams, other.SurpportParams);
		std::swap(this->b_exist_cross_bar_not_generate, other.b_exist_cross_bar_not_generate);
		std::swap(this->b_cross_bar, other.b_cross_bar);
		std::swap(this->b_crossbar_ball, other.b_crossbar_ball);
		std::swap(this->thumbnails, other.thumbnails);
		std::swap(this->_raw_thumbnails_valid, other._raw_thumbnails_valid);
	}
	bool ModelVolume::Isequal(ModelVolume *other)
	{
		// 先按照取点来判断是否相同
		if (this->FrameX == other->FrameX && this->FrameY == other->FrameY && this->FrameZ == other->FrameZ && this->upPickPos == other->upPickPos && this->midPickPos == other->midPickPos && this->dwPickPos == other->dwPickPos && this->mesh.facets_count() == other->mesh.facets_count())
		{

			return true;
		}
		else
		{
			return false;
		}
	}
	void ModelVolume::invalid_volume_thumbnails()
	{
		this->thumbnails.clear();
		this->_raw_thumbnails_valid = false;
	}

	void ModelVolume::update_volume_thumbnails()
	{
		LOGINFO("ModelVolume::update_volume_thumbnails()! Begin.. [%s]", this->name.c_str());
		if (this->_raw_thumbnails_valid && this->thumbnails.size() != 0)
		{
			LOGINFO("no need to ModelVolume::update_volume_thumbnails()!");
			return;
		}
		this->thumbnails.clear();

		if (this->modifier)
			return;
		if (this->surpporter)
			return;

		// 取精确的平面投影
		LOGINFO("ModelVolume::update_volume_thumbnails()! horizontal_projection_slm Begin.. [%s]", this->name.c_str());
		ExPolygons exs;
		// if (this->mesh.facets_count() > 500000) {
		//	BoundingBoxf3 bb = this->mesh.bounding_box();
		//	Polygon pp;
		//	pp.points.clear();
		//	pp.points.resize(4);
		//	pp.points[0].x = bb.min.x;
		//	pp.points[0].y = bb.min.y;
		//	pp.points[1].x = bb.max.x;
		//	pp.points[1].y = bb.min.y;
		//	pp.points[2].x = bb.max.x;
		//	pp.points[2].y = bb.max.y;
		//	pp.points[3].x = bb.min.x;
		//	pp.points[3].y = bb.max.y;
		//	ExPolygon ex;
		//	ex.contour = pp;
		//	exs.push_back(ex);
		// }
		// else {
		exs = this->mesh.horizontal_projection_slm(90.0);
		//}
		LOGINFO("ModelVolume::update_volume_thumbnails()! horizontal_projection_slm End.. [%s]", this->name.c_str());

		for (int i = 0; i < exs.size(); i++)
		{
			ExPolygons ex_simp = exs.at(i).simplify(SCALED_RESOLUTION_SPT);
			for (int j = 0; j < ex_simp.size(); j++)
			{
				if (ex_simp.at(j).contour.points.size() > 3)
				{
					this->thumbnails.push_back(ex_simp.at(j));
				}
			}
		}

		// 未计算出投影
		if (this->thumbnails.size() == 0)
		{
			Polygon conhull = this->mesh.convex_hull();
			if (conhull.points.size() == 0)
			{
				LOGINFO("this object vol mesh convex_hull is empty");
				return;
			}
			ExPolygon ex_conhull;
			ex_conhull.contour = conhull;
			this->thumbnails.push_back(ex_conhull);
		}

		this->_raw_thumbnails_valid = true;
		LOGINFO("ModelVolume::update_volume_thumbnails()! End.. [%s]", this->name.c_str());
	}

	ExPolygons ModelVolume::get_volume_thumbnails()
	{
		this->update_volume_thumbnails();
		ExPolygons _retval;
		// 取出主要的Instance
		ModelInstance *p_major = this->get_object()->instances.at(0);
		if (p_major == NULL)
		{
			CONFESS("this object do not has object instance");
		}
		for (int i = 0; i < this->thumbnails.size(); i++)
		{
			ExPolygon _temp = this->thumbnails.at(i);
			_temp.rotate(p_major->rotation * PI / 180);
			_temp.translate(scale_(p_major->offset.x), scale_(p_major->offset.y));
			_retval.push_back(_temp);
		}
		if (false)
		{
			SVG svg1("get_model_thumbnails.svg");
			svg1.draw(this->thumbnails, "red");
			svg1.draw(_retval, "green");
			svg1.Close();
		}

		return _retval;
	}

	// 获取当前volume的material ID
	t_model_material_id
	ModelVolume::material_id() const
	{
		return this->_material_id;
	}

	// 将某种id的material加入到volume中
	void
	ModelVolume::material_id(t_model_material_id material_id)
	{
		this->_material_id = material_id;

		// ensure this->_material_id references an existing material
		(void)this->object->get_model()->add_material(material_id);
	}

	// 获取当前设定的ModelMaterial
	ModelMaterial *
	ModelVolume::material() const
	{
		return this->object->get_model()->get_material(this->_material_id);
	}

	// 对ModelVolume设定某种材料
	void
	ModelVolume::set_material(t_model_material_id material_id, const ModelMaterial &material)
	{
		this->_material_id = material_id;
		(void)this->object->get_model()->add_material(material_id, material);
	}

	ModelMaterial *
	ModelVolume::assign_unique_material()
	{
		Model *model = this->get_object()->get_model();

		// as material-id "0" is reserved by the AMF spec we start from 1
		this->_material_id = 1 + model->materials.size(); // watchout for implicit cast
		return model->add_material(this->_material_id);
	}

	GLVertexArray
	ModelVolume::Get_VerArray()
	{
		GLVertexArray retval;
		TriangleMesh volume_mesh = this->mesh;
		// 需要对新加的物体做当前矩阵的逆变换
		if (this->object && this->object->verarray_p)
		{
			glm::mat4 Matrix_inv = this->object->verarray_p->get_Matrix();
			if (ModelVertexArray::IsMatrixUnit(Matrix_inv) == false)
			{
				float *Matrix_inv_fp = (float *)glm::value_ptr(Matrix_inv);
				volume_mesh.transform(Matrix_inv_fp);
			}
		}

		retval.load_mesh(volume_mesh, this->surpporter);
		return retval;
	};

	// 获取支撑体支撑的volume
	ModelVolume *
	ModelVolume::Get_spted_obj()
	{
		if (this->surpporter == false)
		{
			LOGINFO("this is not a spt, none spted obj");
			return NULL;
		}
		if (this->p_spted_vol == NULL)
		{
			LOGINFO("this spt has none spted obj");
			return NULL;
		}

		return this->p_spted_vol;
	}

	// 构建支撑体
	bool
	ModelVolume::make_SptMesh()
	{
		LOGINFO("make_SptMesh start!!!");

		if (this->verify_SptParams() == false)
		{
			LOGINFO("this->verify_SptParams() == false!!!");

			return false;
		}

		return this->analyze_SptParams();
	}

	//
	bool
	ModelVolume::make_SptMesh(ModelVolume *pvolume)
	{
		LOGINFO("make_SptMesh(ModelVolume* pvolume) start");

		if (pvolume == NULL || pvolume->p_spted_vol == NULL)
		{
			LOGINFO("pvolume == NULL || pvolume->p_spted_vol == NULL");
			return false;
		}

		// 自动构建横杆支撑流程
		if (pvolume->SurpportType == "auto_crossbar" || pvolume->SurpportType == "auto_assist_crossbar")
		{
			unsigned int crossbar_id = pvolume->SurpportParams["crossbar_id"];
			int Crossbar_PointSize;
			if (pvolume->SurpportType == "auto_crossbar")
				Crossbar_PointSize = pvolume->p_spted_vol->mesh.Crossbar_Points.size();
			else
				Crossbar_PointSize = pvolume->p_spted_vol->mesh.Assit_Crossbar_Points.size();

			if (crossbar_id >= Crossbar_PointSize)
				return false;

			CrossbarPointPair points_pair;
			if (pvolume->SurpportType == "auto_crossbar")
				points_pair = pvolume->p_spted_vol->mesh.Crossbar_Points[crossbar_id];
			else
				points_pair = pvolume->p_spted_vol->mesh.Assit_Crossbar_Points[crossbar_id];

			CrossBarGenerator *crossbarMesh = new CrossBarGenerator(
				points_pair.first,
				pvolume->SurpportType == "auto_crossbar" ? pvolume->SurpportParams["cross_bar_radius"] : pvolume->SurpportParams["assist_cross_bar_radius"],
				pvolume->SurpportType == "auto_crossbar" ? pvolume->SurpportParams["cross_bar_contact_width"] : pvolume->SurpportParams["assist_cross_bar_contact_width"],
				pvolume->SurpportType == "auto_crossbar" ? pvolume->SurpportParams["cross_bar_contact_length"] : pvolume->SurpportParams["assist_cross_bar_contact_length"],
				pvolume->SurpportType == "auto_crossbar" ? pvolume->SurpportParams["cross_bar_is_special"] : pvolume->SurpportParams["assist_cross_bar_is_special"]);

			pvolume->upPickPos = points_pair.first;
			pvolume->dwPickPos = points_pair.second;
			pvolume->crossbar_pointpair = points_pair;
			pvolume->mesh = crossbarMesh->Generate(points_pair.second);

			if(crossbarMesh != nullptr)
			{
				delete crossbarMesh;
				crossbarMesh = nullptr;
			}
			
			return true;
		}

		// 构建横杆支撑流程
		if (pvolume->SurpportType == "crossbar" || pvolume->SurpportType == "assist_crossbar")
		{
			// 已构建晶格支撑不允许添加横杆
			ModelObject *object = pvolume->get_object();
			for (size_t i_volume = 0; i_volume < object->volumes.size(); ++i_volume)
			{
				ModelVolume *volume = object->volumes[i_volume];
				if (volume->lattice_support_ptr != nullptr)
				{
					return false;
				}
			}
			if (pvolume->p_spted_vol != NULL)
				LOGINFO("crossbar spt volume name = %s", pvolume->p_spted_vol->name.c_str());

			CrossBarGenerator *crossbarMesh = new CrossBarGenerator(
				pvolume->upPickPos,
				pvolume->SurpportType == "crossbar" ? pvolume->SurpportParams["cross_bar_radius"] : pvolume->SurpportParams["assist_cross_bar_radius"],
				pvolume->SurpportType == "crossbar" ? pvolume->SurpportParams["cross_bar_contact_width"] : pvolume->SurpportParams["assist_cross_bar_contact_width"],
				pvolume->SurpportType == "crossbar" ? pvolume->SurpportParams["cross_bar_contact_length"] : pvolume->SurpportParams["assist_cross_bar_contact_length"],
				pvolume->SurpportType == "crossbar" ? pvolume->SurpportParams["cross_bar_is_special"] : pvolume->SurpportParams["assist_cross_bar_is_special"]);

			for (size_t i_volume = 0; i_volume < object->volumes.size(); ++i_volume)
			{
				ModelVolume *volume = object->volumes[i_volume];
				if (volume->b_exist_cross_bar_not_generate == true)
				{
					volume->b_exist_cross_bar_not_generate = false;
					volume->b_cross_bar = false;
					pvolume->mesh = crossbarMesh->Generate(volume->upPickPos);
					pvolume->dwPickPos = volume->upPickPos; // 记录crossbar另一头的端点
					LOGINFO("crossbarMesh PickPos{%s %s}",
					pvolume->dwPickPos.dump_perl().c_str(), pvolume->upPickPos.dump_perl().c_str());
					if(crossbarMesh != nullptr)
					{
						delete crossbarMesh;
						crossbarMesh = nullptr;
					}
					
					return true;
				}
			}

			CrossbarPointPair points_pair;
			points_pair.first = pvolume->upPickPos;
			points_pair.second = pvolume->dwPickPos;
			pvolume->crossbar_pointpair = points_pair;

			pvolume->mesh = crossbarMesh->GenerateStartBall();
			pvolume->b_exist_cross_bar_not_generate = true;
			pvolume->b_cross_bar = false;
			pvolume->b_crossbar_ball = true;
			if(crossbarMesh != nullptr)
			{
				delete crossbarMesh;
				crossbarMesh = nullptr;
			}

			return true;
		}

		// 构建晶格基底支撑流程
		if (pvolume->SurpportType == "lattice")
		{
			TriangleMesh target_mesh;
			ModelObject *object = pvolume->get_object();
			ExPolygons support_thumbnails;
			bool lattice_volume_merge = (pvolume->SurpportParams["volume_merge"] != 0.0); // 是否打开了晶格合并支撑标记
			if (lattice_volume_merge)													  // 合并其他所有的主体mesh
			{
				if (true || object->has_LatticeMerge_volume()) // 暂不限制
				{
					for (size_t i_volume = 0; i_volume < object->volumes.size(); ++i_volume)
					{
						ModelVolume *_volume_p = object->volumes[i_volume];
						if (_volume_p->solider == false || _volume_p->modifier) // 排除干扰项
							continue;
						if (_volume_p->b_cross_bar) // 实体横杆
						{
							ExPolygon _ex;
							_ex.contour = _volume_p->mesh.convex_hull();
							support_thumbnails.push_back(_ex);
						}
						else if (_volume_p->surpporter == false) // 实体物体
						{
							target_mesh.merge(_volume_p->mesh);
							ExPolygons _tb = _volume_p->mesh.horizontal_projection_slm(90);
							support_thumbnails.insert(support_thumbnails.end(), _tb.begin(), _tb.end());
						}
					}
				}
				else
				{
					LOGINFO("lattice_volume_merge on, other object no lattice spt");
					return false;
				}
			}
			else
			{
				target_mesh = pvolume->p_spted_vol->mesh;
				support_thumbnails = pvolume->p_spted_vol->mesh.horizontal_projection_slm(90);
			}
			// 横杆特殊处理 取出中轴线
			std::vector<Linef3> crossbar_midLines;
			if (lattice_volume_merge) // 合并操作 取出obj下面所有横杆
			{
				if (true || object->has_LatticeMerge_volume()) // 暂不限制
				{
					for (size_t i_volume = 0; i_volume < object->volumes.size(); ++i_volume)
					{
						ModelVolume *_volume_p = object->volumes[i_volume];
						if (_volume_p->b_cross_bar == true)
						{
							Linef3 _crossbar_line;
							_crossbar_line.a = _volume_p->upPickPos;
							_crossbar_line.b = _volume_p->dwPickPos;
							crossbar_midLines.push_back(_crossbar_line);
						}
					}
				}
				else
				{
					LOGINFO("lattice_volume_merge on, other object no lattice spt");
					return false;
				}
			}
			else
			{ // 没有合并操作 取出所支撑volume物体归属下所有横杆
				ModelVolumePtrs sptedObj_spts = object->map_sptvol[pvolume->p_spted_vol];
				// LOGINFO("1208 - 2 filename = %s", object->name.c_str());
				LOGINFO("[crossbar_midLines] object->map_sptvol[pvolume->p_spted_vol].size() = %d", sptedObj_spts.size());
				for (size_t i_volume = 0; i_volume < sptedObj_spts.size(); ++i_volume)
				{
					ModelVolume *_volume_p = sptedObj_spts[i_volume];
					if (_volume_p->b_cross_bar == true)
					{
						Linef3 _crossbar_line;
						_crossbar_line.a = _volume_p->upPickPos;
						_crossbar_line.b = _volume_p->dwPickPos;
						crossbar_midLines.push_back(_crossbar_line);
						LOGINFO("[crossbar_midLines] volume[%s] is b_cross_bar", _volume_p->name.c_str());
					}
					else
					{
						LOGINFO("[crossbar_midLines] volume[%s] is not b_cross_bar", _volume_p->name.c_str());
					}
				}
			}
			LOGINFO("volumes.size() = %d", object->volumes.size());
			LOGINFO("crossbar_midLines = %d", crossbar_midLines.size());
			// target_mesh.write_binary("target_mesh.stl");
			target_mesh.repair();
			if (target_mesh.UnableRepair())
			{
				LOGINFO("pvolume->SurpportType == lattice, target_mesh Unable Repair");
				return false;
			}

			LatticeSupportMesh *lsMesh = new LatticeSupportMesh(
				target_mesh,
				crossbar_midLines,
				pvolume->SurpportParams["angel_step"],
				pvolume->SurpportParams["bound_num_min"],
				pvolume->SurpportParams["exclude_tuqi_area"],
				pvolume->SurpportParams["suspend_angel"],
				pvolume->SurpportParams["x_cell_length"],
				pvolume->SurpportParams["y_cell_length"],
				pvolume->SurpportParams["z_cell_length"],
				pvolume->SurpportParams["lattice_slice_thickness"],
				pvolume->SurpportParams["lattice_spt_distance"],
				pvolume->SurpportParams["crossbar_spt_distance"],
				pvolume->SurpportParams["up_insert_z"],
				pvolume->SurpportParams["up_contact_height"],
				pvolume->SurpportParams["up_contact_width"],
				pvolume->SurpportParams["up_connection_height"],
				pvolume->SurpportParams["down_contact_width"],
				pvolume->SurpportParams["down_contact_angle"],
				pvolume->SurpportParams["lattice_width"],
				pvolume->SurpportParams["lattice_height"],
				pvolume->SurpportParams["pillar_width"],
				pvolume->SurpportParams["lattice_grid_angle"],
				pvolume->SurpportParams["extand_factor"],
				pvolume->SurpportParams["bottom_strengthen"] != 0.0,
				pvolume->SurpportParams["lattice_merge"] != 0.0,
				pvolume->SurpportParams["arrange_distance"]);

			LOGINFO(" angel_step = %f", pvolume->SurpportParams["angel_step"]);
			LOGINFO(" bound_num_min = %f", pvolume->SurpportParams["bound_num_min"]);
			LOGINFO(" exclude_tuqi_area = %f", pvolume->SurpportParams["exclude_tuqi_area"]);
			LOGINFO(" suspend_angel = %f", pvolume->SurpportParams["suspend_angel"]);
			LOGINFO(" x_cell_length = %f", pvolume->SurpportParams["x_cell_length"]);
			LOGINFO(" y_cell_length = %f", pvolume->SurpportParams["y_cell_length"]);
			LOGINFO(" z_cell_length = %f", pvolume->SurpportParams["z_cell_length"]);
			LOGINFO(" lattice_slice_thickness = %f", pvolume->SurpportParams["lattice_slice_thickness"]);
			LOGINFO(" lattice_spt_distance = %f", pvolume->SurpportParams["lattice_spt_distance"]);
			LOGINFO(" crossbar_spt_distance = %f", pvolume->SurpportParams["crossbar_spt_distance"]);
			LOGINFO(" normal_angel = %f", pvolume->SurpportParams["normal_angel"]);
			LOGINFO(" up_insert_z = %f", pvolume->SurpportParams["up_insert_z"]);
			LOGINFO(" up_contact_height = %f", pvolume->SurpportParams["up_contact_height"]);
			LOGINFO(" up_contact_width = %f", pvolume->SurpportParams["up_contact_width"]);
			LOGINFO(" up_connection_height = %f", pvolume->SurpportParams["up_connection_height"]);
			LOGINFO(" down_contact_width = %f", pvolume->SurpportParams["down_contact_width"]);
			LOGINFO(" down_contact_angle = %f", pvolume->SurpportParams["down_contact_angle"]);
			LOGINFO(" lattice_width = %f", pvolume->SurpportParams["lattice_width"]);
			LOGINFO(" lattice_height = %f", pvolume->SurpportParams["lattice_height"]);
			LOGINFO(" pillar_width = %f", pvolume->SurpportParams["pillar_width"]);
			LOGINFO(" lattice_grid_angle = %f", pvolume->SurpportParams["lattice_grid_angle"]);
			LOGINFO(" extand_factor = %f", pvolume->SurpportParams["extand_factor"]);
			LOGINFO(" bottom_strengthen = %f", pvolume->SurpportParams["bottom_strengthen"]);
			LOGINFO(" lattice_merge = %f", pvolume->SurpportParams["lattice_merge"]);
			LOGINFO(" arrange_distance = %f", pvolume->SurpportParams["arrange_distance"]);

			LOGINFO(" lattice_min_distance = %f", pvolume->SurpportParams["lattice_min_distance"]);
			lsMesh->set_lattice_min_distance(pvolume->SurpportParams["lattice_min_distance"]);

			LOGINFO(" avoid_reten_area = %f", pvolume->SurpportParams["avoid_reten_area"]);
			LOGINFO(" reten_area_pillar_connect = %f", pvolume->SurpportParams["reten_area_pillar_connect"]);
			LOGINFO(" reten_factor = %f", pvolume->SurpportParams["reten_factor"]);

			lsMesh->set_reten_pillar_para(pvolume->SurpportParams["avoid_reten_area"] != 0.0,
										  pvolume->SurpportParams["reten_area_pillar_connect"] != 0.0,
										  pvolume->SurpportParams["reten_factor"]);

			LOGINFO(" avoid_tuqi_area = %f", pvolume->SurpportParams["avoid_tuqi_area"]);
			LOGINFO(" tuqi_area_pillar_connect = %f", pvolume->SurpportParams["tuqi_area_pillar_connect"]);
			LOGINFO(" tuqi_factor = %f", pvolume->SurpportParams["tuqi_factor"]);

			lsMesh->set_tuqi_pillar_para(pvolume->SurpportParams["avoid_tuqi_area"] != 0.0,
										 pvolume->SurpportParams["tuqi_area_pillar_connect"] != 0.0,
										 pvolume->SurpportParams["tuqi_factor"]);

			LOGINFO(" min_pillar_connect_height = %f", pvolume->SurpportParams["min_pillar_connect_height"]);
			LOGINFO(" min_pillar_connect_height_reten = %f", pvolume->SurpportParams["min_pillar_connect_height_reten"]);
			LOGINFO(" min_pillar_connect_height_tuqi = %f", pvolume->SurpportParams["min_pillar_connect_height_tuqi"]);

			lsMesh->set_min_pillar_connect_height(pvolume->SurpportParams["min_pillar_connect_height"],
												  pvolume->SurpportParams["min_pillar_connect_height_reten"],
												  pvolume->SurpportParams["min_pillar_connect_height_tuqi"]);

			LOGINFO(" pillar_connect_jump = %f", pvolume->SurpportParams["pillar_connect_jump"]);
			LOGINFO(" pillar_simplify = %f", pvolume->SurpportParams["pillar_simplify"]);
			lsMesh->set_pillar_connect_jump(pvolume->SurpportParams["pillar_connect_jump"] != 0.0,
											pvolume->SurpportParams["pillar_simplify"] != 0.0);

			LOGINFO(" pillar_connect_jump_reten = %f", pvolume->SurpportParams["pillar_connect_jump_reten"]);
			LOGINFO(" pillar_simplify_reten = %f", pvolume->SurpportParams["pillar_simplify_reten"]);
			lsMesh->set_pillar_connect_jump_reten(pvolume->SurpportParams["pillar_connect_jump_reten"] != 0.0,
												  pvolume->SurpportParams["pillar_simplify_reten"] != 0.0);

			LOGINFO(" pillar_connect_jump_tuqi = %f", pvolume->SurpportParams["pillar_connect_jump_tuqi"]);
			LOGINFO(" pillar_simplify_tuqi = %f", pvolume->SurpportParams["pillar_simplify_tuqi"]);
			lsMesh->set_pillar_connect_jump_tuqi(pvolume->SurpportParams["pillar_connect_jump_tuqi"] != 0.0,
												 pvolume->SurpportParams["pillar_simplify_tuqi"] != 0.0);

			// 已经计算过的投影先进行旋转
			double angle = pvolume->SurpportParams["lattice_grid_angle"];
			if (support_thumbnails.empty() == false && angle != 0.0)
			{
				ExPolygons sts;
				for (int i = 0; i < support_thumbnails.size(); i++)
				{
					ExPolygon ex = support_thumbnails.at(i);
					ex.rotate(-angle / 180.0 * PI);
					sts.push_back(ex);
				}
				support_thumbnails.clear();
				support_thumbnails = sts;
			}
			lsMesh->support_thumbnails = support_thumbnails;

			std::string bedtype = object->Get_bed_type();
			LOGINFO("bedtype = [%s]!!!!!!!!!!!!!", bedtype.c_str());

			if (object->_print != nullptr)
			{
				Polygon bed = object->_print->get_plater_shape(bedtype);
				ModelInstance *p_major = object->instances.at(0);
				if (p_major)
				{
					bed.translate(-scale_(p_major->offset.x), -scale_(p_major->offset.y));
					bed.rotate(-p_major->rotation * PI / 180);

					if (angle != 0.0)
						bed.rotate(-angle / 180.0 * PI);
					lsMesh->bed_type = bed;
				}
			}
			else
			{
				LOGINFO("bedtype = [%s], object->_print != nullptr!!!!!!!!!!!!!", bedtype.c_str());
			}

			if (lsMesh->generate_pillars() == false)
			{
				if(lsMesh != nullptr)
				{
					delete lsMesh;
					lsMesh = nullptr;
				}
				return false;
			}
				
			pvolume->mesh = lsMesh->GeneralMesh();
			// 记录造型器信息
			pvolume->lattice_support_ptr = lsMesh;
			object->can_split = false; // 添加晶格支撑后不可再拆分
			return true;
		}

		// 自动构建晶格顶部支撑流程
		if (pvolume->SurpportType == "lattice_top_spt")
		{
			unsigned int tops_id = pvolume->SurpportParams["tops_id"];

			// 取出晶格造型器
			LatticeSupportMeshPtr _lattice_support_ptr = nullptr;
			ModelObject *object = pvolume->get_object();

			for (size_t i_volume = 0; i_volume < object->map_sptvol[pvolume->p_spted_vol].size(); ++i_volume)
			{
				ModelVolume *volume = object->map_sptvol[pvolume->p_spted_vol][i_volume];
				if (volume->lattice_support_ptr != nullptr)
				{
					_lattice_support_ptr = volume->lattice_support_ptr;
					break;
				}
			}

			if (_lattice_support_ptr == nullptr)
			{
				LOGINFO("Error! pvolume->p_spted_vol->lattice_support_ptr == NULL");
				return false;
			}
			TriangleMesh *_pObj_ptr = _lattice_support_ptr->pObj;
			if (_pObj_ptr == nullptr)
			{
				LOGINFO("Error! pvolume->p_spted_vol->lattice_support_ptr>pObj == NULL");
				return false;
			}
			LatticeSPT_Pointf3s _Lspt_Points_vec = _pObj_ptr->Lspt_Points;
			if (_Lspt_Points_vec.size() == 0)
			{
				LOGINFO("Error! pvolume->p_spted_vol->lattice_support_ptr>pObj->Lspt_Points.size() == 0");
				return false;
			}
			LOGINFO("Lspt_Points.size() = [%d], tops_id = [%d]", _Lspt_Points_vec.size(), tops_id);
			if (tops_id < _Lspt_Points_vec.size())
			{
				int form_id = _Lspt_Points_vec[tops_id].Form_FaceID;
				LOGINFO("form_id = [%d]", form_id);
				// 强制过滤凸起区域
				if (form_id == -1 || TriangleMesh::fact_is_mark(_pObj_ptr->stl.facet_start[form_id].extra, Tuqi_Fact) == false)
				{
					if (_Lspt_Points_vec[tops_id].get_spt_height() < 0.1 ||
						_Lspt_Points_vec[tops_id].GenerateValid == false)
					{
						return false;
					}
					pvolume->mesh = _Lspt_Points_vec[tops_id].GenerateMesh();
				}
			}
			return true;
		}

		// 手动添加晶格顶部支撑流程
		if (pvolume->SurpportType == "lattice_top")
		{
			// 取出晶格造型器
			LatticeSupportMeshPtr lattice_support_mesh_ptr = nullptr;
			ModelObject *object = pvolume->get_object();
			for (size_t i_volume = 0; i_volume < object->map_sptvol[pvolume->p_spted_vol].size(); ++i_volume)
			{
				ModelVolume *volume = object->map_sptvol[pvolume->p_spted_vol][i_volume];
				if (volume->lattice_support_ptr != nullptr)
				{
					lattice_support_mesh_ptr = volume->lattice_support_ptr;
					break;
				}
			}

			if (lattice_support_mesh_ptr == nullptr)
			{
				for (size_t i_volume = 0; i_volume < object->volumes.size(); ++i_volume)
				{
					ModelVolume *volume = object->volumes[i_volume];
					if (volume->lattice_support_ptr != nullptr)
					{
						lattice_support_mesh_ptr = volume->lattice_support_ptr;
						break;
					}
				}
			}

			if (lattice_support_mesh_ptr == nullptr)
			{
				LOGINFO("lattice_support_mesh_ptr == nullptr");
				return false;
			}
			Pointf3 target_point;
			Pointf3 up = pvolume->upPickPos;
			double angleD = lattice_support_mesh_ptr->lattice_grid_angle;
			up.rotate(-angleD / 180 * PI);

			bool r = lattice_support_mesh_ptr->get_nearest_pillar_node_position(up, target_point);

			LatticeSPT_Pointf3 lattice_spt;
			lattice_spt.UpPoint = up;

			lattice_spt.UpPointNormal = Pointf3(0, 0, -1);
			lattice_spt.DwPoint = target_point;
			lattice_spt.set_parameter(*lattice_support_mesh_ptr);
			if (lattice_spt.get_spt_height() < 0.1)
				return false;

			pvolume->mesh = lattice_spt.GenerateMesh();
			return true;
		}

		// 构建面墙流程
		if (pvolume->SurpportType == "boundary_single_wall")
		{
			unsigned int lines_id = pvolume->SurpportParams["lines_id"];
			unsigned int parts_id = pvolume->SurpportParams["parts_id"];
			std::cout << "pvolume->p_spted_vol->mesh.wallLoops_vec[lines_id].size:" << pvolume->p_spted_vol->mesh.wallLoops_vec[lines_id].size() << std::endl;
			LineWall_Mesh *pWall = new LineWall_Mesh(
				pvolume->p_spted_vol->mesh.wallLoops_vec[lines_id],
				false,
				pvolume->SurpportParams["tooth_insert"],
				pvolume->SurpportParams["tooth_depth"],
				pvolume->SurpportParams["tooth_step"],
				pvolume->SurpportParams["hole_height_step"],
				pvolume->SurpportParams["hole_length_step"],
				pvolume->SurpportParams["hole_depth"],
				pvolume->SurpportParams["wall_split_step"],
				pvolume->SurpportParams["wall_split_wedth"]);

			pvolume->mesh = pWall->GeneralMesh(parts_id);
			if(pWall != nullptr)
			{
				delete pWall;	
				pWall = nullptr;
			}
			

			return true;
		}

		// 构建六边形孔洞面墙流程
		if (pvolume->SurpportType == "boundary_hexhole_wall")
		{
			LOGINFO(" thickness_hl = %f", pvolume->SurpportParams["thickness_hl"]);
			LOGINFO(" tooth_depth_hl = %f", pvolume->SurpportParams["tooth_depth_hl"]);
			LOGINFO(" tooth_step_hl = %f", pvolume->SurpportParams["tooth_step_hl"]);
			LOGINFO("0724 down_tooth_depth_hl = %f", pvolume->SurpportParams["down_tooth_depth_hl"]);
			unsigned int lines_id = pvolume->SurpportParams["lines_id"];
			// unsigned int parts_id = pvolume->SurpportParams["parts_id"];
			if (pvolume->SurpportParams["thickness_hl"] > 0.0)
			{
				Pointf3s innerLine = EntityWall_BaseMesh::make_wallline(pvolume->p_spted_vol->mesh.wallLoops_vec[lines_id], 10000, -pvolume->SurpportParams["thickness_hl"]);
				Pointf3s outerLine = EntityWall_BaseMesh::make_wallline(pvolume->p_spted_vol->mesh.wallLoops_vec[lines_id], 10000, 0);
				// for (int i = 0; i < innerLine.size(); i++)
				// {
				// 	LOGINFO("--0726 innerLine [%d] is [%s]", i, innerLine[i].dump_perl().c_str());
				// }
				// for (int i = 0; i < outerLine.size(); i++)
				// {
				// 	LOGINFO("--0726 outerLine [%d] is [%s]", i, outerLine[i].dump_perl().c_str());
				// }
				// LOGINFO("0725 innerLine[0] = %s", innerLine[0].dump_perl().c_str());
				// LOGINFO("0725 Line[0] = %s", pvolume->p_spted_vol->mesh.wallLoops_vec[lines_id][0].dump_perl().c_str());
				// LOGINFO("0725 outerLine[0] = %s", outerLine[0].dump_perl().c_str());
				Hexhole_LineWall_Mesh *pWall_inner = new Hexhole_LineWall_Mesh(
					innerLine,
					false,
					pvolume->SurpportParams["thickness_hl"],
					pvolume->SurpportParams["tooth_insert_hl"],
					pvolume->SurpportParams["tooth_depth_hl"],
					pvolume->SurpportParams["down_tooth_depth_hl"],
					pvolume->SurpportParams["tooth_step_hl"],
					pvolume->SurpportParams["tooth_width_hl"],
					pvolume->SurpportParams["hole_height_step_hl"],
					pvolume->SurpportParams["hole_length_step_hl"],
					pvolume->SurpportParams["hole_depth_hl"],
					pvolume->SurpportParams["wall_split_step_hl"],
					pvolume->SurpportParams["wall_split_wedth_hl"],
					pvolume->SurpportParams["hexhole_length_hl"],
					pvolume->SurpportParams["If_addBeam_hl"],
					pvolume->SurpportParams["Beam_Width_hl"]);

				TriangleMesh WallMesh = pWall_inner->GeneralMesh();

				Hexhole_LineWall_Mesh *pWall_outer = new Hexhole_LineWall_Mesh(
					outerLine,
					false,
					pvolume->SurpportParams["thickness_hl"],
					pvolume->SurpportParams["tooth_insert_hl"],
					pvolume->SurpportParams["tooth_depth_hl"],
					pvolume->SurpportParams["down_tooth_depth_hl"],
					pvolume->SurpportParams["tooth_step_hl"],
					pvolume->SurpportParams["tooth_width_hl"],
					pvolume->SurpportParams["hole_height_step_hl"],
					pvolume->SurpportParams["hole_length_step_hl"],
					pvolume->SurpportParams["hole_depth_hl"],
					pvolume->SurpportParams["wall_split_step_hl"],
					pvolume->SurpportParams["wall_split_wedth_hl"],
					pvolume->SurpportParams["hexhole_length_hl"],
					pvolume->SurpportParams["If_addBeam_hl"],
					pvolume->SurpportParams["Beam_Width_hl"]);

				TriangleMesh WallMesh_Outer = pWall_outer->GeneralMesh();
				// WallMesh_Outer.write_ascii("0725Wall-2.stl");

				WallMesh.MergeCorrespondingMesh(&WallMesh_Outer);
				// WallMesh_Outer.write_ascii("0725WallMesh_Result.stl");
				pvolume->mesh = WallMesh;
				// LOGINFO("0725,the WallMesh facets is %d", pvolume->mesh.stl.stats.number_of_facets);
				pvolume->solider = true;
				if(pWall_inner != nullptr)
				{
					delete pWall_inner;	
					pWall_inner = nullptr;
				}
				if(pWall_outer != nullptr)
				{
					delete pWall_outer;	
					pWall_outer = nullptr;
				}
				return true;
			}
			else
			{
				for (int i = 0; i < pvolume->p_spted_vol->mesh.wallLoops_vec[lines_id].size(); i++)
				{
					LOGINFO("--0726 Line [%d] is [%s]", i, pvolume->p_spted_vol->mesh.wallLoops_vec[lines_id][i].dump_perl().c_str());
				}
				Hexhole_LineWall_Mesh *pWall = new Hexhole_LineWall_Mesh(
					pvolume->p_spted_vol->mesh.wallLoops_vec[lines_id],
					false,
					pvolume->SurpportParams["thickness_hl"],
					pvolume->SurpportParams["tooth_insert_hl"],
					pvolume->SurpportParams["tooth_depth_hl"],
					pvolume->SurpportParams["down_tooth_depth_hl"],
					pvolume->SurpportParams["tooth_step_hl"],
					pvolume->SurpportParams["tooth_width_hl"],
					pvolume->SurpportParams["hole_height_step_hl"],
					pvolume->SurpportParams["hole_length_step_hl"],
					pvolume->SurpportParams["hole_depth_hl"],
					pvolume->SurpportParams["wall_split_step_hl"],
					pvolume->SurpportParams["wall_split_wedth_hl"],
					pvolume->SurpportParams["hexhole_length_hl"],
					pvolume->SurpportParams["If_addBeam_hl"],
					pvolume->SurpportParams["Beam_Width_hl"]);

				pvolume->mesh = pWall->GeneralMesh();
				if(pWall != nullptr)
				{
					delete pWall;	
					pWall = nullptr;
				}
				return true;
			}
		}

		// 构建实体墙流程
		if (pvolume->SurpportType == "boundary_wall")
		{
			unsigned int lines_id = pvolume->SurpportParams["lines_id"];
			unsigned int parts_id = pvolume->SurpportParams["parts_id"];
			EntityWall_Mesh *pWall = new EntityWall_Mesh(
				pvolume->p_spted_vol->mesh.wallLoops_vec[lines_id],
				false,
				pvolume->SurpportParams["tooth_insert_e"],
				pvolume->SurpportParams["tooth_depth_e"],
				pvolume->SurpportParams["thickness_e"],
				pvolume->SurpportParams["contact_thickness_e"],
				pvolume->SurpportParams["tooth_step_e"],
				pvolume->SurpportParams["hole_step_e"],
				pvolume->SurpportParams["hole_height_step_e"],
				pvolume->SurpportParams["hole_length_step_e"],
				pvolume->SurpportParams["hole_depth_e"],
				pvolume->SurpportParams["wall_split_step_e"],
				pvolume->SurpportParams["wall_split_wedth_e"],
				pvolume->SurpportParams["wall_cross_wedth_e"]);
			LOGINFO(" tooth_insert_e = %f", pvolume->SurpportParams["tooth_insert_e"]);
			LOGINFO(" tooth_depth_e = %f", pvolume->SurpportParams["tooth_depth_e"]);
			LOGINFO(" thickness_e = %f", pvolume->SurpportParams["thickness_e"]);
			LOGINFO(" tooth_step_e = %d", pvolume->SurpportParams["tooth_step_e"]);
			LOGINFO(" hole_step_e = %d", pvolume->SurpportParams["hole_step_e"]);
			LOGINFO(" hole_height_step_e = %d", pvolume->SurpportParams["hole_height_step_e"]);
			LOGINFO(" hole_length_step_e = %d", pvolume->SurpportParams["hole_length_step_e"]);
			LOGINFO(" hole_depth_e = %f", pvolume->SurpportParams["hole_depth_e"]);
			LOGINFO(" wall_split_step_e = %d", pvolume->SurpportParams["wall_split_step_e"]);
			LOGINFO(" wall_split_wedth_e = %d", pvolume->SurpportParams["wall_split_wedth_e"]);
			LOGINFO(" wall_cross_wedth_e = %f", pvolume->SurpportParams["wall_cross_wedth_e"]);

			pvolume->mesh = pWall->GeneralMesh(parts_id);
			if(pWall != nullptr)
			{
				delete pWall;	
				pWall = nullptr;
			}

			return true;
		}
		// 借用数字流程构造stl数字
		if (pvolume->SurpportType == "num_mark")
		{

			unsigned int mark_id = pvolume->SurpportParams["mark_num"];
			unsigned int is_side = pvolume->SurpportParams["is_side"];
			double side_depth = pvolume->SurpportParams["side_depth"];
			double num_size = 0.5 + pvolume->SurpportParams["num_size"] / 10;
			LOGINFO(" pvolume->upPickPos = %s", pvolume->upPickPos.dump_perl().c_str());
			LOGINFO(" pvolume->SurpportParams.mark_num = %d", mark_id);
			if (is_side != 0) // 侧壁标记,不做数字标签
			{

				Side_STL_NumMark_Mesh *pMark;
				LOGINFO("------------------start to make STL_numMark------------------");
				if (pvolume->automark)
				{
					LOGINFO(" num_mark auto_mark is true!!!");
					pMark = new Side_STL_NumMark_Mesh(
						pvolume->p_spted_vol->mesh,
						mark_id,
						side_depth,
						num_size);
				}
				else
				{
					LOGINFO(" num_mark auto_mark is false???");
					LOGINFO(" pvolume->midPickPos = %s, pvolume->MidNarmol = %s", pvolume->midPickPos.dump_perl().c_str(), pvolume->MidNarmol.dump_perl().c_str());

					pMark = new Side_STL_NumMark_Mesh(
						pvolume->p_spted_vol->mesh,
						pvolume->midPickPos,
						pvolume->MidNarmol,
						mark_id,
						side_depth,
						num_size);
				}
				pvolume->mesh = pMark->GeneralMesh();
				if(pMark != nullptr)
				{
					delete pMark;	
					pMark = nullptr;
				}
			}
			else
			{
				LOGINFO("now only support the side stl nummark");
				TriangleMesh emptymesh;
				pvolume->mesh = emptymesh;
			}

			return true;
		}

		if (pvolume->SurpportType == "char_mark")
		{
			std::string str = pvolume->CharMark;
			double char_depth = pvolume->SurpportParams["char_depth"];
			double char_height = pvolume->SurpportParams["char_height"];
			double char_size = (pvolume->SurpportParams["char_size"] - 1) / 5.0 + 0.5;
			pvolume->concave = false; // 是否内凹
			double char_gap = pvolume->SurpportParams["char_gap"];
			char_depth = 0;
			if (str.size() == 0)
			{
				return false;
			}
			LOGINFO("make_SptMesh, str = [%s]", str.c_str());
			// LOGINFO("char_depth, double = [%lf]", char_depth);
			// LOGINFO("char_height, double = [%lf]", char_height);
			// LOGINFO("char_size, double = [%lf]", char_size);
			// LOGINFO("concave, double = [%lf]", pvolume->concave);
			// LOGINFO("char_gap, double = [%lf]", char_gap);
			Side_STL_CharMark_Mesh *pMark;
			pvolume->adder = true;
			if (pvolume->automark)
			{
				pMark = new Side_STL_CharMark_Mesh(
					pvolume->p_spted_vol->mesh,
					str,
					char_height, // mark_thickness
					char_depth,	 // insert_thickness
					3.0,
					1.2,
					char_size,
					char_gap);
			}
			else
			{
				if (pvolume->CharSize != 1)
				{
					pMark = new Side_STL_CharMark_Mesh(
						pvolume->p_spted_vol->mesh,
						pvolume->midPickPos,
						pvolume->MidNarmol,
						str,
						char_height, // mark_thickness
						char_depth,	 // insert_thickness
						3.0,
						1.2,
						pvolume->CharSize / 3.5,
						char_gap);
				}
				else
				{
					LOGINFO(" ---0209---- pvolume->midPickPos = %s, pvolume->MidNarmol = %s", pvolume->midPickPos.dump_perl().c_str(), pvolume->MidNarmol.dump_perl().c_str());
					pMark = new Side_STL_CharMark_Mesh(
						pvolume->p_spted_vol->mesh,
						pvolume->midPickPos,
						pvolume->MidNarmol,
						str,
						char_height, // mark_thickness
						char_depth,	 // insert_thickness
						3.0,
						1.2,
						char_size,
						char_gap);
				}
			}
			pvolume->mesh = pMark->GeneralMesh();
			pvolume->mesh.checkonly = true;
			// pvolume->mesh.repair(1);
			pvolume->CharMark_IsValid = pMark->STL_CharMark_IsValid;
			pvolume->midPickPos = pMark->MidPoint;
			int nearid = -1;
			double near_dis = 100000000;
			for (int i = 0; i < pvolume->mesh.facets_count();i++)
			{
				Pointf3 tmp;
				tmp.x = pvolume->mesh.stl.facet_start[i].vertex[0].x;
				tmp.y = pvolume->mesh.stl.facet_start[i].vertex[0].y;
				tmp.z = pvolume->mesh.stl.facet_start[i].vertex[0].z;

				double dis = tmp.distance_to(pvolume->midPickPos);
				if (dis < near_dis)
				{
					nearid = i;
					near_dis = dis;
				}
			}
			TriangleMesh selectedMesh = pvolume->mesh.GetSubMesh(nearid);
			// selectedMesh.write_ascii("selectedMesh.stl");
			pvolume->MidNarmol = selectedMesh.mesh_average_normal();
			pvolume->automark = false;
			if (pvolume->CharMark_IsValid)
			{
				LOGINFO("0607 CharMark_IsValid!");
			}
			else
			{
				LOGINFO("0607 --- CharMark IsNotValid!");
			}
			if(pMark != nullptr)
			{
				delete pMark;	
				pMark = nullptr;
			}

			return true;
		}

		double rotate_deg = pvolume->SurpportParams["rotate_angel"] * PI / 180;
		// 构建mesh
		SingleSPT_BaseMesh *pSPT = NULL;
		if (pvolume->SurpportType == "squareprism")
		{
			pSPT = new Cuber_Mesh(pvolume->dwPickPos, pvolume->upPickPos, rotate_deg, pvolume->SurpportParams["box_X"], pvolume->SurpportParams["box_Y"]);
		}
		else if (pvolume->SurpportType == "crossprism")
		{
			pSPT = new CrossPrism_Mesh(pvolume->dwPickPos, pvolume->upPickPos, rotate_deg, pvolume->SurpportParams["cross_X"], pvolume->SurpportParams["cross_Y"]);
		}
		else if (pvolume->SurpportType == "cylinder")
		{
			pSPT = new Cylinder_Mesh(pvolume->dwPickPos, pvolume->upPickPos, rotate_deg, pvolume->SurpportParams["cyl_R"], 18);
		}
		else if (pvolume->SurpportType == "treebase")
		{
			pSPT = new Cylinder_Mesh(pvolume->dwPickPos, pvolume->upPickPos, rotate_deg, pvolume->SurpportParams["cyl_R"] /**(1.5)*/, 18, true, false);
		}
		else if (pvolume->SurpportType == "facet_single")
		{
			LOGINFO("fsingle_l = %f", pvolume->SurpportParams["fsingle_l"]);
			pSPT = new SliceSingle_Mesh(pvolume->dwPickPos, pvolume->upPickPos, rotate_deg, pvolume->SurpportParams["fsingle_l"], SliceType((int)(pvolume->SurpportParams["shape_type"])));
		}
		else
		{
			LOGINFO("SurpportType error %s", pvolume->SurpportType.c_str());
		}
		// 支撑主体
		if (pSPT != NULL)
		{
			// 添加附件
			if (pvolume->SurpportParams["IsContactDown"])
				pSPT->Add_Contact(true, pvolume->SurpportParams["contactDown_X"], pvolume->SurpportParams["contactDown_Y"], pvolume->SurpportParams["contactDown_Z"]);
			if (pvolume->SurpportParams["IsContactUp"])
				pSPT->Add_Contact(false, pvolume->SurpportParams["contactUp_X"], pvolume->SurpportParams["contactUp_Y"], pvolume->SurpportParams["contactUp_Z"]);
			if (pvolume->SurpportParams["IsShape"])
			{
				LOGINFO("0719 add the uppyramid:: shape_Z is [%f] shape_density is [%f]", pvolume->SurpportParams["shape_Z"], pvolume->SurpportParams["shape_density_1"] / 100);
				pSPT->Add_Pyramid(false, pvolume->SurpportParams["shape_Z"], pvolume->SurpportParams["shape_density_1"] / 100);
			}
			if (pvolume->SurpportParams["IsShape_Down"])
			{
				LOGINFO("0719 add the downpyramid:: shape_Z_Down is [%f] shape_density_Down is [%f]", pvolume->SurpportParams["shape_Z_Down"], pvolume->SurpportParams["shape_density_Down"] / 100);
				pSPT->Add_Pyramid(true, pvolume->SurpportParams["shape_Z_Down"], pvolume->SurpportParams["shape_density_Down"] / 100);
			}
			// 特殊构造
			if (pvolume->SurpportParams["IsContactAutoFix"])
				pSPT->SetMeshAutoFix(false, pvolume->TopNarmol);
			if (pvolume->SurpportParams["IsHollow"])
				pSPT->SetMeshHollow(pvolume->SurpportParams["hollow_density"] / 100);
			pvolume->mesh = pSPT->GeneralMesh();
			if(pSPT != nullptr)
			{
				delete pSPT;	
				pSPT = nullptr;
			}
		}
		else
		{
			LOGINFO("pSPT == NULL!");
			return false;
		}

		return true;
	}

	// 解析支撑参数
	// facet_cross = facet_single + facet_single
	bool ModelVolume::analyze_SptParams()
	{

		LOGINFO("analyze_SptParams start");

		this->modifier = false;
		this->surpporter = true;
		this->solider = (this->SurpportType == "squareprism" ||
						 this->SurpportType == "crossprism" ||
						 this->SurpportType == "cylinder" ||
						 this->SurpportType == "treebase" ||
						 this->SurpportType == "num_mark" ||
						 this->SurpportType == "char_mark" ||
						 this->SurpportType == "boundary_wall" ||
						 this->SurpportType == "crossbar" ||
						 this->SurpportType == "assist_crossbar" ||
						 this->SurpportType == "auto_crossbar" ||
						 this->SurpportType == "auto_assist_crossbar");
		// 实体附加物判定
		this->adder = false;
		this->adder = (this->SurpportType == "num_mark" && this->SurpportParams["is_side"] != 0) ||											  // 实体附加物  侧壁数字标签
					  (this->SurpportType == "char_mark");																					  // 实体附加物  外凸字符标签
		if ((this->SurpportType == "crossbar" || this->SurpportType == "auto_crossbar") && this->SurpportParams["cross_bar_is_special"] == 0) // 实心横杆
			this->adder = true;
		if ((this->SurpportType == "assist_crossbar" || this->SurpportType == "auto_assist_crossbar") && this->SurpportParams["assist_cross_bar_is_special"] == 0) // 实心辅助杆
			this->adder = true;
		this->concave = false; // 齿科暂不提供关于内凹字符的功能 20230504
		// this->concave = this->SurpportType == "char_mark" && this->SurpportParams["is_concave"] != 0; // 实体减除物  内凹字符标签
		this->mark = this->SurpportType == "char_mark";
		if (this->solider)
			LOGINFO("SurpportType[%s] is solid", this->SurpportType.c_str());
		else
			LOGINFO("SurpportType[%s] is nonsolid", this->SurpportType.c_str());

		if ((this->SurpportType == "facet_cross" || this->SurpportType == "facet_single") && this->SurpportParams["IsSawTooth"])
		{ // 非实体多锯齿流程 放在perl中处理
			LOGINFO("IsSawTooth will not in procesing in C++, error");
			return false;
		}
		else if (this->SurpportType == "facet_cross")
		{ // 单十字面片流程
			// 主片
			ModelVolume volume1(this->object, *this);
			volume1.SurpportType = "facet_single";
			volume1.SurpportParams["fsingle_l"] = this->SurpportParams["fcross_l"];
			Slic3r::ModelVolume::make_SptMesh(&volume1);
			// 辅片
			ModelVolume volume2(this->object, *this);
			volume2.SurpportType = "facet_single";
			volume2.SurpportParams["fsingle_l"] = this->SurpportParams["fcross_l"];
			double dwContactZ = (volume1.SurpportParams["IsContactDown"]) * (volume1.SurpportParams["contactDown_Z"]);
			double upContactZ = (volume1.SurpportParams["IsContactUp"]) * (volume1.SurpportParams["contactUp_Z"]);
			volume2.dwPickPos.z += dwContactZ;
			volume2.upPickPos.z -= upContactZ;
			volume2.SurpportParams["IsContactDown"] = 0;
			volume2.SurpportParams["IsContactUp"] = 0;
			volume2.SurpportParams["rotate_angel"] += 90;
			Slic3r::ModelVolume::make_SptMesh(&volume2);

			this->mesh = volume1.mesh;
			this->mesh.merge(volume2.mesh);
		}
		else if (this->SurpportType == "treebase" || this->SurpportType.find("single_wall") != std::string::npos)
		{
			this->SurpportParams["IsContactDown"] = 0;
			this->SurpportParams["IsContactUp"] = 0;
			this->SurpportParams["rotate_angel"] = 0;
			this->SurpportParams["IsContactAutoFix"] = 0;
			this->SurpportParams["IsHollow"] = 0;
			this->SurpportParams["IsShape"] = 0;
			// this->SurpportParams["IsShape_Down"] = 0;
			this->SurpportParams["IsSawTooth"] = 0;
			Slic3r::ModelVolume::make_SptMesh(this);
		}
		else if (this->SurpportType == "crossbar" || this->SurpportType == "assist_crossbar" || this->SurpportType == "auto_crossbar" || this->SurpportType == "auto_assist_crossbar")
		{
			this->b_cross_bar = true; // 做上横杆标记
			Slic3r::ModelVolume::make_SptMesh(this);
		}
		else
		{
		    // 其他流程
			Slic3r::ModelVolume::make_SptMesh(this);
		}

		// 空mesh
		if (this->mesh.facets_count() == 0)
		{
			LOGINFO("this->mesh.facets_count() ==0");
			return false;
		}

		return true;
	}

	// 校正参数
	bool
	ModelVolume::verify_SptParams()
	{
		// 对晶格合并的物体  只能添加晶格支撑
		if (this->object->has_LatticeMerge_volume() && this->SurpportType.find("lattice") == std::string::npos)
		{
			LOGINFO("this->object->has_LatticeMerge_volume() can not add spt except Lattice");
			return false;
		}
		// 对已经添加了合并晶格支撑的物体  不能再加晶格
		if (this->object->has_AddMergeLattice() && this->SurpportType.find("lattice") != std::string::npos)
		{
			LOGINFO("this->object->has_AddMergeLattice() can not add Lattice spt again");
			return false;
		}

		// 晶格相关支撑
		if (this->SurpportType.find("lattice") != std::string::npos)
		{
			return true;
		}

		if (this->SurpportType.find("char_mark") != std::string::npos)
			return true;

		if (this->SurpportType.find("num_mark") != std::string::npos)
			return true;

		if (this->SurpportType.find("auto_crossbar") != std::string::npos)
			return true;
		if (this->SurpportType.find("auto_assist_crossbar") != std::string::npos)
			return true;
		if (this->SurpportType.find("wall") != std::string::npos)
		{
			if (this->p_spted_vol == NULL)
			{
				LOGINFO("p_spted_vol == NULL");
				return false;
			}
			unsigned int lines_id = this->SurpportParams["lines_id"];
			if (this->SurpportType.find("boundary") != std::string::npos)
			{
				unsigned int loopsize = this->p_spted_vol->mesh.wallLoops_vec.size();
				if (lines_id < 0 || lines_id >= loopsize)
				{
					LOGINFO("lines_id out of range [%u / %u]", lines_id, loopsize);
					return false;
				}
			}
			else if (this->SurpportType.find("stripe") != std::string::npos)
			{
				unsigned int loopsize = this->p_spted_vol->mesh.tuqiLoops_vec.size();
				if (lines_id < 0 || lines_id >= loopsize)
				{
					LOGINFO("lines_id out of range [%u / %u]", lines_id, loopsize);
					return false;
				}
			}
			else
			{
				LOGINFO("SurpportType error [%s]", this->SurpportType.c_str());
				return false;
			}

			return true;
		}

		if (this->dwPickPos.z < 0.0)
			this->dwPickPos.z = 0.0;
		if (this->upPickPos.z < 0.0)
			this->upPickPos.z = 0.0;
		// 支撑最小高度不得小于0.1
		if (this->upPickPos.z - this->dwPickPos.z < 0.1)
		{
			LOGINFO("Spt_height too short. up=%s, dw=%s", this->upPickPos.dump_perl().c_str(), this->dwPickPos.dump_perl().c_str());
			return false;
		}
		// 插入深度补偿
		this->upPickPos.z += this->SurpportParams["insert_Z"];
		// 补偿后的支撑总高度
		double Spt_height = this->upPickPos.z - this->dwPickPos.z;

		LOGINFO("1020 volume [%s] Spt_height [%f] up=%s, dw=%s\n", this->name.c_str(), Spt_height, this->upPickPos.dump_perl().c_str(), this->dwPickPos.dump_perl().c_str());

		// 检查是否有足够的空间 添加支撑
		if (this->SurpportParams["IsContactUp"])
			Spt_height -= this->SurpportParams["contactUp_Z"];
		if (this->SurpportParams["IsContactDown"])
			Spt_height -= this->SurpportParams["contactDown_Z"];
		if (Spt_height < 0.1)
		{
			LOGINFO("not enough height, cancel contact");
			this->SurpportParams["IsContactUp"] = 0;
			this->SurpportParams["IsContactDown"] = 0;
		}
		// 检查是否足够的空间加锥体
		if (this->SurpportParams["IsShape"])
		{
			if (Spt_height - this->SurpportParams["shape_Z"] < 0.1)
				this->SurpportParams["shape_Z"] = Spt_height - 0.1;
		}
		LOGINFO("0719 this->SurpportParams shape_Z_Down = %f", this->SurpportParams["shape_Z_Down"]);
		// if (this->SurpportParams["IsShape_Down"])
		// {
		// 	if (Spt_height - this->SurpportParams["shape_Z_Down"] < 0.1)
		// 		this->SurpportParams["shape_Z_Down"] = 0.0;
		// }

		// 非垂直柱体不开启自动贴合功能
		if (this->SurpportParams["IsContactAutoFix"])
		{
			Pointf3 spt_dir = this->upPickPos - this->dwPickPos;
			if (spt_dir.x != 0 || spt_dir.y != 0)
			{
				this->SurpportParams["IsContactAutoFix"] = 0;
			}
		}

		return true;
	}

	Pointf3 ModelVolume::ModifyPoint_CrossBarCenter(Pointf3 pt)
	{
		if (this->b_cross_bar == false)
		{
			LOGINFO("this->b_cross_bar == false");
			return pt;
		}
		Pointf3 _ret = pt;
		// 横杆加在横杆上，进行辅助修正
		Pointf3 crossbar_begin = this->upPickPos;
		Pointf3 crossbar_end = this->dwPickPos;
		Pointf3 dvector = crossbar_begin - crossbar_end;
		LOGINFO("Volume name[%s], upPickPos[%s], dwPickPos[%s] dvector[%s]",
				this->name.c_str(), this->upPickPos.dump_perl().c_str(), this->dwPickPos.dump_perl().c_str(), dvector.dump_perl().c_str());
		double dvector_Length = dvector.length();
		if (dvector_Length <= 0.01)
		{
			LOGINFO("dvector_Length <= 0.01 [%f]", dvector_Length);
			return pt;
		}
		double u =
			(pt.x - crossbar_begin.x) * (crossbar_begin.x - crossbar_end.x) +
			(pt.y - crossbar_begin.y) * (crossbar_begin.y - crossbar_end.y) +
			(pt.z - crossbar_begin.z) * (crossbar_begin.z - crossbar_end.z);
		u = u / (dvector_Length * dvector_Length);
		_ret = crossbar_begin + u * dvector;
		LOGINFO("Orgin upPickPos %s   Modify upPickPos %s", pt.dump_perl().c_str(), _ret.dump_perl().c_str());
		return _ret;
	}
	// 从面片集A中排除所有面片集B的元素
	bool ModelVolume::remove_crossbar_facets(std::vector<int> &region_facets, std::vector<int> crossbar_facets)
	{
		if (region_facets.size() == 0)
		{
			return false;
		}
		LOGINFO("1218 before filter region_facets num == %d)", region_facets.size());
		auto remove_if_condition = [&crossbar_facets](int element)
		{
			return std::find(crossbar_facets.begin(), crossbar_facets.end(), element) != crossbar_facets.end();
		};
		region_facets.erase(std::remove_if(region_facets.begin(), region_facets.end(), remove_if_condition), region_facets.end());
		LOGINFO("1218 after filter region_facets num == %d)", region_facets.size());
		return true;
	}
	// 使用面片过滤，达成横杆和墙支撑的避让策略
	// 原本想将横杆都合并然后与模型求交叉的面片，但是这样就太复杂了
	bool ModelVolume::Avoid_CrossBar_Facets(int expand_degree)
	{
		if (this->surpporter)
		{
			return false;
		}
		if (this->mesh.RPD_selected_facets.empty())
		{
			return false;
		}
		LOGINFO("1218 this->crossbar_num == %d)", this->mesh.Crossbar_Points.size());
		LOGINFO("1218 this->assist crossbar_num == %d)", this->mesh.Assit_Crossbar_Points.size());

		std::vector<CrossbarPointPair> Crossbar_PointPairs;
		Crossbar_PointPairs.clear();
		// Crossbar_PointPairs.insert(Crossbar_PointPairs.end(), this->mesh.Crossbar_Points.begin(), this->mesh.Crossbar_Points.end());
		// Crossbar_PointPairs.insert(Crossbar_PointPairs.end(), this->mesh.Assit_Crossbar_Points.begin(), this->mesh.Assit_Crossbar_Points.end());
		for (int i = 0; i < this->object->volumes.size(); i++)
		{
			if (this->object->volumes[i]->b_cross_bar)
			{
				Crossbar_PointPairs.push_back(this->object->volumes[i]->crossbar_pointpair);
			}
		}
		LOGINFO("1218 this->crossbar_num == %d)", Crossbar_PointPairs.size());
		std::vector<int> crossbar_filter_facets;
		for (int i = 0; i < Crossbar_PointPairs.size(); i++)
		{
			CrossbarPointPair points_pair = Crossbar_PointPairs[i];
			int up_facet_id = this->mesh.GetFacetIdxByPickPoint(points_pair.first);
			std::vector<int> expand_facets1 = this->mesh.ExpandFacetbyRing_BFS(up_facet_id, expand_degree);
			LOGINFO("1218 this up point expand_facets == %d)", expand_facets1.size());
			crossbar_filter_facets.insert(crossbar_filter_facets.end(), expand_facets1.begin(), expand_facets1.end());
			int down_facet_id = this->mesh.GetFacetIdxByPickPoint(points_pair.second);
			std::vector<int> expand_facets2 = this->mesh.ExpandFacetbyRing_BFS(down_facet_id, expand_degree);
			LOGINFO("1218 this down point expand_facets == %d)", expand_facets2.size());
			crossbar_filter_facets.insert(crossbar_filter_facets.end(), expand_facets2.begin(), expand_facets2.end());
		}
		LOGINFO("1218 this crossbar_filter_facets == %d)", crossbar_filter_facets.size());

		remove_crossbar_facets(this->mesh.RPD_selected_facets[0], crossbar_filter_facets);
		remove_crossbar_facets(this->mesh.RPD_selected_facets[1], crossbar_filter_facets);
		remove_crossbar_facets(this->mesh.RPD_selected_facets[2], crossbar_filter_facets);
	}
	int ModelVolume::Get_LatticeTopSize()
	{
		int _ret = 0;
		if (this->lattice_support_ptr == NULL)
		{
			LOGINFO("this->lattice_support_ptr == NULL");
			return _ret;
		}

		if (this->lattice_support_ptr->pObj == NULL)
		{
			LOGINFO("this->lattice_support_ptr->pObj == NULL");
			return _ret;
		}

		_ret = this->lattice_support_ptr->pObj->Lspt_Points.size();

		return _ret;
	}
	void removeClosePoints(LatticeSPT_Pointf3s &points, double threshold)
	{
		for (auto it = points.begin(); it != points.end();)
		{
			bool removed = false;
			for (auto jt = it + 1; jt != points.end();)
			{
				if (it->UpPoint.distance_toXY(jt->UpPoint) < threshold)
				{
					// 如果距离过近，则保留较低的一个点
					if (it->UpPoint.z < jt->UpPoint.z)
					{
						jt = points.erase(jt);
					}
					else
					{
						it = points.erase(it);
						removed = true;
						break;
					}
				}
				else
				{
					++jt;
				}
			}
			if (!removed)
			{
				++it;
			}
		}
	}
	bool Check_Point_in_RPDRegion(TriangleMesh *Pobj, std::vector<int> mergedVector, Pointf targetPoint, double radius)
	{
		std::vector<Pointf> testPoints;
		for (int i = 0; i < 9; i++)
		{
			Pointf tempPoint = Pointf(targetPoint.x + radius * cos(40 * i * PI / 180), targetPoint.y + radius * sin(40 * i * PI / 180));
			SPT_pointf tempsptPoint = SPT_pointf(tempPoint, 0, 0);
			bool hit = Pobj->Cale_CollsionZ(tempsptPoint);
			if (!hit)
			{
				continue;
			}
			int facetID = tempsptPoint.get_mincollsionFaceID();
			auto it = std::find(mergedVector.begin(), mergedVector.end(), facetID);
			if (it != mergedVector.end()) // 考虑到树支撑的粗细进行树支撑和墙支撑的避让。
			{
				return true;
			}
		}
		return false;
	}
	bool ModelVolume::Genenal_LatticeTree_Points(coordf_t _x_cell_length,
												 coordf_t _y_cell_length,
												 coordf_t _z_cell_length,
												 coordf_t _suspend_angel,
												 coordf_t _lattice_slice_thickness,
												 coordf_t _lattice_spt_distance,
												 double tree_support_minispace,
												 int addPoints) // 以晶格采样点的方式来进行生成树枝点
	{

		TriangleMesh Obj = this->mesh;
		TriangleMesh *pObj = &Obj;
		Obj.repair();
		if (Obj.UnableRepair())
		{
			LOGINFO("Genenal_LatticeTree_Points , Obj Unable Repair");
			return false;
		}
		coordf_t lattice_slice_thickness = _lattice_slice_thickness; // 切片层厚
		coordf_t suspend_angel = _suspend_angel;					 // 识别为悬垂面的角度阈值
		coordf_t lattice_spt_distance = _lattice_spt_distance;		 // 识别为悬垂面的角度阈值
		LOGINFO("Genenal_LatticeTree_Points lattice_slice_thickness = %f", lattice_slice_thickness);
		LOGINFO("Genenal_LatticeTree_Points suspend_angel = %f", suspend_angel);
		LOGINFO("Genenal_LatticeTree_Points lattice_spt_distance = %f", lattice_spt_distance);

		DWORD timecount = GetTickCount();
		DWORD tt = timecount;
		// 根据切片获得待加的支撑点  没有faceID信息
		LatticeSPT_PointsGenerate Test_slice(Obj, lattice_slice_thickness, suspend_angel, lattice_spt_distance);
		Test_slice.Slice_Obj();
		LOGINFO("Slice_Obj cost_times =%d", GetTickCount() - timecount);
		timecount = GetTickCount();
		pObj->Lspt_Points = Test_slice.GetSPT_PointsVec();
		LOGINFO("1111 -- Test_slice Lspt_Points size =[%d]", pObj->Lspt_Points.size());
		LOGINFO("GetSPT_PointsVec cost_times =%d", GetTickCount() - timecount);
		timecount = GetTickCount();
		// 对接触的支撑点进行修正  并赋值faceID信息
		pObj->Modify_SuspendPoints(lattice_spt_distance / 5, lattice_spt_distance / 5, lattice_slice_thickness);
		LOGINFO("Modify_SuspendPoints cost_times =%d", GetTickCount() - timecount);
		LOGINFO("1111 -- Modify_SuspendPoints Lspt_Points size =[%d]", pObj->Lspt_Points.size());
		timecount = GetTickCount();
		// 对支撑点进行简化
		coordf_t x_cell_length = _x_cell_length; // 简化晶格X轴向长度
		coordf_t y_cell_length = _y_cell_length; // 简化晶格Y轴向长度
		coordf_t z_cell_length = _z_cell_length; // 简化晶格Z轴向长度
		coordf_t reten_factor = 1.0;			 // 固位网区域简化缩放因子
		coordf_t tuqi_factor = 1.0;				 // 花纹区域简化缩放因子
		LOGINFO("init_contact_support_points x_cell_length = %f", x_cell_length);
		LOGINFO("init_contact_support_points y_cell_length = %f", y_cell_length);
		LOGINFO("init_contact_support_points z_cell_length = %f", z_cell_length);
		LOGINFO("Genenal_LatticeTree_Points reten_factor = %f", reten_factor);
		LOGINFO("Genenal_LatticeTree_Points tuqi_factor = %f", tuqi_factor);
		pObj->Simply_SuspendPoints(x_cell_length, y_cell_length, z_cell_length, reten_factor, tuqi_factor);
		LOGINFO("1111 -- Simply_SuspendPoints size =[%d]", pObj->Lspt_Points.size());
		LatticeSPT_Pointf3s contact_support_points; // 顶部的支撑接触点
		contact_support_points = pObj->Lspt_Points;
		LOGINFO("Simply_SuspendPoints cost_times =%d", GetTickCount() - timecount);
		LOGINFO("Genenal_LatticeTree_Points cost_times =%d", GetTickCount() - tt);
		LOGINFO("contact_support_points size =[%d]", contact_support_points.size());
		removeClosePoints(contact_support_points, tree_support_minispace);
		LOGINFO("removeClosePoints contact_support_points size =[%d]", contact_support_points.size());
		this->mesh.custom_GridPoints.clear();
		this->mesh.RetionPart_Points.clear();
		this->mesh.LatticeMesh_Points.clear();

		std::vector<int> mergedVector;
		if (!this->mesh.RPD_selected_facets.empty())
		{
			for (const auto &innerVector : this->mesh.RPD_selected_facets)
			{
				mergedVector.insert(mergedVector.end(), innerVector.begin(), innerVector.end());
			}
		}
		LOGINFO("merged Facets size =[%d]", mergedVector.size());
		for (int i = 0; i < contact_support_points.size(); i++)
		{
			coord_t tempFaceID = contact_support_points[i].Form_FaceID;
			auto it = std::find(mergedVector.begin(), mergedVector.end(), tempFaceID);
			if (it != mergedVector.end()) // 在加强支撑上面，不使用
			{
				continue;
			}
			if (Check_Point_in_RPDRegion(&this->mesh, mergedVector, contact_support_points[i].UpPoint, 0.4))
			{
				continue;
			}
			size_t side = this->mesh.stl.facet_start[tempFaceID].face_side;
			// if (side != Upper_Face)
			// {
			// 	continue;
			// }
			size_t type = this->mesh.stl.facet_start[tempFaceID].face_type;
			if (type == OcclusalRest_Part || type == Clasp_Part || type == LingualBar_Part)
			{
				continue;
			}

			size_t type_id = this->mesh.stl.facet_start[tempFaceID].face_type;
			size_t Regionid = 10000 * side + 100 * type + type_id;
			SPT_pointf tempP = SPT_pointf(contact_support_points[i].UpPoint, 0, 10000, tempFaceID);
			// LOGINFO("tempP[%s] Regionid[%d] FaceID[%d]", contact_support_points[i].UpPoint.dump_perl().c_str(), Regionid, tempFaceID);
			if (type == RetentionMesh_Part)
			{
				this->mesh.RetionPart_Points.push_back(tempP);
			}
			else
			{
				this->mesh.LatticeMesh_Points.push_back(tempP);
			}
		}
		LOGINFO("LatticeMesh_Points size =[%d]", this->mesh.LatticeMesh_Points.size());
		LOGINFO("RetionPart_Points size =[%d]", this->mesh.RetionPart_Points.size());
		LOGINFO("addPoints is [%d]", addPoints);
		if (addPoints == 1)
		{ // 1内部，2固位网，3两者皆有
			this->mesh.custom_GridPoints.insert(this->mesh.custom_GridPoints.end(), this->mesh.LatticeMesh_Points.begin(), this->mesh.LatticeMesh_Points.end());
		}
		else if (addPoints == 2)
		{
			this->mesh.custom_GridPoints.insert(this->mesh.custom_GridPoints.end(), this->mesh.RetionPart_Points.begin(), this->mesh.RetionPart_Points.end());
		}
		else if (addPoints == 3)
		{
			this->mesh.custom_GridPoints.insert(this->mesh.custom_GridPoints.end(), this->mesh.LatticeMesh_Points.begin(), this->mesh.LatticeMesh_Points.end());
			this->mesh.custom_GridPoints.insert(this->mesh.custom_GridPoints.end(), this->mesh.RetionPart_Points.begin(), this->mesh.RetionPart_Points.end());
		}
		else
		{
			return false;
		}
		if (contact_support_points.empty())
			return false;
		else
			return true;
	}

	bool ModelVolume::Genenal_CrossBar_Tree_Points(double CrossBar_TreePoint_distance)
	{
		LOGINFO("1208 volumename = %s\n", this->name.c_str());
		ModelObject *object = this->get_object();
		LOGINFO("1208 filename = %s\n", object->name.c_str());
		ModelVolumePtrs sptedObj_spts = object->map_sptvol[this];
		// LOGINFO("1208---------------------\n");
		// LOGINFO("1208 [crossbar_midLines] object->map_sptvol[pvolume->p_spted_vol].size() = %d\n", sptedObj_spts.size());
		std::vector<Linef3> crossbar_midLines;
		this->mesh.CrossBar_TreePoints.clear();
		for (size_t i_volume = 0; i_volume < sptedObj_spts.size(); ++i_volume)
		{
			ModelVolume *_volume_p = sptedObj_spts[i_volume];
			if (_volume_p->b_cross_bar == true)
			{
				Linef3 _crossbar_line;
				_crossbar_line.a = _volume_p->upPickPos;
				_crossbar_line.b = _volume_p->dwPickPos;
				crossbar_midLines.push_back(_crossbar_line);
				LOGINFO("[crossbar_midLines] volume[%s] is b_cross_bar", _volume_p->name.c_str());
			}
			else
			{
				LOGINFO("[crossbar_midLines] volume[%s] is not b_cross_bar", _volume_p->name.c_str());
			}
		}
		if (crossbar_midLines.size() == 0)
		{
			return false;
		}
		// 处理横杆中轴线
		for (int i = 0; i < crossbar_midLines.size(); i++)
		{
			LOGINFO("crossbar_midLines.size() = [%d]", crossbar_midLines.size());
			Pointf3s seg_vec = crossbar_midLines[i].GetSegLines(CrossBar_TreePoint_distance, false);
			for (int j = 0; j < seg_vec.size(); j++)
			{
				Pointf3 up = Pointf3(seg_vec[j].x, seg_vec[j].y, seg_vec[j].z);
				// up.rotate(-lattice_grid_angle / 180 * PI);
				SPT_pointf tempP = SPT_pointf(up, 0, 10000, -1);
				this->mesh.CrossBar_TreePoints.push_back(tempP);
			}
		}
		LOGINFO("crossbar_midLines.size() = [%d]", crossbar_midLines.size());
		LOGINFO("contact_support_points.size() = [%d]", this->mesh.CrossBar_TreePoints.size());
		return true;
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//  ModelInstance 定义
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	ModelInstance::ModelInstance(ModelObject *object)
		: rotation(0), scaling_factor(1), object(object)
	{
		offset.x = 0;
		offset.y = 0;
	}

	ModelInstance::ModelInstance(ModelObject *object, const ModelInstance &other)
		: rotation(other.rotation), scaling_factor(other.scaling_factor), offset(other.offset), object(object)
	{
	}

	// 重载操作符 =
	ModelInstance &ModelInstance::operator=(ModelInstance other)
	{
		this->swap(other);
		return *this;
	}

	// 交换两个Instance的值
	void
	ModelInstance::swap(ModelInstance &other)
	{
		std::swap(this->rotation, other.rotation);
		std::swap(this->scaling_factor, other.scaling_factor);
		std::swap(this->offset, other.offset);
	}

	// 将mesh进行Instance的变幻
	void
	ModelInstance::transform_mesh(TriangleMesh *mesh, bool dont_translate) const
	{
		mesh->rotate_z(this->rotation * PI / 180); // rotate around mesh origin
		mesh->scale(this->scaling_factor);		   // scale around mesh origin
		if (!dont_translate)
			mesh->translate(this->offset.x, this->offset.y, 0);
	}

	// 将Mesh进行Instance的变幻  并返回变幻后的boundingbox
	BoundingBoxf3 ModelInstance::transform_mesh_bounding_box(const TriangleMesh *mesh, bool dont_translate) const
	{
		// rotate around mesh origin
		double c = cos(this->rotation * PI / 180);
		double s = sin(this->rotation * PI / 180);
		BoundingBoxf3 bbox;
		for (int i = 0; i < mesh->stl.stats.number_of_facets; ++i)
		{
			const stl_facet &facet = mesh->stl.facet_start[i];
			for (int j = 0; j < 3; ++j)
			{
				stl_vertex v = facet.vertex[j];
				double xold = v.x;
				double yold = v.y;
				v.x = float(c * xold - s * yold);
				v.y = float(s * xold + c * yold);
				v.x *= float(this->scaling_factor);
				v.y *= float(this->scaling_factor);
				v.z *= float(this->scaling_factor);
				if (!dont_translate)
				{
					v.x += this->offset.x;
					v.y += this->offset.y;
				}
				bbox.merge(Pointf3(v.x, v.y, v.z));
			}
		}
		return bbox;
	}

	// 将boundingBox进行当前Instance变幻
	BoundingBoxf3 ModelInstance::transform_bounding_box(const BoundingBoxf3 &bbox, bool dont_translate) const
	{
		// rotate around mesh origin
		double c = cos(this->rotation * PI / 180);
		double s = sin(this->rotation * PI / 180);
		Pointf3 pts[4] = {
			bbox.min,
			bbox.max,
			Pointf3(bbox.min.x, bbox.max.y, bbox.min.z),
			Pointf3(bbox.max.x, bbox.min.y, bbox.max.z)};
		BoundingBoxf3 out;
		for (int i = 0; i < 4; ++i)
		{
			Pointf3 &v = pts[i];
			double xold = v.x;
			double yold = v.y;
			v.x = float(c * xold - s * yold);
			v.y = float(s * xold + c * yold);
			v.x *= this->scaling_factor;
			v.y *= this->scaling_factor;
			v.z *= this->scaling_factor;
			if (!dont_translate)
			{
				v.x += this->offset.x;
				v.y += this->offset.y;
			}
			out.merge(v);
		}
		return out;
	}

	// 将polygon按Instance进行变幻  没有位移变幻
	void
	ModelInstance::transform_polygon(Polygon *polygon) const
	{
		polygon->rotate(this->rotation * PI / 180, Point(0, 0)); // rotate around polygon origin
		polygon->scale(this->scaling_factor);					 // scale around polygon origin
	}

	void
	ModelInstance::reverse_intance()
	{
		this->rotation *= (-1);
		if (this->scaling_factor != 0)
		{
			this->scaling_factor = 1 / this->scaling_factor;
		}
		else
		{
			this->scaling_factor = 1;
		}
		this->offset = this->offset.negative();
	}

	void
	calc_compact_arrange_threads::calc_position_thread(size_t _index, compact_arrange_cale_vec *p_vec, Polygon _bed, bool fast)
	{
		compact_arrange_cale_t *p_arrange = &(p_vec->at(_index));
		coord_t start_x = p_arrange->x_offset;
		coord_t current_y = start_y;

		BoundingBox bed_bb = _bed.bounding_box();
		coord_t bed_width = bed_bb.size().x;
		coord_t bed_max_y = bed_bb.max.y;
		coord_t bed_min_y = bed_bb.min.y;
		coord_t bed_max_x = bed_bb.max.x;
		coord_t bed_min_x = bed_bb.min.x;

		BoundingBoxf3 obj_bb = object->bounding_box();
		coord_t obj_width = scale_(obj_bb.size().x);
		coord_t obj_height = scale_(obj_bb.size().y);
		Pointf3 oc = obj_bb.center();
		Point obj_origin_center = Point(scale_(oc.x), scale_(oc.y));

		Polygon thumbnail_temp(thunbmail.points);
		thumbnail_temp.translate(start_x * step, 0);

		std::map<XYZ_Index, std::vector<int>> temp_hash_facets = object->translate_hash_facets_new(start_x, current_y, 0);
		bool inBed = true;

		// 延y轴向下，直到物体与已排版物体不再相交
		while (model->hash_facets_for_arrange_contain_value(temp_hash_facets))
		{
			p_arrange->res_y--;

			thumbnail_temp.translate(0, -step);
			current_y -= step;

			if (current_y < bed_min_y + obj_height / 2) // 已经超出盘外，表明排不下
			{
				LOGINFO("current_y = [%d], bed_min_y = [%d], obj_height = [%d], current_y < bed_min_y + obj_height / 2",
						current_y, bed_min_y, obj_height);
				inBed = false;
				break;
			}

			if (Slic3r::Geometry::A_Is_In_B(thumbnail_temp, _bed) == false)
			{
				if (false)
				{
					char svg_name[255];
					sprintf(svg_name, "ud_thumbnail_temp[%d][%d].svg", start_x, p_arrange->res_y);
					SVG svg(svg_name);
					svg.draw(thumbnail_temp, "red");
					svg.draw(_bed, "green");
					svg.Close();
				}
				inBed = false;
				break;
			}

			temp_hash_facets = object->translate_hash_facets_new(start_x, p_arrange->res_y, 0);
		}
		p_arrange->can_arrange = !inBed;
	}

	bool
	calc_compact_arrange_threads::get_point(const Polygon _bed, bool fast, Point &offset)
	{
		bool result = false;
		int _threads = boost::thread::hardware_concurrency();
		LOGINFO("【实体】计算堆叠排版位置 线程数%d \n", _threads);
		DWORD dwStart = GetTickCount();
		boost::thread_group *_work_p;
		parallelize<size_t>(
			0,
			cale_vec.size() - 1,
			boost::bind(&calc_compact_arrange_threads::calc_position_thread, this, _1, &cale_vec, _bed, fast),
			_work_p,
			_threads);
		LOGINFO("【实体】计算堆叠排版位置 总耗时【%u】", GetTickCount() - dwStart);
		///////////////////////////////////////////////////////////////////////////////////////////
		int max_y = -100000;
		for (int i = 0; i < cale_vec.size(); i++) // 选出res_y最大的那个cale_t
		{
			if (cale_vec.at(i).can_arrange == false)
				continue; // 排除排不下的
			if (cale_vec.at(i).res_y > max_y)
			{
				max_y = cale_vec.at(i).res_y;
				offset.y = max_y;
				offset.x = cale_vec.at(i).x_offset;
			}
		}

		result = max_y != -100000;

		return result;
	}
	int ModelObject::get_charmark_volume_id()
	{
		if (this->verarray_p == NULL)
		{
			LOGINFO("ModelObject::get_charmark_volumn_id verarray_p == NULL");

			return -1;
		}

		for (int i = 0; i < this->volumes.size(); i++)
		{
			if (this->volumes.at(i)->mark == true)
			{
				return i;
			}
		}
		return -1;
	}

	bool ModelObject::modify_charmark(std::string mark)
	{
		int idx = get_charmark_volume_id();

		if (idx == -1)
		{
			LOGINFO("ModelObject::modify_charmark no char mark volume");
			return false;
		}

		ModelVolume *cm_volume = this->volumes.at(idx);
		ModelVolume *new_volume = new ModelVolume(this, *cm_volume);

		int sence_id = this->verarray_p->GetSelect_idx(cm_volume);
		this->delete_volume_sence_id(sence_id);

		new_volume->CharMark = mark;

		bool result = new_volume->make_SptMesh();

		if (result)
		{
			this->add_volume(*new_volume);
		}

		return result;
	}

}
