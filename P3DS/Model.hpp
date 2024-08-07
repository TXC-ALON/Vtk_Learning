#ifndef slic3r_Model_hpp_
#define slic3r_Model_hpp_

#include "libslic3r.h"
#include "3DScene.hpp"
#include "BoundingBox.hpp"
#include "PrintConfig.hpp"
#include "Layer.hpp"
#include "Point.hpp"
#include "TriangleMesh.hpp"
#include "Geometry.hpp"
#include <map>
#include <string>
#include <utility>
#include <vector>
#include <PrintStep.hpp>

#include <vec3.hpp>					// glm::vec3
#include <vec4.hpp>					// glm::vec4
#include <mat4x4.hpp>				// glm::mat4
#include <gtc/matrix_transform.hpp> // glm::translate, glm::rotate, glm::scale, glm::perspective
#include <gtc/constants.hpp>		// glm::pi
#include <gtc/type_ptr.hpp>

namespace Slic3r
{

	class Print;
	class ModelInstance;
	class ModelMaterial;
	class ModelObject;
	class ModelVolume;
	class ModelVertexArray;
	class calc_compact_arrange_threads;

	typedef std::string t_model_material_id;
	typedef std::string t_model_material_attribute;
	typedef std::map<t_model_material_attribute, std::string> t_model_material_attributes;

	typedef std::map<t_model_material_id, ModelMaterial *> ModelMaterialMap;
	typedef std::vector<ModelObject *> ModelObjectPtrs;
	typedef std::vector<ModelVolume *> ModelVolumePtrs;
	typedef std::vector<ModelInstance *> ModelInstancePtrs;
	typedef std::vector<ModelVolumePtrs> ModelVolume_2dim_vec;
	typedef std::pair<Pointf3, Pointf3> CrossbarPointPair;

	// The print bed content.
	// Description of a triangular model with multiple materials, multiple instances with various affine transformations
	// and with multiple modifier meshes.
	// A model groups multiple objects, each object having possibly multiple instances,
	// all objects may share mutliple materials.
	class Model
	{
	public:
		PrintState<MedolObjectStep> state;
		// Materials are owned by a model and referenced by objects through t_model_material_id.
		// Single material may be shared by multiple models.
		ModelMaterialMap materials;
		// Objects are owned by a model. Each model may have multiple instances, each instance having its own transformation (shift, scale, rotation).
		ModelObjectPtrs objects;
		void copy_obj(int obj_idx, int row, double row_space, int column, double column_space, bool keep_original, Model *model);

		Model();
		Model(const Model &other);
		Model &operator=(Model other);
		void swap(Model &other);
		~Model();
		static Model read_from_file(std::string input_file, std::string file_name);
		ModelObject *add_object();
		ModelObject *add_object(const ModelObject &other, bool copy_volumes = true);
		void delete_object(size_t idx);
		void clear_objects();
		void clear_minorsupport_mark();
		bool Is_Repeated(std::string name);
		ModelMaterial *add_material(t_model_material_id material_id);
		ModelMaterial *add_material(t_model_material_id material_id, const ModelMaterial &other);
		ModelMaterial *get_material(t_model_material_id material_id);
		void delete_material(t_model_material_id material_id);
		void clear_materials();
		bool has_objects_with_no_instances() const;
		bool add_default_instances();
		BoundingBoxf3 bounding_box() const;
		void repair();
		ModelObject *merge_obj(Model &other_model, std::string bed_type);
		ModelObject *Lattice_merge_obj(Model &other_model, std::string bed_type, double spt_Height = 0.0);
		bool has_LatticeMerge_object();
		void center_instances_around_point(const Pointf &point);
		void align_instances_to_origin();
		void translate(coordf_t x, coordf_t y, coordf_t z);
		TriangleMesh support_mesh();
		TriangleMesh mesh();
		TriangleMesh raw_mesh();
		bool _arrange(const Pointfs &sizes, coordf_t dist, const BoundingBoxf *bb, Pointfs &out) const;
		bool arrange_objects(coordf_t dist, const BoundingBoxf *bb = NULL);

		std::vector<size_t> sort_arrange_polygons();
		std::vector<size_t> sort_arrange_by_height();
		std::vector<size_t> sort_arrange_for_simplify_for_zhijia(double x_size);
		std::vector<size_t> sort_arrange_for_complex_for_zhijia();

		bool can_merge();

		// Croaks if the duplicated objects do not fit the print bed.
		void duplicate(size_t copies_num, coordf_t dist, const BoundingBoxf *bb = NULL);
		void duplicate_objects(size_t copies_num, coordf_t dist, const BoundingBoxf *bb = NULL);
		void duplicate_objects_grid(size_t x, size_t y, coordf_t dist);
		void print_info();

		void add_group_instance();

		int object_count;//model中添加的object计数，新增object计数+1，删除时不变，add by gaohui 20240722

	public:
		//////////////////////////////////////////////////////////////////////////
		// 自动堆叠排版
		//////////////////////////////////////////////////////////////////////////
		void init_hash_facets_for_arrange() { hash_facets_for_arrange.clear(); }
		std::set<XYZ_Index> get_hash_facets_for_arrange() { return hash_facets_for_arrange; }
		void update_hash_facets_for_arrange(std::set<XYZ_Index> new_hash_facets);
		void update_hash_facets_for_arrange(std::map<XYZ_Index, std::vector<int>> new_hash_facets);
		bool hash_facets_for_arrange_contain_value(XYZ_Index index);
		bool hash_facets_for_arrange_contain_value(std::map<XYZ_Index, std::vector<int>> new_hash_facets);
		//////////////////////////////////////////////////////////////////////////

		//////////////////////////////////////////////////////////////////////////
		// 碰撞检测
		//////////////////////////////////////////////////////////////////////////
		void clear_model_voxels();
		void build_model_voxels(double step, bool crossbar = true, bool support = true);
		std::set<XYZ_Index> get_model_voxels_for_collision(size_t ignore_index = -1);
		void check_collision();
		size_t get_object_idx(ModelObject *obj);
		//////////////////////////////////////////////////////////////////////////

	private:
		std::set<XYZ_Index> hash_facets_for_arrange; // 自动排版
	};

	// Material, which may be shared across multiple ModelObjects of a single Model.
	class ModelMaterial
	{
		friend class Model;

	public:
		// Attributes are defined by the AMF file format, but they don't seem to be used by Slic3r for any purpose.
		t_model_material_attributes attributes;
		// Dynamic configuration storage for the object specific configuration values, overriding the global configuration.
		DynamicPrintConfig config;

		Model *get_model() const { return this->model; };
		void apply(const t_model_material_attributes &attributes);

	private:
		// Parent, owning this material.
		Model *model;

		ModelMaterial(Model *model);
		ModelMaterial(Model *model, const ModelMaterial &other);
		~ModelMaterial() { this->attributes.clear(); };
	};

	// A printable object, possibly having multiple print volumes (each with its own set of parameters and materials),
	// and possibly having multiple modifier volumes, each modifier volume with its set of parameters and materials.
	// Each ModelObject may be instantiated mutliple times, each instance having different placement on the print bed,
	// different rotation and different uniform scaling.
	class ModelObject
	{
		friend class Model;

	public:
		std::string name;
		std::string input_file;
		std::string Medical_ID;

        bool use_region_segmentation = false;

        int region_segmentation_num = 12;

		ModelVertexArray *verarray_p; // OpenGL 数据管理类指针

		int object_num;//记录该modelobject在model中的唯一编号，add by gaohui 20240722

		// Instances of this ModelObject. Each instance defines a shift on the print bed, rotation around the Z axis and a uniform scaling.
		// Instances are owned by this ModelObject.
		ModelInstancePtrs instances;
		// Printable and modifier volumes, each with its material ID and a set of override parameters.
		// ModelVolumes are owned by this ModelObject.
		ModelVolumePtrs volumes;
		// 描述支撑volume归属关系容器
		std::map<ModelVolume *, ModelVolumePtrs> map_sptvol;
		// Configuration parameters specific to a single ModelObject, overriding the global Slic3r settings.
		DynamicPrintConfig config;
		// Variation of a layer thickness for spans of Z coordinates.
		t_layer_height_ranges layer_height_ranges;

		/* This vector accumulates the total translation applied to the object by the
			center_around_origin() method. Callers might want to apply the same translation
			to new volumes before adding them to this object in order to preserve alignment
			when user expects that. */
		Pointf3 origin_translation;

		// these should be private but we need to expose them via XS until all methods are ported
		BoundingBoxf3 _bounding_box;
		bool _bounding_box_valid;
		BoundingBoxf3 _raw_bounding_box;
		bool _raw_bounding_box_valid;

		bool arranged;					  // 排序标志位，true->已排序，false->未排序
		bool IsAutoXXX;					  // 自动操作标记位
		bool IsIpdModel;				  // 是否来自于iPD的设计模型
		bool AddMergeLattice;			  // 是否加了合并的晶格支撑
		bool Is_Add_minorsupport = false; // 是否添加极简支撑
		bool _raw_thumbnails_valid;
		ExPolygons thumbnails;
		Points num_positions;

		bool can_split = true;

		Print *_print;
		static std::set<int> LimitedFaces;
		PrintObject* pobject;
        PrintObject* print_object() { return pobject; }

		int row = 1;
		int column = 1;
		double row_space = 0.0;
		double column_space = 0.0;

	public:
		void reset_copy_info()
		{
			row = 1;
			column = 1;
			row_space = 0.0;
			column_space = 0.0;
		}
		void set_copy_info(int _row, double _row_space, int _column, double _column_space)
		{
			row = _row;
			column = _column;
			row_space = _row_space;
			column_space = _column_space;
		}
		Model *get_model() const { return this->model; };

		TriangleMesh get_raw_mesh_from_model(int i);
		TriangleMesh get_mesh_from_model(int i);
		TriangleMesh get_meshWithCrossbar_from_model(int i);
		TriangleMesh get_meshWithSupport_from_model(int i, bool crossbar = true, bool support = true);

		// 计算投影和数字标记位置
		void invalid_thumbnails();
		void update_thumbnails();
		void update_num_positions();
		// 2DPlater使用的投影算法
		ExPolygons get_model_thumbnails(bool is_raw = false);
		Points get_num_positions();

		int get_nospt_model_volume_num();  // 获取实体volume数目
		int get_ipd_model_volume_num();	   // 获取ipd实体volume数目
		int get_first_object_volume_idx(); // 获取第一个实体的id
		Pointf3 get_lowest_point_by_volume_idx(int idx);

		// 添加支撑接口
		ModelVolume *get_pvolume_spt(ModelVolume *_p_spted_vol);
		ModelVolume *get_pvolume_spt(ModelVolume *_p_spted_vol, const ModelVolume &volume);
		bool add_pvolume_spt(ModelVolume *p_spt);
		// 记录关系容器
		bool add_pvolume_map(ModelVolume *p_spt);
		bool del_pvolume_map(ModelVolume *p_spt);

		ModelVolume *add_volume(const TriangleMesh &mesh);
		ModelVolume *add_volume(const ModelVolume &volume);
		void delete_volume(size_t idx);
		bool delete_volume_by_volume(ModelVolume *p_volume);
		bool re_delete_volume(ModelVolume *p_volume);
		void delete_volume_sence_id(size_t sence_idx);
		void clear_volumes();
		void clear_spt_volumes(bool delete_crossbar = false);
		int get_sence_id(size_t idx);
		std::vector<int> get_treeBranch_sence_ids(size_t select_idx);
		ModelVolume *get_volume_by_selectid(size_t select_idx);
		int sence_volumes_count();
		bool has_spt_volume(bool exclude_crossbar = false, bool exclude_lattice = false);
		bool has_crossbar_volume();

		bool has_lattice_volume();
		bool has_LatticeMerge_volume(); // 是否是晶格支撑辅助合并
		bool has_AddMergeLattice();		// 是否加了合并的晶格支撑
		void set_AddMergeLattice(bool _add) { AddMergeLattice = _add; };

		bool has_charmark_volume();
		int get_charmark_volume_id();
		bool modify_charmark(std::string mark);

		ModelInstance *add_instance();
		ModelInstance *add_instance(const ModelInstance &instance);
		void delete_instance(size_t idx);
		void delete_last_instance();
		void clear_instances();

		BoundingBoxf3 bounding_box_volume(size_t select_idx);
		BoundingBoxf3 bounding_box();
		void invalidate_bounding_box();
		BoundingBoxf3 raw_bounding_box();
		BoundingBoxf3 raw_bounding_box_volume(size_t select_idx);
		BoundingBoxf3 instance_bounding_box(size_t instance_idx) const;

		void repair(bool auto_repair = true);
		TriangleMesh mesh_output(std::string _type, bool tra_instance = false, bool _repair = false);
		bool write_object_stl(bool binary = false, std::string dir = ""); // 文件名Obj.stl
		bool write_spt_stl(bool binary = false, std::string dir = "");	// 文件名Spt.stl
		bool write_fpt_stl(bool binary = false, std::string dir = "");	// 文件名Fpt.stl
		bool write_instance_xml(std::string dir = "");					// 文件名instance.xml
		bool write_cpt_xml(std::string dir = "");						// 文件名cpt.xml
		bool write_mpt_xml(std::string dir = "");						// 文件名mpt.xml

		bool read_instance_xml(std::string instance_xml);
		bool read_cpt_xml(std::string cpt_xml);
		bool read_mpt_xml(std::string mpt_xml);

		TriangleMesh support_mesh(bool _repair = false, bool _shared_vertices = false);
		TriangleMesh mesh(bool _repair = false, bool _shared_vertices = false);
		TriangleMesh raw_mesh(bool _repair = false);
		TriangleMesh meshWithCrossbar(bool _repair = false);
		TriangleMesh meshWithSupport(bool crossbar = true, bool support = true, bool _repair = false);

		std::vector<TriangleMesh> crossbar_meshs();
		bool UnableRepair() const;
		bool check_needed_repair() const;

		Vectorf3 center_around_origin(bool translate_instance, bool keep_z = false, bool reload = false);
		void put_on_plater(bool only_bed_down = false);
		void translate(const Vectorf3 &vector);
		void translate(coordf_t x, coordf_t y, coordf_t z);

		void scale(float factor);
		void scale(const Pointf3 &versor);
		void scale_to_fit(const Sizef3 &size);
		void rotate(float angle, const Axis &axis);
		void mirror(const Axis &axis);
		void transbyinstance_fixpos(const ModelInstance &instance);
		bool apply_matrix_to_mesh(); // 同步变换数据
		bool reload_mesh_to_array(); // 重载模型
		bool recover_last_operate(); // 恢复上一步操作
		size_t materials_count() const;
		size_t facets_count() const;
		bool needed_repair() const;
		void cut(Axis axis, coordf_t z, Model *model) const;
		bool split_obj(Model &other_model, ModelObjectPtrs *new_objects);
		void split_volumes();
		void update_bounding_box(); // this is a private method but we expose it until we need to expose it via XS
		void print_info();

		void adjust_object(int angle, int step); // 调整object姿态，使其包围盒z范围最小
		void reverse_object();					 // 翻转物体
		void rotate_around_X(double angle);
		bool Pre_adjust_object_zhijia(int facets_threshold, double tuqi_facets_percent); // 调整支架姿态
		bool Pro_adjust_object_zhijia(int angle_threshold_min, int angle_threshold_max, double z_min, double z_max);
		bool arrange_object_slm(const Polygons bed, const coord_t nr1, const coord_t nr2, const std::string direction, const coordf_t area, const bool crown, const double tolerance = SCALED_RESOLUTION_ARRANGE);
		int arrange_object_slm_compact_new_complex(const Polygon &origin_bed, double step, double x_length, int last_idx, std::string direction, bool fast = true, bool crossbar = true, bool support = true);
		int arrange_object_slm_compact_new_complex_multithread(const Polygon &origin_bed, double step, double x_length, int last_idx, std::string direction, bool fast = true, bool crossbar = true, bool support = true);

		int arrange_object_slm_compact_new_complex_align_x(const Polygon &origin_bed, double step, double x_length, int last_idx, std::string direction_x, std::string direction_y, bool fast = true, bool crossbar = true, bool support = true);

		void pre_adjust_zhijia_for_arrange(int angle, int step); // 绕z轴旋转，调整支架的姿态，使其包围盒x轴范围最大

		Polygons get_next_bed(const Polygons bed, const coordf_t object_dist, const coordf_t area, const bool crown, const double tolerance = SCALED_RESOLUTION_ARRANGE);
		// 排版用的投影算法
		Polygon get_thumbnail(double tolerance = SCALED_RESOLUTION_ARRANGE);
		Polygon get_boundingbox_thumbnail();

		int get_IpdRpdType();
		bool Is_In_plater(const Polygon &bed_shape, bool Is_boundary); // 判断物体是否在盘上

		bool check_if_add_minorsupport() { return this->Is_Add_minorsupport; }
		void Set_Minorsupport(bool _Minorsupport) { Is_Add_minorsupport = _Minorsupport; };

		void reload_obj_facet_color();
		void reload_rpd_facet_color();
		size_t Get_modelobject_Index();
		size_t Get_GroupID() { return group_id; };
		void Set_GroupID(size_t _group_id) { group_id = _group_id; };
		std::string get_char_mark() { return char_mark; }
		void set_char_mark(std::string _mark) { char_mark = _mark; }

		std::string Get_bed_type() { return bed_type; };
		void Set_bed_type(std::string _bed_type) { bed_type = _bed_type; };

		void RefreshFaceColorByPos(size_t select_idx, Pointf3 pickPos, bool IsRemove);
		void RefreshNoSupportFaceColorByPos(size_t select_idx, Pointf3 pickPos, bool IsRemove);
		void GetNoSupportHoleLimitFaceColorByPos(size_t select_idx, Pointf3 pickPos, bool IsRemove);
		//取消不加支撑区域选择的地方
		void CancelNoSupportArea();
		double get_surface_area(bool iSpt);

	public:
		void build_object_voxels(double step, bool crossbar = true, bool support = true);
		std::set<XYZ_Index> get_object_voxels() { return object_voxels; };
		void clear_object_voxels();
		void check_collision();
		bool isCollision = false;
		size_t get_idx_in_model();

		std::string get_preset_name() { return preset_name; }
		void set_preset_name(std::string name) { preset_name = name; }

		size_t get_print_order() { return print_order; }
		void set_print_order(size_t _order) { this->print_order = _order; }

		bool get_transparent() { return transparent; }
		void set_transparent(bool trans) { transparent = trans; }

	public:
		ModelVolumePtrs temp_Delete_vec;
		ModelVolume_2dim_vec Undo_Vec;
		ModelVolume_2dim_vec Redo_Vec;
		size_t init_length = 1000;
		void add_items_to_tempvec(size_t idx);
		void push_delete_items();
		void pop_delete_items();
		void clear_vecs();
		void undo_delete();
		void redo_delete();
		void clear_object_undo_vecs();

	private:
		// Parent object, owning this ModelObject.
		Model *model;

		ModelObject(Model *model);
		ModelObject(Model *model, const ModelObject &other, bool copy_volumes = true);
		ModelObject &operator=(ModelObject other);
		void swap(ModelObject &other);
		~ModelObject();
		// 排版用的投影算法
		Polygon _get_thumbnail(double tolerance = SCALED_RESOLUTION_ARRANGE);

		//打印顺序
		size_t print_order = 0;

		//透明度
		bool transparent = false;

		// 组编号
		int group_id;
        std::string preset_name = ""; //工艺参数名称

		std::string char_mark;
		std::string bed_type = "None";
		std::set<XYZ_Index> object_voxels;
		double _step;
	};

	// OpenGL 顶点数组
	class ModelVertexArray
	{
		friend class ModelObject;

	private:
		ModelObject *object;
		//ModelVolumePtrs sence_volumes;
		glm::mat4 translate_Matrix; // 平移矩阵
		glm::mat4 rotate_Matrix;	// 旋转矩阵
		glm::mat4 scale_Matrix;		// 缩放矩阵
		//  组合后模型矩阵 传给perl的指针指向的内容
		glm::mat4 t_Matrix;
		// 记录上一次同步mesh的模型矩阵
		glm::mat4 last_translate;
		glm::mat4 last_rotate;
		glm::mat4 last_scale;

		//记录显示复制物体的临时矩阵
		glm::mat4 temp_Matrix_for_copy;

		//GLVertexArrayCollection vercol_obj; // 物体的数据数组
		//GLVertexArrayCollection vercol_spt; // 支撑的数据数组

	public:
		ModelObject *get_object() const { return this->object; };
		ModelVertexArray(ModelObject *object);
		// ModelVertexArray(ModelObject *object, const ModelVertexArray &other);
		// ModelVertexArray& operator= (ModelVertexArray other);
		// void swap(ModelVertexArray &other);
		~ModelVertexArray(){};
		// 顶点数据操作
		bool add_array(ModelVolume *volume_p);
		bool delete_array(const size_t select_idx);
		bool delete_array(ModelVolume *volume_p);
		void clear_array();
		void clear_array_spt();
		void reload_array();
		void reload_obj_facet_color();
		void reload_rpd_facet_color();

		// 模型矩阵操作
		void translate(const coordf_t _x, const coordf_t _y, const coordf_t _z);
		void scale(const coordf_t _x, const coordf_t _y, const coordf_t _z);
		void rotate(float angle, const Axis &axis);
		// perl 调用接口
		ModelVolume *GetVolume(const size_t select_idx);
		int GetSelect_idx(ModelVolume *volume_p);

		void *get_Matrix_ptr(bool no_Z = false);
		void *get_transfered_Matrix_ptr(const coordf_t _x, const coordf_t _y, const coordf_t _z, bool no_Z = false);
		glm::mat4 get_Matrix();
		glm::mat4 get_transfer_Matrix();
		GLVertexArrayCollection *get_Aarray_ptr(bool Is_spt);
		void clear_matrix();
		void recover_last_matrix();
		// 调试接口
		static void Log_Matrix(glm::mat4 _Matrix);
		static bool IsMatrixUnit(glm::mat4 _Matrix);

	private:
		static void Log_Matrix(const float *pSource);
		void Combin_Matrix(); // 组合模型矩阵
		glm::vec3 Get_WorldAxis(const Axis &axis);
	};

	// An object STL, or a modifier volume, over which a different set of parameters shall be applied.
	// ModelVolume instances are owned by a ModelObject.
	class ModelVolume
	{
		friend class ModelObject;

	public:
		std::string name;
		// The triangular model.
		TriangleMesh mesh;
		// Configuration parameters specific to an object model geometry or a modifier volume,
		// overriding the global Slic3r settings and the ModelObject settings.
		// 配置参数，具体到对象模型几何或修改量，
		// 覆盖全局的Slic3r设置和ModelObject设置。
		DynamicPrintConfig config;
		// Is it an object to be printed, or a modifier volume?
		bool modifier;
		// 是否是支撑体
		bool surpporter;
		// 是否是实体支撑
		bool solider;
		// 实体附加体 比如侧壁标记
		bool adder;
		// 实体附加体是否内凹
		bool concave = false;
		// 是否是标记
		bool mark = false;
		// 标记是否自动计算位置
		bool automark;
		// 是否被合并过
		bool merged;
		// 是否是晶格支撑合并主体
		bool Lattice_merged;
		// 支撑体类型
		std::string SurpportType;
		// 字符标记
		std::string CharMark;
		// 接触点法线
		Pointf3 TopNarmol;
		Pointf3 ButtomNarmol;
		Pointf3 MidNarmol;

		// 接触点坐标
		Pointf3 upPickPos;
		Pointf3 dwPickPos;
		Pointf3 midPickPos;

		// 支撑体形状参数容器
		std::map<std::string, double> SurpportParams;
		// 支撑体支撑的物体指针
		ModelVolume *p_spted_vol;
		// 横杆的上下接触点
		CrossbarPointPair crossbar_pointpair;

		// 是否存在待生成横杆
		bool b_exist_cross_bar_not_generate = false;
		// 是否是横杆
		bool b_cross_bar = false;
		// 是否是横杆末端球
		bool b_crossbar_ball = false;
		// 晶格支撑
		LatticeSupportMesh *lattice_support_ptr = nullptr;

		bool IsIpdVolume; // 是否来自于iPD的设计模型
		int IpdRpdType = 0;
		double Rpd_Rotate_Angle_X = 0.0;
		std::string dentalCode;
		Pointf3 FrameX;
		Pointf3 FrameY;
		Pointf3 FrameZ;
		double CharSize = 1.0;		  // 提供给字符大小的控制值
		bool CharMark_IsValid = true; // 衡量该charmark是否完全贴合模型
		bool _raw_thumbnails_valid;
		ExPolygons thumbnails;
		void invalid_volume_thumbnails();
		void update_volume_thumbnails();
		ExPolygons get_volume_thumbnails();
		void set_Rpd_Rotate_Angle_X(double angle) { Rpd_Rotate_Angle_X = angle; };
		void set_charsize(double size) { CharSize = size; };
		double get_charsize() { return CharSize; };
		bool get_charmark_quality() { return CharMark_IsValid; };

	public:
		// A parent object owning this modifier volume.
		ModelObject *get_object() const { return this->object; };
		t_model_material_id material_id() const;
		void material_id(t_model_material_id material_id);
		ModelMaterial *material() const;
		void set_material(t_model_material_id material_id, const ModelMaterial &material);

		ModelMaterial *assign_unique_material();
		bool make_SptMesh();		  // 根据参数构建支撑体 最上层接口
		GLVertexArray Get_VerArray(); // 获取OpenGL顶点数组
		ModelVolume *Get_spted_obj(); // 获取支撑体支撑的volume
		int Get_LatticeTopSize();	  // 获取晶格支撑点个数
		bool Isequal(ModelVolume *other);
		Pointf3 ModifyPoint_CrossBarCenter(Pointf3 pt);
		bool Avoid_CrossBar_Facets(int expand_degree);
		bool remove_crossbar_facets(std::vector<int> &region_facets, std::vector<int> crossbar_facets);

	private:
		// Parent object owning this ModelVolume.
		ModelObject *object;
		t_model_material_id _material_id;

		ModelVolume(ModelObject *object, const TriangleMesh &mesh);
		ModelVolume(ModelObject *object, const ModelVolume &other);
		ModelVolume &operator=(ModelVolume other);
		void swap(ModelVolume &other);
		~ModelVolume();

		bool verify_SptParams();						// 校验支撑参数
		bool analyze_SptParams();						// 解析支撑参数
		static bool make_SptMesh(ModelVolume *pvolume); // 构造支撑
	public:
		bool Genenal_LatticeTree_Points(
			coordf_t _x_cell_length,
			coordf_t _y_cell_length,
			coordf_t _z_cell_length,
			coordf_t _suspend_angel,
			coordf_t _lattice_slice_thickness,
			coordf_t _lattice_spt_distance,
			double tree_support_minispace,
			int addPoints);													   // 以晶格采样点的方式来进行生成树枝点
		bool Genenal_CrossBar_Tree_Points(double CrossBar_TreePoint_distance); // 从卡环取点生成树支撑
	};

	// A single instance of a ModelObject.
	// Knows the affine transformation of an object.

	class ModelInstance
	{
		friend class ModelObject;

	public:
		double rotation; // Rotation around the Z axis, in radians around mesh center point
		double scaling_factor;
		Pointf offset; // in unscaled coordinates

		ModelObject *get_object() const { return this->object; };

		ModelInstance *clone() { return new ModelInstance(this->object, *this); };
		void reverse_intance();

		// To be called on an external mesh
		void transform_mesh(TriangleMesh *mesh, bool dont_translate = false) const;
		// Calculate a bounding box of a transformed mesh. To be called on an external mesh.
		BoundingBoxf3 transform_mesh_bounding_box(const TriangleMesh *mesh, bool dont_translate = false) const;
		// Transform an external bounding box.
		BoundingBoxf3 transform_bounding_box(const BoundingBoxf3 &bbox, bool dont_translate = false) const;
		// To be called on an external polygon. It does not translate the polygon, only rotates and scales.
		void transform_polygon(Polygon *polygon) const;

	private:
		// Parent object, owning this instance.
		ModelObject *object;

		ModelInstance(ModelObject *object);
		ModelInstance(ModelObject *object, const ModelInstance &other);
		ModelInstance &operator=(ModelInstance other);
		void swap(ModelInstance &other);
	};

	// 多线程堆叠排版数据结构
	struct compact_arrange_cale_t
	{
		coord_t x_offset;
		coord_t res_y;
		bool can_arrange;
		compact_arrange_cale_t(coord_t _x_offset) : x_offset(_x_offset), res_y(0), can_arrange(false){};
	};
	typedef std::vector<compact_arrange_cale_t> compact_arrange_cale_vec;

	// 多线程计算类
	class calc_compact_arrange_threads
	{
	public:
		compact_arrange_cale_vec cale_vec;
		coord_t start_y;
		coord_t step;

		Model *model;
		TriangleMesh *object;
		Polygon thunbmail;
		calc_compact_arrange_threads(int _len, int _direc, coord_t _step, Model *_model, TriangleMesh *_object, Polygon _thunbmail, coord_t _start_y)
			: model(_model), object(_object), thunbmail(_thunbmail), start_y(_start_y), step(_step)
		{
			cale_vec.clear();
			for (int i = 0; i < _len; i++)
			{
				cale_vec.push_back(compact_arrange_cale_t(i * _direc));
			}
		};
		~calc_compact_arrange_threads(){};

		bool get_point(const Polygon _bed, bool fast, Point &offset);
		void calc_position_thread(size_t _index, compact_arrange_cale_vec *p_vec, Polygon _bed, bool fast);

	private:
	};

}

#endif
