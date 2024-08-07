#include "TriangleMesh.hpp"
#include <float.h>
#include <string>
namespace Slic3r
{

	// 快速构建一个球体  半径rho  精度步长fa
	// Generates mesh for a sphere centered about the origin, using the generated angle
	// to determine the granularity.
	// Default angle is 1 degree.
	TriangleMesh
	TriangleMeshGeneral::make_sphere(double rho, double fa, SphereType stype)
	{
		Pointf3s vertices;
		std::vector<Point3> facets;

		// Algorithm:
		// Add points one-by-one to the sphere grid and form facets using relative coordinates.
		// Sphere is composed effectively of a mesh of stacked circles.

		// adjust via rounding to get an even multiple for any provided angle.
		double angle = 2 * PI / fa;
		double angle_range = (stype == SpWhole) ? (2 * PI) : (PI);
		// Ring to be scaled to generate the steps of the sphere
		std::vector<double> ring;
		for (double i = 0; i < angle_range; i += angle)
		{
			ring.push_back(i);
		}
		const size_t steps = ring.size();
		const double increment = (double)(1.0 / (double)steps);

		// special case: first ring connects to 0,0,0
		// insert and form facets.
		vertices.push_back(Pointf3(0.0, 0.0, -rho));
		size_t id = vertices.size();
		for (size_t i = 0; i < ring.size(); i++)
		{
			// Fixed scaling
			const double z = -rho + increment * rho * 2.0;
			// radius of the circle for this step.
			const double r = sqrt(std::abs(rho * rho - z * z));
			Pointf3 b(0, r, z);
			b.rotate(ring[i], Pointf3(0, 0, z));
			vertices.push_back(b);
			if (i == 0)
			{
				facets.push_back(Point3(1, ring.size(), 0));
			}
			else
			{
				facets.push_back(Point3(id, id - 1, 0));
			}
			id++;
		}

		// General case: insert and form facets for each step, joining it to the ring below it.
		for (size_t s = 2; s < steps - 1; s++)
		{
			const double z = -rho + increment * (double)s * 2.0 * rho;
			const double r = sqrt(std::abs(rho * rho - z * z));

			for (size_t i = 0; i < ring.size(); i++)
			{
				Pointf3 b(0, r, z);
				b.rotate(ring[i], Pointf3(0, 0, z));
				vertices.push_back(b);
				if (i == 0)
				{
					// wrap around
					facets.push_back(Point3(id + ring.size() - 1, id - 1, id));
					facets.push_back(Point3(id, id - 1, id - ring.size()));
				}
				else
				{
					facets.push_back(Point3(id, (id - 1) - ring.size(), id - ring.size()));
					facets.push_back(Point3(id, id - 1, id - 1 - ring.size()));
				}
				id++;
			}
		}

		// special case: last ring connects to 0,0,rho*2.0
		// only form facets.
		vertices.push_back(Pointf3(0.0, 0.0, rho));
		for (size_t i = 0; i < ring.size(); i++)
		{
			if (i == 0)
			{
				// third vertex is on the other side of the ring.
				facets.push_back(Point3(id, id - 1, id - ring.size()));
			}
			else
			{
				facets.push_back(Point3(id, id - ring.size() + (i - 1), id - ring.size() + i));
			}
		}
		id++;
		TriangleMesh mesh(vertices, facets);
		if (stype == SpTop)
			mesh.rotate_y(PI / 2);
		else if (stype == SpBottom)
			mesh.rotate_y(-PI / 2);
		mesh.repair(false);

		return mesh;
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Single_BaseMesh
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	Single_BaseMesh::Single_BaseMesh(const Pointf3 &_dwPoint, const Pointf3 &_upPoint, double _reg)
	{
		this->dwBasePoint = _dwPoint;
		this->upBasePoint = _upPoint;
		this->shape_rotate = _reg;
		dwouter_points.clear(); // 下表面外轮廓顶点链表
		dwinner_points.clear(); // 下表面内轮廓顶点链表
		upouter_points.clear(); // 上表面外轮廓顶点链表
		upinner_points.clear(); // 上表面内轮廓顶点链表
	};

	Single_BaseMesh::~Single_BaseMesh()
	{
		Pointf3s().swap(dwouter_points);
		Pointf3s().swap(dwinner_points);
		Pointf3s().swap(upouter_points);
		Pointf3s().swap(upinner_points);
	};

	// 构建轮廓环
	bool
	Single_BaseMesh::GeneralExPoints()
	{
		int Points_Num = shape_points.size();
		dwouter_points.clear();
		dwinner_points.clear();
		upouter_points.clear();
		upinner_points.clear();

		for (int i = 0; i < Points_Num; i++)
		{
			Vectorf3 shapePoint(shape_points[i].x, shape_points[i].y, 0); // 轮廓点偏移量

			Pointf3 _dwPoint = dwBasePoint;		// 下表面接触点
			dwinner_points.push_back(_dwPoint); // 下表面内轮廓顶点链表
			_dwPoint.translate(shapePoint);
			dwouter_points.push_back(_dwPoint); // 下表面外轮廓顶点链表

			Pointf3 _upPoint = upBasePoint;		// 上表面接触点
			upinner_points.push_back(_upPoint); // 上表面内轮廓顶点链表
			_upPoint.translate(shapePoint);
			upouter_points.push_back(_upPoint); // 上表面外轮廓顶点链表
		}

		return true;
	};

	bool
	Single_BaseMesh::ScaleLoopPoints(Pointf3s *ploop_points, const Pointf3 basePoint, double sacle_dentity)
	{
		if (ploop_points == NULL)
			return false;

		// shape_dentity参数检查
		if (sacle_dentity < 0.000001)
			sacle_dentity = 0.0;
		else if (sacle_dentity > 0.999999)
			sacle_dentity = 1.0;

		for (int i = 0; i < ploop_points->size(); i++)
		{
			Pointf3 *pPoint = &(ploop_points->at(i));
			pPoint->translate(basePoint.negative()); // 将中心点移至原点
			pPoint->scale(sacle_dentity);
			pPoint->translate(basePoint); // 将中心点移回
		}

		return true;
	}

	bool
	Single_BaseMesh::TranslateLoopPoints(Pointf3s *ploop_points, const Vectorf3 offsetvector)
	{
		if (ploop_points == NULL)
			return false;

		for (int i = 0; i < ploop_points->size(); i++)
		{
			Pointf3 *pPoint = &(ploop_points->at(i));
			pPoint->translate(offsetvector);
		}

		return true;
	}

	// 旋转二维点集
	bool
	Single_BaseMesh::RotatePointfs(Pointfs *shape_points, const double _reg)
	{
		if (shape_points == NULL)
			return false;

		for (int i = 0; i < shape_points->size(); i++)
		{
			Pointf *pPoint = &(shape_points->at(i));
			pPoint->rotate(_reg);
		}

		return true;
	}

	bool
	Single_BaseMesh::translate_bynormal(Pointf3 &orgin_toppoint, const Pointf3 basePoint, const Vectorf3 normalVec)
	{
		Vectorf3 vecbasePoint = basePoint;
		Vectorf3 nvecbasePoint = basePoint.negative();
		orgin_toppoint.translate(nvecbasePoint);
		orgin_toppoint.z = orgin_toppoint.z - (normalVec.x * orgin_toppoint.x + normalVec.y * orgin_toppoint.y);
		orgin_toppoint.translate(vecbasePoint);

		return true;
	}

	// 移动支撑体的上下表面
	bool
	Single_BaseMesh::OffsetMeshFlat(bool IsDw, const Vectorf3 offsetVec)
	{
		if (IsDw)
		{ // 下表面台柱 下表面向上移动
			this->dwBasePoint.translate(offsetVec);
			Single_BaseMesh::TranslateLoopPoints(&dwinner_points, offsetVec);
			Single_BaseMesh::TranslateLoopPoints(&dwouter_points, offsetVec);
		}
		else
		{ // 上表面台柱 上表面向下移动
			this->upBasePoint.translate(offsetVec);
			Single_BaseMesh::TranslateLoopPoints(&upinner_points, offsetVec);
			Single_BaseMesh::TranslateLoopPoints(&upouter_points, offsetVec);
		}

		return true;
	}

	// 偏移实体
	bool
	Single_BaseMesh::OffsetMesh(const Vectorf3 offsetVec)
	{
		this->OffsetMeshFlat(true, offsetVec);
		this->OffsetMeshFlat(false, offsetVec);

		return true;
	}

	// 设置中空形状
	bool
	Single_BaseMesh::SetMeshHollow(double hollow_dentity)
	{
		dwinner_points = dwouter_points;
		upinner_points = upouter_points;
		Single_BaseMesh::ScaleLoopPoints(&dwinner_points, dwBasePoint, hollow_dentity);
		Single_BaseMesh::ScaleLoopPoints(&upinner_points, upBasePoint, hollow_dentity);

		return true;
	}

	// 平面自适应法相
	bool
	Single_BaseMesh::SetMeshFlatAutoFix(bool IsDw, Vectorf3 normalVec)
	{
		if (normalVec.z <= 0)
		{
			LOGINFO("法向量不合法，Noraml[%f, %f, %f]", normalVec.x, normalVec.y, normalVec.z);
			normalVec.x = 0;
			normalVec.y = 0;
			normalVec.z = 1;
		}

		for (int i = 0; i < this->shape_points.size(); i++)
		{
			if (IsDw)
			{
				Single_BaseMesh::translate_bynormal(dwouter_points[i], dwBasePoint, normalVec);
				Single_BaseMesh::translate_bynormal(dwinner_points[i], dwBasePoint, normalVec);
				// 下表面不能超过上表面
				dwouter_points[i].z = std::min(dwouter_points[i].z, upouter_points[i].z);
				dwinner_points[i].z = std::min(dwinner_points[i].z, upinner_points[i].z);
				// 最低不能低于0
				dwouter_points[i].z = std::max(dwouter_points[i].z, 0.0);
				dwinner_points[i].z = std::max(dwinner_points[i].z, 0.0);
			}
			else
			{
				Single_BaseMesh::translate_bynormal(upouter_points[i], upBasePoint, normalVec);
				Single_BaseMesh::translate_bynormal(upinner_points[i], upBasePoint, normalVec);
				upouter_points[i].z = std::max(upouter_points[i].z, dwouter_points[i].z);
				upouter_points[i].z = std::max(upouter_points[i].z, dwinner_points[i].z);
				// 最低不能低于0
				upouter_points[i].z = std::max(upouter_points[i].z, 0.0);
				upouter_points[i].z = std::max(upouter_points[i].z, 0.0);
			}
		}

		return true;
	}

	bool
	Single_BaseMesh::SetMeshAutoFix(Vectorf3 normalVec)
	{
		this->SetMeshFlatAutoFix(false, normalVec);
		this->SetMeshFlatAutoFix(true, normalVec);
		return true;
	}

	// 构建mesh
	TriangleMesh
	Single_BaseMesh::GeneralMesh()
	{
		// 参数检查
		int Points_Num = shape_points.size();
		if (Points_Num < 2 || Points_Num != dwouter_points.size() || Points_Num != dwinner_points.size() || Points_Num != upouter_points.size() || Points_Num != upinner_points.size())
		{
			LOGINFO("Points_Num[%d] error!! ", Points_Num);
			return TriangleMesh();
		}
		// 非实体单片
		bool IsNoSolid = (Points_Num == 2);

		std::vector<Point3> facets; // 三角面片顶点索引
		Pointf3s vertices;			// 顶点数组
		if (IsNoSolid)
		{
			vertices.push_back(dwouter_points[0]); // 下表面外轮廓顶点
			vertices.push_back(upouter_points[0]); // 上表面外轮廓顶点
			vertices.push_back(dwouter_points[1]); // 下表面内轮廓顶点
			vertices.push_back(upouter_points[1]); // 上表面内轮廓顶点

			facets.push_back(Point3(1, 0, 2));
			facets.push_back(Point3(1, 2, 3));
		}
		else
		{
			unsigned int id = 0;
			for (unsigned int i = 0; i < Points_Num; i++)
			{
				vertices.push_back(dwouter_points[i]); // 下表面外轮廓顶点
				vertices.push_back(upouter_points[i]); // 上表面外轮廓顶点
				vertices.push_back(dwinner_points[i]); // 下表面内轮廓顶点
				vertices.push_back(upinner_points[i]); // 上表面内轮廓顶点

				// 生成定点索引
				if (i == 0) // 从第二点开始
					continue;
				id = vertices.size() - 1;

				// bottom
				facets.push_back(Point3(id - 7, id - 1, id - 5));
				facets.push_back(Point3(id - 7, id - 3, id - 1));
				// top
				facets.push_back(Point3(id, id - 6, id - 4));
				facets.push_back(Point3(id, id - 2, id - 6));
				// front
				facets.push_back(Point3(id - 2, id - 3, id - 7));
				facets.push_back(Point3(id - 2, id - 7, id - 6));
				// behind
				facets.push_back(Point3(id, id - 4, id - 5));
				facets.push_back(Point3(id, id - 5, id - 1));
			}

			// 最后闭合成型
			// bottom
			facets.push_back(Point3(id - 3, 0, 2));
			facets.push_back(Point3(id - 3, 2, id - 1));
			// top
			facets.push_back(Point3(3, 1, id - 2));
			facets.push_back(Point3(3, id - 2, id));
			// front
			facets.push_back(Point3(1, 0, id - 3));
			facets.push_back(Point3(1, id - 3, id - 2));
			// behind
			facets.push_back(Point3(3, id - 1, 2));
			facets.push_back(Point3(3, id, id - 1));
		}

		// 构建mesh
		TriangleMesh mesh(vertices, facets);
		mesh.checkonly = IsNoSolid;
		mesh.repair();
		if (mesh.UnableRepair())
		{
			LOGINFO("GeneralMesh Unable Repair");
			return TriangleMesh();
		}

		return mesh;
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// SingleSPT_BaseMesh
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 构建Mesh
	TriangleMesh
	SingleSPT_BaseMesh::GeneralMesh()
	{
		TriangleMesh mesh = this->Single_BaseMesh::GeneralMesh();
		// 合并所有附件
		if (pDwContact)
			mesh.merge(pDwContact->GeneralMesh());
		if (pUpContact)
			mesh.merge(pUpContact->GeneralMesh());
		if (pDwPyramid)
			mesh.merge(pDwPyramid->GeneralMesh());
		if (pUpPyramid)
			mesh.merge(pUpPyramid->GeneralMesh());

		mesh.repair();
		if (mesh.UnableRepair())
		{
			LOGINFO("SingleSPT_BaseMesh::GeneralMesh Unable Repair");
			return TriangleMesh();
		}
		return mesh;
	}

	// 添加台柱
	bool
	SingleSPT_BaseMesh::Add_Contact(bool IsDw, double _w, double _l, double _h)
	{
		if (_h > this->GetMeshHeigt() - 0.1)
		{
			LOGINFO("Contact height error ,Need height[%f], this mesh height[%f]", _h, this->GetMeshHeigt());
			return false;
		}

		// 首先移除原来的台柱
		this->Del_Contact(IsDw);

		Pointf3 dwPoint;	// 上表面接触点
		Pointf3 upPoint;	// 下表面接触点
		Vectorf3 offsetVec; // 平面移动向量

		if (IsDw)
		{ // 下表面台柱 下表面向上移动
			offsetVec = Pointf3(0, 0, _h);
			dwPoint = this->dwBasePoint;
			this->Single_BaseMesh::OffsetMeshFlat(IsDw, offsetVec);
			upPoint = this->dwBasePoint;
			upPoint.z += 0.1;
			this->pDwContact = new Contact_Mesh(dwPoint, upPoint, this->shape_rotate, _w, _l);
		}
		else
		{ // 上表面台柱 上表面向下移动
			offsetVec = Pointf3(0, 0, -_h);
			upPoint = this->upBasePoint;
			this->Single_BaseMesh::OffsetMeshFlat(IsDw, offsetVec);
			dwPoint = this->upBasePoint;
			dwPoint.z -= 0.1;
			// if (dwPoint.z < 0) dwPoint.z = 0;
			this->pUpContact = new Contact_Mesh(dwPoint, upPoint, this->shape_rotate, _w, _l);
		}

		return true;
	}

	// 移除台柱
	void
	SingleSPT_BaseMesh::Del_Contact(bool IsDw)
	{
		double contact_height = 0.0;

		if (IsDw)
		{ // 下台柱 删除台柱  下表面向下移动h
			if (pDwContact)
			{
				contact_height = pDwContact->GetMeshHeigt();
				delete pDwContact;
				pDwContact = NULL;

				Vectorf3 offsetVec = Pointf3(0, 0, -contact_height);
				this->Single_BaseMesh::OffsetMeshFlat(IsDw, offsetVec);
			}
		}
		else
		{ // 上台柱 删除台柱  上表面向上移动h
			if (pUpContact)
			{
				contact_height = pUpContact->GetMeshHeigt();
				delete pUpContact;
				pUpContact = NULL;

				Vectorf3 offsetVec = Pointf3(0, 0, contact_height);
				this->Single_BaseMesh::OffsetMeshFlat(IsDw, offsetVec);
			}
		}
	}

	// 添加锥体
	bool
	SingleSPT_BaseMesh::Add_Pyramid(bool IsDw, double _h, double _Scale_density)
	{

		if (_h > this->GetMeshHeigt() - 0.1)
		{
			LOGINFO("Pyramid height error ,Need height[%f], this mesh height[%f]", _h, this->GetMeshHeigt());
			return false;
		}

		// 首先移除原来的锥体
		this->Del_Pyramid(IsDw);

		Pointf3 dwPoint;	// 上表面接触点
		Pointf3 upPoint;	// 下表面接触点
		Vectorf3 offsetVec; // 平面移动向量

		if (IsDw)
		{ // 下表面锥体 下表面向上移动
			LOGINFO("0711 Down Pyramid");
			offsetVec = this->upBasePoint - this->dwBasePoint;
			offsetVec = (_h / offsetVec.z) * offsetVec;
			LOGINFO("offsetVec = %s", offsetVec.dump_perl().c_str());
			dwPoint = this->dwBasePoint;
			this->Single_BaseMesh::OffsetMeshFlat(IsDw, offsetVec);
			upPoint = this->dwBasePoint;
			this->pDwPyramid = new Pyramid_Mesh(dwPoint, upPoint, this->shape_rotate, &shape_points, _Scale_density, IsDw);
		}
		else
		{ // 上表面锥体 上表面向下移动
			LOGINFO("0711 Up Pyramid");
			offsetVec = this->upBasePoint - this->dwBasePoint;
			offsetVec = (-_h / offsetVec.z) * offsetVec;
			LOGINFO("offsetVec = %s", offsetVec.dump_perl().c_str());
			upPoint = this->upBasePoint;
			this->Single_BaseMesh::OffsetMeshFlat(IsDw, offsetVec);
			dwPoint = this->upBasePoint;
			this->pUpPyramid = new Pyramid_Mesh(dwPoint, upPoint, this->shape_rotate, &shape_points, _Scale_density, IsDw);
		}

		return true;
	}

	// 移除锥体
	void
	SingleSPT_BaseMesh::Del_Pyramid(bool IsDw)
	{
		double pyramid_height = 0.0;

		if (IsDw)
		{ // 下台柱 删除台柱  下表面向下移动h
			if (pDwPyramid)
			{
				pyramid_height = pDwContact->GetMeshHeigt();
				delete pDwPyramid;
				pDwPyramid = NULL;

				// Vectorf3 offsetVec = Pointf3(0, 0, -pyramid_height);
				Vectorf3 offsetVec = this->upBasePoint - this->dwBasePoint;
				offsetVec = (-pyramid_height / offsetVec.z) * offsetVec;
				LOGINFO("offsetVec = %s", offsetVec.dump_perl().c_str());
				this->Single_BaseMesh::OffsetMeshFlat(IsDw, offsetVec);
			}
		}
		else
		{ // 上台柱 删除台柱  上表面向上移动h
			if (pUpPyramid)
			{
				pyramid_height = pUpContact->GetMeshHeigt();
				delete pUpPyramid;
				pUpPyramid = NULL;

				// Vectorf3 offsetVec = Pointf3(0, 0, pyramid_height);
				Vectorf3 offsetVec = this->upBasePoint - this->dwBasePoint;
				offsetVec = (pyramid_height / offsetVec.z) * offsetVec;
				LOGINFO("offsetVec = %s", offsetVec.dump_perl().c_str());
				this->Single_BaseMesh::OffsetMeshFlat(IsDw, offsetVec);
			}
		}
	}

	// 获取实体高度
	double
	SingleSPT_BaseMesh::GetTotalMeshHeigt()
	{
		double retval;
		retval = this->GetMeshHeigt();
		if (pUpContact)
			retval += pUpContact->GetMeshHeigt();
		if (pDwContact)
			retval += pDwContact->GetMeshHeigt();
		if (pUpPyramid)
			retval += pUpPyramid->GetMeshHeigt();
		if (pDwPyramid)
			retval += pDwPyramid->GetMeshHeigt();

		return retval;
	}

	// 偏移实体上下表面
	bool
	SingleSPT_BaseMesh::OffsetMeshFlat(bool IsDw, const Vectorf3 offsetVec)
	{
		this->Single_BaseMesh::OffsetMeshFlat(IsDw, offsetVec);
		if (IsDw)
		{
			if (pDwContact)
				pDwContact->OffsetMesh(offsetVec);
			if (pDwPyramid)
				pDwPyramid->OffsetMesh(offsetVec);
		}
		else
		{
			if (pUpPyramid)
				pUpPyramid->OffsetMesh(offsetVec);
			if (pUpContact)
				pUpContact->OffsetMesh(offsetVec);
		}

		return true;
	}

	// 偏移实体
	bool
	SingleSPT_BaseMesh::OffsetMesh(const Vectorf3 offsetVec)
	{
		this->OffsetMeshFlat(true, offsetVec);
		this->OffsetMeshFlat(false, offsetVec);

		return true;
	}

	// 设置中空形状
	bool
	SingleSPT_BaseMesh::SetMeshHollow(double hollow_dentity)
	{
		this->Single_BaseMesh::SetMeshHollow(hollow_dentity);
		if (pDwPyramid)
			pDwPyramid->SetMeshHollow(hollow_dentity);
		if (pUpPyramid)
			pUpPyramid->SetMeshHollow(hollow_dentity);

		return true;
	}

	// 平面自适应法相
	bool
	SingleSPT_BaseMesh::SetMeshAutoFix(bool IsDw, Vectorf3 normalVec)
	{
		this->Single_BaseMesh::SetMeshFlatAutoFix(IsDw, normalVec);
		if (IsDw)
		{
			if (pDwContact)
				pDwContact->SetMeshAutoFix(normalVec);
			if (pDwPyramid)
				pDwPyramid->SetMeshAutoFix(normalVec);
		}
		else
		{
			if (pUpPyramid)
				pUpPyramid->SetMeshAutoFix(normalVec);
			if (pUpContact)
				pUpContact->SetMeshAutoFix(normalVec);
		}

		return true;
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Cylinder_Mesh
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	bool
	Cylinder_Mesh::GeneralShapePoints()
	{
		Pointfs vertices;
		// double angle = (2 * PI / floor(2 * PI / fa));
		double angle = 2 * PI / fa;
		vertices.push_back(Pointf(sin(0) * ra, cos(0) * ra));
		for (double i = 0; i < 2 * PI; i += angle)
		{
			Pointf b(0, ra);
			b.rotate(i, Pointf(0, 0));
			vertices.push_back(b);
		}
		std::reverse(vertices.begin(), vertices.end());

		this->shape_points.clear();
		this->shape_points.insert(this->shape_points.end(), vertices.begin(), vertices.end());

		return Single_BaseMesh::RotatePointfs(&shape_points, this->shape_rotate);
	}

	TriangleMesh
	Cylinder_Mesh::GeneralMesh()
	{
		TriangleMesh _mesh = this->SingleSPT_BaseMesh::GeneralMesh();
		if (Topsp)
		{
			LOGINFO("Cylinder_Mesh SpTop");
			TriangleMesh _top = TriangleMeshGeneral::make_sphere(ra, 12, SpTop);
			_top.translate(upBasePoint.x, upBasePoint.y, upBasePoint.z);
			_mesh.merge(_top);
		}
		if (Btmsp)
		{
			LOGINFO("Cylinder_Mesh SpTop");
			TriangleMesh _btm = TriangleMeshGeneral::make_sphere(ra, 12, SpBottom);
			_btm.translate(dwBasePoint.x, dwBasePoint.y, dwBasePoint.z);
			_mesh.merge(_btm);
		}
		if (Topsp || Btmsp)
			_mesh.repair();

		_mesh.repair();
		if (_mesh.UnableRepair())
		{
			LOGINFO("Cylinder_Mesh::GeneralMesh Unable Repair");
			return TriangleMesh();
		}

		return _mesh;
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Cuber_Mesh
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	bool
	Cuber_Mesh::GeneralShapePoints()
	{
		this->shape_points.clear();
		this->shape_points.push_back(Pointf(l / 2, w / 2));
		this->shape_points.push_back(Pointf(l / 2, -w / 2));
		this->shape_points.push_back(Pointf(-l / 2, -w / 2));
		this->shape_points.push_back(Pointf(-l / 2, w / 2));

		return Single_BaseMesh::RotatePointfs(&shape_points, this->shape_rotate);
	}

	bool
	NumMarkBar_Mesh::GeneralShapePoints()
	{
		this->shape_points.clear();
		this->shape_points.push_back(Pointf(l / 2, w / 2));
		this->shape_points.push_back(Pointf(l / 2 + w / 2, 0));
		this->shape_points.push_back(Pointf(l / 2, -w / 2));
		this->shape_points.push_back(Pointf(-l / 2, -w / 2));
		this->shape_points.push_back(Pointf(-l / 2 - w / 2, 0));
		this->shape_points.push_back(Pointf(-l / 2, w / 2));

		return Single_BaseMesh::RotatePointfs(&shape_points, this->shape_rotate);
	}

	// 台柱体
	bool
	Contact_Mesh::GeneralShapePoints()
	{
		this->shape_points.clear();
		this->shape_points.push_back(Pointf(l / 2, w / 2));
		this->shape_points.push_back(Pointf(l / 2, -w / 2));
		this->shape_points.push_back(Pointf(-l / 2, -w / 2));
		this->shape_points.push_back(Pointf(-l / 2, w / 2));

		return Single_BaseMesh::RotatePointfs(&shape_points, this->shape_rotate);
	}

	bool
	SliceSingle_Mesh::GeneralShapePoints()
	{
		this->shape_points.clear();
		switch (this->sType)
		{
		case sTypeRight:
			this->shape_points.push_back(Pointf(0, 0));
			this->shape_points.push_back(Pointf(l, 0));
			break;
		case sTypeLeft:
			this->shape_points.push_back(Pointf(-l, 0));
			this->shape_points.push_back(Pointf(0, 0));
			break;
		case sTypeMiddle:
			this->shape_points.push_back(Pointf(-l / 2, 0));
			this->shape_points.push_back(Pointf(l / 2, 0));
			break;
		default:
			this->shape_points.push_back(Pointf(-l / 2, 0));
			this->shape_points.push_back(Pointf(l / 2, 0));
			break;
		}

		return Single_BaseMesh::RotatePointfs(&shape_points, this->shape_rotate);
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// CrossPrism_Mesh
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	bool
	CrossPrism_Mesh::GeneralShapePoints()
	{

		this->shape_points.clear();
		this->shape_points.push_back(Pointf(l / 2, w + l / 2));
		this->shape_points.push_back(Pointf(l / 2, l / 2));
		this->shape_points.push_back(Pointf(l / 2 + w, l / 2));

		this->shape_points.push_back(Pointf(l / 2 + w, -l / 2));
		this->shape_points.push_back(Pointf(l / 2, -l / 2));
		this->shape_points.push_back(Pointf(l / 2, -l / 2 - w));

		this->shape_points.push_back(Pointf(-l / 2, -w - l / 2));
		this->shape_points.push_back(Pointf(-l / 2, -l / 2));
		this->shape_points.push_back(Pointf(-l / 2 - w, -l / 2));

		this->shape_points.push_back(Pointf(-l / 2 - w, l / 2));
		this->shape_points.push_back(Pointf(-l / 2, l / 2));
		this->shape_points.push_back(Pointf(-l / 2, l / 2 + w));

		return Single_BaseMesh::RotatePointfs(&shape_points, this->shape_rotate);
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Prism_Mesh
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	bool
	Pyramid_Mesh::GeneralShapePoints()
	{
		if (pshapePoints == NULL)
			return false;

		this->shape_points.clear();
		this->shape_points.insert(this->shape_points.end(), pshapePoints->begin(), pshapePoints->end());

		// return Single_BaseMesh::RotatePointfs(&shape_points, this->shape_rotate);
		//  pshapePoints 已经被rotate过  不需要再次旋转
		return true;
	}

	bool
	Pyramid_Mesh::GeneralExPoints()
	{
		this->Single_BaseMesh::GeneralExPoints();

		// 锥体和支撑实体接触的表面如果完全一样，会被判定为共边，引起切片死循环
		// 稍微缩一点  规避这种情况
		const double protect_Scale_density = 0.9999;

		if (IsDw == false)
		{ // 上锥体
			Single_BaseMesh::ScaleLoopPoints(&upinner_points, upBasePoint, Scale_density);
			Single_BaseMesh::ScaleLoopPoints(&upouter_points, upBasePoint, Scale_density);
			Single_BaseMesh::ScaleLoopPoints(&dwinner_points, dwBasePoint, protect_Scale_density);
			Single_BaseMesh::ScaleLoopPoints(&dwouter_points, dwBasePoint, protect_Scale_density);
		}
		else
		{ // 下锥体
			Single_BaseMesh::ScaleLoopPoints(&dwinner_points, dwBasePoint, Scale_density);
			Single_BaseMesh::ScaleLoopPoints(&dwouter_points, dwBasePoint, Scale_density);
			Single_BaseMesh::ScaleLoopPoints(&upinner_points, dwBasePoint, protect_Scale_density);
			Single_BaseMesh::ScaleLoopPoints(&upouter_points, dwBasePoint, protect_Scale_density);
		}

		return true;
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// LineWall_BaseMesh
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	LineWall_BaseMesh::LineWall_BaseMesh(
		Pointf3s _wallps,
		bool _IsLoop,
		double _insert_depth,
		double _tooth_depth,
		unsigned int _tooth_step,
		unsigned int _split_step,
		unsigned int _split_wedth)
	{
		this->clear_date();
		this->wallbase_points = _wallps;
		this->IsLoop = _IsLoop;
		this->tooth_step = _tooth_step;
		this->tooth_depth = _tooth_depth;
		this->insert_depth = _insert_depth;
		this->split_step = _split_step;
		this->split_wedth = _split_wedth;
	}

	LineWall_BaseMesh::~LineWall_BaseMesh()
	{
		this->clear_date();
	}

	void LineWall_BaseMesh::clear_date()
	{
		Pointf3s().swap(this->wallbase_points);
		Pointf3s().swap(this->wallshape_points);
		Walls_UpShape.clear();
		Walls_DwShape.clear();
	}

	// 构造锯齿
	bool
	LineWall_BaseMesh::make_tooth(
		Pointf3s &ploop_points,
		double _tooth_depth, // 有正负 正向下挖  负向上凸
		unsigned int _tooth_step,
		bool Isstagger)
	{
		if (ploop_points.size() < 3)
		{
			LOGINFO("date error ploop_points.size error");
			return false;
		}
		unsigned int half_tooth_step = (_tooth_step / 2 == 0) ? 1 : _tooth_step / 2;
		// 修改顶点的z值
		for (int i = 0; i < ploop_points.size(); i++)
		{
			if (Isstagger)
			{
				if ((i + 1) % _tooth_step == 0)
					ploop_points.at(i).z -= _tooth_depth;
			}
			else
			{
				if ((i + 1 + half_tooth_step) % _tooth_step == 0)
					ploop_points.at(i).z -= _tooth_depth;
			}
			// 值保护
			if (ploop_points.at(i).z < 0.0)
				ploop_points.at(i).z = 0.0;
		}

		return true;
	}
	// 只为上接触点进行优化使用，造型从锯齿变为梯形。
	bool
	LineWall_BaseMesh::make_tooth_top(
		Pointf3s &ploop_points,
		double _tooth_depth, // 有正负 正向下挖  负向上凸
		unsigned int _tooth_step,
		unsigned int _tooth_width)
	{
		if (ploop_points.size() < 3)
		{
			LOGINFO("date error ploop_points.size error");
			return false;
		}
		if (_tooth_step < 0 || _tooth_width < 0)
		{
			LOGINFO("_tooth_step == %d  _tooth_width == %derror", _tooth_step, _tooth_width);
			return false;
		}
		unsigned int downPoints = (_tooth_step / 2 == 0) ? 1 : _tooth_step;
		// 修改顶点的z值
		for (int i = 0; i < ploop_points.size(); i++)
		{
			if (((i) % (_tooth_step + _tooth_width) < _tooth_width))
				ploop_points.at(i).z -= _tooth_depth;

			// 值保护
			if (ploop_points.at(i).z < 0.0)
				ploop_points.at(i).z = 0.0;
		}
		return true;
	}
	bool
	LineWall_BaseMesh::make_tooth_limit(
		Pointf3s &ploop_points,
		const Pointf3s plimit_points_up,
		const Pointf3s plimit_points_dw,
		double _tooth_depth, // 有正负 正向下挖  负向上凸
		unsigned int _tooth_step,
		bool Isstagger)
	{
		if (ploop_points.size() != plimit_points_up.size() || ploop_points.size() != plimit_points_dw.size())
		{
			LOGINFO("date error ploop_points.size()[%d], plimit_points_up.size()[%d, plimit_points_dw.size()[%d] ] ",
					ploop_points.size(), plimit_points_up.size(), plimit_points_dw.size());
			return false;
		}
		// 顶点数量太少
		if (ploop_points.size() <= 5)
		{
			LOGINFO("date error ploop_points.size <= 5");
			return false;
		}

		unsigned int half_tooth_step = (_tooth_step / 2 == 0) ? 1 : _tooth_step / 2;
		// 修改顶点的z值
		for (int i = 0; i < ploop_points.size(); i++)
		{
			// 值保护
			if (ploop_points.at(i).z > plimit_points_up.at(i).z - std::fabs(_tooth_depth) * 2 || ploop_points.at(i).z < plimit_points_dw.at(i).z + std::fabs(_tooth_depth) * 2)
				continue;

			if (Isstagger)
			{
				if ((i + 1) % _tooth_step == 0)
					ploop_points.at(i).z -= _tooth_depth;
			}
			else
			{
				if ((i + 1 + half_tooth_step) % _tooth_step == 0)
					ploop_points.at(i).z -= _tooth_depth;
			}
			// 值保护
			if (ploop_points.at(i).z < 0.0)
				ploop_points.at(i).z = 0.0;
		}

		return true;
	}

	Pointf3s
	LineWall_BaseMesh::make_wallline(
		const Pointf3s base_wallline,
		double wall_Z)
	{
		if (wall_Z < 0.0)
		{
			LOGINFO("wall_Z < 0.0");
			wall_Z = 0.0;
		}

		Pointf3s points_vec;
		points_vec.clear();
		for (size_t i = 0; i < base_wallline.size(); i++)
		{
			if (base_wallline[i].z >= wall_Z)
			{
				points_vec.push_back(Pointf3(base_wallline.at(i).x, base_wallline.at(i).y, wall_Z));
			}
			else
			{
				points_vec.push_back(base_wallline[i]);
			}
		}

		return points_vec;
	}
	// 重载记录的maketoothlimit
	bool
	LineWall_BaseMesh::make_tooth_limit(
		Pointf3s &ploop_points,
		Pointf3s &RecordHoleLine,
		const Pointf3s plimit_points_up,
		const Pointf3s plimit_points_dw,
		double _tooth_depth, // 有正负 正向下挖  负向上凸
		unsigned int _tooth_step,
		bool Isstagger)
	{
		if (ploop_points.size() != plimit_points_up.size() || ploop_points.size() != plimit_points_dw.size())
		{
			LOGINFO("date error ploop_points.size()[%d], plimit_points_up.size()[%d, plimit_points_dw.size()[%d] ] ",
					ploop_points.size(), plimit_points_up.size(), plimit_points_dw.size());
			return false;
		}
		// 顶点数量太少
		if (ploop_points.size() <= 3)
		{
			LOGINFO("date error ploop_points.size <= 3");
			return false;
		}

		unsigned int half_tooth_step = (_tooth_step / 2 == 0) ? 1 : _tooth_step / 2;
		// 修改顶点的z值
		for (int i = 0; i < ploop_points.size(); i++)
		{
			// 值保护
			if (ploop_points.at(i).z > plimit_points_up.at(i).z - std::fabs(_tooth_depth) * 2 || ploop_points.at(i).z < plimit_points_dw.at(i).z + std::fabs(_tooth_depth) * 2)
			{
				continue;
			}

			if (Isstagger)
			{

				if ((i + 1) % _tooth_step == 0)
				{

					ploop_points.at(i).z -= _tooth_depth;
					RecordHoleLine.at(i).z = -1.0;
				}
			}
			else
			{
				if ((i + 1 + half_tooth_step) % _tooth_step == 0)
				{

					ploop_points.at(i).z -= _tooth_depth;
					RecordHoleLine.at(i).z = -1.0;
				}
			}
			// 值保护

			if (ploop_points.at(i).z < 0.0)
				ploop_points.at(i).z = 0.0;

			// LOGINFO("ploop_points.at(%d).z[%f]--RecordHoleLine.at(%d).[%f]", i, ploop_points.at(i).z, i, RecordHoleLine.at(i).z);
		}

		return true;
	}

	bool
	LineWall_BaseMesh::GeneralShapePoints()
	{
		if (wallbase_points.size() < 2)
		{
			LOGINFO("wallbase_points.size[%d] error", wallbase_points.size());
			return false;
		}
		this->wallshape_points.clear();
		this->wallshape_points = wallbase_points;

		// loop首尾相连
		if (this->IsLoop)
			this->wallshape_points.push_back(this->wallshape_points.front());

		// 增加insert深度
		double Length_wall = 0.0;
		for (unsigned int i = 0; i < this->wallshape_points.size(); i++)
		{
			if (i != 0)
			{
				Length_wall += std::sqrt(
					(this->wallshape_points[i].x - this->wallshape_points[i - 1].x) * (this->wallshape_points[i].x - this->wallshape_points[i - 1].x) + (this->wallshape_points[i].y - this->wallshape_points[i - 1].y) * (this->wallshape_points[i].y - this->wallshape_points[i - 1].y));
			}
			this->wallshape_points[i].z += this->insert_depth;
		}
		if (Length_wall < 5.0)
		{
			LOGINFO("Length_wall too small [%f]", Length_wall);
			// return false;
		}

		// 构造锯齿
		bool result = LineWall_BaseMesh::make_tooth(wallshape_points, tooth_depth, tooth_step, false);
		for (unsigned int i = 0; i < wallshape_points.size(); i++)
		{
			splitwall_maxz = std::fmax(splitwall_maxz, wallshape_points.at(i).z);
			splitwall_minz = std::fmin(splitwall_minz, wallshape_points.at(i).z);
		}
		return result;
	}

	// 构建Mesh
	TriangleMesh
	LineWall_BaseMesh::GeneralMesh()
	{
		// 参数检查
		int Walls_Num = this->Walls_UpShape.size();

		if (this->Walls_UpShape.size() == 0 || this->Walls_DwShape.size() == 0 || this->Walls_UpShape.size() != this->Walls_DwShape.size())
		{
			LOGINFO("Walls_UpShape.size[%d] != Walls_DwShape.size[%d] error!! ",
					this->Walls_UpShape.size(),
					this->Walls_DwShape.size());
			return TriangleMesh();
		}

		// 构建mesh
		TriangleMesh wallsmesh;
		std::list<Pointf3s>::iterator upIt = Walls_UpShape.begin();
		std::list<Pointf3s>::iterator dwIt = Walls_DwShape.begin();
		while (upIt != Walls_UpShape.end() && dwIt != Walls_DwShape.end())
		{
			int Points_Num = this->wallshape_points.size();
			if (Points_Num == 0 || upIt->size() != dwIt->size())
			{
				LOGINFO("Points_Num[%d/%d] error!! ", upIt->size(), dwIt->size());
				return TriangleMesh();
			}

			// 构建mesh
			std::vector<Point3> facets; // 三角面片顶点索引
			Pointf3s vertices;			// 顶点数组
			unsigned int id = 0;
			for (unsigned int i = 0; i < Points_Num; i++)
			{
				// 取出一对边
				Pointf3s &upshape = *upIt;
				Pointf3s &dwshape = *dwIt;
				// 压入一对顶点
				vertices.push_back(upshape[i]);
				vertices.push_back(dwshape[i]);

				if (i == 0) // 从第二点开始
					continue;
				if (split_step > 0 &&
					split_wedth > 0 &&
					i % (split_step + split_wedth) > (split_step - 1))
					continue;

				id = vertices.size() - 1;
				facets.push_back(Point3(id, id - 1, id - 3));
				facets.push_back(Point3(id, id - 3, id - 2));
			}

			TriangleMesh wall_mesh(vertices, facets);
			wall_mesh.checkonly = true;
			wall_mesh.repair();
			wallsmesh.merge(wall_mesh);

			upIt++;
			dwIt++;
		}
		wallsmesh.merge(this->GeneralMesh_CrossBeam(0.2, 1));
		wallsmesh.checkonly = true;
		wallsmesh.repair();

		if (wallsmesh.UnableRepair())
		{
			LOGINFO("LineWall_BaseMesh::GeneralMesh Unable Repair");
			return TriangleMesh();
		}
		return wallsmesh;
	}

	// 构建Mesh
	TriangleMesh
	LineWall_BaseMesh::GeneralMesh_CrossBeam(double _wedth, double _depth)
	{
		// 构建mesh
		std::vector<Point3> facets; // 三角面片顶点索引
		Pointf3s vertices;			// 顶点数组

		// 构建mesh
		for (int i = 0; i < wallshape_points.size(); i++)
		{
			Pointf3 MidPf3 = wallshape_points[i];
			Pointf3 LeftPf3 = wallshape_points[i];
			Pointf3 RightPf3 = wallshape_points[i];

			if (i == 0 || i == wallshape_points.size() - 1)
				continue;
			if (
				split_step > 3 && (split_step - split_wedth) > 1 && (i % split_step >= (split_step - split_wedth)) && ((i + 1) % split_step >= (split_step - split_wedth)))
				continue;

			Vectorf v1(
				wallshape_points[i].x - wallshape_points[i - 1].x,
				wallshape_points[i].y - wallshape_points[i - 1].y);
			Vectorf v2(
				wallshape_points[i].x - wallshape_points[i + 1].x,
				wallshape_points[i].y - wallshape_points[i + 1].y);
			Vectorf vmid(v1.x + v2.x, v1.y + v2.y);
			if (vmid.length() < 0.001)
			{
				vmid.x = -v1.y;
				vmid.y = v1.x;
			}
			vmid.normalize();
			LeftPf3.x += vmid.x * _wedth;
			LeftPf3.y += vmid.y * _wedth;
			RightPf3.x -= vmid.x * _wedth;
			RightPf3.y -= vmid.y * _wedth;

			// 该点是尖顶
			if (wallshape_points[i].z >= wallbase_points[i].z)
			{
				LeftPf3.z -= _depth;
				RightPf3.z -= _depth;
				if (LeftPf3.z < 0)
					LeftPf3.z = 0;
				if (RightPf3.z < 0)
					RightPf3.z = 0;
			}

			vertices.push_back(LeftPf3);
			vertices.push_back(Pointf3(LeftPf3.x, LeftPf3.y, 0));
			vertices.push_back(MidPf3);
			vertices.push_back(Pointf3(MidPf3.x, MidPf3.y, 0));
			vertices.push_back(RightPf3);
			vertices.push_back(Pointf3(RightPf3.x, RightPf3.y, 0));
			size_t id = vertices.size() - 1;
			facets.push_back(Point3(id, id - 1, id - 3));
			facets.push_back(Point3(id, id - 3, id - 2));
			facets.push_back(Point3(id - 2, id - 3, id - 5));
			facets.push_back(Point3(id - 2, id - 5, id - 4));

			LOGINFO("Left%s  Mid%s  Right%s wallshape_points[i-1]%s wallshape_points[i+1]%s vmid%s",
					LeftPf3.dump_perl().c_str(),
					MidPf3.dump_perl().c_str(),
					RightPf3.dump_perl().c_str(),
					wallshape_points[i - 1].dump_perl().c_str(),
					wallshape_points[i + 1].dump_perl().c_str(),
					vmid.dump_perl().c_str());
		}

		TriangleMesh beamsmesh(vertices, facets);
		beamsmesh.checkonly = true;
		beamsmesh.repair();

		return beamsmesh;
	}

	// 构建部分Mesh
	TriangleMesh
	LineWall_BaseMesh::GeneralMesh(int part_index)
	{
		// 参数检查
		int Walls_Num = this->Walls_UpShape.size();

		if (this->Walls_UpShape.size() == 0 || this->Walls_DwShape.size() == 0 || this->Walls_UpShape.size() != this->Walls_DwShape.size())
		{
			LOGINFO("Walls_UpShape.size[%d] != Walls_DwShape.size[%d] error!! ",
					this->Walls_UpShape.size(),
					this->Walls_DwShape.size());
			return TriangleMesh();
		}

		// 构建mesh
		TriangleMesh wallsmesh;
		std::list<Pointf3s>::iterator upIt = Walls_UpShape.begin();
		std::list<Pointf3s>::iterator dwIt = Walls_DwShape.begin();
		while (upIt != Walls_UpShape.end() && dwIt != Walls_DwShape.end())
		{
			int Points_Num = this->wallshape_points.size();
			if (Points_Num == 0 || upIt->size() != dwIt->size() || part_index <= 0 || part_index >= Points_Num)
			{
				LOGINFO("Error!! Points_Num[%d/%d] Part_index[%d]", upIt->size(), dwIt->size(), part_index);
				break;
			}
			if (split_step > 0 &&
				split_wedth > 0 &&
				part_index % (split_step + split_wedth) > (split_step - 1))
			{
				LOGINFO("Part_index[%d] in the gap", part_index);
				break;
			}

			// 构建mesh
			std::vector<Point3> facets; // 三角面片顶点索引
			Pointf3s vertices;			// 顶点数组
			unsigned int id = 0;

			// 取出一对边
			Pointf3s &upshape = *upIt;
			Pointf3s &dwshape = *dwIt;
			// 压入一对顶点
			vertices.push_back(upshape[part_index - 1]);
			vertices.push_back(dwshape[part_index - 1]);
			vertices.push_back(upshape[part_index]);
			vertices.push_back(dwshape[part_index]);

			id = vertices.size() - 1;
			facets.push_back(Point3(id, id - 1, id - 3));
			facets.push_back(Point3(id, id - 3, id - 2));

			TriangleMesh wall_mesh(vertices, facets);
			wall_mesh.checkonly = true;
			wall_mesh.repair();
			wallsmesh.merge(wall_mesh);

			upIt++;
			dwIt++;
		}

		if (wallsmesh.facets_count())
		{
			wallsmesh.merge(this->GeneralMesh_CrossBeam(0.2, 1, part_index));
			wallsmesh.checkonly = true;
			wallsmesh.repair();
		}
		if (wallsmesh.UnableRepair())
		{
			LOGINFO("LineWall_BaseMesh::GeneralMesh Unable Repair");
			return TriangleMesh();
		}
		return wallsmesh;
	}

	// 构建Mesh
	TriangleMesh
	LineWall_BaseMesh::GeneralMesh_CrossBeam(double _wedth, double _depth, int part_index)
	{
		// 构建mesh
		std::vector<Point3> facets; // 三角面片顶点索引
		Pointf3s vertices;			// 顶点数组

		// 构建mesh
		if (part_index <= 0 || part_index >= wallshape_points.size() - 1)
		{
			LOGINFO("out of part index [%d/%d]", part_index, wallshape_points.size());
			return TriangleMesh();
		}
		if (
			split_step > 3 && (split_step - split_wedth) > 1 && (part_index % split_step >= (split_step - split_wedth)) && ((part_index + 1) % split_step >= (split_step - split_wedth)))
		{
			LOGINFO("part_index[%d] in the gaps ", part_index);
			return TriangleMesh();
		}

		Pointf3 MidPf3 = wallshape_points[part_index];
		Pointf3 LeftPf3 = wallshape_points[part_index];
		Pointf3 RightPf3 = wallshape_points[part_index];

		Vectorf v1(
			wallshape_points[part_index].x - wallshape_points[part_index - 1].x,
			wallshape_points[part_index].y - wallshape_points[part_index - 1].y);
		Vectorf v2(
			wallshape_points[part_index].x - wallshape_points[part_index + 1].x,
			wallshape_points[part_index].y - wallshape_points[part_index + 1].y);

		if (v1.length() > 0.001)
		{
			v1.normalize();
		}

		if (v2.length() > 0.001)
		{
			v2.normalize();
		}

		Vectorf vmid(v1.x + v2.x, v1.y + v2.y);
		if (vmid.length() < 0.001)
		{
			vmid.x = -v1.y;
			vmid.y = v1.x;
		}
		if (vmid.length() < 0.001)
		{
			LOGINFO("wall part index [%d/%d] vmid[%f, %f].length() < 0.001",
					part_index,
					wallshape_points.size(),
					vmid.x,
					vmid.y);
			return TriangleMesh();
		}
		vmid.normalize();
		LeftPf3.x += vmid.x * _wedth;
		LeftPf3.y += vmid.y * _wedth;
		RightPf3.x -= vmid.x * _wedth;
		RightPf3.y -= vmid.y * _wedth;

		// 该点是尖顶
		if (wallshape_points[part_index].z >= wallbase_points[part_index].z)
		{
			LeftPf3.z -= _depth;
			RightPf3.z -= _depth;
			if (LeftPf3.z < 0)
				LeftPf3.z = 0;
			if (RightPf3.z < 0)
				RightPf3.z = 0;
		}

		vertices.push_back(LeftPf3);
		vertices.push_back(Pointf3(LeftPf3.x, LeftPf3.y, 0));
		vertices.push_back(MidPf3);
		vertices.push_back(Pointf3(MidPf3.x, MidPf3.y, 0));
		vertices.push_back(RightPf3);
		vertices.push_back(Pointf3(RightPf3.x, RightPf3.y, 0));
		size_t id = vertices.size() - 1;
		facets.push_back(Point3(id, id - 1, id - 3));
		facets.push_back(Point3(id, id - 3, id - 2));
		facets.push_back(Point3(id - 2, id - 3, id - 5));
		facets.push_back(Point3(id - 2, id - 5, id - 4));

		LOGINFO("Left%s  Mid%s  Right%s wallshape_points[i-1]%s wallshape_points[i+1]%s vmid%s",
				LeftPf3.dump_perl().c_str(),
				MidPf3.dump_perl().c_str(),
				RightPf3.dump_perl().c_str(),
				wallshape_points[part_index - 1].dump_perl().c_str(),
				wallshape_points[part_index + 1].dump_perl().c_str(),
				vmid.dump_perl().c_str());

		TriangleMesh beamsmesh(vertices, facets);
		beamsmesh.checkonly = true;
		beamsmesh.repair();

		return beamsmesh;
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// LineWall_Mesh
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// 构建某一层墙体上下半边
	bool
	LineWall_Mesh::GeneralWallLines(Pointf3s wall_top, Pointf3s wall_base, double WallHeight, bool &Isstagger)
	{
		if (wall_top.empty())
		{
			LOGINFO("wall_top.empty() GeneralWallLines finish");
			return false;
		}
		// 计算最低点
		double splitwall_maxz = DBL_MIN;
		for (unsigned int i = 0; i < wall_top.size(); i++)
		{
			splitwall_maxz = std::fmax(splitwall_maxz, wall_top.at(i).z);
		}
		if (WallHeight >= splitwall_maxz)
		{
			LOGINFO("WallHeight >= splitwall_maxz");
			return false;
		}

		Pointf3s line_buttom = LineWall_BaseMesh::make_wallline(wall_top, WallHeight);
		double next_WallHeight = WallHeight + this->hole_height_step; // 取下层高度
		Pointf3s line_top = LineWall_BaseMesh::make_wallline(wall_top, next_WallHeight);
		LineWall_BaseMesh::make_tooth_limit(line_buttom, wall_top, wall_base,
											-(hole_depth), hole_length_step, Isstagger);
		Isstagger = !Isstagger;
		LineWall_BaseMesh::make_tooth_limit(line_top, wall_top, wall_base,
											hole_depth, hole_length_step, Isstagger);
		this->Walls_DwShape.push_back(line_buttom);
		this->Walls_UpShape.push_back(line_top);

		return true;
	}

	// 构建墙体上下半边
	bool
	LineWall_Mesh::GeneralExPoints()
	{
		// 造型顶点为空
		if (this->wallshape_points.size() == 0)
		{
			LOGINFO("wallshape_points size == 0");
			return false;
		}
		// 齿的高度大于间隔
		if (hole_depth > hole_height_step)
		{
			LOGINFO("hole_weidth[%f] > hole_height_step[%f]", hole_depth, hole_height_step);
			return false;
		}

		// 按照从下向上的顺序构建墙体
		double wall_splitz = 0.0;
		bool Isstagger = false;
		//  构造边界
		Pointf3s wall_base = LineWall_BaseMesh::make_wallline(wallshape_points, 0.0);
		while (this->GeneralWallLines(wallshape_points, wall_base, wall_splitz, Isstagger))
		{
			wall_splitz += this->hole_height_step;
		}

		return true;
	}

	bool
	Hexhole_LineWall_Mesh::GeneralShapePoints()
	{
		if (wallbase_points.size() < 2)
		{
			LOGINFO("wallbase_points.size[%d] error", wallbase_points.size());
			return false;
		}
		this->wallshape_points.clear();
		this->wallshape_points = wallbase_points;

		// loop首尾相连
		if (this->IsLoop)
			this->wallshape_points.push_back(this->wallshape_points.front());

		// 增加insert深度
		double Length_wall = 0.0;
		for (unsigned int i = 0; i < this->wallshape_points.size(); i++)
		{
			if (i != 0)
			{
				Length_wall += std::sqrt(
					(this->wallshape_points[i].x - this->wallshape_points[i - 1].x) * (this->wallshape_points[i].x - this->wallshape_points[i - 1].x) + (this->wallshape_points[i].y - this->wallshape_points[i - 1].y) * (this->wallshape_points[i].y - this->wallshape_points[i - 1].y));
			}
			this->wallshape_points[i].z += this->insert_depth;
		}
		if (Length_wall < 5.0)
		{
			LOGINFO("Length_wall too small [%f]", Length_wall);
			// return false;
		}

		// 构造锯齿
		bool result = LineWall_BaseMesh::make_tooth_top(wallshape_points, tooth_depth, tooth_step, tooth_width_hl);
		for (unsigned int i = 0; i < wallshape_points.size(); i++)
		{
			splitwall_maxz = std::fmax(splitwall_maxz, wallshape_points.at(i).z);
			splitwall_minz = std::fmin(splitwall_minz, wallshape_points.at(i).z);
		}
		return result;
	}

	// 构造孔洞界线

	bool
	Hexhole_LineWall_Mesh::GeneralWallLines(Pointf3s wall_top, Pointf3s wall_base, double WallHeight, bool &Isstagger)
	{
		LOGINFO("---1123---Hexhole_LineWall_Mesh::GeneralWallLines");
		if (wall_top.empty())
		{
			LOGINFO("wall_top.empty() GeneralWallLines finish");
			return false;
		}
		// 计算最高点

		if (WallHeight >= splitwall_maxz)
		{
			LOGINFO("WallHeight[%lf] >= splitwall_maxz[%lf]", WallHeight, splitwall_maxz);
			return false;
		}

		Pointf3s line_buttom = LineWall_BaseMesh::make_wallline(wall_top, WallHeight);
		double next_WallHeight = WallHeight + this->hole_height_step;	  // 取下层高度
		double next_WallHeight2 = next_WallHeight + this->hexhole_length; // 取下层高度
		Pointf3s line_top = LineWall_BaseMesh::make_wallline(wall_top, next_WallHeight);
		Pointf3s hole_bottom = line_top;
		Pointf3s hole_top = line_top;
		for (auto &it : hole_top)
		{
			it.z += this->hexhole_length;
		}
		hole_top = LineWall_BaseMesh::make_wallline(wall_top, next_WallHeight2);

		LineWall_BaseMesh::make_tooth_limit(line_buttom, wall_top, wall_base,
											-(hole_depth), hole_length_step, Isstagger);
		Isstagger = !Isstagger;

		LineWall_BaseMesh::make_tooth_limit(line_top, wall_top, wall_base,
											(hole_depth), hole_length_step, Isstagger);
		//

		LineWall_BaseMesh::make_tooth_limit(hole_top, hole_bottom, wall_top, wall_base,
											(hole_depth), hole_length_step, Isstagger); // 将hole_bottom作为标记,不需要加的就为-1.

		this->Walls_DwShape.push_back(line_buttom);
		this->Walls_UpShape.push_back(line_top);
		this->Walls_DwShape.push_back(hole_bottom); // 含有孔洞信息
		this->Walls_UpShape.push_back(hole_top);

		return true;
	}

	bool
	Hexhole_LineWall_Mesh::GeneralExPoints()
	{
		LOGINFO("----0710----Hexhole_LineWall_Mesh If_addBeam_hl [%d]", this->If_addBeam_hl);
		// 造型顶点为空
		if (this->wallshape_points.size() == 0)
		{
			LOGINFO("wallshape_points size == 0");
			return false;
		}
		// 齿的高度大于间隔
		// if ((hole_depth + hexhole_length) > hole_height_step)
		// {
		// 	LOGINFO("hole_weidth[%f]+ _hexhole_length[%f]> hole_height_step[%f]", hole_depth, hexhole_length, hole_height_step);
		// 	return false;
		// }
		if ((hole_depth) > hole_height_step)
		{
			LOGINFO("hole_weidth[%f]+> hole_height_step[%f]", hole_depth, hole_height_step);
			return false;
		}
		// if ((tooth_depth + down_tooth_depth_hl) > splitwall_minz)
		// {
		// 	LOGINFO("tooth_depth[%f]+ down_tooth_depth_hl[%f]> splitwall_minz[%f]", tooth_depth, down_tooth_depth_hl, splitwall_minz);
		// 	return false;
		// }

		if (this->tooth_depth > 0)
		{
			this->line_Tooth_Top_Up = wallshape_points;
			this->line_Tooth_Top_Dw = wallshape_points;
			for (int i = 0; i < line_Tooth_Top_Up.size(); i++)
			{
				line_Tooth_Top_Dw[i].z -= this->tooth_depth;
				if (line_Tooth_Top_Dw.at(i).z < 0)
				{
					line_Tooth_Top_Dw.at(i).z = 0.0;
				}
				// LOGINFO("[%d] 0-P [%s] 1-P [%s] ", i, line_Tooth_Top_Up[i].dump_perl().c_str(), line_Tooth_Top_Dw[i].dump_perl().c_str());
			}
			wallshape_points = this->line_Tooth_Top_Dw;
		}

		// 按照从下向上的顺序构建墙体
		double wall_splitz = 0.0;
		bool Isstagger = false;
		Pointf3s wall_base = LineWall_BaseMesh::make_wallline(wallshape_points, 0.0);
		LOGINFO("this->down_tooth_depth_hl is [%lf]", this->down_tooth_depth_hl);
		if (this->down_tooth_depth_hl > 0)
		{
			// LOGINFO("0725 this->down_tooth_depth_hl is [%f]", this->down_tooth_depth_hl);
			Pointf3s line_Tooth_buttom_Dw = LineWall_BaseMesh::make_wallline(wallshape_points, 0.0);
			Pointf3s line_Tooth_buttom_Up = LineWall_BaseMesh::make_wallline(wallshape_points, down_tooth_depth_hl);
			for (int i = 0; i < line_Tooth_buttom_Dw.size(); i++)
			{
				if (((i) % (tooth_step_hl + tooth_width_hl) < tooth_width_hl))
				{
					line_Tooth_buttom_Dw.at(i).z += down_tooth_depth_hl;
					if (line_Tooth_buttom_Dw.at(i).z > wallshape_points[i].z)
					{
						line_Tooth_buttom_Dw.at(i).z = wallshape_points[i].z;
					}
				}
			}
			wall_splitz += down_tooth_depth_hl;
			wall_base = LineWall_BaseMesh::make_wallline(wallshape_points, down_tooth_depth_hl);
			this->Walls_DwShape.push_back(line_Tooth_buttom_Dw);
			this->Walls_UpShape.push_back(line_Tooth_buttom_Up);
		}
		while (this->GeneralWallLines(wallshape_points, wall_base, wall_splitz, Isstagger))
		{
			wall_splitz += this->hole_height_step;
			wall_splitz += this->hexhole_length;
			// LOGINFO("----1024---- now the wall_splitz is [%lf]", wall_splitz);
		}
		if (this->tooth_depth > 0)
		{
			this->Walls_DwShape.push_back(this->line_Tooth_Top_Dw);
			this->Walls_UpShape.push_back(this->line_Tooth_Top_Up);
		}
		return true;
	}
	TriangleMesh
	Hexhole_LineWall_Mesh::GeneralMesh(int part_index)
	{
		LOGINFO("----1024---- build the hex hole linewall mesh by part_index");
		// 参数检查
		int Walls_Num = this->Walls_UpShape.size();

		if (this->Walls_UpShape.size() == 0 || this->Walls_DwShape.size() == 0 || this->Walls_UpShape.size() != this->Walls_DwShape.size())
		{
			LOGINFO("Walls_UpShape.size[%d] != Walls_DwShape.size[%d] error!! ",
					this->Walls_UpShape.size(),
					this->Walls_DwShape.size());
			return TriangleMesh();
		}

		// 构建mesh
		TriangleMesh wallsmesh;
		int wallsmesh_facect_num = 0;
		std::list<Pointf3s>::iterator upIt = Walls_UpShape.begin();
		std::list<Pointf3s>::iterator dwIt = Walls_DwShape.begin();
		while (upIt != Walls_UpShape.end() && dwIt != Walls_DwShape.end())
		{
			int Points_Num = this->wallshape_points.size();
			if (Points_Num == 0 || upIt->size() != dwIt->size() || part_index <= 0 || part_index >= Points_Num)
			{
				LOGINFO("Error!! Points_Num[%d/%d] Part_index[%d]", upIt->size(), dwIt->size(), part_index);
				break;
			}
			if (split_step > 0 &&
				split_wedth > 0 &&
				part_index % (split_step + split_wedth) > (split_step - 1))
			{
				LOGINFO("Part_index[%d] in the gap", part_index);
				break;
			}

			// 构建mesh
			std::vector<Point3> facets; // 三角面片顶点索引
			Pointf3s vertices;			// 顶点数组
			unsigned int id = 0;

			// 取出一对边
			Pointf3s &upshape = *upIt;
			Pointf3s &dwshape = *dwIt;
			// 压入一对顶点
			vertices.push_back(upshape[part_index - 1]);
			vertices.push_back(dwshape[part_index - 1]);
			vertices.push_back(upshape[part_index]);
			vertices.push_back(dwshape[part_index]);
			id = vertices.size() - 1;
			if ((dwshape[part_index - 1].z != -1.0) && (dwshape[part_index].z != -1.0))
			{

				facets.push_back(Point3(id, id - 1, id - 3));
				facets.push_back(Point3(id, id - 3, id - 2));
				wallsmesh_facect_num += 2;
				TriangleMesh wall_mesh(vertices, facets);
				wall_mesh.checkonly = true;
				wall_mesh.repair();
				wallsmesh.merge(wall_mesh);
			}
			upIt++;
			dwIt++;
		}

		if (wallsmesh.facets_count())
		{
			LOGINFO("wallsmesh.facets_count() is %d ", wallsmesh.facets_count());
			if (this->If_addBeam_hl)
			{
				wallsmesh.merge(this->GeneralMesh_CrossBeam(0.2, 1, part_index));
			}

			// LOGINFO("Add the GeneralMesh_CrossBeam");
			wallsmesh.checkonly = true;
			wallsmesh.repair();
			// LOGINFO("after add crossbeam wallsmesh.facets_count() is %d ", wallsmesh.facets_count());
		}
		if (wallsmesh.UnableRepair())
		{
			LOGINFO("Hexhole_LineWall_Mesh::GeneralMesh Unable Repair");
			return TriangleMesh();
		}
		return wallsmesh;
	}
	// 六边形孔墙基类 生成面片
	TriangleMesh
	Hexhole_LineWall_Mesh::GeneralMesh()
	{
		LOGINFO("----0710---- build the hex hole linewall mesh");
		// 参数检查
		int Walls_Num = this->Walls_UpShape.size();
		LOGINFO("0710---Walls_Num is [%d] !! ", Walls_Num);
		if (this->Walls_UpShape.size() == 0 || this->Walls_DwShape.size() == 0 || this->Walls_UpShape.size() != this->Walls_DwShape.size())
		{
			LOGINFO("Walls_UpShape.size[%d] != Walls_DwShape.size[%d] error!! ",
					this->Walls_UpShape.size(),
					this->Walls_DwShape.size());
			return TriangleMesh();
		}

		// 构建mesh
		TriangleMesh wallsmesh;
		std::list<Pointf3s>::iterator upIt = Walls_UpShape.begin();
		std::list<Pointf3s>::iterator dwIt = Walls_DwShape.begin();
		while (upIt != Walls_UpShape.end() && dwIt != Walls_DwShape.end())
		{
			int Points_Num = this->wallshape_points.size();
			LOGINFO("1123---Points_Num is [%d] !! ", Points_Num);
			if (Points_Num == 0 || upIt->size() != dwIt->size()) //
			{
				LOGINFO("Points_Num[%d/%d] error!! ", upIt->size(), dwIt->size());
				return TriangleMesh();
			}

			// 构建mesh
			std::vector<Point3> facets; // 三角面片顶点索引
			Pointf3s vertices;			// 顶点数组
			unsigned int id = 0;
			for (unsigned int i = 0; i < Points_Num; i++)
			{
				// 取出一对边
				Pointf3s &upshape = *upIt;
				Pointf3s &dwshape = *dwIt;
				// 压入一对顶点
				vertices.push_back(upshape[i]);
				vertices.push_back(dwshape[i]);

				if (i == 0) // 从第二点开始
					continue;
				if (split_step > 0 &&
					split_wedth > 0 &&
					i % (split_step + split_wedth) > (split_step - 1))
					continue;

				id = vertices.size() - 1;
				if ((dwshape[i - 1].z != -1.0) && (dwshape[i].z != -1.0))
				{

					facets.push_back(Point3(id, id - 1, id - 3));
					facets.push_back(Point3(id, id - 3, id - 2));
				}
			}

			TriangleMesh wall_mesh(vertices, facets);
			wall_mesh.checkonly = true;
			wall_mesh.repair();
			wallsmesh.merge(wall_mesh);
			upIt++;
			dwIt++;
		}

		if (If_addBeam_hl == 1)
		{
			wallsmesh.merge(this->GeneralMesh_CrossBeam(Beam_Width_hl, 1));
		}

		wallsmesh.checkonly = true;
		wallsmesh.repair();

		if (wallsmesh.UnableRepair())
		{
			LOGINFO("Hexhole_LineWall_Mesh::GeneralMesh Unable Repair");
			return TriangleMesh();
		}
		return wallsmesh;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////
	// 实体墙基类
	///////////////////////////////////////////////////////////////////////////////////////////////
	EntityWall_BaseMesh::EntityWall_BaseMesh(
		Pointf3s _wallps,
		bool _IsLoop,
		double _insert_depth,
		double _tooth_depth,
		double _thickness,
		double _contact_thickness,
		unsigned int _tooth_step,
		unsigned int _hole_step,
		unsigned int _split_step,
		unsigned int _split_wedth,
		double _cross_width)
	{
		this->clear_date();
		this->wallbase_points = _wallps;
		this->IsLoop = _IsLoop;
		this->tooth_step = _tooth_step;
		this->hole_step = _hole_step;
		this->tooth_depth = _tooth_depth;
		this->insert_depth = _insert_depth;
		this->thickness = _thickness;
		this->contact_thickness = _contact_thickness;
		this->split_step = _split_step;
		this->split_wedth = _split_wedth;
		this->cross_width = _cross_width;

		for (unsigned int i = 0; i < _wallps.size(); i++)
		{
			if (_wallps[i].z < 1)
			{
				// LOGINFO("error Point  this->wallbase_points[%d] = %s", i, _wallps[i].dump_perl().c_str());
			}
		}
	}

	EntityWall_BaseMesh::~EntityWall_BaseMesh()
	{
		this->clear_date();
	}

	void
	EntityWall_BaseMesh::clear_date()
	{
		Pointf3s().swap(this->wallbase_points);
		Pointf3s().swap(this->wallshape_points);
		Pointf3s().swap(this->wallmiddle_points);
		Pointf3s().swap(this->Walls_UpLeftShape);
		Pointf3s().swap(this->Walls_UpRightShape);
		Pointf3s().swap(this->Walls_CtLeftShape);
		Pointf3s().swap(this->Walls_CtRightShape);
		Pointf3s().swap(this->Walls_DwLeftShape);
		Pointf3s().swap(this->Walls_DwRightShape);
	}

	// 构造锯齿
	bool
	EntityWall_BaseMesh::make_tooth(
		Pointf3s &ploop_points,
		double _tooth_depth, // 有正负 正向下挖  负向上凸
		unsigned int _tooth_step,
		unsigned int _hole_step)
	{
		if (ploop_points.size() < 3)
		{
			LOGINFO("date error ploop_points.size error");
			return false;
		}

		unsigned int step = _tooth_step + _hole_step;
		// 修改顶点的Z值
		for (int i = 0; i < ploop_points.size(); i++)
		{
			if (i % step >= _tooth_step)
				ploop_points.at(i).z -= _tooth_depth;
			// 值保护
			if (ploop_points.at(i).z < 0.0)
				ploop_points.at(i).z = 0.0;
		}
		return true;
	}

	// 构造界线
	Pointf3s
	EntityWall_BaseMesh::make_wallline(
		const Pointf3s base_wallline,
		double wall_Z,
		double distance // 偏移距离
	)
	{
		if (wall_Z < 0.0)
		{
			LOGINFO("wall_Z < 0.0");
			wall_Z = 0.0;
		}

		Pointf3s points_vec;
		points_vec.clear();
		for (size_t i = 0; i < base_wallline.size(); i++)
		{
			Pointf3 point = base_wallline[i];
			// 顶点偏移
			if (i == 0)
			{
				Vectorf v2(
					base_wallline[i + 1].x - base_wallline[i].x,
					base_wallline[i + 1].y - base_wallline[i].y);
				v2.rotate(PI / 2);
				v2.normalize();

				point.x += v2.x * distance;
				point.y += v2.y * distance;
				if (point.z >= wall_Z)
					point.z = wall_Z;
			}
			else if (i == base_wallline.size() - 1)
			{
				Vectorf v1(
					base_wallline[i - 1].x - base_wallline[i].x,
					base_wallline[i - 1].y - base_wallline[i].y);
				v1.rotate(-PI / 2);
				v1.normalize();

				point.x += v1.x * distance;
				point.y += v1.y * distance;
				if (point.z >= wall_Z)
					point.z = wall_Z;
			}
			else
			{
				Vectorf v1(
					base_wallline[i - 1].x - base_wallline[i].x,
					base_wallline[i - 1].y - base_wallline[i].y);
				Vectorf v2(
					base_wallline[i + 1].x - base_wallline[i].x,
					base_wallline[i + 1].y - base_wallline[i].y);

				v1.normalize();
				v2.normalize();

				double angle = atan2(v2.x, v2.y) - atan2(v1.x, v1.y);
				angle = angle <= 0 ? angle + 2 * PI : angle;

				Vectorf vmid = v2;
				vmid.rotate(angle / 2);
				vmid.normalize();
				point.x += vmid.x * distance;
				point.y += vmid.y * distance;

				if (point.z >= wall_Z)
					point.z = wall_Z;
			}
			points_vec.push_back(point);
		}

		return points_vec;
	}

	bool
	EntityWall_BaseMesh::GeneralShapePoints()
	{
		if (wallbase_points.size() < 2)
		{
			LOGINFO("wallbase_points.size[%d] error", wallbase_points.size());
			return false;
		}

		this->wallmiddle_points.clear();
		this->wallmiddle_points = wallbase_points;

		for (unsigned int i = 0; i < this->wallmiddle_points.size(); i++)
		{
			this->wallmiddle_points.at(i).z -= this->tooth_depth;
			if (this->wallmiddle_points.at(i).z < 0.0)
				this->wallmiddle_points.at(i).z = 0.0;
		}

		this->wallshape_points.clear();
		this->wallshape_points = wallbase_points;

		// loop首尾相连
		if (this->IsLoop)
			this->wallshape_points.push_back(this->wallshape_points.front());

		// 增加insert深度
		double Length_wall = 0.0;
		for (unsigned int i = 0; i < this->wallshape_points.size(); i++)
		{
			if (i != 0)
			{
				Length_wall += std::sqrt(
					(this->wallshape_points[i].x - this->wallshape_points[i - 1].x) * (this->wallshape_points[i].x - this->wallshape_points[i - 1].x) + (this->wallshape_points[i].y - this->wallshape_points[i - 1].y) * (this->wallshape_points[i].y - this->wallshape_points[i - 1].y));
			}
			this->wallshape_points[i].z += this->insert_depth;
		}
		if (Length_wall < 5.0)
		{
			LOGINFO("Length_wall too small [%f]", Length_wall);
			// return false;
		}
		// 构造锯齿
		return EntityWall_BaseMesh::make_tooth(wallshape_points, tooth_depth, tooth_step, hole_step);
	}

	bool
	EntityWall_BaseMesh::CheckShapePoint()
	{
		if (this->Walls_UpLeftShape.size() == 0 || this->Walls_DwLeftShape.size() == 0 || this->Walls_CtLeftShape.size() == 0 || this->Walls_UpLeftShape.size() != this->Walls_DwLeftShape.size() || this->Walls_CtLeftShape.size() != this->Walls_DwLeftShape.size())
		{
			LOGINFO("Walls_UpLeftShape.size[%d] != Walls_DwLeftShape.size[%d] != this->Walls_CtLeftShape.size() error!! ",
					this->Walls_UpLeftShape.size(),
					this->Walls_DwLeftShape.size(),
					this->Walls_CtLeftShape.size());
			return false;
		}

		if (this->Walls_UpRightShape.size() == 0 || this->Walls_DwRightShape.size() == 0 || this->Walls_CtRightShape.size() == 0 || this->Walls_UpRightShape.size() != this->Walls_DwRightShape.size() || this->Walls_CtRightShape.size() != this->Walls_DwRightShape.size())
		{
			LOGINFO("Walls_UpRightShape.size[%d] != Walls_DwRightShape.size[%d] != this->Walls_CtRightShape.size() error!! ",
					this->Walls_UpRightShape.size(),
					this->Walls_DwRightShape.size(),
					this->Walls_CtRightShape.size());
			return false;
		}

		if (this->Walls_UpRightShape.size() == 0 || this->Walls_DwRightShape.size() == 0 || this->Walls_UpRightShape.size() != this->Walls_DwRightShape.size())
		{
			LOGINFO("Walls_UpRightShape.size[%d] != Walls_DwRightShape.size[%d] error!! ",
					this->Walls_UpRightShape.size(),
					this->Walls_DwRightShape.size());
			return false;
		}

		if (this->Walls_UpRightShape.size() != this->Walls_UpLeftShape.size())
		{
			LOGINFO("Walls_UpRightShape.size[%d] != Walls_UpLeftShape.size[%d] error!! ",
					this->Walls_UpRightShape.size(),
					this->Walls_UpLeftShape.size());
			return false;
		}

		if (this->Walls_CtRightShape.size() != this->Walls_CtLeftShape.size())
		{
			LOGINFO("Walls_CtRightShape.size[%d] != Walls_CtLeftShape.size[%d] error!! ",
					this->Walls_CtRightShape.size(),
					this->Walls_CtLeftShape.size());
			return false;
		}

		if (this->Walls_DwRightShape.size() != this->Walls_DwLeftShape.size())
		{
			LOGINFO("Walls_DwRightShape.size[%d] != Walls_DwLeftShape.size[%d] error!! ",
					this->Walls_DwRightShape.size(),
					this->Walls_DwLeftShape.size());
			return false;
		}

		if (this->Walls_DwRightShape.size() != this->Walls_DwLeftShape.size())
		{
			LOGINFO("Walls_DwRightShape.size[%d] != Walls_DwLeftShape.size[%d] error!! ",
					this->Walls_DwRightShape.size(),
					this->Walls_DwLeftShape.size());
			return false;
		}

		return true;
	}

	// 构建mesh
	TriangleMesh
	EntityWall_BaseMesh::GeneralMesh()
	{
		// 构建mesh
		int Points_Num = this->wallshape_points.size();
		if (Points_Num == 0 || this->CheckShapePoint() == false)
		{
			return TriangleMesh();
		}

		std::vector<Point3> facets; // 三角面片顶点索引
		Pointf3s vertices;			// 顶点数组
		unsigned int id = 0;

		for (unsigned int i = 0; i < Points_Num; i++)
		{
			// 取出两对边
			Pointf3 &upleftshape = Walls_UpLeftShape[i];
			Pointf3 &dwleftshape = Walls_DwLeftShape[i];
			Pointf3 &uprightshape = Walls_UpRightShape[i];
			Pointf3 &dwrightshape = Walls_DwRightShape[i];
			Pointf3 &ctleftshape = Walls_CtLeftShape[i];
			Pointf3 &ctrightshape = Walls_CtRightShape[i];

			vertices.push_back(upleftshape);
			vertices.push_back(uprightshape);
			vertices.push_back(ctleftshape);
			vertices.push_back(ctrightshape);
			vertices.push_back(dwleftshape);
			vertices.push_back(dwrightshape);

			// LOGINFO("Walls_UpLeftShape [%s]", upleftshape.dump_perl().c_str());
			// LOGINFO("Walls_UpRightShape [%s]", uprightshape.dump_perl().c_str());
			// LOGINFO("Walls_DwLeftShape [%s]", dwleftshape.dump_perl().c_str());
			// LOGINFO("Walls_DwRightShape [%s]", dwrightshape.dump_perl().c_str());

			id = vertices.size() - 1; //
			LOGINFO("id[%d]", id);

			LOGINFO("split_step[%d], split_wedth[%d]", split_step, split_wedth);

			unsigned int split_cycle = split_step + split_wedth;
			// 侧面1
			if (i % (split_cycle) == 0)
			{
				facets.push_back(Point3(id - 5, id - 4, id - 3));
				facets.push_back(Point3(id - 3, id - 4, id - 2));

				facets.push_back(Point3(id - 3, id - 2, id - 1));
				facets.push_back(Point3(id - 1, id - 2, id));
				continue;
			}

			if ((i % (split_cycle) > split_step) && (i % (split_cycle) < split_step + split_wedth))
				continue;

			// 正面left
			facets.push_back(Point3(id - 11, id - 9, id - 5));
			facets.push_back(Point3(id - 5, id - 9, id - 3));
			facets.push_back(Point3(id - 9, id - 7, id - 3));
			facets.push_back(Point3(id - 3, id - 7, id - 1));

			// 正面right
			facets.push_back(Point3(id - 10, id - 4, id - 8));
			facets.push_back(Point3(id - 8, id - 4, id - 2));
			facets.push_back(Point3(id - 8, id - 2, id - 6));
			facets.push_back(Point3(id - 6, id - 2, id));

			// 上面
			facets.push_back(Point3(id - 10, id - 11, id - 4));
			facets.push_back(Point3(id - 4, id - 11, id - 5));

			// 下面
			facets.push_back(Point3(id - 7, id - 6, id - 1));
			facets.push_back(Point3(id - 1, id - 6, id));

			// 另一个侧面
			if (i % (split_step + split_wedth) == split_step || i == Points_Num - 1)
			{
				facets.push_back(Point3(id - 4, id - 5, id - 2));
				facets.push_back(Point3(id - 2, id - 5, id - 3));
				facets.push_back(Point3(id - 2, id - 3, id));
				facets.push_back(Point3(id, id - 3, id - 1));
			}
		}

		TriangleMesh wallmesh(vertices, facets);
		wallmesh.checkonly = false;
		wallmesh.merge(this->GeneralMesh_CrossBeam(this->cross_width, 0.0));
		wallmesh.repair();
		if (wallmesh.UnableRepair())
		{
			LOGINFO("GeneralMesh Unable Repair");
			return TriangleMesh();
		}

		return wallmesh;
	}

	// 构建部分Mesh
	TriangleMesh
	EntityWall_BaseMesh::GeneralMesh(unsigned int part_index)
	{
		// 构建mesh
		int Points_Num = this->wallshape_points.size();
		if (Points_Num == 0 ||
			this->CheckShapePoint() == false ||
			// part_index == 0 ||
			part_index > Points_Num)
		{
			LOGINFO("EntityWall_BaseMesh [%d/%d] GeneralMesh failed", part_index, Points_Num);
			return TriangleMesh();
		}

		std::vector<Point3> facets; // 三角面片顶点索引
		Pointf3s vertices;			// 顶点数组

		unsigned int toothhole_step = tooth_step + hole_step;
		// 只取每个齿周期的第一个顶点
		if (part_index % toothhole_step != 0)
		{
			LOGINFO("EntityWall_BaseMesh [%d/%d] GeneralMesh failed", part_index, Points_Num);
			return TriangleMesh();
		}
		int cycle_index = part_index / toothhole_step;
		unsigned int part_index_end;
		if (cycle_index % split_step == 0)
		{
			part_index_end = part_index + tooth_step;
		}
		else
		{
			part_index_end = part_index + toothhole_step;
		}

		TriangleMesh WallCorss;
		for (unsigned int i = part_index; i < part_index_end; i++)
		{
			if (i == 0)
				continue;		 // 第一个顶点
			if (i >= Points_Num) // 超出数组范围
				break;
			// 过滤间隙部位
			// unsigned int split_cycle = split_step + split_wedth;
			// if ((i % (split_cycle) > split_step) && (i % (split_cycle) < split_step + split_wedth))
			//	continue;
			// 取出两对边
			vertices.push_back(Walls_UpLeftShape[i - 1]);
			vertices.push_back(Walls_UpRightShape[i - 1]);
			vertices.push_back(Walls_CtLeftShape[i - 1]);
			vertices.push_back(Walls_CtRightShape[i - 1]);
			vertices.push_back(Walls_DwLeftShape[i - 1]);
			vertices.push_back(Walls_DwRightShape[i - 1]);
			vertices.push_back(Walls_UpLeftShape[i]);
			vertices.push_back(Walls_UpRightShape[i]);
			vertices.push_back(Walls_CtLeftShape[i]);
			vertices.push_back(Walls_CtRightShape[i]);
			vertices.push_back(Walls_DwLeftShape[i]);
			vertices.push_back(Walls_DwRightShape[i]);

			if (Walls_UpLeftShape[i - 1].z - Walls_DwLeftShape[i - 1].z <= insert_depth ||
				Walls_UpRightShape[i - 1].z - Walls_DwRightShape[i - 1].z <= insert_depth)
			{
				LOGINFO("Walls_UpLeftShape[%d] = %s Walls_DwLeftShape[%d] = %s Walls_UpRightShape[%d] = %s Walls_DwRightShape[%d] = %s",
						i - 1,
						Walls_UpLeftShape[i - 1].dump_perl().c_str(),
						i - 1,
						Walls_DwLeftShape[i - 1].dump_perl().c_str(),
						i - 1,
						Walls_UpRightShape[i - 1].dump_perl().c_str(),
						i - 1,
						Walls_DwRightShape[i - 1].dump_perl().c_str());
				return TriangleMesh();
			}
			if (Walls_UpLeftShape[i - 1].z < Walls_CtLeftShape[i - 1].z ||
				Walls_CtLeftShape[i - 1].z <= Walls_DwLeftShape[i - 1].z ||
				Walls_UpLeftShape[i - 1].z <= Walls_DwLeftShape[i - 1].z)
			{
				LOGINFO("Walls_UpLeftShape[%d] = %s Walls_CtLeftShape[%d] = %s Walls_DwLeftShape[%d] = %s",
						i - 1,
						Walls_UpLeftShape[i - 1].dump_perl().c_str(),
						i - 1,
						Walls_CtLeftShape[i - 1].dump_perl().c_str(),
						i - 1,
						Walls_DwLeftShape[i - 1].dump_perl().c_str());
				return TriangleMesh();
			}
			if (Walls_UpRightShape[i - 1].z < Walls_CtRightShape[i - 1].z ||
				Walls_CtRightShape[i - 1].z <= Walls_DwRightShape[i - 1].z ||
				Walls_UpRightShape[i - 1].z <= Walls_DwRightShape[i - 1].z)
			{
				LOGINFO("Walls_UpRightShape[%d] = %s Walls_CtRightShape[%d] = %s Walls_DwRightShape[%d] = %s",
						i - 1,
						Walls_UpRightShape[i - 1].dump_perl().c_str(),
						i - 1,
						Walls_CtRightShape[i - 1].dump_perl().c_str(),
						i - 1,
						Walls_DwRightShape[i - 1].dump_perl().c_str());
				return TriangleMesh();
			}
			if (Walls_DwRightShape[i - 1].z < 0 || Walls_DwLeftShape[i - 1].z < 0)
			{
				LOGINFO("Walls_DwRightShape[%d] = %s Walls_DwLeftShape[%d] = %s",
						i - 1,
						Walls_DwRightShape[i - 1].dump_perl().c_str(),
						i - 1,
						Walls_DwLeftShape[i - 1].dump_perl().c_str());
				return TriangleMesh();
			}

			unsigned int id = vertices.size() - 1;

			// 正面left
			facets.push_back(Point3(id - 11, id - 9, id - 5));
			facets.push_back(Point3(id - 5, id - 9, id - 3));
			facets.push_back(Point3(id - 9, id - 7, id - 3));
			facets.push_back(Point3(id - 3, id - 7, id - 1));
			// 正面right
			facets.push_back(Point3(id - 10, id - 4, id - 8));
			facets.push_back(Point3(id - 8, id - 4, id - 2));
			facets.push_back(Point3(id - 8, id - 2, id - 6));
			facets.push_back(Point3(id - 6, id - 2, id));
			// 上面
			facets.push_back(Point3(id - 10, id - 11, id - 4));
			facets.push_back(Point3(id - 4, id - 11, id - 5));
			// 下面
			facets.push_back(Point3(id - 7, id - 6, id - 1));
			facets.push_back(Point3(id - 1, id - 6, id));
			// 右侧面
			facets.push_back(Point3(id - 4, id - 5, id - 2));
			facets.push_back(Point3(id - 2, id - 5, id - 3));
			facets.push_back(Point3(id - 2, id - 3, id));
			facets.push_back(Point3(id, id - 3, id - 1));
			// 左侧面
			facets.push_back(Point3(id - 10, id - 8, id - 11));
			facets.push_back(Point3(id - 11, id - 8, id - 9));
			facets.push_back(Point3(id - 8, id - 6, id - 9));
			facets.push_back(Point3(id - 9, id - 6, id - 7));
			// 加强筋
			WallCorss.merge(this->GeneralMesh_CrossBeam(i, this->cross_width, 0.0));
		}

		TriangleMesh wallmesh(vertices, facets);
		wallmesh.checkonly = false;
		wallmesh.merge(WallCorss);
		wallmesh.repair();
		if (wallmesh.UnableRepair())
		{
			LOGINFO("GeneralMesh Unable Repair");
			return TriangleMesh();
		}

		return wallmesh;
	}

	// 构建加强筋Mesh
	TriangleMesh
	EntityWall_BaseMesh::GeneralMesh_CrossBeam(double _wedth, double _depth)
	{
		TriangleMesh beamsmesh;
		beamsmesh.checkonly = false;
		// 构建mesh
		for (int i = 0; i < wallshape_points.size(); i++)
		{
			// 除去首尾
			if (i == 0 || i == wallshape_points.size() - 1)
				continue;
			// 除去打断掉的顶点
			unsigned int split_cycle = split_step + split_wedth;
			if ((i % (split_cycle) > split_step) && (i % (split_cycle) < split_step + split_wedth))
				continue;
			// 取hole第一个顶点
			if (i % (tooth_step + hole_step) != tooth_step + 1)
				continue;
			// 计算加强筋的角度
			Vectorf v1(
				wallshape_points[i - 1].x - wallshape_points[i].x,
				wallshape_points[i - 1].y - wallshape_points[i].y);
			Vectorf v2(
				wallshape_points[i + 1].x - wallshape_points[i].x,
				wallshape_points[i + 1].y - wallshape_points[i].y);
			v1.normalize();
			v2.normalize();
			double angle = atan2(v2.x, v2.y) - atan2(v1.x, v1.y);
			angle = angle <= 0 ? angle + 2 * PI : angle;
			Vectorf vmid = v2;
			vmid.rotate(angle / 2);
			vmid.normalize();

			double rotate_deg = acos(fabs(vmid.x));
			if (vmid.x * vmid.y < 0)
				rotate_deg = PI - rotate_deg;

			//
			Pointf3 upPickPos = wallshape_points[i];
			Pointf3 dwPickPos = wallshape_points[i];
			upPickPos.z -= _depth;
			dwPickPos.z = 0.0;

			SingleSPT_BaseMesh *pSPT = new Cuber_Mesh(
				dwPickPos,
				upPickPos,
				rotate_deg,
				thickness,
				_wedth);
			if (pSPT != NULL)
				beamsmesh.merge(pSPT->GeneralMesh());
		}

		beamsmesh.repair();

		if (beamsmesh.UnableRepair())
		{
			LOGINFO("EntityWall_BaseMesh::GeneralMesh Unable Repair");
			return TriangleMesh();
		}
		return beamsmesh;
	}

	// 构建部分加强筋Mesh
	TriangleMesh
	EntityWall_BaseMesh::GeneralMesh_CrossBeam(unsigned int part_id, double _wedth, double _depth)
	{
		TriangleMesh beamsmesh;
		beamsmesh.checkonly = false;
		// 构建mesh

		// 除去首尾
		if (part_id == 0 || part_id == wallshape_points.size() - 1)
			return TriangleMesh();
		// 除去打断掉的顶点
		// unsigned int split_cycle = split_step + split_wedth;
		// if ((part_id % (split_cycle) > split_step) && (part_id % (split_cycle) < split_step + split_wedth))
		//	return TriangleMesh();
		// 取hole第一个顶点
		if (part_id % (tooth_step + hole_step) != tooth_step)
			return TriangleMesh();
		// 计算加强筋的角度
		Vectorf v1(
			wallshape_points[part_id - 1].x - wallshape_points[part_id].x,
			wallshape_points[part_id - 1].y - wallshape_points[part_id].y);
		Vectorf v2(
			wallshape_points[part_id + 1].x - wallshape_points[part_id].x,
			wallshape_points[part_id + 1].y - wallshape_points[part_id].y);
		v1.normalize();
		v2.normalize();
		double angle = atan2(v2.x, v2.y) - atan2(v1.x, v1.y);
		angle = angle <= 0 ? angle + 2 * PI : angle;
		Vectorf vmid = v2;
		vmid.rotate(angle / 2);
		vmid.normalize();

		double rotate_deg = acos(fabs(vmid.x));
		if (vmid.x * vmid.y < 0)
			rotate_deg = PI - rotate_deg;

		//
		Pointf3 upPickPos = wallshape_points[part_id];
		Pointf3 dwPickPos = wallshape_points[part_id];
		upPickPos.z -= _depth;
		dwPickPos.z = 0.0;

		SingleSPT_BaseMesh *pSPT = new Cuber_Mesh(
			dwPickPos,
			upPickPos,
			rotate_deg,
			thickness,
			_wedth);
		if (pSPT != NULL)
			beamsmesh.merge(pSPT->GeneralMesh());
		beamsmesh.repair();
		if (beamsmesh.UnableRepair())
		{
			LOGINFO("EntityWall_BaseMesh::GeneralMesh_CrossBeam Unable Repair");
			return TriangleMesh();
		}
		return beamsmesh;
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// EntityWall_Mesh
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 构建实体墙的上下边,不掏孔
	bool
	EntityWall_Mesh::GeneralPoints()
	{
		// 造型顶点为空
		if (this->wallshape_points.size() == 0)
		{
			LOGINFO("wallshape_points size == 0");
			return false;
		}

		// 计算最高点
		double splitwall_maxz = DBL_MIN;
		for (unsigned int i = 0; i < wallshape_points.size(); i++)
		{
			splitwall_maxz = std::fmax(splitwall_maxz, wallshape_points.at(i).z);
		}

		// 计算最低点
		double splitwall_maxz_middle = DBL_MIN;
		for (unsigned int i = 0; i < wallmiddle_points.size(); i++)
		{
			splitwall_maxz_middle = std::fmax(splitwall_maxz_middle, wallmiddle_points.at(i).z);
		}

		this->Walls_DwLeftShape = EntityWall_BaseMesh::make_wallline(wallshape_points, 0.0, -this->thickness / 2);
		this->Walls_DwRightShape = EntityWall_BaseMesh::make_wallline(wallshape_points, 0.0, this->thickness / 2);
		this->Walls_CtLeftShape = EntityWall_BaseMesh::make_wallline(wallmiddle_points, splitwall_maxz_middle, -this->thickness / 2);
		this->Walls_CtRightShape = EntityWall_BaseMesh::make_wallline(wallmiddle_points, splitwall_maxz_middle, this->thickness / 2);
		this->Walls_UpLeftShape = EntityWall_BaseMesh::make_wallline(wallshape_points, splitwall_maxz, -this->contact_thickness / 2);
		this->Walls_UpRightShape = EntityWall_BaseMesh::make_wallline(wallshape_points, splitwall_maxz, this->contact_thickness / 2);

		// LOGINFO("thickness [%f]", this->thickness);
		// LOGINFO("splitwall_maxz [%f]", splitwall_maxz);
		// LOGINFO("splitwall_maxz [%f]", splitwall_maxz_middle);
		// LOGINFO("tooth_depth [%f]", this->tooth_depth);
		// LOGINFO("thickness [%f]", this->thickness);
		// LOGINFO("contact_thickness [%f]", this->contact_thickness);
		// LOGINFO("Walls_UpLeftShape [%d]", this->Walls_UpLeftShape.size());
		// LOGINFO("Walls_UpRightShape [%d]", this->Walls_UpRightShape.size());
		// LOGINFO("Walls_CtLeftShape [%d]", this->Walls_CtLeftShape.size());
		// LOGINFO("Walls_CtRightShape [%d]", this->Walls_CtRightShape.size());
		// LOGINFO("Walls_DwLeftShape [%d]", this->Walls_DwLeftShape.size());
		// LOGINFO("Walls_DwRightShape [%d]", this->Walls_DwRightShape.size());

		return true;
	}

	// 构建墙体上下半边
	bool
	EntityWall_Mesh::GeneralExPoints()
	{
		// 造型顶点为空
		if (this->wallshape_points.size() == 0)
		{
			LOGINFO("wallshape_points size == 0");
			return false;
		}

		return true;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////
	// 单个数字标记类
	///////////////////////////////////////////////////////////////////////////////////////////////
	double
	NumSingle_Mesh::GetHitOnPlat()
	{
		if (pObj == NULL)
		{
			LOGINFO("pObj == NULL");
			return DBL_MAX;
		}
		double min_z = DBL_MAX;
		double max_z = -DBL_MAX;
		std::list<NumMarkPart>::iterator p_It = Num_Parts.begin();
		while (p_It != Num_Parts.end())
		{
			if (p_It->part_point.z <= 0.0)
				return DBL_MAX;
			if (min_z > p_It->part_point.z)
				min_z = p_It->part_point.z;
			if (max_z < p_It->part_point.z)
				max_z = p_It->part_point.z;
			p_It++;
		}

		return max_z - min_z;
	}

	bool
	NumSingle_Mesh::CalePartsHit()
	{
		if (pObj == NULL)
		{
			LOGINFO("pObj == NULL");
			return false;
		}
		bool retval = true;
		std::list<NumMarkPart>::iterator p_It = Num_Parts.begin();
		while (p_It != Num_Parts.end())
		{
			Pointf basePoint;
			basePoint.x = p_It->part_point.x;
			basePoint.y = p_It->part_point.y;
			bool IsHit = pObj->Cale_CollsionZ_byXY(basePoint, p_It->part_point, p_It->part_normal);
			if (IsHit)
			{
				p_It->part_normal = p_It->part_normal.negative();
				p_It->part_point.z += insert_thickness;
			}
			else
			{
				LOGINFO("Error! [%d] has no collsion point", p_It->part_type);
				p_It->part_point.z = -1.0; // 无效值
				retval = false;
			}
			p_It++;
		}

		return retval;
	}

	TriangleMesh
	NumSingle_Mesh::GeneralMesh()
	{
		TriangleMesh num_mesh;
		if (pObj == NULL)
		{
			LOGINFO("pObj == NULL");
			return num_mesh;
		}
		std::list<NumMarkPart>::iterator p_It = Num_Parts.begin();
		while (p_It != Num_Parts.end())
		{
			if (p_It->part_point.z > 0)
				num_mesh.merge(this->GeneralMesh(*p_It));
			p_It++;
		}
		num_mesh.repair();
		if (num_mesh.UnableRepair())
		{
			LOGINFO("NumSingle_Mesh::GeneralMesh Unable Repair");
			return TriangleMesh();
		}

		return num_mesh;
	}

	// 产生某个部件
	TriangleMesh
	NumSingle_Mesh::GeneralMesh(const NumMarkPart &_PartPoint)
	{
		// 确定部位的角度
		double rotate_deg = 0 * PI / 180;
		if (
			_PartPoint.part_type == NumPart_Mid || _PartPoint.part_type == NumPart_UpMid || _PartPoint.part_type == NumPart_DwMid || _PartPoint.part_type == NumPart_UnderLine)
		{
			rotate_deg = 0 * PI / 180;
		}
		else if (
			_PartPoint.part_type == NumPart_UpR || _PartPoint.part_type == NumPart_DwR || _PartPoint.part_type == NumPart_UpL || _PartPoint.part_type == NumPart_DwL)
		{
			rotate_deg = 90 * PI / 180;
		}
		// 构建脚本点
		Pointf3 dwPickPos;
		dwPickPos.x = _PartPoint.part_point.x;
		dwPickPos.y = _PartPoint.part_point.y;
		if (mark_thickness > 0.0)
		{
			dwPickPos.z = _PartPoint.part_point.z - mark_thickness - insert_thickness;
		}
		else
		{
			dwPickPos.z = 0.0;
		}

		if (dwPickPos.z < 0.0)
			dwPickPos.z = 0.0;
		// dwPickPos.z = 0.0;
		double _Length = (_PartPoint.part_type == NumPart_UnderLine) ? (NumMark_Length + 2 * NumMark_Width + 2 * NumMark_gap) : NumMark_Length;
		LOGINFO("dwPickPos = %s _PartPoint.part_point = %s, _PartPoint.part_normal = %s",
				dwPickPos.dump_perl().c_str(),
				_PartPoint.part_point.dump_perl().c_str(),
				_PartPoint.part_normal.dump_perl().c_str());
		NumMarkBar_Mesh _SPT = NumMarkBar_Mesh(dwPickPos, _PartPoint.part_point, rotate_deg, NumMark_Width, _Length);
		//_SPT.Add_Contact(false, NumMark_Width, _Length / 2, 0.2);// 接触台柱
		_SPT.SetMeshAutoFix(false, _PartPoint.part_normal);
		return _SPT.GeneralMesh();
	}

	bool
	NumSingle_Mesh::GeneralShapePoints()
	{
		Num_Parts.clear();
		if (hasUnderLine)
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_UnderLine, NumMark_Length, NumMark_Width, NumMark_gap));
		switch (NumType)
		{
		case Slic3r::Num_0:
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_UpMid, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_DwMid, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_UpL, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_UpR, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_DwL, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_DwR, NumMark_Length, NumMark_Width, NumMark_gap));
			break;
		case Slic3r::Num_1:
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_UpR, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_DwR, NumMark_Length, NumMark_Width, NumMark_gap));
			break;
		case Slic3r::Num_2:
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_Mid, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_UpMid, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_DwMid, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_UpR, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_DwL, NumMark_Length, NumMark_Width, NumMark_gap));
			break;
		case Slic3r::Num_3:
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_Mid, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_UpMid, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_DwMid, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_UpR, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_DwR, NumMark_Length, NumMark_Width, NumMark_gap));
			break;
		case Slic3r::Num_4:
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_Mid, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_UpR, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_UpL, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_DwR, NumMark_Length, NumMark_Width, NumMark_gap));
			break;
		case Slic3r::Num_5:
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_Mid, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_UpMid, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_DwMid, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_UpL, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_DwR, NumMark_Length, NumMark_Width, NumMark_gap));
			break;
		case Slic3r::Num_6:
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_Mid, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_UpMid, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_DwMid, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_UpL, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_DwR, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_DwL, NumMark_Length, NumMark_Width, NumMark_gap));
			break;
		case Slic3r::Num_7:
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_UpMid, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_UpR, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_DwR, NumMark_Length, NumMark_Width, NumMark_gap));
			break;
		case Slic3r::Num_8:
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_Mid, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_UpMid, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_DwMid, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_UpL, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_UpR, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_DwL, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_DwR, NumMark_Length, NumMark_Width, NumMark_gap));
			break;
		case Slic3r::Num_9:
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_Mid, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_UpMid, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_DwMid, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_UpL, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_UpR, NumMark_Length, NumMark_Width, NumMark_gap));
			Num_Parts.push_back(NumMarkPart(MidPoint, NumPart_DwR, NumMark_Length, NumMark_Width, NumMark_gap));
			break;
		default:
			break;
		}
		// 交点计算
		return this->CalePartsHit();
	}

	bool
	NumSingle_Mesh::GeneralExPoints()
	{
		return true;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////
	// 数字标记类
	///////////////////////////////////////////////////////////////////////////////////////////////
	double
	NumMark_Mesh::GetNumOnPlat()
	{
		if (pObj == NULL)
		{
			LOGINFO("pObj == NULL");
			return DBL_MAX;
		}
		// 从各个部件构建数字
		double retval = 0.0;
		for (unsigned int i = 0; i < SingleNum_vec.size(); i++)
		{
			LOGINFO("SingleNum_vec[i].MidPoint = %s", SingleNum_vec[i].MidPoint.dump_perl().c_str());
			NumSingle_Mesh single_mesh =
				NumSingle_Mesh(
					pObj,
					SingleNum_vec[i].MidPoint,
					SingleNum_vec[i].SingleType,
					mark_thickness,
					NumMark_Length,
					NumMark_Width,
					NumMark_gap,
					NumMark_Size,
					false);
			double NumOnPlat = single_mesh.GetHitOnPlat();
			if (NumOnPlat == DBL_MAX)
			{
				return DBL_MAX;
			}
			retval += NumOnPlat;
		}

		return retval;
	}

	// 拆解数字
	bool
	NumMark_Mesh::GeneralShapePoints()
	{
		size_t _num = Num;
		while (_num > 0)
		{
			SingleNum_vec.push_back(NumSingleInfo(MidPoint, _num % 10));
			_num = _num / 10;
		}

		return true;
	}

	// 计算每个数字的中心点位置
	bool
	NumMark_Mesh::GeneralExPoints()
	{
		double _xl = NumMark_Length + 2 * NumMark_Width + 2 * NumMark_gap;
		double _yl = 2 * NumMark_Length + 3 * NumMark_Width + 4 * NumMark_gap;
		double _xl_gap = 0.3;
		double _yl_gap = 0.3;

		double mark_Length_x = _xl * SingleNum_vec.size() + _xl_gap * (SingleNum_vec.size() - 1);
		LOGINFO("mark_Length_x = [%f], NumMark_Size = [%f]", mark_Length_x, NumMark_Size);

		for (unsigned int i = 0; i < SingleNum_vec.size(); i++)
		{
			double dx = i * (_xl + _xl_gap) - mark_Length_x / 2;
			dx *= NumMark_Size;
			LOGINFO("i = [%d], dx = [%f]", i, dx);

			SingleNum_vec.at(i).MidPoint.x += dx;
		}

		// mark_Length = SingleNum_vec.size() + 0.2*(SingleNum_vec.size() - 1);
		// for (unsigned int i = 0; i < SingleNum_vec.size(); i++)
		//{
		//	SingleNum_vec.at(i).MidPoint.x += (i * 1.4 - (mark_Length - 1) / 2);
		// }

		return true;
	}

	TriangleMesh
	NumMark_Mesh::GeneralMesh()
	{
		TriangleMesh num_mesh;
		if (pObj == NULL)
		{
			LOGINFO("pObj == NULL");
			return num_mesh;
		}
		// 从各个部件构建数字
		for (unsigned int i = 0; i < SingleNum_vec.size(); i++)
		{
			LOGINFO("SingleNum_vec[i].MidPoint = %s", SingleNum_vec[i].MidPoint.dump_perl().c_str());
			NumSingle_Mesh single_mesh = NumSingle_Mesh(
				pObj,
				SingleNum_vec[i].MidPoint,
				SingleNum_vec[i].SingleType,
				mark_thickness,
				NumMark_Length,
				NumMark_Width,
				NumMark_gap,
				NumMark_Size,
				hasUnderLine);
			num_mesh.merge(single_mesh.GeneralMesh());
		}
		num_mesh.repair();
		if (num_mesh.UnableRepair())
		{
			LOGINFO("NumMark_Mesh::GeneralMesh Unable Repair");
			return TriangleMesh();
		}

		return num_mesh;
	}

	// 侧壁位置计算
	TriangleMesh
	Side_NumMark_Mesh::GeneralMesh()
	{
		TriangleMesh retval_mesh;

		if (autoGeneral)
		{
			retval_mesh = GeneralMeshAuto();
		}
		else
		{
			retval_mesh = GeneralMeshManual();
		}

		return retval_mesh;
	}

	// 根据已知中心位置计算数字标记
	TriangleMesh
	Side_NumMark_Mesh::GeneralMeshManual()
	{
		TriangleMesh retval_mesh;
		// 将模型侧过来
		Pointf3 mPoint(this->MidPoint.x, this->MidPoint.y, this->MidPoint.z);

		Vector3 from1(MidNormal.x, MidNormal.y, 0.0);
		from1.normalize();
		Vector3 to1(0.0, 1.0, 0.0);

		Vector3 from2(0.0, 1.0, 0.0);
		Vector3 to2(0.0, 0.0, -1.0);

		Pointf3 p1(0, 0, 0);
		Pointf3 p2(0, 0, 0);
		// this->obj_copy.write_ascii("obj_copy.stl");
		TriangleMesh test_mesh1 = this->obj_copy.rotate(from1, to1, mPoint, p2);
		// test_mesh1.write_ascii("test_mesh1.stl");
		LOGINFO("this->MidNormal = [%s], this->MidPoint = [%s], mPoint = [%s]", this->MidNormal.dump_perl().c_str(), this->MidPoint.dump_perl().c_str(), mPoint.dump_perl().c_str());

		TriangleMesh test_mesh2 = test_mesh1.rotate(from2, to2, mPoint, p2);
		// test_mesh2.write_ascii("test_mesh2.stl");
		LOGINFO("this->MidNormal = [%s], this->MidPoint = [%s], mPoint = [%s]", this->MidNormal.dump_perl().c_str(), this->MidPoint.dump_perl().c_str(), mPoint.dump_perl().c_str());
		mPoint.z = 0.0;

		// test_mesh2.translate(-mPoint.x, -mPoint.y, -mPoint.z);
		// test_mesh2.rotate_z(PI);
		// test_mesh2.translate(mPoint.x, mPoint.y, mPoint.z);

		double minZ = test_mesh2.bounding_box().min.z;
		LOGINFO("this->mark_thickness = [%f], minZ = [%f]", this->mark_thickness, minZ);
		test_mesh2.translate(0, 0, this->mark_thickness - minZ);
		// test_mesh2.write_ascii("test_mesh3.stl");

		NumMark_Mesh *pMark = new NumMark_Mesh(
			&test_mesh2,
			mPoint,
			this->Num,
			this->mark_thickness,
			NumMark_Length,
			NumMark_Width,
			NumMark_gap,
			NumMark_Size,
			false);

		pMark->GetNumOnPlat();
		retval_mesh = pMark->GeneralMesh();
		delete pMark;

		retval_mesh.translate(0, 0, minZ - this->mark_thickness);
		retval_mesh = retval_mesh.rotate(to2, from2, p1, p2);
		// retval_mesh.write_ascii("retval_mesh2.stl");
		retval_mesh = retval_mesh.rotate(to1, from1, p1, p2);
		// retval_mesh.write_ascii("retval_mesh3.stl");

		return retval_mesh;
	}

	// 侧壁位置自动计算数字标记
	TriangleMesh
	Side_NumMark_Mesh::GeneralMeshAuto()
	{
		TriangleMesh retval_mesh;
		TriangleMesh test_mesh = this->obj_copy;
		// 将模型侧过来
		test_mesh.rotate_x(-PI * 90 / 180);
		double minZ = test_mesh.bounding_box().min.z;
		test_mesh.translate(0, 0, this->mark_thickness - minZ);
		// 计算数字标记的中心点
		double min_OnPlat = DBL_MAX;
		int min_rotate_num = 0;
		int rotate_num = 0;
		while (rotate_num < 36)
		{ // 360度搜索
			ExPolygons exs = test_mesh.horizontal_projection_slm(90.0);
			// Pointf midPoint = TriangleMesh::get_ExMidPoint(exs);
			std::vector<Pointf> circle_points = TriangleMesh::get_ExPoints(exs, 0.2);
			LOGINFO("Side_NumMark_Mesh  rotate_num[%d]  circle_points size = %d", rotate_num, circle_points.size());
			for (int i = 0; i < circle_points.size(); i++)
			{
				Pointf midPoint = circle_points[i];
				this->MidPoint.x = midPoint.x;
				this->MidPoint.y = midPoint.y;
				this->MidPoint.z = 0.0;
				NumMark_Mesh *pMark = new NumMark_Mesh(
					&test_mesh,
					this->MidPoint,
					this->Num,
					this->mark_thickness,
					NumMark_Length,
					NumMark_Width,
					NumMark_gap,
					NumMark_Size,
					false);
				double _OnPlat = pMark->GetNumOnPlat();
				if (_OnPlat <= min_OnPlat)
				{
					min_OnPlat = _OnPlat;
					min_rotate_num = rotate_num;
					retval_mesh = pMark->GeneralMesh();
					// if (min_OnPlat < mark_thickness * 5) break; // 结果较为理想 退出搜索
				}
				delete pMark;
			}

			rotate_num++;
			test_mesh.rotate_y(PI * 10 / 180);
		}

		retval_mesh.rotate_y(-PI * min_rotate_num * 10 / 180);
		retval_mesh.translate(0, 0, minZ - this->mark_thickness);
		retval_mesh.rotate_x(PI * 90 / 180);

		return retval_mesh;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////
	// 单个STL数字标记类
	///////////////////////////////////////////////////////////////////////////////////////////////

	// 读取指定文件夹下的stl数字模型
	TriangleMesh
	STL_NumSingle_Mesh::GetSTLNumber()
	{
		TriangleMesh mesh_STL_num;
		std::string input_file = GetRunPath() + "\\STL_NumMark\\";
		LOGINFO("input_file is %s", input_file.c_str());
		std::string stlsuffix = ".stl";
		char stl_Num = NumType + '0';
		std::string num_stl = stl_Num + stlsuffix;
		std::string obj_file = input_file + num_stl;
		LOGINFO("obj_file is %s", obj_file.c_str());
		mesh_STL_num.ReadSTLFile(obj_file);
		mesh_STL_num.scale(STLMark_size);
		mesh_STL_num.check_topology();
		if (mesh_STL_num.facets_count() != 0)
		{
			mesh_STL_num.checkonly = true;
			mesh_STL_num.rotate_y(M_PI);
			LOGINFO("read the STL number success!");
		}
		else
		{
			LOGINFO("This STL file couldn't be read because it's empty.");
		}
		return mesh_STL_num;
	}

	struct pointtest
	{
		double z;
		bool hit = false;
	};
	// 单个数字的标签生成
	TriangleMesh
	STL_NumSingle_Mesh::GeneralMesh()
	{

		TriangleMesh num_mesh;
		if (pObj == NULL)
		{
			LOGINFO("pObj == NULL");
			return num_mesh;
		}
		else
		{
			num_mesh = STL_NumSingle_Mesh::GetSTLNumber(); // 取得stl数字模型
			num_mesh.translate(STL_Midpoint.x, STL_Midpoint.y, STL_Midpoint.z);
			TriangleMesh num_meshreverse = num_mesh;
			int num_hitPoints = 0;
			double averageheight = 0.0;
			const int stl_facets_num = num_mesh.stl.stats.number_of_facets;
			// 现在将判断是否hit分三步，利用map以避免重复的遍历
			// 第一步将点都取出塞进map
			std::map<Pointf, pointtest> calPoints;
			for (int i = 0; i < stl_facets_num; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					Pointf xyPoint;
					xyPoint.x = num_mesh.stl.facet_start[i].vertex[j].x;
					xyPoint.y = num_mesh.stl.facet_start[i].vertex[j].y;
					pointtest takeplace;
					takeplace.hit = false;
					takeplace.z = 0.0;
					calPoints.insert(std::pair<Pointf, pointtest>(xyPoint, takeplace));
				}
			}
			Pointf3 xyPointNormal;
			Pointf3 fromPoint;
			// 第二步，从map里保存z值，并判断是否hit
			for (auto iter = calPoints.begin(); iter != calPoints.end(); ++iter)
			{
				bool IsHit = pObj->Cale_CollsionZ_byXY(iter->first, fromPoint, xyPointNormal);
				iter->second.z = fromPoint.z;
				if (iter->second.z > 0.000001)
				{
					iter->second.hit = true;
				}
				else
				{
					iter->second.hit = false;
				}
			}
			// 第三步遍历每个点，找到对应的hit以及z值。
			for (int i = 0; i < stl_facets_num; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					Pointf basePoint;
					basePoint.x = num_mesh.stl.facet_start[i].vertex[j].x;
					basePoint.y = num_mesh.stl.facet_start[i].vertex[j].y;
					auto get = calPoints.find(basePoint);
					if (get->second.hit == true)
					{
						num_mesh.stl.facet_start[i].vertex[j].z = get->second.z;
						if (get->second.z > NumbercomZMax)
						{
							NumbercomZMax = get->second.z;
						}
						if (get->second.z < NumbercomZMin)
						{
							NumbercomZMin = get->second.z;
						}
						num_hitPoints++;
						averageheight += get->second.z;
					}
				}
				// 这里是采用高低差的方法来进行标签是否平滑的判断
				NumbercomDif = NumbercomZMax - NumbercomZMin;
				// LOGINFO("this [%d] time NumbercomDif is %lf , by [%lf] - [%lf] ", i, NumbercomDif, NumbercomZMax, NumbercomZMin);
				averageheight = (averageheight / num_hitPoints);
				hit_radio = (double(num_hitPoints) / calPoints.size());
				// LOGINFO("the averageheight is %lf , the hitnum is %d ,the stl_points_num is %d ,the hit radio is %lf ", averageheight, num_hitPoints, calPoints.size(), hit_radio);
			}
			LOGINFO("------------------after the hit ------------------------");
			// 处理未击中的点以及将面片整体进行上下调整，上表面上抬以和模型相交，下表面下降进行下一步的缝合
			for (int i = 0; i < stl_facets_num; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					Pointf basePoint;
					basePoint.x = num_mesh.stl.facet_start[i].vertex[j].x;
					basePoint.y = num_mesh.stl.facet_start[i].vertex[j].y;
					auto get = calPoints.find(basePoint);
					if (get->second.hit = false)
					{
						num_mesh.stl.facet_start[i].vertex[j].z = averageheight;
					}
					num_meshreverse.stl.facet_start[i].vertex[j].z = num_mesh.stl.facet_start[i].vertex[j].z - STLMark_thickness;
					num_mesh.stl.facet_start[i].vertex[j].z += insert_thickness;
				}
			}
			// 将下面的num_meshreverse 进行法向翻转。
			stl_reverse_all_facets(&(num_meshreverse.stl)); // 将facet的0，1互换
			// 缝合,插入侧面面片以及底部面片
			int stl_neighbor_num = 0;
			for (int i = 0; i < stl_facets_num; i++)
			{
				stl_facet new_facet;
				for (int j = 0; j < 3; j++)
				{
					if (num_mesh.stl.neighbors_start[i].neighbor[j] == -1) // 判断是否为边缘面片
					{
						stl_neighbor_num++;
						stl_facet new_facet;
						if (j == 0)
						{
							// 不需要设置法向量，在stl_add_facet()里面会重置为0
							new_facet.vertex[0] = num_mesh.stl.facet_start[i].vertex[0];
							new_facet.vertex[1] = num_meshreverse.stl.facet_start[i].vertex[1];
							new_facet.vertex[2] = num_mesh.stl.facet_start[i].vertex[1];
							stl_add_facet(&(num_mesh.stl), &new_facet);
							new_facet.vertex[0] = num_mesh.stl.facet_start[i].vertex[1];
							new_facet.vertex[1] = num_meshreverse.stl.facet_start[i].vertex[1];
							new_facet.vertex[2] = num_meshreverse.stl.facet_start[i].vertex[0];
							stl_add_facet(&(num_mesh.stl), &new_facet);
						}
						else if (j == 1)
						{
							new_facet.vertex[0] = num_mesh.stl.facet_start[i].vertex[1];
							new_facet.vertex[1] = num_meshreverse.stl.facet_start[i].vertex[0];
							new_facet.vertex[2] = num_mesh.stl.facet_start[i].vertex[2];
							stl_add_facet(&(num_mesh.stl), &new_facet);
							new_facet.vertex[0] = num_mesh.stl.facet_start[i].vertex[2];
							new_facet.vertex[1] = num_meshreverse.stl.facet_start[i].vertex[0];
							new_facet.vertex[2] = num_meshreverse.stl.facet_start[i].vertex[2];
							stl_add_facet(&(num_mesh.stl), &new_facet);
						}
						else
						{
							new_facet.vertex[0] = num_mesh.stl.facet_start[i].vertex[2];
							new_facet.vertex[1] = num_meshreverse.stl.facet_start[i].vertex[2];
							new_facet.vertex[2] = num_mesh.stl.facet_start[i].vertex[0];
							stl_add_facet(&(num_mesh.stl), &new_facet);
							new_facet.vertex[0] = num_mesh.stl.facet_start[i].vertex[0];
							new_facet.vertex[1] = num_meshreverse.stl.facet_start[i].vertex[2];
							new_facet.vertex[2] = num_meshreverse.stl.facet_start[i].vertex[1];
							stl_add_facet(&(num_mesh.stl), &new_facet);
						}
					}
				}
				// 顺便将底部的num_meshreverse每个面片也塞进去。
				new_facet.vertex[0] = num_meshreverse.stl.facet_start[i].vertex[0];
				new_facet.vertex[1] = num_meshreverse.stl.facet_start[i].vertex[1];
				new_facet.vertex[2] = num_meshreverse.stl.facet_start[i].vertex[2];
				stl_add_facet(&(num_mesh.stl), &new_facet);
			}
			LOGINFO("------------------- we insert side facets num is [%d]--------------------- ", stl_neighbor_num);
			LOGINFO("After the all the insert ,the stl_num is %d", num_mesh.stl.stats.number_of_facets);
			num_mesh.repair();
			if (num_mesh.UnableRepair())
			{
				LOGINFO("STL_NumMark_Mesh::GeneralMesh Unable Repair");
				return TriangleMesh();
			}
			return num_mesh;
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////
	// STL数字标记类
	///////////////////////////////////////////////////////////////////////////////////////////////
	double
	STL_NumMark_Mesh::GetNumOnPlat()
	{
		if (pObj == NULL)
		{
			LOGINFO("pObj == NULL");
			return DBL_MAX;
		}
		// mesh后，得到每一个数字的高低差，然后进行存储
		double retval = 0.0;
		for (unsigned int i = 0; i < SingleNum_vec.size(); i++)
		{
			STL_NumSingle_Mesh single_mesh =
				STL_NumSingle_Mesh(
					pObj,
					SingleNum_vec[i].MidPoint,
					SingleNum_vec[i].SingleType,
					STLmark_thickness,
					STLNumMark_Size);
			double NumOnPlat = single_mesh.NumbercomDif;
			if (NumOnPlat == DBL_MAX)
			{
				return DBL_MAX;
			}
			retval += NumOnPlat;
		}

		return retval;
	}

	// 拆分数字
	bool
	STL_NumMark_Mesh::GeneralShapePoints()
	{
		SingleNum_vec.clear();
		size_t _num = Num;
		while (_num > 0)
		{
			SingleNum_vec.push_back(STL_NumSingleInfo(STL_MidPoint, _num % 10));
			_num = _num / 10;
		}
		return true;
	}
	bool
	STL_NumMark_Mesh::GeneralExPoints()
	{
		double _xl_gap = 0.2;
		double _xl = STLNumMark_Length;
		double mark_Length_x = _xl * SingleNum_vec.size() + _xl_gap * (SingleNum_vec.size() - 1);

		for (unsigned int i = 0; i < SingleNum_vec.size(); i++)
		{
			double dx = i * (_xl + _xl_gap) - mark_Length_x / 2;
			dx *= STLNumMark_Size;
			SingleNum_vec.at(i).MidPoint.x += dx;
		}
		return true;
	}
	// 将多个数字进行组合
	TriangleMesh
	STL_NumMark_Mesh::GeneralMesh()
	{
		TriangleMesh STL_num_mesh;

		if (pObj == NULL)
		{
			LOGINFO("STL_pObj == NULL");
			return STL_num_mesh;
		}
		// 从各个单字组成数字
		for (unsigned int i = 0; i < SingleNum_vec.size(); i++)
		{
			STL_NumSingle_Mesh single_mesh = STL_NumSingle_Mesh(
				this->pObj,
				this->SingleNum_vec[i].MidPoint,
				this->SingleNum_vec[i].SingleType,
				this->STLmark_thickness,
				this->STLNumMark_Size);
			STL_num_mesh.merge(single_mesh.GeneralMesh());
			this->STLzDifSUM += single_mesh.NumbercomDif;
			this->HitRadioSum += single_mesh.hit_radio;
		}

		STL_num_mesh.repair();
		if (STL_num_mesh.UnableRepair())
		{
			LOGINFO("STL_NumMark_Mesh::GeneralMesh Unable Repair");
			return TriangleMesh();
		}
		return STL_num_mesh;
	}

	bool
	STL_NumMark_Mesh::RSearch_point(size_t rotate_num, TriangleMesh ObjMesh, std::vector<finalSTLcal> *FinalCal, size_t Num, double thickness, double Size)
	{
		DWORD dwStart = GetTickCount();
		if (rotate_num < 0 || rotate_num > 36)
		{
			return false;
		}
		TriangleMesh retest_mesh;
		TriangleMesh test_mesh = ObjMesh; // 这里模型已经侧过来了

		// 计算数字标记的中心点
		test_mesh.rotate_y(rotate_num * PI * 10 / 180);
		double minZ = test_mesh.bounding_box().min.z;
		test_mesh.translate(0, 0, thickness - minZ);
		ExPolygons exs = test_mesh.horizontal_projection_slm(90.0);

		DWORD expointsStart = GetTickCount();
		std::vector<Pointf> circle_points = TriangleMesh::get_ExPoints(exs, 0.2);
		LOGINFO("------------第[%d]圈轮廓点判定计算耗时【%u】", rotate_num, GetTickCount() - expointsStart);

		Pointf midPoint = circle_points.back();
		Pointf3 MidPoint;
		MidPoint.x = midPoint.x;
		MidPoint.y = midPoint.y;
		MidPoint.z = 0.0;
		STL_NumMark_Mesh *pMark = new STL_NumMark_Mesh(
			&test_mesh,
			MidPoint,
			Num,
			thickness,
			Size);

		DWORD generalStart = GetTickCount();
		retest_mesh = pMark->GeneralMesh();
		LOGINFO("------------第[%d]圈general计算耗时【%u】", rotate_num, GetTickCount() - generalStart);

		retest_mesh.translate(0, 0, minZ - thickness);
		(*FinalCal)[rotate_num].F_rotate_Num = rotate_num;
		(*FinalCal)[rotate_num].F_mesh = retest_mesh;
		(*FinalCal)[rotate_num].F_STLzDifSUM = pMark->STLzDifSUM;
		(*FinalCal)[rotate_num].F_HitRadio = (pMark->HitRadioSum) / ((double)(pMark->SingleNum_vec.size()));
		(*FinalCal)[rotate_num].F_MidPoint = pMark->STL_MidPoint;

		LOGINFO("-----------------this [%d] time the F_HitRadio is %lf the FinalCal[i].F_STLzDifSUM is %lf -------------------", (*FinalCal)[rotate_num].F_rotate_Num, (*FinalCal)[rotate_num].F_HitRadio, (*FinalCal)[rotate_num].F_STLzDifSUM);
		delete pMark;
		LOGINFO("this [%d] time the diffZsum is %lf the HitRadio is %lf", rotate_num, (*FinalCal)[rotate_num].F_STLzDifSUM, (*FinalCal)[rotate_num].F_HitRadio);
		LOGINFO("------------第【%d】次旋转总计算耗时【%u】", rotate_num, GetTickCount() - dwStart);
		return true;
	}

	// 生成标签
	TriangleMesh
	Side_STL_NumMark_Mesh::GeneralMesh()
	{
		TriangleMesh retval_mesh;
		if (autoGeneral)
		{
			retval_mesh = STL_GeneralMeshAuto();
		}
		else
		{
			retval_mesh = STL_GeneralMeshManual();
		}

		return retval_mesh;
	}

	// 侧壁位置自动生成标签
	TriangleMesh
	Side_STL_NumMark_Mesh::STL_GeneralMeshAuto()
	{
		DWORD Start = GetTickCount();
		TriangleMesh retval_mesh;
		TriangleMesh retest_mesh;
		TriangleMesh test_mesh = this->obj_copy;
		// 将模型侧过来
		test_mesh.rotate_x(-PI * 90 / 180);
		// 计算数字标记的中心点
		double min_OnPlat = DBL_MAX;
		unsigned int min_rotate_num = 0;
		int rotate_num = 0;
		std::vector<finalSTLcal> FinalCal(36);
		boost::thread_group *_work_p;
		int threads = (2 * boost::thread::hardware_concurrency() / 3);
		LOGINFO("this threads is %d", threads);
		STL_NumMark_Mesh *mesh_p = nullptr;
		parallelize<size_t>(
			0,
			35,
			boost::bind(&STL_NumMark_Mesh::RSearch_point, mesh_p, _1, test_mesh, &FinalCal, this->Num, this->STL_mark_thickness, this->STL_NumMark_Size),
			_work_p,
			threads);

		for (int i = 0; i < 36; i++)
		{
			if ((FinalCal[i].F_STLzDifSUM <= min_OnPlat) && (FinalCal[i].F_STLzDifSUM > 0.0000001) && ((FinalCal[i].F_HitRadio) > 0.95))
			{
				min_OnPlat = FinalCal[i].F_STLzDifSUM;
				// LOGINFO("this [%d] time the min_OnPlat is %lf", FinalCal[i].F_rotate_Num, min_OnPlat);
				min_rotate_num = FinalCal[i].F_rotate_Num;
			}
		}
		retval_mesh = FinalCal[min_rotate_num].F_mesh;
		this->MidPoint = FinalCal[min_rotate_num].F_MidPoint;
		this->MidPoint = this->MidPoint.rotate_y(-PI * min_rotate_num * 10 / 180);
		this->MidPoint = this->MidPoint.rotate_x(PI * 90 / 180);
		LOGINFO("---1111111111111", this->Num, GetTickCount() - Start);
		this->autoGeneral = false;
		// LOGINFO("-------------------------------this min diffZsum is %lf----------------------", min_OnPlat);
		retval_mesh.rotate_y(-PI * min_rotate_num * 10 / 180);
		retval_mesh.rotate_x(PI * 90 / 180);
		LOGINFO("------------自动生成一个[%d]标签计算耗时【%u】", this->Num, GetTickCount() - Start);
		return retval_mesh;
	}

	// 侧壁位置手动生成标签
	TriangleMesh
	Side_STL_NumMark_Mesh::STL_GeneralMeshManual()
	{
		LOGINFO("start to mannul make stlmark");
		DWORD manualStart = GetTickCount();
		TriangleMesh retval_mesh;
		// 将模型侧过来
		Pointf3 mPoint(this->MidPoint.x, this->MidPoint.y, this->MidPoint.z);
		LOGINFO("the mannual MidPoint is  = %s", this->MidPoint.dump_perl().c_str());
		Vector3 from1(MidNormal.x, MidNormal.y, 0.0);
		from1.normalize();
		Vector3 to1(0.0, 1.0, 0.0);
		Vector3 from2(0.0, 1.0, 0.0);
		Vector3 to2(0.0, 0.0, -1.0);
		Pointf3 p1(0, 0, 0);
		Pointf3 p2(0, 0, 0);
		TriangleMesh test_mesh1 = this->obj_copy.rotate(from1, to1, mPoint, p2);
		LOGINFO("this->MidNormal = [%s], this->MidPoint = [%s], mPoint = [%s]", this->MidNormal.dump_perl().c_str(), this->MidPoint.dump_perl().c_str(), mPoint.dump_perl().c_str());
		TriangleMesh test_mesh2 = test_mesh1.rotate(from2, to2, mPoint, p2);
		LOGINFO("this->MidNormal = [%s], this->MidPoint = [%s], mPoint = [%s]", this->MidNormal.dump_perl().c_str(), this->MidPoint.dump_perl().c_str(), mPoint.dump_perl().c_str());
		mPoint.z = 0.0;
		double minZ = test_mesh2.bounding_box().min.z;
		LOGINFO("this->mark_thickness = [%f], minZ = [%f]", this->STL_mark_thickness, minZ);
		test_mesh2.translate(0, 0, this->STL_mark_thickness - minZ);
		STL_NumMark_Mesh *pMark = new STL_NumMark_Mesh(
			&test_mesh2,
			mPoint,
			this->Num,
			this->STL_mark_thickness,
			this->STL_NumMark_Size);
		retval_mesh = pMark->GeneralMesh();
		delete pMark;
		retval_mesh.translate(0, 0, minZ - this->STL_mark_thickness);
		retval_mesh = retval_mesh.rotate(to2, from2, p1, p2);
		retval_mesh = retval_mesh.rotate(to1, from1, p1, p2);
		LOGINFO("------------手动生成一个[%d]标签计算耗时【%u】", this->Num, GetTickCount() - manualStart);
		return retval_mesh;
	}

	/////////////////////////////////////////////
	// 字符标签
	/////////////////////////////////////////////

	NumSingleType Side_STL_CharMark_Mesh::getTypeFromChar(char s)
	{
		NumSingleType _type = NumSingleType::Num_0;
		if (s <= '9' && s >= '0')
		{
			size_t _num = (size_t)s - (size_t)('0');
			_type = (NumSingleType)_num;
		}
		else if (s <= 'z' && s >= 'a')
		{
			size_t _num = (size_t)s - (size_t)('a') + 36;
			_type = (NumSingleType)_num;
		}
		else if (s <= 'Z' && s >= 'A')
		{
			size_t _num = (size_t)s - (size_t)('A') + 10;
			_type = (NumSingleType)_num;
		}
		else if (s == '-')
		{
			_type = Sym_Min;
		}
		else if (s == '+')
		{
			_type = Sym_Plus;
		}
		else if (s == '#')
		{
			_type = Sym_Sharp;
		}
		else if (s == '*')
		{
			_type = Sym_Star;
		}
		else if (s == '!')
		{
			_type = Sym_Exc;
		}
		else if (s == '?')
		{
			_type = Sym_Que;
		}
		else if (s == '$')
		{
			_type = Sym_Dol;
		}
		else if (s == '%')
		{
			_type = Sym_Per;
		}
		else if (s == '&')
		{
			_type = Sym_And;
		}
		else if (s == '@')
		{
			_type = Sym_At;
		}
		else if (s == '^')
		{
			_type = Sym_Pow;
		}
		else if (s == ':')
		{
			_type = Sym_Colon;
		}
		else
		{
			LOGINFO("_num error");
		}
		return _type;
	}

	// 初始化静态变量

	bool Side_STL_CharMark_Mesh::Init_Charmark_Mesh()
	{
		std::string MeshDataPath = GetRunPath() + "\\CharMark\\";
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
			if (findData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) // 是否是目录
			{
				LOGINFO("Is dir--%s", findData.cFileName);
			}
			else
			{
				string _filepath = MeshDataPath + findData.cFileName;
				Charmark *sc_p = new Charmark(_filepath);
				if (sc_p->cy_mesh.facets_count() > 0) // 读取成功
				{
					LOGINFO("sc_p->FilePath == %s 读取成功! x[%f], y[%f], z[%f],face count[%d]",
							sc_p->FilePath.c_str(), sc_p->x, sc_p->y, sc_p->z, sc_p->cy_mesh.facets_count());
					std::string file = findData.cFileName;
					if (file.empty() == false)
					{
						char s = file[0];

						if (s == 's' && file.find("star") != string::npos)
						{
							s = '*';
						}
						if (s == 'q' && file.find("que") != string::npos)
						{
							s = '?';
						}
						if (s == 'c' && file.find("colon") != string::npos)
						{
							s = ':';
						}
						if ((s <= '9' && s >= '0') ||
							(s <= 'z' && s >= 'a') ||
							(s <= 'Z' && s >= 'A') ||
							(s == '-') || (s == '#') ||
							(s == '*') || (s == '?') ||
							(s == '!') || (s == '%') ||
							(s == '$') || (s == '^') ||
							(s == '&') || (s == '+') ||
							(s == '@') || (s == ':'))
						{
							NumSingleType _type = getTypeFromChar(s);
							Slic3r::Char_Meshs[_type] = sc_p;

							LOGINFO("Add a NumSingleType to Char_Meshs successfully! char = [%c]", s);
						}
					}
				}
			}
		} while (FindNextFileA(hFind, &findData));
		LOGINFO("Slic3r::Char_Meshs.size() = %d", Slic3r::Char_Meshs.size());
		return Slic3r::Char_Meshs.size() > 0;
	}

	Charmark::Charmark(std::string _filepath)
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
		if (cy_mesh.facets_count() != 0)
		{
			cy_mesh.checkonly = true;
			cy_mesh.rotate_y(M_PI);
			LOGINFO("read the STL char success!");
		}
		else
		{
			LOGINFO("This STL file couldn't be read because it's empty.");
			return;
		}
		// if (cy_mesh.facets_count() == 0)
		// {
		// 	LOGINFO("[%s] 文件读stl失败", _filepath.c_str());
		// 	return;
		// }
		// cy_mesh.repair();
		// if (cy_mesh.repaired == false)
		// {
		// 	LOGINFO("[%s] 文件stl修复失败", _filepath.c_str());
		// 	return;
		// }

		// cy_mesh.mirror_y();
		// cy_mesh.mirror_x();

		FilePath = _filepath;
		BoundingBoxf3 _bb3 = cy_mesh.bounding_box();
		x = _bb3.size().x;
		y = _bb3.size().y;
		z = _bb3.size().z;
	}

	void
	Charmark::Print_Info()
	{
		LOGINFO("Charmark FilePath [%s], Mesh Faces[%d], x[%f], y[%f], z[%f]",
				FilePath.c_str(), cy_mesh.facets_count(), x, y, z);
	}
	// 读取指定文件夹下的stl字符模型
	TriangleMesh
	STL_Char_Single_Mesh::GetSTLChar()
	{
		if (pObj == NULL)
		{
			LOGINFO("pObj == NULL");
			return TriangleMesh();
		}
		LOGINFO("Slic3r::Char_Meshs.size[%d]", Slic3r::Char_Meshs.size());
		LOGINFO("NumType[%d]", NumType);
		TriangleMesh mesh(Char_Meshs[NumType]->cy_mesh);
		mesh.scale(STLCharMark_size);
		return mesh;
	}
	void my_stl_remove_facet(stl_file *stl, int facet_number)
	{
		int neighbor[3];
		int vnot[3];
		int i;
		int j;

		if (stl->error)
			return;

		stl->stats.facets_removed += 1;
		/* Update list of connected edges */
		j = ((stl->neighbors_start[facet_number].neighbor[0] == -1) +
			 (stl->neighbors_start[facet_number].neighbor[1] == -1) +
			 (stl->neighbors_start[facet_number].neighbor[2] == -1));
		if (j == 2)
		{
			stl->stats.connected_facets_1_edge -= 1;
		}
		else if (j == 1)
		{
			stl->stats.connected_facets_2_edge -= 1;
			stl->stats.connected_facets_1_edge -= 1;
		}
		else if (j == 0)
		{
			stl->stats.connected_facets_3_edge -= 1;
			stl->stats.connected_facets_2_edge -= 1;
			stl->stats.connected_facets_1_edge -= 1;
		}

		stl->facet_start[facet_number] =
			stl->facet_start[stl->stats.number_of_facets - 1];
		/* I could reallocate at this point, but it is not really necessary. */
		stl->neighbors_start[facet_number] =
			stl->neighbors_start[stl->stats.number_of_facets - 1];
		stl->stats.number_of_facets -= 1;

		for (i = 0; i < 3; i++)
		{
			neighbor[i] = stl->neighbors_start[facet_number].neighbor[i];
			vnot[i] = stl->neighbors_start[facet_number].which_vertex_not[i];
		}
		int neighbor0 = stl->neighbors_start[facet_number].neighbor[0];
		int neighbor1 = stl->neighbors_start[facet_number].neighbor[1];
		int neighbor2 = stl->neighbors_start[facet_number].neighbor[2];
		for (int i = 0; i < 3; i++)
		{
			if (stl->neighbors_start[neighbor0].neighbor[i] == facet_number)
			{
				stl->neighbors_start[neighbor0].neighbor[i] = -1;
			}
			if (stl->neighbors_start[neighbor1].neighbor[i] == facet_number)
			{
				stl->neighbors_start[neighbor1].neighbor[i] = -1;
			}
			if (stl->neighbors_start[neighbor2].neighbor[i] == facet_number)
			{
				stl->neighbors_start[neighbor2].neighbor[i] = -1;
			}
			// 20230609 by zhangzheng stl_remove_facet的算法是将要删除的面皮移到stl的最后，并没有释放，原本的拓扑关系并没有变，
			// 理论上我们删除面片后就是应该要更新被删除面片的邻居面片的邻居面片(它本身为-1)
		}
		for (i = 0; i < 3; i++)
		{
			if (neighbor[i] != -1)
			{
				if (stl->neighbors_start[neighbor[i]].neighbor[(vnot[i] + 1) % 3] !=
					stl->stats.number_of_facets)
				{
					// stl->stats.unable_repair = true;
					LOGINFO("in stl_remove_facet: neighbor = %d numfacets = %d this  is wrong \n",
							stl->neighbors_start[neighbor[i]].neighbor[(vnot[i] + 1) % 3],
							stl->stats.number_of_facets);
					return;
				}
				stl->neighbors_start[neighbor[i]].neighbor[(vnot[i] + 1) % 3] = facet_number;
			}
		}
	}
	// 单个字符的标签生成
	TriangleMesh
	STL_Char_Single_Mesh::GeneralMesh()
	{

		TriangleMesh num_mesh;
		if (pObj == NULL)
		{
			LOGINFO("pObj == NULL");
			return num_mesh;
		}
		else
		{
			LOGINFO("start get charsinglestl");
			num_mesh = STL_Char_Single_Mesh::GetSTLChar(); // 取得stl字符模型
			LOGINFO("end get charsinglestl");
			num_mesh.translate(STL_Midpoint.x, STL_Midpoint.y, STL_Midpoint.z);
			int num_hitPoints = 0;
			const int stl_facets_num = num_mesh.stl.stats.number_of_facets;
			LOGINFO("this num_mesh facet num  is %d", stl_facets_num);
			//  std::vector<std::vector <bool> > testhit(stl_facets_num, std::vector<bool>(3, false));
			// 现在将判断是否hit分三步，利用map以避免重复的遍历
			// 第一步将点都取出塞进map
			std::map<Pointf, pointtest> calPoints;
			for (int i = 0; i < stl_facets_num; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					Pointf xyPoint;
					xyPoint.x = num_mesh.stl.facet_start[i].vertex[j].x;
					xyPoint.y = num_mesh.stl.facet_start[i].vertex[j].y;
					pointtest takeplace;
					takeplace.hit = false;
					takeplace.z = 0.0;
					calPoints.insert(std::pair<Pointf, pointtest>(xyPoint, takeplace));
				}
			}
			LOGINFO("this calPoints size  is %d", calPoints.size());
			Pointf3 xyPointNormal;
			Pointf3 fromPoint;
			// 第二步，从map里保存z值，并判断是否hit
			int i = 0;
			for (auto iter = calPoints.begin(); iter != calPoints.end(); ++iter)
			{

				bool IsHit = pObj->Cale_CollsionZ_byXY(iter->first, fromPoint, xyPointNormal);
				if (IsHit)
				{
					iter->second.z = fromPoint.z;
					iter->second.hit = true;
				}
				else
				{
					iter->second.z = -100000;
					iter->second.hit = false;
				}
				// LOGINFO("this  %d iter->second.z is %f", i, iter->second.z);
				i++;
			}
			// 第三步遍历每个点，找到对应的hit以及z值。
			std::set<size_t> record_hit_facets;
			for (int i = 0; i < stl_facets_num; i++)
			{
				double facet_max_z = -DBL_MAX;
				double facet_min_z = DBL_MAX;
				std::vector<double> facet_z;
				facet_z.clear();
				for (int j = 0; j < 3; j++)
				{
					Pointf basePoint;
					basePoint.x = num_mesh.stl.facet_start[i].vertex[j].x;
					basePoint.y = num_mesh.stl.facet_start[i].vertex[j].y;
					num_mesh.stl.facet_start[i].vertex[j].z = -100;
					auto get = calPoints.find(basePoint);
					facet_z.push_back(get->second.z);
					if (get->second.hit == true)
					{
						num_mesh.stl.facet_start[i].vertex[j].z = get->second.z;
						if (get->second.z > NumbercomZMax)
						{
							NumbercomZMax = get->second.z;
						}
						if (get->second.z < NumbercomZMin)
						{
							NumbercomZMin = get->second.z;
						}
						num_hitPoints++;
					}
					else
					{
						// num_mesh.stl.facet_start[i].vertex[j].z = -100000;
						record_hit_facets.insert(i);
					}
					// LOGINFO("0608  [%d][%d] facet x = %lf,y = %lf,z = %lf\n", i, j, num_mesh.stl.facet_start[i].vertex[j].x, num_mesh.stl.facet_start[i].vertex[j].y, num_mesh.stl.facet_start[i].vertex[j].z);
				}
				std::sort(facet_z.begin(), facet_z.end());
			}
			// 删除未击中的面片
			for (auto iter = record_hit_facets.rbegin(); iter != record_hit_facets.rend(); ++iter)
			{
				my_stl_remove_facet((&num_mesh.stl), *iter);
			}
			int after_stl_facets_num = num_mesh.stl.stats.number_of_facets;

			NumbercomDif = NumbercomZMax - NumbercomZMin;
			LOGINFO("this ComDif is %lf , by [%lf] - [%lf] ", NumbercomDif, NumbercomZMax, NumbercomZMin);

			hit_radio = (double(after_stl_facets_num) / stl_facets_num);
			LOGINFO("the hitnum is %d ,the stl_points_num is %d ,the hit radio is %lf ", num_hitPoints, calPoints.size(), hit_radio);

			TriangleMesh num_meshreverse = num_mesh;
			LOGINFO("------------------after the hit ------------------------");
			// 处理未击中的点以及将面片整体进行上下调整，上表面上抬以和模型相交，下表面下降进行下一步的缝合
			for (int i = 0; i < after_stl_facets_num; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					Pointf basePoint;
					basePoint.x = num_mesh.stl.facet_start[i].vertex[j].x;
					basePoint.y = num_mesh.stl.facet_start[i].vertex[j].y;
					auto get = calPoints.find(basePoint);
					num_meshreverse.stl.facet_start[i].vertex[j].z = num_mesh.stl.facet_start[i].vertex[j].z - STLCharMark_thickness;
					num_mesh.stl.facet_start[i].vertex[j].z += insert_thickness;
				}
			}
			// 将下面的num_meshreverse 进行法向翻转。
			stl_reverse_all_facets(&(num_meshreverse.stl)); // 将facet的0，1互换
			// 缝合,插入侧面面片以及底部面片
			int stl_neighbor_num = 0;
			for (int i = 0; i < after_stl_facets_num; i++)
			{
				stl_facet new_facet;
				for (int j = 0; j < 3; j++)
				{
					if (num_mesh.stl.neighbors_start[i].neighbor[j] == -1) // 判断是否为边缘面片
					{
						stl_neighbor_num++;
						stl_facet new_facet;
						if (j == 0)
						{
							// 不需要设置法向量，在stl_add_facet()里面会重置为0
							new_facet.vertex[0] = num_mesh.stl.facet_start[i].vertex[0];
							new_facet.vertex[1] = num_meshreverse.stl.facet_start[i].vertex[1];
							new_facet.vertex[2] = num_mesh.stl.facet_start[i].vertex[1];
							stl_add_facet(&(num_mesh.stl), &new_facet);
							new_facet.vertex[0] = num_mesh.stl.facet_start[i].vertex[1];
							new_facet.vertex[1] = num_meshreverse.stl.facet_start[i].vertex[1];
							new_facet.vertex[2] = num_meshreverse.stl.facet_start[i].vertex[0];
							stl_add_facet(&(num_mesh.stl), &new_facet);
						}
						else if (j == 1)
						{
							new_facet.vertex[0] = num_mesh.stl.facet_start[i].vertex[1];
							new_facet.vertex[1] = num_meshreverse.stl.facet_start[i].vertex[0];
							new_facet.vertex[2] = num_mesh.stl.facet_start[i].vertex[2];
							stl_add_facet(&(num_mesh.stl), &new_facet);
							new_facet.vertex[0] = num_mesh.stl.facet_start[i].vertex[2];
							new_facet.vertex[1] = num_meshreverse.stl.facet_start[i].vertex[0];
							new_facet.vertex[2] = num_meshreverse.stl.facet_start[i].vertex[2];
							stl_add_facet(&(num_mesh.stl), &new_facet);
						}
						else
						{
							new_facet.vertex[0] = num_mesh.stl.facet_start[i].vertex[2];
							new_facet.vertex[1] = num_meshreverse.stl.facet_start[i].vertex[2];
							new_facet.vertex[2] = num_mesh.stl.facet_start[i].vertex[0];
							stl_add_facet(&(num_mesh.stl), &new_facet);
							new_facet.vertex[0] = num_mesh.stl.facet_start[i].vertex[0];
							new_facet.vertex[1] = num_meshreverse.stl.facet_start[i].vertex[2];
							new_facet.vertex[2] = num_meshreverse.stl.facet_start[i].vertex[1];
							stl_add_facet(&(num_mesh.stl), &new_facet);
						}
					}
				}
				// 顺便将底部的num_meshreverse每个面片也塞进去。
				new_facet.vertex[0] = num_meshreverse.stl.facet_start[i].vertex[0];
				new_facet.vertex[1] = num_meshreverse.stl.facet_start[i].vertex[1];
				new_facet.vertex[2] = num_meshreverse.stl.facet_start[i].vertex[2];
				stl_add_facet(&(num_mesh.stl), &new_facet);
			}

			LOGINFO("------------------- origin facets num is [%d]--------------------- ", stl_facets_num);
			LOGINFO("------------------- after delete facets num is [%d]--------------------- ", after_stl_facets_num);
			LOGINFO("------------------- we insert side facets num is [%d]--------------------- ", stl_neighbor_num * 2);
			LOGINFO("After the all the insert ,the stl_num is %d", num_mesh.stl.stats.number_of_facets);
			num_mesh.stl.stats.unable_repair = false;
			num_mesh.checkonly = true;
			if (after_stl_facets_num != stl_facets_num || NumbercomDif > 4)
			{
				this->singeCharvaild = false;
			}
			if (this->singeCharvaild == false)
			{
				LOGINFO("-----0615 this->singeCharvaild = false");
			}
			else
			{
				LOGINFO("-----0615 this->singeCharvaild = true");
			}
			return num_mesh;
		}
	}
	///////////////////////////////////////////////////////////////////////////////////////////////
	// STL字符标记类
	///////////////////////////////////////////////////////////////////////////////////////////////
	double
	STL_CharMark_Mesh::GetNumOnPlat()
	{
		if (pObj == NULL)
		{
			LOGINFO("pObj == NULL");
			return DBL_MAX;
		}
		// mesh后，得到每一个字符的高低差，然后进行存储
		double retval = 0.0;
		for (unsigned int i = 0; i < SingleChar_vec.size(); i++)
		{
			for (unsigned int j = 0; j < SingleChar_vec.at(i).size(); j++)
			{
				LOGINFO("SingleChar_vec.at(i).at(j).MidPoint = %s", SingleChar_vec.at(i).at(j).MidPoint.dump_perl().c_str());
				STL_Char_Single_Mesh single_mesh =
					STL_Char_Single_Mesh(
						pObj,
						SingleChar_vec.at(i).at(j).MidPoint,
						SingleChar_vec.at(i).at(j).SingleType,
						STLCharmark_thickness,
						STLInsert_thickness,
						STLCharMark_Size,
						STLCharMark_gap);
				double NumOnPlat = single_mesh.NumbercomDif;
				if (NumOnPlat == DBL_MAX)
				{
					return DBL_MAX;
				}
				retval += NumOnPlat;
			}
		}

		return retval;
	}

	// 拆分字符
	bool
	STL_CharMark_Mesh::GeneralShapePoints()
	{
		if (STL_Str.empty())
			return true;

		LOGINFO("GeneralShapePoints Str = [%s]", STL_Str.c_str());
		const char *ss = STL_Str.c_str();

		std::vector<NumSingleInfos> temp_vecs;
		NumSingleInfos temp_vec;
		LOGINFO("start");

		while (*ss != '\0')
		{
			if (*ss == '\n')
			{
				LOGINFO("find char = [%c]\n", *ss);
				if (temp_vec.empty() == false)
				{
					temp_vecs.push_back(temp_vec);
					temp_vec.clear();
				}
			}
			else
			{
				temp_vec.push_back(NumSingleInfo(STL_MidPoint, *ss));
			}
			ss++;
		}

		if (temp_vec.empty() == false)
		{
			temp_vecs.push_back(temp_vec);
		}

		LOGINFO("end");

		// 倒序
		SingleChar_vec.clear();
		SingleChar_vec.reserve(temp_vecs.size());

		std::vector<NumSingleInfos>::reverse_iterator riter;
		for (riter = temp_vecs.rbegin(); riter != temp_vecs.rend(); riter++)
		{
			NumSingleInfos vec;
			NumSingleInfos::reverse_iterator riter_v;
			for (riter_v = (*riter).rbegin(); riter_v != (*riter).rend(); riter_v++)
			{
				vec.push_back(*riter_v);
				// LOGINFO("*riter_v = [%s]", (*riter_v).SingleType);
			}
			LOGINFO("vec.size = [%d]", vec.size());

			SingleChar_vec.push_back(vec);
		}
		LOGINFO("SingleChar_vec.size = [%d]", SingleChar_vec.size());
		return true;
	}
	bool
	STL_CharMark_Mesh::GeneralExPoints()
	{
		double _xl = STLCharMark_Width;	 // *CharMark_Width;
		double _yl = STLCharMark_Length; // *CharMark_Length;
		double _xl_gap = STLCharMark_gap;
		double _yl_gap = STLCharMark_gap;

		double mark_Length_y = _yl * (SingleChar_vec.size() - 1) + _yl_gap * (SingleChar_vec.size() - 1);
		LOGINFO("mark_Length_y = [%f], CharMark_size = [%f]", mark_Length_y, STLCharMark_Size);

		for (unsigned int i = 0; i < SingleChar_vec.size(); i++)
		{
			double dy = i * (_yl + _yl_gap) - mark_Length_y / 2;
			dy *= STLCharMark_Size;
			double mark_Length_x = _xl * SingleChar_vec.at(i).size() + _xl_gap * (SingleChar_vec.at(i).size() - 1);
			LOGINFO("mark_Length_x = [%f], dy = [%f]", mark_Length_x, dy);

			for (unsigned int j = 0; j < SingleChar_vec.at(i).size(); j++)
			{
				double dx = j * (_xl + _xl_gap) - mark_Length_x / 2;
				dx *= STLCharMark_Size;
				LOGINFO("i = [%d], dx = [%f]", j, dx);
				SingleChar_vec.at(i).at(j).MidPoint.x += dx;
				SingleChar_vec.at(i).at(j).MidPoint.y += dy;
			}
		}
		return true;
	}
	// 将多个数字进行组合
	TriangleMesh
	STL_CharMark_Mesh::GeneralMesh()
	{

		TriangleMesh STL_num_mesh;
		if (pObj == NULL)
		{
			LOGINFO("STL_pObj == NULL");
			return STL_num_mesh;
		}
		// 从各个单字组成
		for (unsigned int i = 0; i < SingleChar_vec.size(); i++)
		{
			for (unsigned int j = 0; j < SingleChar_vec.at(i).size(); j++)
			{
				LOGINFO("SingleChar_vec.at(%d).at(%d).MidPoint = %s", i, j, SingleChar_vec.at(i).at(j).MidPoint.dump_perl().c_str());
				LOGINFO("this->STLCharmark_thickness is [%f],", this->STLCharmark_thickness);
				LOGINFO("this->STLInsert_thickness is [%f],", this->STLInsert_thickness);
				LOGINFO("this->STLCharMark_Size is [%f],", this->STLCharMark_Size);
				STL_Char_Single_Mesh single_mesh = STL_Char_Single_Mesh(
					this->pObj,
					SingleChar_vec.at(i).at(j).MidPoint,
					SingleChar_vec.at(i).at(j).SingleType,
					this->STLCharmark_thickness,
					this->STLInsert_thickness,
					this->STLCharMark_Size,
					this->STLCharMark_gap,
					this->autoGeneral);
				STL_num_mesh.merge(single_mesh.GeneralMesh());
				if (single_mesh.singeCharvaild == false)
				{
					this->charMeshIsValid = false;
				}
				this->STLzDifSUM += single_mesh.NumbercomDif;
				this->HitRadioSum += single_mesh.hit_radio;
			}
		}
		LOGINFO("0609 the charmark STLzDifSUM is %lf", this->STLzDifSUM);
		if (this->charMeshIsValid == false)
		{
			LOGINFO("-----0615 this->charMeshIsValid = false");
		}
		else
		{
			LOGINFO("-----0615 this->charMeshIsValid = true");
		}
		// STL_num_mesh.write_ascii("STL_num_mesh.stl");
		STL_num_mesh.checkonly = true;
		// STL_num_mesh.write_ascii("STL_num_mesh_afterrepair.stl");
		if (STL_num_mesh.UnableRepair())
		{
			LOGINFO("STL_CharMark_Mesh::GeneralMesh Unable Repair");
			return TriangleMesh();
		}
		return STL_num_mesh;
	}

	bool
	STL_CharMark_Mesh::RSearch_point(size_t rotate_num, TriangleMesh ObjMesh, std::vector<finalSTLcal> *FinalCal, std::string STL_Str, double thickness, double insert_thickness, double charSize, double STLCharMark_gap)
	{
		LOGINFO("-----0323 RSearch_point [%d] -- [%s]", rotate_num, STL_Str.c_str());
		DWORD dwStart = GetTickCount();
		if (rotate_num < 0 || rotate_num > 36)
		{
			return false;
		}
		TriangleMesh retest_mesh;
		TriangleMesh test_mesh = ObjMesh; // 这里模型已经侧过来了
		// 计算数字标记的中心点
		test_mesh.rotate_y(rotate_num * PI * 10 / 180);

		double minZ = test_mesh.bounding_box().min.z;
		test_mesh.translate(0, 0, thickness - minZ);

		ExPolygons exs = test_mesh.horizontal_projection_slm(90.0);
		DWORD expointsStart = GetTickCount();
		std::vector<Pointf> circle_points = TriangleMesh::get_ExPoints(exs, 0.2);
		LOGINFO("------------第[%d]圈轮廓点判定计算耗时【%u】", rotate_num, GetTickCount() - expointsStart);

		Pointf midPoint = circle_points.back();
		LOGINFO("-----circle_points.back() is %s", midPoint.dump_perl().c_str());
		Pointf3 MidPoint;
		MidPoint.x = midPoint.x;
		MidPoint.y = midPoint.y;
		MidPoint.z = 0.0;

		STL_CharMark_Mesh *pMark = new STL_CharMark_Mesh(
			&test_mesh,
			MidPoint,
			STL_Str,
			thickness,
			insert_thickness,
			3.5,
			1.6,
			charSize,
			STLCharMark_gap,
			true);
		DWORD generalStart = GetTickCount();
		retest_mesh = pMark->GeneralMesh();
		LOGINFO("------------第[%d]圈general计算耗时【%u】", rotate_num, GetTickCount() - generalStart);
		retest_mesh.translate(0, 0, minZ - thickness);
		(*FinalCal)[rotate_num].F_rotate_Num = rotate_num;
		(*FinalCal)[rotate_num].F_mesh = retest_mesh;
		(*FinalCal)[rotate_num].F_STLzDifSUM = pMark->STLzDifSUM;
		(*FinalCal)[rotate_num].F_HitRadio = (pMark->HitRadioSum) / ((double)(pMark->SingleChar_vec.size()));
		(*FinalCal)[rotate_num].F_meshValid = pMark->charMeshIsValid;
		
		(*FinalCal)[rotate_num].F_MidPoint = pMark->STL_MidPoint;
		(*FinalCal)[rotate_num].F_MidPoint.translate(0, 0, minZ - thickness);
		(*FinalCal)[rotate_num].F_MidPoint = (*FinalCal)[rotate_num].F_MidPoint.rotate_y(-PI * rotate_num * 10 / 180);
		(*FinalCal)[rotate_num].F_MidPoint = (*FinalCal)[rotate_num].F_MidPoint.rotate_x(PI * 90 / 180);
		if (!(*FinalCal)[rotate_num].F_meshValid)
		{
			LOGINFO("this [%d] time the Rcharmark is not valid", rotate_num);
		}
		LOGINFO("-----------------this [%d] time the F_HitRadio is %lf F_STLzDifSUM is %lf -------------------", (*FinalCal)[rotate_num].F_rotate_Num, (*FinalCal)[rotate_num].F_HitRadio, (*FinalCal)[rotate_num].F_STLzDifSUM);
		delete pMark;
		LOGINFO("this [%d] time the diffZsum is %lf the HitRadio is %lf", rotate_num, (*FinalCal)[rotate_num].F_STLzDifSUM, (*FinalCal)[rotate_num].F_HitRadio);
		LOGINFO("------------第【%d】次旋转总计算耗时【%u】", rotate_num, GetTickCount() - dwStart);
		return true;
	}

	// 生成标签
	TriangleMesh
	Side_STL_CharMark_Mesh::GeneralMesh()
	{
		TriangleMesh retval_mesh;
		if (this->autoGeneral)
		{
			LOGINFO("-----0214 start auto general mesh STLChar");
			retval_mesh = STL_GeneralMeshAuto();
		}
		else
		{
			LOGINFO("-----0214 start Manual general mesh STLChar");
			retval_mesh = STL_GeneralMeshManual();
		}
		LOGINFO("-----0609  Side_STL_CharMark_Mesh finish\n");

		// if ((1 - this->STL_CharMark_HitRadio > EPS) || this->STL_CharMark_zDifSUM < 0) //
		// {
		// 	this->STL_CharMark_IsValid = false;
		// 	LOGINFO("-----0608 this->STL_CharMark_IsValid = false");
		// }
		if (this->STL_CharMark_IsValid == false)
		{
			LOGINFO("-----0615 this->STL_CharMark_IsValid = false");
		}
		else
		{
			LOGINFO("-----0615 this->STL_CharMark_IsValid = true");
		}
		retval_mesh.checkonly = false;
		retval_mesh.repair(1);
		return retval_mesh;
	}

	// 侧壁位置自动生成标签
	TriangleMesh
	Side_STL_CharMark_Mesh::STL_GeneralMeshAuto()
	{
		DWORD Start = GetTickCount();
		TriangleMesh retval_mesh;
		TriangleMesh retest_mesh;
		TriangleMesh test_mesh = this->obj_copy;
		// 将模型侧过来
		test_mesh.rotate_x(-PI * 90 / 180);
		// 计算数字标记的中心点
		double min_OnPlat = DBL_MAX;
		unsigned int min_rotate_num = 0;
		int rotate_num = 0;
		std::vector<finalSTLcal> FinalCal(36);
		boost::thread_group *_work_p;
		int threads = (3 * boost::thread::hardware_concurrency() / 4);
		LOGINFO("this threads is %d", threads);
		STL_CharMark_Mesh *mesh_p = nullptr;
		parallelize<size_t>(
			0,
			35,
			boost::bind(&STL_CharMark_Mesh::RSearch_point, mesh_p, _1, test_mesh, &FinalCal, this->STL_Str, this->STL_Charmark_thickness, this->STL_Insert_thickness, this->STL_CharMark_Size, this->STL_CharMark_gap),
			_work_p,
			threads);

		for (int i = 0; i < 36; i++)
		{
			if ((FinalCal[i].F_STLzDifSUM <= min_OnPlat) && (FinalCal[i].F_meshValid) && ((FinalCal[i].F_HitRadio) > 0.95))
			{
				min_OnPlat = FinalCal[i].F_STLzDifSUM;
				LOGINFO("this [%d] time the min_OnPlat is %lf", FinalCal[i].F_rotate_Num, min_OnPlat);
				min_rotate_num = FinalCal[i].F_rotate_Num;
			}
		}
		LOGINFO("0609 1637 --- \n");
		retval_mesh = FinalCal[min_rotate_num].F_mesh;
		LOGINFO("----min_rotate_num[%d]-----this min diffZsum is %5lf----------------------", min_rotate_num, min_OnPlat);
		retval_mesh.rotate_y(-PI * min_rotate_num * 10 / 180);
		retval_mesh.rotate_x(PI * 90 / 180);
		this->STL_CharMark_HitRadio = FinalCal[min_rotate_num].F_HitRadio / this->STL_Str.size();
		this->STL_CharMark_zDifSUM = FinalCal[min_rotate_num].F_STLzDifSUM;
		this->STL_CharMark_IsValid = FinalCal[min_rotate_num].F_meshValid;
		this->MidPoint = FinalCal[min_rotate_num].F_MidPoint;
		
		LOGINFO("-----0608 this->MidPoint %f%f%F", this->MidPoint.x, this->MidPoint.y, this->MidPoint.z);
		LOGINFO("-----0608 this->STL_CharMark_IsValid = false");
		LOGINFO("the STL_CharMark_HitRadio is %lf", this->STL_CharMark_HitRadio);
		LOGINFO("------------自动生成一个[%s]标签计算耗时【%u】", this->STL_Str.c_str(), GetTickCount() - Start);
		return retval_mesh;
	}

	// 侧壁位置手动生成标签
	TriangleMesh
	Side_STL_CharMark_Mesh::STL_GeneralMeshManual()
	{
		LOGINFO("start to mannul make stlmark");
		DWORD manualStart = GetTickCount();
		TriangleMesh retval_mesh;
		// 将模型侧过来
		Pointf3 mPoint(this->MidPoint.x, this->MidPoint.y, this->MidPoint.z);
		LOGINFO("the mannual MidPoint is  = %s", this->MidPoint.dump_perl().c_str());
		Vector3 from1(MidNormal.x, MidNormal.y, MidNormal.z);
		from1.normalize();
		LOGINFO("normalize->MidNormal = [%s], this->MidPoint = [%s], mPoint = [%s]", this->MidNormal.dump_perl().c_str(), this->MidPoint.dump_perl().c_str(), mPoint.dump_perl().c_str());
		Vector3 to1(0.0, 1.0, 0.0);
		Vector3 from2(0.0, 1.0, 0.0);
		Vector3 to2(0.0, 0.0, -1.0);
		Pointf3 p1(0, 0, 0);
		Pointf3 p2(0, 0, 0);
		TriangleMesh test_mesh1 = this->obj_copy.rotate(from1, to1, mPoint, p1);
		LOGINFO("rotate1->MidNormal = [%s],  mPoint = [%s]", this->MidNormal.dump_perl().c_str(), mPoint.dump_perl().c_str());
		TriangleMesh test_mesh2 = test_mesh1.rotate(from2, to2, mPoint, p2);
		LOGINFO("rotate2->MidNormal = [%s],  mPoint = [%s]", this->MidNormal.dump_perl().c_str(), mPoint.dump_perl().c_str());
		mPoint.z = 0.0;
		double minZ = test_mesh2.bounding_box().min.z;
		LOGINFO("this->mark_thickness = [%f], minZ = [%f]", this->STL_Charmark_thickness, minZ);
		test_mesh2.translate(0, 0, this->STL_Charmark_thickness - minZ);
		STL_CharMark_Mesh *pMark = new STL_CharMark_Mesh(
			&test_mesh2,
			mPoint,
			this->STL_Str,
			this->STL_Charmark_thickness,
			this->STL_Insert_thickness,
			this->STL_CharMark_Length, // 字符高度
			this->STL_CharMark_Width,  // 字符宽度
			this->STL_CharMark_Size,
			this->STL_CharMark_gap,
			this->autoGeneral);
		retval_mesh = pMark->GeneralMesh();
		retval_mesh.translate(0, 0, minZ - this->STL_Charmark_thickness);
		retval_mesh = retval_mesh.rotate(to2, from2, p1, p2);
		retval_mesh = retval_mesh.rotate(to1, from1, p1, p2);
		this->STL_CharMark_zDifSUM = pMark->STLzDifSUM;
		this->STL_CharMark_HitRadio = double(pMark->HitRadioSum) / this->STL_Str.size();
		this->STL_CharMark_IsValid = pMark->charMeshIsValid;

		delete pMark;
		LOGINFO("0607 the diffZsum is %lf the HitRadio is %lf", this->STL_CharMark_zDifSUM, this->STL_CharMark_HitRadio);
		LOGINFO("------------手动生成一个[%s]标签计算耗时【%u】", this->STL_Str.c_str(), GetTickCount() - manualStart);
		return retval_mesh;
	}

}
