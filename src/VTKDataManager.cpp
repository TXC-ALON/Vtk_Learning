#include <VTKDataManager.h>

VTKDataManager *VTKDataManager::getInstance()
{
	static VTKDataManager instance;
	return &instance;
}

void VTKDataManager::InitVTKData()
{
	ObjectActorMap.clear();
	SupportActorMap.clear();
}

void VTKDataManager::ReleaseVTKData()
{
	ObjectActorMap.clear();
	SupportActorMap.clear();
}
/*
vtkSmartPointer<vtkActor>
VTKDataManager::GetObjectActor(ModelObject* objectPtr)
{
	qDebug() << "VTKDataManager::GetObjectActor start";
	//获取ModelObject的编号
	int idx = objectPtr->object_num;
	if (idx == -1)
	{
		qDebug() << "VTKDataManager::GetObjectActor objectPtr->object_num == -1";
		return NULL;
	}
	else
	{
		qDebug() << "VTKDataManager::GetObjectActor objectPtr->object_num = " << idx;
	}

	qDebug() << "VTKDataManager::GetObjectActor end";

	return ObjectActorMap[idx];
}

vtkSmartPointer<vtkActor>
VTKDataManager::GetSupportActor(ModelObject* objectPtr)
{
	qDebug() << "VTKDataManager::GetSupportActor start";
	//获取ModelObject的编号
	int idx = objectPtr->object_num;
	if (idx == -1)
	{
		qDebug() << "VTKDataManager::GetSupportActor objectPtr->object_num == -1";
		return NULL;
	}
	else
	{
		qDebug() << "VTKDataManager::GetSupportActor objectPtr->object_num = " << idx;
	}

	qDebug() << "VTKDataManager::GetSupportActor end";

	return SupportActorMap[idx];
}

*/

bool VTKDataManager::toADMesh(vtkSmartPointer<vtkPolyData> vtkMesh, stl_file *adMesh)
{
	qDebug() << "VTKDataManager::toADMesh start";

	// Vertices
	const vtkIdType inNPts = vtkMesh->GetNumberOfPoints();
	// Cells
	const vtkIdType inNCells = vtkMesh->GetNumberOfCells();

	if (inNPts == 0 || inNCells == 0)
	{
		qDebug() << "VTKDataManager::toADMesh inNPts == 0 || inNCells == 0";
		return false;
	}

	// convert vtkMesh data to stl_file
	qDebug() << "VTKDataManager::toADMesh convert vtkMesh data to stl_file";
	{
		// initialize stl_file
		stl_initialize(adMesh);
		adMesh->stats.number_of_facets = inNCells;
		adMesh->stats.original_num_facets = adMesh->stats.number_of_facets;
		stl_allocate(adMesh);

		int first = 1;

		// add facet
		qDebug() << "VTKDataManager::toADMesh add facet";
		stl_facet facet;
		for (vtkIdType i = 0; i < inNCells; i++)
		{
			vtkSmartPointer<vtkIdList> cellPoints = vtkSmartPointer<vtkIdList>::New();
			vtkMesh->GetCellPoints(i, cellPoints);

			vtkIdType IDNumber = cellPoints->GetNumberOfIds();
			if (IDNumber != 3)
			{
				qDebug() << "find a cell, points number != 3.";
				continue;
			}

			for (int j = 0; j < 3; j++)
			{
				double point[3];
				vtkMesh->GetPoint(cellPoints->GetId(j), point);
				facet.vertex[j].x = point[0];
				facet.vertex[j].y = point[1];
				facet.vertex[j].z = point[2];
				// std::cout << "Vertex " << i + 1 << ": (" << point[0]
				//     << ", " << point[1] << ", "
				//     << point[2] << ")"
				//     std::endl;
			}

			/* Write the facet into memory. */
			memcpy(adMesh->facet_start + i, &facet, SIZEOF_STL_FACET);
			stl_facet_stats(adMesh, facet, first);
			first = 0;
		}

		adMesh->stats.size.x = adMesh->stats.max.x - adMesh->stats.min.x;
		adMesh->stats.size.y = adMesh->stats.max.y - adMesh->stats.min.y;
		adMesh->stats.size.z = adMesh->stats.max.z - adMesh->stats.min.z;
		adMesh->stats.bounding_diameter = sqrt(
			adMesh->stats.size.x * adMesh->stats.size.x +
			adMesh->stats.size.y * adMesh->stats.size.y +
			adMesh->stats.size.z * adMesh->stats.size.z);

		// stl_write_ascii(adMesh, "d:\\toadmesh.stl", "");
	}

	// check_topology
	qDebug() << "VTKDataManager::toADMesh check_topology";
	{
		// checking exact
		// 精确的检查模型中的三角形面片
		stl_check_facets_exact(adMesh);
		// 有一条孤立边的三角面片个数
		adMesh->stats.facets_w_1_bad_edge = (adMesh->stats.connected_facets_2_edge - adMesh->stats.connected_facets_3_edge);
		// 有两条孤立边的三角面片个数
		adMesh->stats.facets_w_2_bad_edge = (adMesh->stats.connected_facets_1_edge - adMesh->stats.connected_facets_2_edge);
		// 有三条孤立边的三角面片个数
		adMesh->stats.facets_w_3_bad_edge = (adMesh->stats.number_of_facets - adMesh->stats.connected_facets_1_edge);
		// 统计坏边个数
		adMesh->repair_stats.bad_edges = adMesh->stats.facets_w_1_bad_edge +
										 adMesh->stats.facets_w_2_bad_edge * 2 +
										 adMesh->stats.facets_w_3_bad_edge * 3;

		// checking nearby
		// int last_edges_fixed = 0;
		// 过滤值为最小的边长
		float tolerance = adMesh->stats.shortest_edge;
		// 步长为包围盒的对角线长度
		float increment = adMesh->stats.bounding_diameter / 10000.0;
		// 迭代次数
		int iterations = 2;
		// 迭代终止条件：无孤立边
		if (adMesh->stats.connected_facets_3_edge < adMesh->stats.number_of_facets)
		{
			for (int i = 0; i < iterations; i++)
			{
				if (adMesh->stats.connected_facets_3_edge < adMesh->stats.number_of_facets)
				{
					// printf("Checking nearby. Tolerance= %f Iteration=%d of %d...", tolerance, i + 1, iterations);
					stl_check_facets_nearby(adMesh, tolerance);
					// printf("  Fixed %d edges.\n", adMesh->stats.edges_fixed - last_edges_fixed);
					// last_edges_fixed = adMesh->stats.edges_fixed;
					tolerance += increment;
				}
				else
				{
					break;
				}
			}
		}
	}

	// generate shared v_indices
	qDebug() << "VTKDataManager::toADMesh generate shared v_indices";
	stl_generate_shared_vertices(adMesh);

	qDebug() << "VTKDataManager::toADMesh end";
	return true;
}

bool VTKDataManager::toVTK(stl_file *adMesh, vtkSmartPointer<vtkPolyData> vtkMesh)
{
	qDebug() << "VTKDataManager::toVTK start";
	QTime startTime = QTime::currentTime();

	if (adMesh->stats.unable_repair || adMesh->v_shared == nullptr || adMesh->v_indices == nullptr)
	{
		qDebug() << "adMesh->stats.unable_repair || adMesh->v_shared == nullptr || adMesh->v_indices == nullptr";
		return false;
	}

	// 顶点
	qDebug() << "VTKDataManager::toVTK pts creat";
	vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
	const vtkIdType outNPts = adMesh->stats.shared_vertices;
	pts->Allocate(outNPts);

	for (vtkIdType i = 0; i < outNPts; i++)
	{
		stl_vertex *v = &adMesh->v_shared[i];
		pts->InsertNextPoint(v->x, v->y, v->z);
	}
	pts->Squeeze();

	// 三角形
	qDebug() << "VTKDataManager::toVTK cells creat";
	vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
	const vtkIdType outNCells = adMesh->stats.number_of_facets;
	cells->AllocateEstimate(outNCells, 3);

	for (vtkIdType i = 0; i < outNCells; i++)
	{
		vtkSmartPointer<vtkIdList> ids = vtkSmartPointer<vtkIdList>::New();
		ids->Allocate(3);

		v_indices_struct *f = &adMesh->v_indices[i];
		for (vtkIdType j = 0; j < 3; j++)
		{
			ids->InsertNextId(f->vertex[j]);
		}
		cells->InsertNextCell(ids);
	}
	cells->Squeeze();

	// VTK dataset
	qDebug() << "VTKDataManager::toVTK dataset creat";
	vtkMesh->Reset(); // always start from new mesh
	vtkMesh->SetPoints(pts);
	vtkMesh->SetPolys(cells);

	QTime endTime = QTime::currentTime();
	int elapsed = startTime.msecsTo(endTime); // 以毫秒为单位
	qDebug() << "VTKDataManager::toVTK stl_file to vtkpolydata cost time:" << elapsed << "ms";
	qDebug() << "VTKDataManager::toVTK end";
	return true;
}