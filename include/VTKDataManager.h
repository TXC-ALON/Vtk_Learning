#pragma once
#ifndef VTKDataManager_H
#define VTKDataManager_H

#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkLookupTable.h>
#include <vtkNamedColors.h>
#include <QTime>
#include <QDebug>
#include <stl.h>
// #include "Model.hpp"
// #include "Print.hpp"
// 单例类，全局数据管理类，存储VTK视窗中的模型等相关数据，前后端和数据分离
class VTKDataManager
{
public:
	static VTKDataManager *getInstance();
	void InitVTKData();
	void ReleaseVTKData();

public:
	// Model MainModel;//记录和管理模型数据
	// Print MainPrint;//记录和管理切片数据
	QMap<int, vtkSmartPointer<vtkActor>> ObjectActorMap;  // 用来存放物体的actor,所有的物体volume的mesh合并后转化
	QMap<int, vtkSmartPointer<vtkActor>> SupportActorMap; // 用来存放支撑的actor,所有的支撑volume的mesh合并后转化

public:
	// vtkSmartPointer<vtkActor> GetObjectActor(ModelObject *objectPtr);
	// vtkSmartPointer<vtkActor> GetSupportActor(ModelObject *objectPtr);

	// 将admesh数据转换为VTK数据
	bool toVTK(stl_file *adMesh, vtkSmartPointer<vtkPolyData> vtkMesh);
	// 将VTK数据转换为admesh数据
	bool toADMesh(vtkSmartPointer<vtkPolyData> vtkMesh, stl_file *adMesh);

private:
	VTKDataManager() = default;
	~VTKDataManager() = default;

	VTKDataManager(const VTKDataManager &) = delete;
	VTKDataManager &operator=(const VTKDataManager &) = delete;
};
#endif // VTKDataManager_H
