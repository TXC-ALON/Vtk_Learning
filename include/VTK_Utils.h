#pragma once
#include <Vtk_headers.h>

double Deg2Rad(double degrees);
vtkSmartPointer<vtkPolyData> ReadPolyData(std::string const &fileName);
vtkSmartPointer<vtkMatrix4x4> GetModelRotationMatrix(double m, double n, double l, double angleX, double angleY, double angleZ);
void SetModelRotationMatrix(vtkActor *actor, double m, double n, double l, double angleX, double angleY, double angleZ);
void SetModelTranslateMatrix(vtkActor *actor, double m, double n, double l, double dx, double dy, double dz);
