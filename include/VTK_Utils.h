#pragma once
#include <Vtk_headers.h>

double Deg2Rad(double degrees);
vtkSmartPointer<vtkPolyData> ReadPolyData(std::string const &fileName);
vtkSmartPointer<vtkMatrix4x4> GetModelRotationMatrix(double m, double n, double l, double angleX, double angleY, double angleZ);
void SetModelRotationMatrix(vtkActor *actor, double m, double n, double l, double angleX, double angleY, double angleZ);
void SetModelTranslateMatrix(vtkActor *actor, double m, double n, double l, double dx, double dy, double dz);
vtkSmartPointer<vtkActor> createArrowActor(double scaleFactor, double *color, double *direction);
void Add_Arrow_Axes(vtkSmartPointer<vtkRenderer> renderer, double scaleFactor = 20);
void Add_Line_Axes(vtkSmartPointer<vtkRenderer> renderer, double scaleFactor = 20);