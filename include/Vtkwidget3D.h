#ifndef VTKWIDGET_H
#define VTKWIDGET_H

#include <QWidget>
#include <QVTKOpenGLNativeWidget.h>
#include <Vtk_headers.h>
class VtkWidget3D : public QWidget
{
    Q_OBJECT

public:
    VtkWidget3D(QWidget *parent = nullptr);
    ~VtkWidget3D();

public:
    QVTKOpenGLNativeWidget* p_vtkWidget;
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> window;
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor;
};

#endif // VTKWIDGET_H