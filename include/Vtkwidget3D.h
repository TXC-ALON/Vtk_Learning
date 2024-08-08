#ifndef VTKWIDGET_H
#define VTKWIDGET_H

#include <QWidget>
#include <QVTKOpenGLNativeWidget.h>
#include <Vtk_headers.h>
class VtkWidget3D : public QVTKOpenGLNativeWidget
{
    Q_OBJECT

public:
    VtkWidget3D(QWidget *parent = nullptr);
    ~VtkWidget3D();
    void setupWidget();
    void initializeRenderer();

public:
    vtkSmartPointer<vtkRenderer> m_renderer;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_renderWindow;

public:
    // QVTKOpenGLNativeWidget *p_vtkWidget;
    // vtkSmartPointer<vtkRenderer> renderer;
    // // vtkSmartPointer<vtkRenderWindow> window;
    // vtkSmartPointer<vtkGenericOpenGLRenderWindow> window;
    // vtkSmartPointer<vtkGenericRenderWindowInteractor> renderWindowInteractor;

protected:
    // 事件处理槽函数
    void mousePressEvent(QMouseEvent *event);
signals:
    void mousePressedSignal(const QPoint &pos);
};

#endif // VTKWIDGET_H