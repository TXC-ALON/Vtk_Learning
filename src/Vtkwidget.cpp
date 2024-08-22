
#include <QVBoxLayout>
#include "VTKWidget3D.h"
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkObjectFactory.h>
VtkWidget3D::VtkWidget3D(QWidget *parent) : QVTKOpenGLNativeWidget(parent)
{
    setupWidget();
}

VtkWidget3D::~VtkWidget3D()
{
    // Clean up if necessary
}
void VtkWidget3D::setupWidget()
{
    initializeRenderer();
}

void VtkWidget3D::initializeRenderer()
{
    // 创建 VTK 渲染器
    m_renderer = vtkSmartPointer<vtkRenderer>::New();
    // 设置背景颜色
    m_renderer->SetBackground(0.733, 0.871, 0.984);
    // 设置渲染窗口
    m_renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    this->setRenderWindow(m_renderWindow);
    this->renderWindow()->AddRenderer(m_renderer);
    this->m_renderer->GetActiveCamera()->SetPosition(0, 400, 0); // 相机位置，这里假设相机在z轴正方向
    this->m_renderer->GetActiveCamera()->SetFocalPoint(0, 0, 0); // 焦点位置，通常设置为场景的中心
    this->m_renderer->GetActiveCamera()->SetViewUp(0, 0, 1);     // 视图向上向量，这里假设向上为y轴正方向
    this->m_renderer->GetActiveCamera()->Azimuth(-45);           // 方位角，绕y轴旋转45度
    this->m_renderer->GetActiveCamera()->Elevation(30);          // 仰角，绕x轴旋转45度
}
void VtkWidget3D::mousePressEvent(QMouseEvent *event)
{
    // 处理鼠标按下事件
    vtkRenderWindowInteractor *interactor = this->interactor();
    if (!interactor)
        return;

    int *pos = interactor->GetEventPosition();
    vtkRenderWindow *renderer = this->renderWindow();
    if (renderer)
    {
        double worldPos[4];
        double nearPoint[4] = {pos[0], pos[1], 0.0, 1.0};
        double farPoint[4] = {pos[0], pos[1], 1.0, 1.0};
        std::cout << "Mouse position in world coordinates: " << pos[0] << ", " << pos[1] << ", " << pos[2] << ", " << pos[3] << std::endl;
        emit mousePressedSignal(QPoint(pos[0], pos[1]));
    }
}