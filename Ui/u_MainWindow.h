#pragma once
#include <Vtkwidget3D.h>
#include <Qt_headers.h>
#include <Vtk_headers.h>

#include <cmath>
#include <cstdlib>
#include <random>
#include <vtkInteractorStyleTrackballCamera.h>
#include <ThreeDAxesCallback.h>
#include <ThreeDAxesRepresentation.h>
#include <ThreeDAxesWidget.h>
#include <VTK_Utils.h>

class ssCallback : public vtkCommand
{
public:
    static ssCallback *New()
    {
        return new ssCallback;
    }

    virtual void Execute(vtkObject *caller, unsigned long eventId, void *callData) override
    {
        std::cout << eventId << std::endl;
    }
};
class XKeyPressCallback : public vtkCommand
{
public:
    static XKeyPressCallback *New()
    {
        return new XKeyPressCallback;
    }

    virtual void Execute(vtkObject *caller, unsigned long eventId, void *callData) override
    {
        if (eventId == vtkCommand::KeyPressEvent)
        {
            vtkRenderWindowInteractor *iren = static_cast<vtkRenderWindowInteractor *>(caller);
            std::string key = iren->GetKeySym();
            std::cout << key << std::endl;
            if (iren->GetKeyCode() == '0')
            {
                std::cout << "Reset" << std::endl;
                renderer->GetActiveCamera()->SetPosition(0, 30, 0);  // 相机位置，这里假设相机在z轴正方向
                renderer->GetActiveCamera()->SetFocalPoint(0, 0, 0); // 焦点位置，通常设置为场景的中心
                renderer->GetActiveCamera()->SetViewUp(0, 0, 1);     // 视图向上向量，这里假设向上为y轴正方向
                renderer->GetActiveCamera()->Azimuth(-45);           // 方位角，绕y轴旋转45度
                renderer->GetActiveCamera()->Elevation(30);          // 仰角，绕x轴旋转45度
                                                                     // iren->GetRenderWindow()->Render();                   // 重新渲染窗口以显示重置效果
                iren->GetRenderWindow()->Render();                   // 重新渲染窗口以显示效果
                window->Render();
            }
            else if (iren->GetKeyCode() == '7')
            {
                renderer->GetActiveCamera()->SetPosition(30, 0, 0);  // 相机位置，这里假设相机在x轴正方向
                renderer->GetActiveCamera()->SetFocalPoint(0, 0, 0); // 焦点位置，通常设置为场景的中心
                renderer->GetActiveCamera()->SetViewUp(0, 0, 1);     // 视图向上向量，这里假设向上为y轴正方向
                iren->GetRenderWindow()->Render();                   // 重新渲染窗口以显示效果
            }

            else if (iren->GetKeyCode() == '8')
            {
                renderer->GetActiveCamera()->SetPosition(0, 30, 0);  // 相机位置，这里假设相机在y轴正方向
                renderer->GetActiveCamera()->SetFocalPoint(0, 0, 0); // 焦点位置，通常设置为场景的中心
                renderer->GetActiveCamera()->SetViewUp(0, 0, 1);     // 视图向上向量，这里假设向上为z轴正方向
                iren->GetRenderWindow()->Render();                   // 重新渲染窗口以显示效果
            }

            else if (iren->GetKeyCode() == '9')
            {
                renderer->GetActiveCamera()->SetPosition(0, 0, 30);  // 相机位置，这里假设相机在z轴正方向
                renderer->GetActiveCamera()->SetFocalPoint(0, 0, 0); // 焦点位置，通常设置为场景的中心
                renderer->GetActiveCamera()->SetViewUp(0, 1, 0);     // 视图向上向量，这里假设向上为y轴正方向
                iren->GetRenderWindow()->Render();                   // 重新渲染窗口以显示效果
            }
        }
    }
    vtkRenderer *renderer;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> window;
};

class Ui_MainWindow
{
public:
    Ui_MainWindow()
    {
        this->rotateWidget = ThreeDAxesWidget::New();
        this->rotateWidget->EnabledOff();
        this->callback = ThreeDAxesCallback::New();
    }
    QDockWidget *controlDock;
    QWidget *centralWidget;
    VtkWidget3D *vtkWidget;
    QPushButton *openFileButton;
    QPushButton *rotateButton;
    QPushButton *ShowAxesButton;

    vtkSmartPointer<vtkAxesActor> Axes;
    vtkSmartPointer<vtkOrientationMarkerWidget> widget;
    vtkSmartPointer<XKeyPressCallback> keycallback;
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor;
    ThreeDAxesCallback *callback = nullptr;
    ThreeDAxesWidget *rotateWidget;

public:
    void setupUi(QMainWindow *MainWindow);
};
namespace Ui
{
    class MainWindow : public Ui_MainWindow
    {
    };
} // namespace Ui
