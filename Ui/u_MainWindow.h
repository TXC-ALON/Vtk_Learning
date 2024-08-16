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
namespace
{
    /**
     * Deform the sphere source using a random amplitude and modes and render it in
     * the window
     *
     * @param sphere the original sphere source
     * @param mapper the mapper for the scene
     * @param window the window to render to
     * @param randEng the random number generator engine
     */
    void Randomize(vtkSphereSource *sphere, vtkMapper *mapper,
                   vtkGenericOpenGLRenderWindow *window, std::mt19937 &randEng);
}
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
    void
    setupUi(QMainWindow *MainWindow)
    {
        MainWindow->resize(1200, 900);
        // Control area setup
        controlDock = new QDockWidget(MainWindow);
        MainWindow->addDockWidget(Qt::LeftDockWidgetArea, controlDock);
        QLabel *controlDockTitle = new QLabel("Control Dock", MainWindow);
        controlDockTitle->setMargin(20);
        controlDock->setTitleBarWidget(controlDockTitle);

        QVBoxLayout *dockLayout = new QVBoxLayout();
        QWidget *layoutContainer = new QWidget(MainWindow);
        layoutContainer->setLayout(dockLayout);
        controlDock->setWidget(layoutContainer);

        QPushButton *randomizeButton = new QPushButton("Randomize", MainWindow);
        dockLayout->addWidget(randomizeButton);
        openFileButton = new QPushButton("Open", MainWindow);
        dockLayout->addWidget(openFileButton);
        rotateButton = new QPushButton("Rotate", MainWindow);
        dockLayout->addWidget(rotateButton);
        ShowAxesButton = new QPushButton("ShowAxes", MainWindow);
        dockLayout->addWidget(ShowAxesButton);

        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        // Vtk
        // 窗口
        vtkWidget = new VtkWidget3D(centralWidget);
        vtkWidget->setObjectName(QString::fromUtf8("widgetVTK"));
        vtkWidget->setMinimumSize(QSize(1200, 900));

        MainWindow->setCentralWidget(centralWidget);

        vtkNew<vtkInteractorStyleTrackballCamera> interactorStyle;
        vtkWidget->interactor()->SetInteractorStyle(interactorStyle);
        vtkNew<XKeyPressCallback> keycallback;
        keycallback->renderer = vtkWidget->m_renderer;
        keycallback->window = vtkWidget->m_renderWindow;
        vtkWidget->interactor()->AddObserver(vtkCommand::KeyPressEvent, keycallback);
        // 添加坐标轴
        // Add_Arrow_Axes(vtkWidget->m_renderer, 15);
        Add_Line_Axes(vtkWidget->m_renderer, 15);
    }
};
namespace Ui
{
    class MainWindow : public Ui_MainWindow
    {
    };
} // namespace Ui
namespace
{
    void Randomize(vtkSphereSource *sphere, vtkMapper *mapper,
                   vtkGenericOpenGLRenderWindow *window, std::mt19937 &randEng)
    {
        // Generate randomness.
        double randAmp = 0.1 + ((randEng() % 1000) / 1000.0) * 0.2;
        double randThetaFreq = 1.0 + (randEng() % 9);
        double randPhiFreq = 1.0 + (randEng() % 9);

        // Extract and prepare data.
        sphere->Update();
        vtkSmartPointer<vtkPolyData> newSphere;
        newSphere.TakeReference(sphere->GetOutput()->NewInstance());
        newSphere->DeepCopy(sphere->GetOutput());
        vtkNew<vtkDoubleArray> height;
        height->SetName("Height");
        height->SetNumberOfComponents(1);
        height->SetNumberOfTuples(newSphere->GetNumberOfPoints());
        newSphere->GetPointData()->AddArray(height);

        // Deform the sphere.
        for (int iP = 0; iP < newSphere->GetNumberOfPoints(); iP++)
        {
            double pt[3] = {0.0};
            newSphere->GetPoint(iP, pt);
            double theta = std::atan2(pt[1], pt[0]);
            double phi =
                std::atan2(pt[2], std::sqrt(std::pow(pt[0], 2) + std::pow(pt[1], 2)));
            double thisAmp =
                randAmp * std::cos(randThetaFreq * theta) * std::sin(randPhiFreq * phi);
            height->SetValue(iP, thisAmp);
            pt[0] += thisAmp * std::cos(theta) * std::cos(phi);
            pt[1] += thisAmp * std::sin(theta) * std::cos(phi);
            pt[2] += thisAmp * std::sin(phi);
            newSphere->GetPoints()->SetPoint(iP, pt);
        }
        newSphere->GetPointData()->SetScalars(height);

        // Reconfigure the pipeline to take the new deformed sphere.
        mapper->SetInputDataObject(newSphere);
        mapper->SetScalarModeToUsePointData();
        mapper->ColorByArrayComponent("Height", 0);
        window->Render();
    }
}
