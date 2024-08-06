#pragma once
#include <Vtkwidget3D.h>
#include <Qt_headers.h>
#include <Vtk_headers.h>

#include <cmath>
#include <cstdlib>
#include <random>

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
class Ui_MainWindow
{
public:
    Ui_MainWindow()
    {
        vtkNew<vtkSphereSource> temp_sphere;
        sphere = temp_sphere;
    }
    QDockWidget *controlDock;
    VtkWidget3D vtkWidget;
    vtkSmartPointer<vtkSphereSource> sphere;

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

        // Vtk
        vtkNew<vtkGenericOpenGLRenderWindow> tempwindow;
        vtkWidget.window = tempwindow;
        vtkWidget.p_vtkWidget = new QVTKOpenGLNativeWidget(MainWindow);
        MainWindow->setCentralWidget(vtkWidget.p_vtkWidget);
        // VTK setup
        vtkWidget.p_vtkWidget->setRenderWindow(vtkWidget.window.Get());

        sphere->SetRadius(1.0);
        sphere->SetThetaResolution(100);
        sphere->SetPhiResolution(100);

        vtkNew<vtkDataSetMapper> mapper;
        mapper->SetInputConnection(sphere->GetOutputPort());

        vtkNew<vtkActor> actor;
        actor->SetMapper(mapper);
        actor->GetProperty()->SetEdgeVisibility(true);
        actor->GetProperty()->SetRepresentationToSurface();
        vtkNew<vtkRenderer> renderer;
        renderer->SetBackground(0.733, 0.871, 0.984);
        renderer->AddActor(actor);

        this->vtkWidget.window->AddRenderer(renderer);
        // Setup initial status
        std::mt19937 randEng(0);
        ::Randomize(sphere, mapper, this->vtkWidget.window.GetPointer(), randEng);

        // connect the buttons
        QObject::connect(randomizeButton, &QPushButton::released,
                         [sphere = sphere.GetPointer(), mapper = mapper.GetPointer(), window = this->vtkWidget.window.GetPointer(), &randEng]()
                         { ::Randomize(sphere, mapper, window, randEng); });
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
