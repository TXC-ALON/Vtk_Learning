#pragma once
#include "Vtkwidget3D.h"
#include <Qt_headers.h>
#include <Vtk_headers.h>
QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QDockWidget *controlDock;
    VtkWidget3D vtkWidget;

public:
    void setupUi(QMainWindow *MainWindow)
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

        vtkNew<vtkSphereSource> sphere;
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
    }
};
namespace Ui
{
    class MainWindow : public Ui_MainWindow
    {
    };
} // namespace Ui

QT_END_NAMESPACE