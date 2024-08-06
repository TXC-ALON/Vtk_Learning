#include "Mainwindow.h"
#include <QVBoxLayout>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkActor.h>
#include <vtkDataSetMapper.h>
#include <vtkDoubleArray.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkPointData.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>

#include <cmath>
#include <cstdlib>
#include <random>
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
    setupUi(this);
}

void MainWindow::setupUi(QMainWindow *mainWindow)
{
    this->resize(1200, 900);
    // Control area setup
    controlDock = new QDockWidget(this);
    addDockWidget(Qt::LeftDockWidgetArea, controlDock);
    QLabel *controlDockTitle = new QLabel("Control Dock", this);
    controlDockTitle->setMargin(20);
    controlDock->setTitleBarWidget(controlDockTitle);

    QVBoxLayout *dockLayout = new QVBoxLayout();
    QWidget *layoutContainer = new QWidget(this);
    layoutContainer->setLayout(dockLayout);
    controlDock->setWidget(layoutContainer);

    QPushButton *randomizeButton = new QPushButton("Randomize", this);
    dockLayout->addWidget(randomizeButton);

    // // Render area setup
    // QVTKOpenGLNativeWidget *vtkRenderWidget = new QVTKOpenGLNativeWidget(this);
    // setCentralWidget(vtkRenderWidget);

    // // VTK setup
    // vtkNew<vtkRenderWindow> window;
    // vtkRenderWidget->setRenderWindow(window.Get());
    vtkNew<vtkGenericOpenGLRenderWindow> tempwindow;
    vtkWidget.window = tempwindow;
    vtkWidget.p_vtkWidget = new QVTKOpenGLNativeWidget(this);
    setCentralWidget(vtkWidget.p_vtkWidget);
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

    vtkWidget.window->AddRenderer(renderer);
}

MainWindow::~MainWindow()
{
    ;
}