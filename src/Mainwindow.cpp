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
#include <VTKDataManager.h>
MainWindow::~MainWindow()
{
    if (pRotateWidget != nullptr)
    {
        delete pRotateWidget;
        pRotateWidget = nullptr;
    }
}
void MainWindow::ShowAxes()
{
    if (!ui->rotateWidget->GetEnabled())
    {
        std::cout << "to enable" << std::endl;
        vtkSmartPointer<vtkActor> actor = VTKDataManager::getInstance()->ObjectActorMap[1];
        if (actor != nullptr)
        {
            ui->rotateWidget->SetInteractor(ui->vtkWidget->interactor());
            ui->rotateWidget->SetCurrentRenderer(ui->vtkWidget->m_renderer);
            ui->rotateWidget->CreateDefaultRepresentation();
            ui->rotateWidget->GetRepresentation()->SetPlaceFactor(1);
            ui->rotateWidget->GetRepresentation()->PlaceWidget(actor->GetBounds());
            ui->callback->SetProp3D(actor);
            ui->rotateWidget->AddObserver(vtkCommand::StartInteractionEvent, ui->callback, 1.0);
            ui->rotateWidget->AddObserver(vtkCommand::InteractionEvent, ui->callback, 1.0);
            ui->rotateWidget->AddObserver(vtkCommand::EndInteractionEvent, ui->callback, 1.0);
            ui->rotateWidget->EnabledOn();
            ui->vtkWidget->m_renderWindow->Render();
        }
        else
        {
            return;
        }
    }
    else
    {
        std::cout << "close" << std::endl;
        ui->rotateWidget->EnabledOff();
        ui->vtkWidget->m_renderWindow->Render();
    }
}
void MainWindow::openFileSlot()
{
    std::cout << "openFileSlot" << std::endl;

    vtkSmartPointer<vtkSphereSource> sphere;
    vtkSmartPointer<vtkCylinderSource> cylinder;
    vtkSmartPointer<vtkDataSetMapper> mapper;
    vtkSmartPointer<vtkActor> actor;
    sphere = vtkSmartPointer<vtkSphereSource>::New();
    cylinder = vtkSmartPointer<vtkCylinderSource>::New();
    cylinder->SetCenter(0.0, 0.0, 0.0);
    cylinder->SetRadius(5.0);
    cylinder->SetHeight(2.0);
    cylinder->SetResolution(100);
    // vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
    sphere->SetRadius(1.0);
    sphere->SetThetaResolution(100);
    sphere->SetPhiResolution(100);
    sphere->Update();

    vtkSmartPointer<vtkDataSetMapper> sphere_mapper = vtkSmartPointer<vtkDataSetMapper>::New();
    vtkSmartPointer<vtkDataSetMapper> cylinder_mapper = vtkSmartPointer<vtkDataSetMapper>::New();

    sphere_mapper->SetInputConnection(sphere->GetOutputPort());
    cylinder_mapper->SetInputConnection(cylinder->GetOutputPort());
    actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(cylinder_mapper);
    actor->GetProperty()->SetEdgeVisibility(true);
    actor->GetProperty()->SetRepresentationToSurface();
    VTKDataManager::getInstance()->ObjectActorMap.insert(1, actor);
    ui->vtkWidget->m_renderer->AddActor(actor);
}
void MainWindow::RotateSlot()
{
    std::cout << "RotateSlot" << std::endl;
    pRotateWidget->setVisible(true);
    pRotateWidget->InitOpenPage();
}

void MainWindow::connectSignals(QWidget *widget)
{
    // setAcceptDrops(true);
    connect(ui->rotateButton, &QPushButton::clicked, this, &MainWindow::RotateSlot);
    connect(ui->openFileButton, &QPushButton::clicked, this, &MainWindow::openFileSlot);
    connect(ui->ShowAxesButton, &QPushButton::clicked, this, &MainWindow::ShowAxes);
}

void MainWindow::setupWidget(QWidget *widget)
{
    // setupWidgetLayoutAndStyle(widget);
    connectSignals(widget);
}