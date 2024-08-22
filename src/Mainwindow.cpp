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
#include <Toys.h>
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
        // ui->rotateWidget->CreateDefaultRepresentation();
        ui->rotateWidget->EnabledOff();
        ui->vtkWidget->m_renderWindow->Render();
    }
}
void MainWindow::openFileSlot()
{
    std::cout << "openFileSlot" << std::endl;
    std::cout << "doubleSpinBox_Type is " << ui->doubleSpinBox_Type->value() << std::endl;
    if (ui->doubleSpinBox_Type->value() == 0)
    {
        vtkSmartPointer<vtkSphereSource> sphere;
        vtkSmartPointer<vtkDataSetMapper> mapper;
        vtkSmartPointer<vtkActor> actor;
        sphere = vtkSmartPointer<vtkSphereSource>::New();
        sphere->SetRadius(5.0);
        sphere->SetThetaResolution(50);
        sphere->SetPhiResolution(50);
        sphere->Update();
        vtkSmartPointer<vtkDataSetMapper> sphere_mapper = vtkSmartPointer<vtkDataSetMapper>::New();
        sphere_mapper->SetInputConnection(sphere->GetOutputPort());

        actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(sphere_mapper);
        actor->GetProperty()->SetEdgeVisibility(true);
        actor->GetProperty()->SetRepresentationToSurface();

        ui->vtkWidget->m_renderer->AddActor(actor);
        VTKDataManager::getInstance()->ObjectActorMap.insert(1, actor);
    }
    else if (ui->doubleSpinBox_Type->value() == 1)
    {
        vtkSmartPointer<vtkCylinderSource> cylinder;

        vtkSmartPointer<vtkDataSetMapper> mapper;
        vtkSmartPointer<vtkActor> actor;
        cylinder = vtkSmartPointer<vtkCylinderSource>::New();
        cylinder->SetCenter(0.0, 0.0, 0.0);
        cylinder->SetRadius(5.0);
        cylinder->SetHeight(2.0);
        cylinder->SetResolution(100);
        vtkSmartPointer<vtkDataSetMapper> cylinder_mapper = vtkSmartPointer<vtkDataSetMapper>::New();
        cylinder_mapper->SetInputConnection(cylinder->GetOutputPort());
        actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(cylinder_mapper);
        actor->GetProperty()->SetEdgeVisibility(true);
        actor->GetProperty()->SetRepresentationToSurface();
        ui->vtkWidget->m_renderer->AddActor(actor);
        VTKDataManager::getInstance()->ObjectActorMap.insert(1, actor);
    }
    else if (ui->doubleSpinBox_Type->value() == 2)
    {
        vtkSmartPointer<vtkParametricMobius> mobius = vtkSmartPointer<vtkParametricMobius>::New();

        // 创建一个用于生成几何数据的源
        vtkSmartPointer<vtkParametricFunctionSource> functionSource = vtkSmartPointer<vtkParametricFunctionSource>::New();
        functionSource->SetParametricFunction(mobius);
        functionSource->Update();

        // 创建一个映射器并设置输入数据
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(functionSource->GetOutputPort());

        // 创建一个演员，并设置它的映射器
        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);

        // 设置演员的属性，如颜色
        actor->GetProperty()->SetColor(1.0, 0.5, 0.3); // 橙色
        actor->GetProperty()->SetEdgeVisibility(true);
        actor->GetProperty()->SetRepresentationToSurface();
        actor->SetScale(2);

        // 创建一个渲染器
        ui->vtkWidget->m_renderer->AddActor(actor);
        VTKDataManager::getInstance()->ObjectActorMap.insert(1, actor);
    }
    else if (ui->doubleSpinBox_Type->value() == 3)
    {
        // 创建自定义的莫比乌斯环
        vtkSmartPointer<CustomMobius> customMobius = vtkSmartPointer<CustomMobius>::New();

        // 创建一个用于生成几何数据的源
        vtkSmartPointer<vtkParametricFunctionSource> functionSource = vtkSmartPointer<vtkParametricFunctionSource>::New();
        functionSource->SetParametricFunction(customMobius);
        functionSource->Update();

        // 创建一个映射器并设置输入数据
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(functionSource->GetOutputPort());

        // 创建一个演员，并设置它的映射器
        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);

        // 设置演员的属性，如颜色
        actor->GetProperty()->SetColor(0.3, 0.7, 0.9); // 青色  (0.3, 0.7, 0.9) 這個顏色好看
        // actor->GetProperty()->SetEdgeVisibility(true);
        actor->GetProperty()->SetRepresentationToSurface();
        actor->SetScale(10);
        ui->vtkWidget->m_renderer->AddActor(actor);
        VTKDataManager::getInstance()->ObjectActorMap.insert(1, actor);
    }
    else if (ui->doubleSpinBox_Type->value() == 4)
    {
        // 创建自定义的莫比乌斯环
        vtkSmartPointer<HeartShapeMobius> customMobius = vtkSmartPointer<HeartShapeMobius>::New();

        // 创建一个用于生成几何数据的源
        vtkSmartPointer<vtkParametricFunctionSource> functionSource = vtkSmartPointer<vtkParametricFunctionSource>::New();
        functionSource->SetParametricFunction(customMobius);

        functionSource->SetUResolution(100); // 增加U方向的采样点数
        functionSource->SetVResolution(10);  // 增加V方向的采样点数（如果适用）
        functionSource->Update();

        // 创建一个映射器并设置输入数据
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(functionSource->GetOutputPort());

        // 创建一个演员，并设置它的映射器
        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);

        // 设置演员的属性，如颜色
        actor->GetProperty()->SetColor(1, 0.753, 0.816); //
        actor->GetProperty()->SetEdgeVisibility(true);
        actor->GetProperty()->SetRepresentationToSurface();
        ui->vtkWidget->m_renderer->AddActor(actor);
        VTKDataManager::getInstance()->ObjectActorMap.insert(1, actor);
    }
}
void MainWindow::RotateSlot()
{
    std::cout << "RotateSlot" << std::endl;
    pRotateWidget->setVisible(true);
    pRotateWidget->InitOpenPage();
}
void MainWindow::openPlatSelectSlot()
{
    std::cout << "openPlatSelectSlot" << std::endl;
}
void MainWindow::connectSignals(QWidget *widget)
{
    // setAcceptDrops(true);
    connect(pRotateWidget, &RotateWidget::sendData, this, &MainWindow::receiveData);
    connect(pRotateWidget, &RotateWidget::render_update, this, &MainWindow::Render_Update);

    connect(ui->rotateButton, &QPushButton::clicked, this, &MainWindow::RotateSlot);
    connect(ui->openFileButton, &QPushButton::clicked, this, &MainWindow::openFileSlot);
    connect(ui->ShowAxesButton, &QPushButton::clicked, this, &MainWindow::ShowAxes);
    connect(ui->bedshapesComboBox, &QComboBox::currentTextChanged, this, &MainWindow::onSelectionChanged);
}
void MainWindow::onSelectionChanged()
{
    QString selectedBedShape = ui->bedshapesComboBox->currentText();
    std::string bedshape_str = ui->baseMap[selectedBedShape.toStdString()];
    // std::cout << "Base Name: " << selectedBedShape.toStdString() << ", Base Data: " << bedshape_str << std::endl;
    ui->add_bedshape2(bedshape_str, ui->vtkWidget->m_renderer);
    ui->vtkWidget->m_renderWindow->Render();
}
void MainWindow::setupWidget(QWidget *widget)
{
    QIcon icon("D:/0Learning/Vtk/0805_qt5_vtk/resource/Proslice.ico");
    setWindowIcon(icon);
    // setupWidgetLayoutAndStyle(widget);
    connectSignals(widget);
}