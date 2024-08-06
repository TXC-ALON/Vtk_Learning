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

MainWindow::~MainWindow()
{
    if (pRotateWidget != nullptr)
    {
        delete pRotateWidget;
        pRotateWidget = nullptr;
    }
}

void MainWindow::RotateSlot()
{
    std::cout << "RotateSlot" << std::endl;
    // TODO 打开零件旋转页面
    pRotateWidget->setVisible(true);
    pRotateWidget->InitOpenPage();
}

void MainWindow::connectSignals(QWidget *widget)
{
    // 接受拖动事件
    // setAcceptDrops(true);
    // 信号槽
    connect(ui->rotateButton, &QPushButton::clicked, this, &MainWindow::RotateSlot);
}

void MainWindow::setupWidget(QWidget *widget)
{
    // setupWidgetLayoutAndStyle(widget);
    connectSignals(widget);
}