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

#include <QApplication>
#include <QDockWidget>
#include <QGridLayout>
#include <QLabel>
#include <QMainWindow>
#include <QPointer>
#include <QPushButton>
#include <QVBoxLayout>

#include <cmath>
#include <cstdlib>
#include <random>
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
    setupUi(this);
}

void MainWindow::setupUi(QMainWindow *mainWindow)
{
    // Render area.
    vtkWidget->p_vtkWidget = new QVTKOpenGLNativeWidget();
    mainWindow->setCentralWidget(vtkWidget->p_vtkWidget);
    // VTK part.
    vtkWidget->p_vtkWidget->setRenderWindow(window.Get());
}

MainWindow::~MainWindow()
{
    delete vtkWidget;
}