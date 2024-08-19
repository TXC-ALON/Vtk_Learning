#include <QApplication>
#include <QMainWindow>
#include <QDockWidget>
#include <QLabel>
#include <QVBoxLayout>
#include <QPushButton>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkSphereSource.h>
#include <vtkDataSetMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <QPointer>
#include <functional>
#include <random>
#include <vtkProperty.h>
#include "Mainwindow.h"
#include <Eigen/Dense>
#include <stl.h>
#include <JsonHandler.h>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    MainWindow mainWindow;
    mainWindow.show();
    return app.exec();
}