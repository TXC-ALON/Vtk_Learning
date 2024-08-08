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

int main(int argc, char *argv[])
{
    stl_vertex test;
    test.x = 1;
    test.y = 1.8;
    test.z = 1.999;
    test.stl_scale(2);
    std::cout << test.x << std::endl;
    std::cout << "Eigen version: "
              << EIGEN_WORLD_VERSION << "."
              << EIGEN_MAJOR_VERSION << "."
              << EIGEN_MINOR_VERSION << std::endl;
    QApplication app(argc, argv);
    MainWindow mainWindow;
    mainWindow.show();
    return app.exec();
}