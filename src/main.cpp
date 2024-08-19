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
#include <JSONHandler.h>

#define TEST_JSON "D:/0Learning/Vtk/0805_qt5_vtk/Test/file.json"

#define OUTPUT_JSON "D:/0Learning/Vtk/0805_qt5_vtk/Test/output.json"

int main(int argc, char *argv[])
{

    JSONHandler jsonHandler;

    // 读取 JSON 文件
    if (jsonHandler.readJsonFile(TEST_JSON))
    {
        qDebug() << "JSON 文件读取成功";
    }
    else
    {
        qDebug() << "读取 JSON 文件失败";
        return -1;
    }

    // 获取嵌套的 street 值
    QJsonValue cityValue = jsonHandler.getValue({"address", "location", "city"});
    qDebug() << "city:" << cityValue.toString();

    // 设置 JSON 值
    jsonHandler.setValue({"address", "location", "city"}, "456 Wonderland Ave");
    QJsonValue cityValue2 = jsonHandler.getValue({"address", "location", "city"});
    foreach (const QString &key, jsonHandler.jsonObject.keys())
    {
        qDebug() << key << "::" << jsonHandler.jsonObject.value(key);
    }
    qDebug() << "city2:" << cityValue2.toString();
    QApplication app(argc, argv);
    MainWindow mainWindow;
    mainWindow.show();
    return app.exec();
}