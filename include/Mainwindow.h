#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <Vtkwidget3D.h>
#include <Qt_headers.h>
#include <Ui/u_MainWindow.h>
#include "RotateWidget.h"
namespace Ui
{
    class MainWindow;
}
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr) : QMainWindow(parent),
                                            ui(new Ui::MainWindow)
    {
        ui->setupUi(this);
        setupWidget(this);
        pRotateWidget = new RotateWidget(this);
    }
    ~MainWindow();

public:
    Ui::MainWindow *ui;
    RotateWidget *pRotateWidget;
private slots:
    // // 导入文件页面
    // void openFileSlot();
    // // 基台选择页面
    // void openPlatSelectSlot();
    // // 自动修复页面
    // void AutoRepairSlot();
    // // 姿态平移页面
    // void MoveSlot();
    // 姿态旋转页面
    void RotateSlot();

private:
    void connectSignals(QWidget *widget);
    void setupWidget(QWidget *widget);
};

#endif // MAINWINDOW_H