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
    // void openFileSlot();
    // void openPlatSelectSlot();
    // void AutoRepairSlot();
    // void MoveSlot();
    void RotateSlot();

private:
    void connectSignals(QWidget *widget);
    void setupWidget(QWidget *widget);
};

#endif // MAINWINDOW_H