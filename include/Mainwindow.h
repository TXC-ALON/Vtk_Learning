#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <Vtkwidget3D.h>
#include <Qt_headers.h>
#include <Ui/u_MainWindow.h>

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
    }
    ~MainWindow();

public:
    Ui::MainWindow *ui;
    void randomize()
    {
        // Randomize the sphere, mapper, and renderer here
        // Placeholder for randomization logic
    }
};

#endif // MAINWINDOW_H