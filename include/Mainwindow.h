#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "Vtkwidget.h"
#include "Qt_headers.h"
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public:
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    VtkWidget *vtkWidget;
    vtkNew<vtkGenericOpenGLRenderWindow> window;

    void setupUi(QMainWindow *mainwindow);
    void randomize()
    {
        // Randomize the sphere, mapper, and renderer here
        // Placeholder for randomization logic
    }
};

#endif // MAINWINDOW_H