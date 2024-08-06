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
    QDockWidget *controlDock;
    VtkWidget vtkWidget;
    void setupUi(QMainWindow *mainwindow);
    void randomize()
    {
        // Randomize the sphere, mapper, and renderer here
        // Placeholder for randomization logic
    }
};

#endif // MAINWINDOW_H