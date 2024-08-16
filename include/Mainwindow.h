#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <Vtkwidget3D.h>
#include <Qt_headers.h>
#include <P3DS_headers.h>
#include "../Ui/u_MainWindow.h"
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
        pRotateWidget = new RotateWidget(this);
        ui->setupUi(this);
        setupWidget(this);
        recent_skein_directory = "";
    }
    ~MainWindow();

public:
    Ui::MainWindow *ui;
    RotateWidget *pRotateWidget;
    QString recent_skein_directory;
    QString recent_out_directory;

private slots:
    void openFileSlot();
    void ShowAxes();
    // void openPlatSelectSlot();
    // void AutoRepairSlot();
    // void MoveSlot();
    void RotateSlot();
    void receiveData(QString str)
    {
        // in Mainwindow
        std::cout << str.toStdString() << std::endl;
    }
    void Render_Update()
    {
        std::cout << "Render_Update" << std::endl;
        ui->vtkWidget->m_renderWindow->Render();
    }

private:
    void connectSignals(QWidget *widget);
    void setupWidget(QWidget *widget);

    // bool loadFile(const QString &filePath);
    // bool loadFileList(const QStringList files);
    // bool loadDirectory(const QString &dir);
};

#endif // MAINWINDOW_H