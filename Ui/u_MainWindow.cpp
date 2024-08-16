#include "u_MainWindow.h"

void Ui_MainWindow::setupUi(QMainWindow *MainWindow)
{
    MainWindow->resize(1200, 900);
    // Control area setup
    controlDock = new QDockWidget(MainWindow);
    MainWindow->addDockWidget(Qt::LeftDockWidgetArea, controlDock);
    QLabel *controlDockTitle = new QLabel("Control Dock", MainWindow);
    controlDockTitle->setMargin(20);
    controlDock->setTitleBarWidget(controlDockTitle);

    QVBoxLayout *dockLayout = new QVBoxLayout();
    QWidget *layoutContainer = new QWidget(MainWindow);
    layoutContainer->setLayout(dockLayout);
    controlDock->setWidget(layoutContainer);

    QPushButton *randomizeButton = new QPushButton("Randomize", MainWindow);
    dockLayout->addWidget(randomizeButton);
    openFileButton = new QPushButton("Open", MainWindow);
    dockLayout->addWidget(openFileButton);
    rotateButton = new QPushButton("Rotate", MainWindow);
    dockLayout->addWidget(rotateButton);
    ShowAxesButton = new QPushButton("ShowAxes", MainWindow);
    dockLayout->addWidget(ShowAxesButton);

    centralWidget = new QWidget(MainWindow);
    centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
    // Vtk
    // 窗口
    vtkWidget = new VtkWidget3D(centralWidget);
    vtkWidget->setObjectName(QString::fromUtf8("widgetVTK"));
    vtkWidget->setMinimumSize(QSize(1200, 900));

    MainWindow->setCentralWidget(centralWidget);

    vtkNew<vtkInteractorStyleTrackballCamera> interactorStyle;
    vtkWidget->interactor()->SetInteractorStyle(interactorStyle);
    vtkNew<XKeyPressCallback> keycallback;
    keycallback->renderer = vtkWidget->m_renderer;
    keycallback->window = vtkWidget->m_renderWindow;
    vtkWidget->interactor()->AddObserver(vtkCommand::KeyPressEvent, keycallback);
    // 添加坐标轴
    // Add_Arrow_Axes(vtkWidget->m_renderer, 15);
    Add_Line_Axes(vtkWidget->m_renderer, 15);
}