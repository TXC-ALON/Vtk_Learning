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
    doubleSpinBox_Type = new QDoubleSpinBox(MainWindow);
    doubleSpinBox_Type->setObjectName(QString::fromUtf8("doubleSpinBox_Type"));
    doubleSpinBox_Type->setMinimumSize(QSize(100, 25));
    doubleSpinBox_Type->setDecimals(0);
    doubleSpinBox_Type->setValue(1);

    dockLayout->addWidget(doubleSpinBox_Type);
    openFileButton = new QPushButton("Open", MainWindow);
    dockLayout->addWidget(openFileButton);
    rotateButton = new QPushButton("Rotate", MainWindow);
    dockLayout->addWidget(rotateButton);
    ShowAxesButton = new QPushButton("ShowAxes", MainWindow);
    dockLayout->addWidget(ShowAxesButton);

    // Vtk 主窗口
    centralWidget = new QWidget(MainWindow);
    centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
    vtkWidget = new VtkWidget3D(centralWidget);
    vtkWidget->setObjectName(QString::fromUtf8("widgetVTK"));
    vtkWidget->setMinimumSize(QSize(1200, 900));
    gridLayout = new QGridLayout(centralWidget);
    gridLayout->setSpacing(0);
    gridLayout->setContentsMargins(11, 11, 11, 11);
    gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
    gridLayout->setContentsMargins(0, 0, 0, 0);
    gridLayout->addWidget(vtkWidget, 1, 0, 2, 1);
    MainWindow->setCentralWidget(centralWidget);
    // 交互
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