#include "u_MainWindow.h"
#include <Vtk_headers.h>
#include <vtkPolygon.h>
#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolygon.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <sstream>
#include <string>
#include <vtkPoints.h>
#define BEDSHAPE_PATH "D:/0Learning/Vtk/0805_qt5_vtk/resource/bedshape/bedshape.ini"

// 将单个点对字符串转换为 vtkPoints 的点
void InsertPointIntoVtkPoints(vtkSmartPointer<vtkPoints> points, const std::string &pointPair)
{
    double x, y;
    std::string modifiedPair = pointPair;
    std::replace(modifiedPair.begin(), modifiedPair.end(), 'x', ' '); // 将 'x' 替换为空格
    std::istringstream pointStream(modifiedPair);
    pointStream >> x >> y;              // 读取 x 和 y 坐标
    points->InsertNextPoint(x, y, 0.0); // 插入点到 vtkPoints，假设 z 坐标为 0
}

// 从指定路径读取txt文件并将点数据直接插入 vtkPoints
vtkSmartPointer<vtkPoints> ReadBedShapeFromFile(const std::string &filePath)
{
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    std::ifstream file(filePath);
    std::string line;

    if (file.is_open())
    {
        while (std::getline(file, line))
        {
            // 查找包含 "bed_shape" 的行
            if (line.find("bed_shape") != std::string::npos)
            {
                // 提取 "bed_shape = " 后面的部分
                size_t pos = line.find("=");
                if (pos != std::string::npos)
                {
                    std::string pointsString = line.substr(pos + 1);
                    std::istringstream ss(pointsString);
                    std::string pointPair;

                    // 逐个点对解析并插入到 vtkPoints 中
                    while (std::getline(ss, pointPair, ','))
                    {
                        InsertPointIntoVtkPoints(points, pointPair);
                    }
                }
            }
        }
        file.close();
    }
    else
    {
        std::cerr << "无法打开文件: " << filePath << std::endl;
    }

    return points;
}

void add_bedshape(vtkSmartPointer<vtkRenderer> renderer)
{
    std::string filePath = BEDSHAPE_PATH; // 替换为你的文件路径
    vtkSmartPointer<vtkPoints> vtkPoints = ReadBedShapeFromFile(filePath);
    vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
    vtkNew<vtkNamedColors> colors;
    int numPoints = vtkPoints->GetNumberOfPoints();
    polygon->GetPointIds()->SetNumberOfIds(numPoints); // 设置多边形顶点数量
    std::cout << "numPoints is " << numPoints << std::endl;
    for (int i = 0; i < numPoints; ++i)
    {
        polygon->GetPointIds()->SetId(i, i); // 将每个点的 ID 添加到多边形
    }

    // Add the polygon to a list of polygons
    vtkNew<vtkCellArray> polygons;
    polygons->InsertNextCell(polygon);

    // Create a PolyData
    vtkNew<vtkPolyData> polygonPolyData;
    polygonPolyData->SetPoints(vtkPoints);
    polygonPolyData->SetPolys(polygons);

    // Create a mapper and actor
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(polygonPolyData);

    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);
    // actor->GetProperty()->SetEdgeVisibility(true);
    // actor->GetProperty()->SetRepresentationToSurface();
    actor->GetProperty()->SetColor(0.7412, 0.7451, 0.6667); //
    // actor->GetProperty()->SetOpacity(0.5); // 0.5表示50%的透明度
    renderer->AddActor(actor);
}
// 将一组字符串形式的二维点转换为 vtkPoints

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

    add_bedshape(vtkWidget->m_renderer);
}