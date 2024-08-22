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
#include <vtkTriangleFilter.h>

#include "Qt_headers.h"
#define BEDSHAPE_PATH "D:/0Learning/Vtk/0805_qt5_vtk/resource/bedshape/bedshape.ini"

// 将单个点对字符串转换为 vtkPoints 的点
void InsertPointIntoVtkPoints(vtkSmartPointer<vtkPoints> points, const std::string &pointPair)
{
    double x, y;
    std::string modifiedPair = pointPair;
    std::replace(modifiedPair.begin(), modifiedPair.end(), 'x', ' '); // 将 'x' 替换为空格
    std::istringstream pointStream(modifiedPair);
    pointStream >> x >> y; // 读取 x 和 y 坐标
    // std::cout << x << " " << y << std::endl;
    for (vtkIdType i = 0; i < points->GetNumberOfPoints(); i++)
    {
        double point[3];
        points->GetPoint(i, point); // 获取第i个点的坐标
        // std::cout << "check Point " << i << ": (" << point[0] << ", " << point[1] << ", " << point[2] << ")" << std::endl;
        if (fabs(point[0] - x) < 0.0001 && fabs(point[1] - y) < 0.0001)
        {
            return; // 不插入重复的点
        }
    }
    points->InsertNextPoint(x, y, 0.0); // 插入点到 vtkPoints，假设 z 坐标为 0
}

// 从指定路径读取txt文件并将点数据直接插入 vtkPoints
vtkSmartPointer<vtkPoints> ReadBedShapeFromFile(const std::string &filePath, std::vector<Point> &boost_points)
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
        // for (vtkIdType i = 0; i < points->GetNumberOfPoints(); i++)
        // {
        //     double point[3];
        //     points->GetPoint(i, point); // 获取第i个点的坐标
        //     std::cout << "Point " << i << ": (" << point[0] << ", " << point[1] << ", " << point[2] << ")" << std::endl;
        // }
        file.close();
    }
    else
    {
        std::cerr << "无法打开文件: " << filePath << std::endl;
    }

    for (vtkIdType i = 0; i < points->GetNumberOfPoints(); i++)
    {
        double point[3];
        points->GetPoint(i, point); // 获取第i个点的坐标
        Point tempPoint;
        tempPoint = Point(point[0], point[1]);
        boost_points.push_back(tempPoint);
    }
    return points;
}
vtkSmartPointer<vtkPoints> ReadBedShapeFromStdString(const std::string BedShapeString, std::vector<Point> &boost_points)
{
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    std::string pointsString = BedShapeString;
    std::istringstream ss(pointsString);
    std::string pointPair;

    // 逐个点对解析并插入到 vtkPoints 中
    while (std::getline(ss, pointPair, ','))
    {
        InsertPointIntoVtkPoints(points, pointPair);
    }

    // for (vtkIdType i = 0; i < points->GetNumberOfPoints(); i++)
    // {
    //     double point[3];
    //     points->GetPoint(i, point); // 获取第i个点的坐标
    //     std::cout << "Point " << i << ": (" << point[0] << ", " << point[1] << ", " << point[2] << ")" << std::endl;
    // }

    for (vtkIdType i = 0; i < points->GetNumberOfPoints(); i++)
    {
        double point[3];
        points->GetPoint(i, point); // 获取第i个点的坐标
        Point tempPoint;
        tempPoint = Point(point[0], point[1]);
        boost_points.push_back(tempPoint);
    }
    return points;
}

void add_bedshape_polylines(double *bounds, vtkSmartPointer<vtkRenderer> renderer)
{
    // 获取边界盒
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    // XY平面的间距
    double xSpacing = 10.0; // 例如，每隔1个单位添加一条水平线
    double ySpacing = 10.0; // 每隔1个单位添加一条垂直线

    // 创建点和线
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    double x_grid_min = bounds[0];
    double x_grid_max = bounds[1];
    double y_grid_min = bounds[2];
    double y_grid_max = bounds[3];
    x_grid_min = std::floor(x_grid_min / xSpacing) * xSpacing;
    x_grid_max = std::ceil(x_grid_max / xSpacing) * xSpacing;

    y_grid_min = std::floor(y_grid_min / ySpacing) * ySpacing;
    y_grid_max = std::ceil(y_grid_max / ySpacing) * ySpacing;
    int numPointsX = static_cast<int>((x_grid_max - x_grid_min) / xSpacing);
    int numPointsY = static_cast<int>((y_grid_max - y_grid_min) / ySpacing);
    std::cout << "x_grid_min is " << x_grid_min << std::endl;
    std::cout << "x_grid_max is " << x_grid_max << std::endl;
    std::cout << "y_grid_min is " << y_grid_min << std::endl;
    std::cout << "y_grid_max is " << y_grid_max << std::endl;

    std::cout << "numPointsX is " << numPointsX << std::endl;
    std::cout << "numPointsY is " << numPointsY << std::endl;
    // 添加水平线
    for (int i = 0; i <= numPointsX; i++)
    {
        points->InsertNextPoint(x_grid_min + xSpacing * i, y_grid_min, 0.0); // 假设在XY平面
        points->InsertNextPoint(x_grid_min + xSpacing * i, y_grid_max, 0.0);
        lines->InsertNextCell(2);
        lines->InsertCellPoint(points->GetNumberOfPoints() - 2);
        lines->InsertCellPoint(points->GetNumberOfPoints() - 1);
    }

    for (int i = 0; i <= numPointsY; i++)
    {
        points->InsertNextPoint(x_grid_min, y_grid_min + ySpacing * i, 0.0); // 假设在XY平面
        points->InsertNextPoint(x_grid_max, y_grid_min + ySpacing * i, 0.0);
        lines->InsertNextCell(2);
        lines->InsertCellPoint(points->GetNumberOfPoints() - 2);
        lines->InsertCellPoint(points->GetNumberOfPoints() - 1);
    }
    // for (vtkIdType i = 0; i < points->GetNumberOfPoints(); i++)
    // {
    //     double point[3];
    //     points->GetPoint(i, point); // 获取第i个点的坐标
    //     std::cout << "Point " << i << ": (" << point[0] << ", " << point[1] << ", " << point[2] << ")" << std::endl;
    // }
    // 将点和线添加到PolyData
    polyData->SetPoints(points);
    polyData->SetLines(lines);

    // 创建Mapper和Actor
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    // 设置颜色等属性
    actor->GetProperty()->SetColor(0, 0, 0);
    renderer->AddActor(actor);
}
void Ui_MainWindow::add_polylines(std::vector<Segment> segments, vtkSmartPointer<vtkRenderer> renderer)
{
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    for (const auto &segment : segments)
    {
        const Point &p1 = segment.first;              // 获取线段的起点
        const Point &p2 = segment.second;             // 获取线段的终点
        points->InsertNextPoint(p1.x(), p1.y(), 0.0); // 假设在XY平面
        points->InsertNextPoint(p2.x(), p2.y(), 0.0); // 假设在XY平面
        lines->InsertNextCell(2);
        lines->InsertCellPoint(points->GetNumberOfPoints() - 2);
        lines->InsertCellPoint(points->GetNumberOfPoints() - 1);
        // std::cout << "Segment from (" << p1.x() << ", " << p1.y() << ") to (" << p2.x() << ", " << p2.y() << ")" << std::endl;
    }
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    // 将点和线添加到PolyData
    polyData->SetPoints(points);
    polyData->SetLines(lines);

    // 创建Mapper和Actor
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);
    if (Bedshape_Polylines == nullptr)
    {
        Bedshape_Polylines = vtkSmartPointer<vtkActor>::New();
    }
    Bedshape_Polylines->SetMapper(mapper);

    // 设置颜色等属性
    Bedshape_Polylines->GetProperty()->SetColor(0, 0, 0);
    debug_qiu({10, 20, 30}, {1, 0, 0}, renderer);
    debug_qiu({10, 20, 0}, {0, 0, 1}, renderer);

    // debug_qiu({-95.7551, -70, 0}, {1, 0, 0}, renderer);
    // debug_qiu({-110.713, -70, 0}, {0, 1, 0}, renderer);
    // debug_qiu({-116.619, -70, 0}, {0, 0, 1}, renderer);
    // debug_qiu({116.619, -70, 0}, {1, 1, 0}, renderer);
    // debug_qiu({95.7551, -70, 0}, {0, 1, 1}, renderer);
    // debug_qiu({110.713, -70, 0}, {1, 0, 1}, renderer);

    renderer->AddActor(Bedshape_Polylines);
}
void Ui_MainWindow::debug_qiu(std::vector<double> position, std::vector<double> color, vtkSmartPointer<vtkRenderer> renderer)
{
    vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
    sphereSource->SetRadius(1.0); // 设置球的半径
    sphereSource->Update();       // 更新球体源

    // 创建映射器
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(sphereSource->GetOutputPort());

    // 创建actor
    vtkSmartPointer<vtkActor> sphereActor = vtkSmartPointer<vtkActor>::New();
    sphereActor->SetMapper(mapper);
    sphereActor->GetProperty()->SetColor(color[0], color[1], color[2]); //
    // 设置小球的位置
    sphereActor->SetPosition(position[0], position[1], position[2]);
    renderer->AddActor(sphereActor);
}
void Ui_MainWindow::add_bedshape2(std::string bedshape_str, vtkSmartPointer<vtkRenderer> renderer)
{

    std::string filePath = BEDSHAPE_PATH; // 替换为你的文件路径
    std::vector<Point> boost_points;
    vtkSmartPointer<vtkPoints> vtkPoints = ReadBedShapeFromStdString(bedshape_str, boost_points);
    // std::cout << "add_bedshape2" << std::endl;
    // std::cout << "boost_points size is " << boost_points.size() << std::endl;

    vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
    vtkNew<vtkNamedColors> colors;
    int numPoints = vtkPoints->GetNumberOfPoints();
    polygon->GetPointIds()->SetNumberOfIds(numPoints); // 设置多边形顶点数量
    std::cout << "BedShape numPoints is " << numPoints << std::endl;
    for (int i = 0; i < numPoints; ++i)
    {
        polygon->GetPointIds()->SetId(i, i); // 将每个点的 ID 添加到多边形
    }

    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
    cells->InsertNextCell(polygon);

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(vtkPoints);
    polyData->SetPolys(cells);

    vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
    triangleFilter->SetInputData(polyData);
    triangleFilter->Update();

    // Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(triangleFilter->GetOutputPort());
    if (Bedshape == nullptr)
    {
        Bedshape = vtkSmartPointer<vtkActor>::New();
        Bedshape->SetObjectName("BedShape");
    }
    Bedshape->SetMapper(mapper);
    // Bedshape->GetProperty()->SetEdgeVisibility(true);
    //  actor->GetProperty()->SetRepresentationToSurface();
    Bedshape->GetProperty()->SetColor(0.7412, 0.7451, 0.6667); //
    // actor->GetProperty()->SetOpacity(0.5); // 0.5表示50%的透明度
    renderer->AddActor(Bedshape);

    double *bounds = Bedshape->GetBounds();
    std::cout << "Bounds: ["
              << bounds[0] << ", " << bounds[1] << ", "
              << bounds[2] << ", " << bounds[3] << ", "
              << bounds[4] << ", " << bounds[5] << "]"
              << std::endl;
    Bedshape_Bound = 10;
    for (int i = 0; i < 6; i++)
    {
        Bedshape_Bound = std::max(fabs(bounds[i]), Bedshape_Bound);
    }
    std::cout << "Bedshape_Bound " << Bedshape_Bound * 1.5 << std::endl;
    Add_Line_Axes(vtkWidget->m_renderer, Bedshape_Bound * 1.5);

    if (1)
    {
        Boost_Polygon temp_poly = Boost_Polygon(boost_points);
        std::vector<Segment> segments = temp_poly.generateGridLines(10);
        // // 遍历线段向量并打印每个线段的端点
        // for (const auto &segment : segments)
        // {
        //     const Point &p1 = segment.first;  // 获取线段的起点
        //     const Point &p2 = segment.second; // 获取线段的终点

        //     std::cout << "Segment from (" << p1.x() << ", " << p1.y() << ") to (" << p2.x() << ", " << p2.y() << ")" << std::endl;
        // }
        add_polylines(segments, renderer);
    }
}
void Ui_MainWindow::add_bedshape(vtkSmartPointer<vtkRenderer> renderer)
{
    std::string filePath = BEDSHAPE_PATH; // 替换为你的文件路径
    std::vector<Point> boost_points;
    vtkSmartPointer<vtkPoints> vtkPoints = ReadBedShapeFromFile(filePath, boost_points);
    std::cout << "add_bedshape" << std::endl;
    std::cout << "boost_points size is " << boost_points.size() << std::endl;
    Boost_Polygon temp_poly = Boost_Polygon(boost_points);
    std::vector<Segment> segments = temp_poly.generateGridLines(20);
    // 遍历线段向量并打印每个线段的端点
    // for (const auto &segment : segments)
    // {
    //     const Point &p1 = segment.first;  // 获取线段的起点
    //     const Point &p2 = segment.second; // 获取线段的终点

    //     std::cout << "Segment from (" << p1.x() << ", " << p1.y() << ") to (" << p2.x() << ", " << p2.y() << ")" << std::endl;
    // }
    vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
    vtkNew<vtkNamedColors> colors;
    int numPoints = vtkPoints->GetNumberOfPoints();
    polygon->GetPointIds()->SetNumberOfIds(numPoints); // 设置多边形顶点数量
    std::cout << "numPoints is " << numPoints << std::endl;
    for (int i = 0; i < numPoints; ++i)
    {
        polygon->GetPointIds()->SetId(i, i); // 将每个点的 ID 添加到多边形
    }

    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
    cells->InsertNextCell(polygon);

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(vtkPoints);
    polyData->SetPolys(cells);

    vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
    triangleFilter->SetInputData(polyData);
    triangleFilter->Update();

    // Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(triangleFilter->GetOutputPort());

    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);
    // actor->GetProperty()->SetEdgeVisibility(true);
    // actor->GetProperty()->SetRepresentationToSurface();
    actor->GetProperty()->SetColor(0.7412, 0.7451, 0.6667); //
    // actor->GetProperty()->SetOpacity(0.5); // 0.5表示50%的透明度
    renderer->AddActor(actor);

    double *bounds = actor->GetBounds();
    add_polylines(segments, renderer);
}
// 将一组字符串形式的二维点转换为 vtkPoints
void Ui_MainWindow::read_bedshapes(std::string filepath)
{
    std::ifstream file(filepath); // 替换为你的文件路径

    if (file.is_open())
    {
        std::string line;
        while (std::getline(file, line))
        {
            std::istringstream iss(line);
            std::string baseName, baseData;
            size_t pos = line.find(" = ");
            if (pos != std::string::npos)
            {
                std::string key = line.substr(0, pos);
                std::string value = line.substr(pos + 3);
                baseMap[key] = value;
            }
        }

        file.close();
    }
    else
    {
        std::cerr << "Unable to open file";
    }

    // // 打印map中的内容，以检查是否正确读取
    // for (const auto &pair : baseMap)
    // {
    //     std::cout << "Base Name: " << pair.first << ", Base Data: " << pair.second << std::endl;
    // }
    if (!baseMap.empty())
    {
        bedshapesComboBox->clear();

        // 将map中的键添加到下拉框中
        for (const auto &pair : baseMap)
        {
            QString qStr = QString::fromStdString(pair.first);
            bedshapesComboBox->addItem(qStr);
        }
        bedshapesComboBox->setCurrentIndex(2);
    }
}

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

    bedshapesComboBox = new QComboBox(MainWindow);
    bedshapesComboBox->setObjectName(QString::fromUtf8("bedshapesComboBox"));
    bedshapesComboBox->setMinimumSize(QSize(300, 0));
    bedshapesComboBox->setEditable(false);
    bedshapesComboBox->setIconSize(QSize(16, 16));
    dockLayout->addWidget(bedshapesComboBox);
    read_bedshapes(BEDSHAPES_PATH);
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

    QString selectedBedShape = bedshapesComboBox->currentText();
    std::string bedshape_str = baseMap[selectedBedShape.toStdString()];
    add_bedshape2(bedshape_str, vtkWidget->m_renderer);

    // add_bedshape(vtkWidget->m_renderer);
}

void Ui_MainWindow::Add_Line_Axes(vtkSmartPointer<vtkRenderer> renderer, double scaleFactor)
{
    vtkNew<vtkAxes> modelAxesSource;
    modelAxesSource->SetScaleFactor(scaleFactor);
    modelAxesSource->SetOrigin(0, 0, 0);

    vtkNew<vtkTubeFilter> tubeFilter;
    tubeFilter->SetInputConnection(modelAxesSource->GetOutputPort());
    tubeFilter->SetRadius(0.5);
    tubeFilter->SetNumberOfSides(50);

    vtkNew<vtkPolyDataMapper> modelAxesMapper;
    modelAxesMapper->SetInputConnection(tubeFilter->GetOutputPort());
    if (modelAxes == nullptr)
    {
        modelAxes = vtkSmartPointer<vtkActor>::New();
    }

    modelAxes->SetMapper(modelAxesMapper);
    renderer->AddActor(modelAxes);
}