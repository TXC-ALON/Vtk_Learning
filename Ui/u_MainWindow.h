#pragma once
#include <Vtkwidget3D.h>
#include <Qt_headers.h>
#include <Vtk_headers.h>

#include <cmath>
#include <cstdlib>
#include <random>
#include <vtkInteractorStyleTrackballCamera.h>
#include <ThreeDAxesCallback.h>
#include <ThreeDAxesRepresentation.h>
#include <ThreeDAxesWidget.h>
#include <VTK_Utils.h>
#include <Toys.h>
#define BEDSHAPES_PATH "D:/0Learning/Vtk/0805_qt5_vtk/resource/bedshape/bedshapes.ini"
class ssCallback : public vtkCommand
{
public:
    static ssCallback *New()
    {
        return new ssCallback;
    }

    virtual void Execute(vtkObject *caller, unsigned long eventId, void *callData) override
    {
        std::cout << eventId << std::endl;
    }
};
class XKeyPressCallback : public vtkCommand
{
public:
    static XKeyPressCallback *New()
    {
        return new XKeyPressCallback;
    }

    virtual void Execute(vtkObject *caller, unsigned long eventId, void *callData) override
    {

        double bound = Get_BedShape_Bound();

        if (eventId == vtkCommand::KeyPressEvent)
        {
            vtkRenderWindowInteractor *iren = static_cast<vtkRenderWindowInteractor *>(caller);
            std::string key = iren->GetKeySym();
            std::cout << key << std::endl;
            double Position_Base = 4 * bound;
            if (iren->GetKeyCode() == '0')
            {
                std::cout << "Reset" << std::endl;
                renderer->GetActiveCamera()->SetParallelProjection(0);
                renderer->GetActiveCamera()->SetPosition(Position_Base, 0, 0); // 相机位置，这里假设相机在z轴正方向
                renderer->GetActiveCamera()->SetFocalPoint(0, 0, 0);           // 焦点位置，通常设置为场景的中心
                renderer->GetActiveCamera()->SetViewUp(0, 0, 1);               // 视图向上向量，这里假设向上为y轴正方向
                renderer->GetActiveCamera()->Azimuth(45);                      // 方位角，45度
                renderer->GetActiveCamera()->Elevation(30);                    // 仰角，30度
                // iren->GetRenderWindow()->Render();                   // 重新渲染窗口以显示重置效果
                iren->GetRenderWindow()->Render(); // 重新渲染窗口以显示效果
                window->Render();
            }
            else if (iren->GetKeyCode() == '7')
            {
                renderer->GetActiveCamera()->SetParallelProjection(1);
                renderer->GetActiveCamera()->SetParallelScale(bound);
                renderer->GetActiveCamera()->SetPosition(Position_Base, 0, 0); // 相机位置，这里假设相机在x轴正方向
                renderer->GetActiveCamera()->SetFocalPoint(0, 0, 0);           // 焦点位置，通常设置为场景的中心
                renderer->GetActiveCamera()->SetViewUp(0, 0, 1);               // 视图向上向量，这里假设向上为z轴正方向
                iren->GetRenderWindow()->Render();                             // 重新渲染窗口以显示效果
            }

            else if (iren->GetKeyCode() == '8')
            {
                renderer->GetActiveCamera()->SetParallelProjection(1);
                renderer->GetActiveCamera()->SetParallelScale(bound);
                renderer->GetActiveCamera()->SetPosition(0, Position_Base, 0); // 相机位置，这里假设相机在y轴正方向
                renderer->GetActiveCamera()->SetFocalPoint(0, 0, 0);           // 焦点位置，通常设置为场景的中心
                renderer->GetActiveCamera()->SetViewUp(0, 0, 1);               // 视图向上向量，这里假设向上为z轴正方向
                iren->GetRenderWindow()->Render();                             // 重新渲染窗口以显示效果
            }

            else if (iren->GetKeyCode() == '9')
            {
                renderer->GetActiveCamera()->SetParallelProjection(1);
                renderer->GetActiveCamera()->SetParallelScale(bound);
                renderer->GetActiveCamera()->SetPosition(0, 0, Position_Base); // 相机位置，这里假设相机在z轴正方向
                renderer->GetActiveCamera()->SetFocalPoint(0, 0, 0);           // 焦点位置，通常设置为场景的中心
                renderer->GetActiveCamera()->SetViewUp(0, 1, 0);               // 视图向上向量，这里假设向上为y轴正方向
                iren->GetRenderWindow()->Render();                             // 重新渲染窗口以显示效果
            }
        }
    }
    vtkRenderer *renderer;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> window;
    double Get_BedShape_Bound()
    {
        double res_bound = 0;
        // 从渲染器中获取所有演员
        vtkCollectionSimpleIterator sit;
        vtkSmartPointer<vtkPropCollection> props = renderer->GetActors();
        props->InitTraversal(sit);
        vtkProp *prop;
        while ((prop = props->GetNextProp(sit)))
        {
            vtkSmartPointer<vtkActor> actor = vtkActor::SafeDownCast(prop);
            if (actor)
            {
                // std::cout << "Actor name: " << actor->GetObjectName() << std::endl;
                if (actor->GetObjectName() == "BedShape")
                {
                    for (int i = 0; i < 6; i++)
                    {
                        double bounds[6];
                        actor->GetBounds(bounds);
                        std::cout << "Actor bounds: ("
                                  << bounds[0] << ", " << bounds[1] << ") "
                                  << "(" << bounds[2] << ", " << bounds[3] << ") "
                                  << "(" << bounds[4] << ", " << bounds[5] << ")" << std::endl;
                        res_bound = std::max(fabs(bounds[i]), res_bound);
                    }
                }
            }
        }

        std::cout << "res_bound :" << res_bound << std::endl;
        res_bound += 10;
        return res_bound;
    }
};

class Ui_MainWindow
{
public:
    Ui_MainWindow()
    {
        this->rotateWidget = ThreeDAxesWidget::New();
        this->rotateWidget->EnabledOff();
        this->callback = ThreeDAxesCallback::New();
    }
    QDockWidget *controlDock;
    QWidget *centralWidget;

    QGridLayout *gridLayout;
    VtkWidget3D *vtkWidget;
    QPushButton *openFileButton;
    QPushButton *rotateButton;
    QComboBox *bedshapesComboBox;
    QPushButton *ShowAxesButton;
    QDoubleSpinBox *doubleSpinBox_Type;
    vtkSmartPointer<vtkAxesActor> Axes;
    vtkSmartPointer<vtkActor> modelAxes; // Line_Axes
    vtkSmartPointer<vtkActor> Bedshape = nullptr;
    vtkSmartPointer<vtkActor> Bedshape_Polylines = nullptr;

    vtkSmartPointer<vtkOrientationMarkerWidget> widget;
    vtkSmartPointer<XKeyPressCallback> keycallback;
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor;
    ThreeDAxesCallback *callback = nullptr;
    ThreeDAxesWidget *rotateWidget;
    std::map<std::string, std::string> baseMap;
    double Bedshape_Bound = 10;

public:
    void read_bedshapes(std::string filepath);
    void setupUi(QMainWindow *MainWindow);
    void add_bedshape(vtkSmartPointer<vtkRenderer> renderer);
    void add_bedshape2(std::string bedshape_str, vtkSmartPointer<vtkRenderer> renderer);
    void add_polylines(std::vector<Segment> segments, vtkSmartPointer<vtkRenderer> renderer);
    void debug_qiu(std::vector<double> position, std::vector<double> color, vtkSmartPointer<vtkRenderer> renderer);
    void Add_Line_Axes(vtkSmartPointer<vtkRenderer> renderer, double scaleFactor);
};
namespace Ui
{
    class MainWindow : public Ui_MainWindow
    {
    };
} // namespace Ui
