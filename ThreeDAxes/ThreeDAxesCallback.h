
#include <vtkCamera.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>
#include <vtkProperty.h>
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkNew.h>
#include <vtkArrowSource.h>
#include <vtkTransform.h>
#include <vtkAxes.h>
#include <vtkCylinderSource.h>
#include <ThreeDAxesWidget.h>
#include <ThreeDAxesRepresentation.h>

class ThreeDAxesCallback : public vtkCommand
{
public:
    static ThreeDAxesCallback *New()
    {
        return new ThreeDAxesCallback;
    }
    void SetDefaultMatrix(vtkMatrix4x4 *matrix)
    {
        if (m_matrix != nullptr)
        {
            vtkMatrix4x4::Multiply4x4(m_matrix, matrix, m_matrix);
        }
    }
    vtkSmartPointer<vtkProp3D> m_prop3D;
    void SetProp3D(vtkSmartPointer<vtkProp3D> prop)
    {
        m_prop3D = prop;

        vtkMatrix4x4 *martix = prop->GetUserMatrix();
        if (m_matrix != nullptr && nullptr != martix)
        {
            vtkMatrix4x4::Multiply4x4(m_matrix, martix, m_matrix);
        }
        else
        {
            vtkMatrix4x4 *m = vtkMatrix4x4::New();
            prop->SetUserMatrix(m);
        }
    }
    void SetFollowActors(std::vector<vtkActor *> actors)
    {
        m_followActors.clear();
        for (int i = 0; i < actors.size(); i++)
        {
            m_followActors.push_back(actors[i]);
        }
        return;
    }
    vtkSmartPointer<vtkPolyData> m_polyData;
    void SetPolyData(vtkSmartPointer<vtkPolyData> polydata) { m_polyData = polydata; }
    vtkPolyData *GetPolyData() { return m_polyData; }
    virtual void Execute(vtkObject *caller, unsigned long eventId, void *callData) override
    {
        vtkSmartPointer<ThreeDAxesWidget> boxWidget = dynamic_cast<ThreeDAxesWidget *>(caller);
        vtkNew<vtkTransform> t;
        dynamic_cast<ThreeDAxesRepresentation *>(boxWidget->GetRepresentation())->GetTransform(t);
        vtkMatrix4x4 *m = vtkMatrix4x4::New();
        t->GetMatrix(m);
        if (m_matrix != nullptr)
        {
            vtkMatrix4x4::Multiply4x4(m, m_matrix, m);
        }

        this->m_prop3D->SetUserMatrix(m);
        for (int i = 0; i < m_followActors.size(); i++)
        {
            m_followActors[i]->SetUserMatrix(m);
        }
    }
    ThreeDAxesCallback() {}

private:
    vtkMatrix4x4 *m_matrix = vtkMatrix4x4::New();
    std::vector<vtkActor *> m_followActors;
};
