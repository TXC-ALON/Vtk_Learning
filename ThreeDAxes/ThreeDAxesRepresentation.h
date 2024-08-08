#pragma once

#include "vtkInteractionWidgetsModule.h" // For export macro
#include "vtkWidgetRepresentation.h"

class vtkActor;
class vtkAxesActor;
class vtkPolyDataMapper;
class vtkLineSource;
class vtkSphereSource;
class vtkRegularPolygonSource;
class vtkCellPicker;
class vtkProperty;
class vtkPolyData;
class vtkPoints;
class vtkPolyDataAlgorithm;
class vtkPointHandleRepresentation3D;
class vtkTransform;
class vtkPlane;
class vtkPlanes;
class vtkBox;
class vtkDoubleArray;
class vtkMatrix4x4;
class vtkCellArray;
class vtkTextMapper;
class vtkFollower;
class vtkVectorText;
class vtkImplicitPolyDataDistance;
#include <vtkArrowSource.h>
#include <vtkTransformPolyDataFilter.h>

class ThreeDAxesRepresentation : public vtkWidgetRepresentation
{
public:
    /**
     * Instantiate the class.
     */
    static ThreeDAxesRepresentation *New()
    {
        return new ThreeDAxesRepresentation();
    }

    //@{
    /**
     * Standard methods for the class.
     */
    vtkTypeMacro(ThreeDAxesRepresentation, vtkWidgetRepresentation);
    //@}

    //@{
    /**
     * Set/Get the InsideOut flag. This data member is used in conjunction
     * with the GetPlanes() method. When off, the normals point out of the
     * box. When on, the normals point into the hexahedron.  InsideOut is off
     * by default.
     */
    vtkSetMacro(InsideOut, vtkTypeBool);
    vtkGetMacro(InsideOut, vtkTypeBool);
    vtkBooleanMacro(InsideOut, vtkTypeBool);
    //@}

    /**
     * Retrieve a linear transform characterizing the transformation of the
     * box. Note that the transformation is relative to where PlaceWidget()
     * was initially called. This method modifies the transform provided. The
     * transform can be used to control the position of vtkProp3D's, as well as
     * other transformation operations (e.g., vtkTransformPolyData).
     */
    virtual void GetTransform(vtkTransform *t);

    /**
     * Set the position, scale and orientation of the box widget using the
     * transform specified. Note that the transformation is relative to
     * where PlaceWidget() was initially called (i.e., the original bounding
     * box).
     */
    virtual void SetTransform(vtkTransform *t);

    //@{
    /**
     * Get the handle properties (the little balls are the handles). The
     * properties of the handles, when selected or normal, can be
     * specified.
     */
    vtkGetObjectMacro(HandleProperty, vtkProperty);
    vtkGetObjectMacro(SelectedHandleProperty, vtkProperty);
    //@}

    //@{
    /**
     * Switches handles (the spheres) on or off by manipulating the underlying
     * actor visibility.
     */
    virtual void HandlesOn();
    virtual void HandlesOff();
    //@}

    //@{
    /**
     * These are methods that satisfy vtkWidgetRepresentation's API.
     */
    void PlaceWidget(double bounds[6]) override;
    void BuildRepresentation() override;
    int ComputeInteractionState(int X, int Y, int modify = 0) override;
    void StartWidgetInteraction(double e[2]) override;
    void WidgetInteraction(double e[2]) override;
    void EndWidgetInteraction(double e[2]) override; // 2024.1.2
    double *GetBounds() VTK_SIZEHINT(6) override;
    //@}

    //@{
    /**
     * Methods supporting, and required by, the rendering process.
     */
    void ReleaseGraphicsResources(vtkWindow *) override;
    int RenderOpaqueGeometry(vtkViewport *) override;
    int RenderTranslucentPolygonalGeometry(vtkViewport *) override;
    vtkTypeBool HasTranslucentPolygonalGeometry() override;
    //@}

    // Used to manage the state of the widget
    enum
    {
        Outside = 0,
        RotateYZ,
        RotateXZ,
        RotateXY,
        Translating,
        Rotating,
        MoveX,
        MoveY,
        MoveZ
    };

    /**
     * The interaction state may be set from a widget (e.g., vtkBoxWidget2) or
     * other object. This controls how the interaction with the widget
     * proceeds. Normally this method is used as part of a handshaking
     * process with the widget: First ComputeInteractionState() is invoked that
     * returns a state based on geometric considerations (i.e., cursor near a
     * widget feature), then based on events, the widget may modify this
     * further.
     */
    void SetInteractionState(int state);

    /*
     * Register internal Pickers within PickingManager
     */
    void RegisterPickers() override;

    //@{
    /**
     * Set/Get the x,y,z Direction.
     */
    void SetXDirection(double x[3]);
    void SetYDirection(double x[3]);
    void SetZDirection(double x[3]);

    vtkGetMacro(TranslationAxis, int);
    vtkSetClampMacro(TranslationAxis, int, -1, 2);
    vtkGetVectorMacro(XDirection, double, 3);
    vtkGetVectorMacro(YDirection, double, 3);
    vtkGetVectorMacro(ZDirection, double, 3);

    void SetXTranslationAxisOn()
    {
        this->TranslationAxis = Axis::XAxis;
    }
    void SetYTranslationAxisOn()
    {
        this->TranslationAxis = Axis::YAxis;
    }
    void SetZTranslationAxisOn()
    {
        this->TranslationAxis = Axis::ZAxis;
    }
    void SetTranslationAxisOff()
    {
        this->TranslationAxis = Axis::NONE;
    }
    bool IsTranslationConstrained()
    {
        return this->TranslationAxis != Axis::NONE;
    }
    //@}
    //@}

    //@{
    /**
     * Specify the origin of the widget (in world coordinates). The origin
     * is the point where the widget places itself. Note that rotations and
     * scaling occurs around the origin.
     */
    void SetOrigin(const double o[3]) { this->SetOrigin(o[0], o[1], o[2]); }
    void SetOrigin(double ox, double oy, double oz);
    vtkGetVector3Macro(Origin, double);
    //@}

protected:
    ThreeDAxesRepresentation();
    ~ThreeDAxesRepresentation() override;

    // Manage how the representation appears
    double LastEventPosition[3];
    double LastEventOrientation[4];
    double StartEventOrientation[4];

    // glyphs representing hot spots (e.g., handles)
    vtkActor *Handle; // ��ǰѡ��
    vtkPolyDataMapper *HandleMapper;
    vtkSphereSource *HandleGeometry;

    double Origin[3];
    // axes
    double N[6][3];
    int HighlightHandle(vtkProp *prop); // returns cell id
    int HighlightCircle(vtkProp *prop);
    int HighlightArrow(vtkProp *prop);

    virtual void SizeHandles();

    virtual void PositionHandles();
    virtual void PositionHandles(double *ipts);
    virtual void ComputeNormals();

    // Arrow
    double ArrowLength;
    vtkActor **ArrowActor; // ��ͷ
    vtkPolyDataMapper **ArrowMapper;
    vtkPolyData **ArrowGeometry;

    // circle
    double CircleRadius;
    vtkActor **CircleActor; // Բ��
    vtkPolyDataMapper **CircleMapper;
    vtkRegularPolygonSource **CircleGeometry;

    vtkPoints *HCirclePoints;
    vtkCellArray *HCircleCellArray;
    vtkPolyData *HCircle;
    vtkPolyDataMapper *HCircleMapper;
    vtkActor *HCircleActor; // ����Բ��

    vtkPoints *HCircleAreaPoints;
    vtkCellArray *HCircleAreaCellArray;
    vtkPolyData *HCircleArea;
    vtkPolyDataMapper *HCircleAreaMapper;
    vtkActor *HCircleAreaActor; // ����Բ�����

    vtkVectorText *TextInput;
    vtkPolyDataMapper *TextMapper;
    vtkFollower *TextActor; // 文字

    // 坐标轴
    int TranslationAxis;
    double XDirection[3];
    double YDirection[3];
    double ZDirection[3];
    double ViewDirection[3];

    // Do the picking
    vtkCellPicker *ArrowPicker;
    vtkCellPicker *CirclePicker;
    vtkCellPicker *HandlePicker;
    vtkActor *CurrentHandle;
    int CurrentHexFace;
    vtkCellPicker *LastPicker;

    // StartMove to EndMove
    vtkTransform *Transform;
    // Total Transform
    vtkTransform *TotalTransform;
    // Support GetBounds() method
    vtkBox *BoundingBox;

    // Properties used to control the appearance of selected objects and
    // the manipulator in general.
    // Arrow
    vtkProperty *XArrowProperty;
    vtkProperty *YArrowProperty;
    vtkProperty *ZArrowProperty;
    vtkProperty *FaceProperty;
    vtkProperty *OutlineProperty;
    // Circle
    vtkProperty *YZCircleProperty;
    vtkProperty *XZCircleProperty;
    vtkProperty *XYCircleProperty;
    vtkProperty *ViewCircleProperty;
    // Selected
    vtkProperty *SelectedFaceProperty;
    vtkProperty *SelectedOutlineProperty;
    vtkProperty *SelectedYZCircleProperty;
    vtkProperty *SelectedXZCircleProperty;
    vtkProperty *SelectedXYCircleProperty;
    vtkProperty *SelectedViewCircleProperty;
    // HighlightCircle
    vtkProperty *SelectedHCircleProperty;
    vtkProperty *SelectedHCircleAreaProperty;
    // Handle
    vtkProperty *HandleProperty;
    vtkProperty *SelectedHandleProperty;

    virtual void CreateDefaultProperties();

    // Control the orientation of the normals
    vtkTypeBool InsideOut;

    void GenerateRotateCircle();
    vtkPoints *Points; // used by others as well
    void GenerateOutline(int i, bool hight = false);
    // Helper methods
    virtual void Translate(const double *p1, const double *p2);
    virtual void Rotate(int X, int Y, const double *p0, const double *p1, const double *p2, const double *vpn);
    virtual void TranslateX(const double *p1, const double *p2);
    virtual void TranslateY(const double *p1, const double *p2);
    virtual void TranslateZ(const double *p1, const double *p2);

    void UpdatePose(const double *p1, const double *d1, const double *p2, const double *d2);

private:
    ThreeDAxesRepresentation(const ThreeDAxesRepresentation &) = delete;
    void operator=(const ThreeDAxesRepresentation &) = delete;
};
