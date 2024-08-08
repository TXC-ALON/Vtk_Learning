#pragma once

#include "vtkAbstractWidget.h"
#include "vtkInteractionWidgetsModule.h" // For export macro

class ThreeDAxesRepresentation;
class vtkHandleWidget;
class ThreeDAxesWidget : public vtkAbstractWidget
{
public:
    /**
     * Instantiate the object.
     */
    static ThreeDAxesWidget *New()
    {
        return new ThreeDAxesWidget;
    }

    //@{
    /**
     * Standard class methods for type information and printing.
     */
    vtkTypeMacro(ThreeDAxesWidget, vtkAbstractWidget);
    //@}

    /**
     * Specify an instance of vtkWidgetRepresentation used to represent this
     * widget in the scene. Note that the representation is a subclass of vtkProp
     * so it can be added to the renderer independent of the widget.
     */
    void SetRepresentation(ThreeDAxesRepresentation *r)
    {
        this->Superclass::SetWidgetRepresentation(reinterpret_cast<vtkWidgetRepresentation *>(r));
    }

    //@{
    /**
     * Control the behavior of the widget (i.e., how it processes
     * events). Translation, rotation, scaling and face movement can all be enabled and
     * disabled. Scaling refers to scaling of the whole widget at once,
     * (default is through right mouse button) while face movement refers to
     * scaling of the widget one face (axis) at a time (default through grabbing
     * one of the representation spherical handles).
     */
    vtkSetMacro(TranslationEnabled, vtkTypeBool);
    vtkGetMacro(TranslationEnabled, vtkTypeBool);
    vtkBooleanMacro(TranslationEnabled, vtkTypeBool);
    vtkSetMacro(RotationEnabled, vtkTypeBool);
    vtkGetMacro(RotationEnabled, vtkTypeBool);
    vtkBooleanMacro(RotationEnabled, vtkTypeBool);
    //@}

    /**
     * Create the default widget representation if one is not set. By default,
     * this is an instance of the vtkBoxRepresentation class.
     */
    void CreateDefaultRepresentation() override;
    ThreeDAxesRepresentation *GetRepresentation();
    /**
     * Override superclasses' SetEnabled() method because the line
     * widget must enable its internal handle widgets.
     */
    void SetEnabled(int enabling) override;

protected:
    ThreeDAxesWidget();
    ~ThreeDAxesWidget() override;
    ThreeDAxesRepresentation *zxWidgetRep;
    // zxRotateRepresentation* WidgetRep;
    //  Manage the state of the widget
    int WidgetState;
    enum _WidgetState
    {
        Start = 0,
        Active
    };

    // These methods handle events
    static void SelectAction(vtkAbstractWidget *);
    static void EndSelectAction(vtkAbstractWidget *);
    static void TranslateAction(vtkAbstractWidget *);
    static void MoveAction(vtkAbstractWidget *);

    // Control whether scaling, rotation, and translation are supported
    vtkTypeBool TranslationEnabled;
    vtkTypeBool RotationEnabled;

private:
    ThreeDAxesWidget(const ThreeDAxesWidget &) = delete;
    void operator=(const ThreeDAxesWidget &) = delete;
};
