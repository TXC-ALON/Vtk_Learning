#include "ThreeDAxesWidget.h"
#include "ThreeDAxesRepresentation.h"
#include "vtkCallbackCommand.h"
#include "vtkCommand.h"
#include "vtkEvent.h"
#include "vtkEventData.h"
#include "vtkObjectFactory.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkWidgetCallbackMapper.h"
#include "vtkWidgetEvent.h"
#include "vtkWidgetEventTranslator.h"
#include <vtkAutoInit.h>

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

#include "ThreeDAxesWidget.h"
#include "ThreeDAxesRepresentation.h"

//----------------------------------------------------------------------------
ThreeDAxesWidget::ThreeDAxesWidget()
{
    this->WidgetState = ThreeDAxesWidget::Start;
    this->ManagesCursor = 1;

    this->TranslationEnabled = true;
    this->RotationEnabled = true;

    // Define widget events
    this->CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonPressEvent, vtkEvent::NoModifier, 0,
                                            0, nullptr, vtkWidgetEvent::Select, this, ThreeDAxesWidget::SelectAction);
    this->CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonReleaseEvent, vtkEvent::NoModifier,
                                            0, 0, nullptr, vtkWidgetEvent::EndSelect, this, ThreeDAxesWidget::EndSelectAction);
    this->CallbackMapper->SetCallbackMethod(vtkCommand::MouseMoveEvent, vtkWidgetEvent::Move, this, ThreeDAxesWidget::MoveAction);
}

//----------------------------------------------------------------------------
ThreeDAxesWidget::~ThreeDAxesWidget()
{
}

//----------------------------------------------------------------------------
void ThreeDAxesWidget::SetEnabled(int enabling)
{
    int enabled = this->Enabled;

    // We do this step first because it sets the CurrentRenderer
    this->Superclass::SetEnabled(enabling);
}

//----------------------------------------------------------------------
void ThreeDAxesWidget::SelectAction(vtkAbstractWidget *w)
{
    // We are in a static method, cast to ourself
    ThreeDAxesWidget *self = reinterpret_cast<ThreeDAxesWidget *>(w);

    // Get the event position
    int X = self->Interactor->GetEventPosition()[0];
    int Y = self->Interactor->GetEventPosition()[1];

    // Okay, make sure that the pick is in the current renderer
    if (!self->CurrentRenderer || !self->CurrentRenderer->IsInViewport(X, Y))
    {
        self->WidgetState = ThreeDAxesWidget::Start;
        return;
    }

    // Begin the widget interaction which has the side effect of setting the
    // interaction state.
    double e[2];
    e[0] = static_cast<double>(X);
    e[1] = static_cast<double>(Y);
    self->WidgetRep->StartWidgetInteraction(e);
    int interactionState = self->WidgetRep->GetInteractionState();
    if (interactionState == ThreeDAxesRepresentation::Outside)
    {
        return;
    }

    // Test for states that involve face or handle picking here so
    // selection highlighting doesn't happen if that interaction is disabled.
    // Non-handle-grabbing transformations are tested in the "Action" methods.

    // Rotation
    if ((interactionState == ThreeDAxesRepresentation::RotateYZ ||
         interactionState == ThreeDAxesRepresentation::RotateXZ ||
         interactionState == ThreeDAxesRepresentation::RotateXY ||
         interactionState == ThreeDAxesRepresentation::Rotating) &&
        self->RotationEnabled == 0)
    {
        return;
    }
    // Translation x y z
    if ((interactionState == ThreeDAxesRepresentation::MoveX ||
         interactionState == ThreeDAxesRepresentation::MoveY ||
         interactionState == ThreeDAxesRepresentation::MoveZ) &&
        self->TranslationEnabled == 0)
    {
        return;
    }
    // Translation
    if (interactionState == ThreeDAxesRepresentation::Translating && self->TranslationEnabled == 0)
    {
        return;
    }

    // We are definitely selected
    self->WidgetState = ThreeDAxesWidget::Active;
    self->GrabFocus(self->EventCallbackCommand);

    // The SetInteractionState has the side effect of highlighting the widget
    reinterpret_cast<ThreeDAxesRepresentation *>(self->WidgetRep)->SetInteractionState(interactionState);

    // start the interaction
    self->EventCallbackCommand->SetAbortFlag(1);
    self->StartInteraction();
    self->InvokeEvent(vtkCommand::StartInteractionEvent, nullptr);
    self->Render();
}

//----------------------------------------------------------------------
void ThreeDAxesWidget::TranslateAction(vtkAbstractWidget *w)
{
    // We are in a static method, cast to ourself
    ThreeDAxesWidget *self = reinterpret_cast<ThreeDAxesWidget *>(w);

    if (self->TranslationEnabled == 0)
    {
        return;
    }

    // Get the event position
    int X = self->Interactor->GetEventPosition()[0];
    int Y = self->Interactor->GetEventPosition()[1];

    // Okay, make sure that the pick is in the current renderer
    if (!self->CurrentRenderer || !self->CurrentRenderer->IsInViewport(X, Y))
    {
        self->WidgetState = ThreeDAxesWidget::Start;
        return;
    }

    // Begin the widget interaction which has the side effect of setting the
    // interaction state.
    double e[2];
    e[0] = static_cast<double>(X);
    e[1] = static_cast<double>(Y);
    self->WidgetRep->StartWidgetInteraction(e);
    int interactionState = self->WidgetRep->GetInteractionState();
    if (interactionState == ThreeDAxesRepresentation::Outside)
    {
        return;
    }

    // We are definitely selected
    self->WidgetState = ThreeDAxesWidget::Active;
    self->GrabFocus(self->EventCallbackCommand);
    reinterpret_cast<ThreeDAxesRepresentation *>(self->WidgetRep)
        ->SetInteractionState(ThreeDAxesRepresentation::Translating);

    // start the interaction
    self->EventCallbackCommand->SetAbortFlag(1);
    self->StartInteraction();
    self->InvokeEvent(vtkCommand::StartInteractionEvent, nullptr);
    self->Render();
}

//----------------------------------------------------------------------
void ThreeDAxesWidget::MoveAction(vtkAbstractWidget *w)
{
    ThreeDAxesWidget *self = reinterpret_cast<ThreeDAxesWidget *>(w);

    // See whether we're active
    if (self->WidgetState == ThreeDAxesWidget::Start)
    {
        return;
    }

    // compute some info we need for all cases
    int X = self->Interactor->GetEventPosition()[0];
    int Y = self->Interactor->GetEventPosition()[1];

    // Okay, adjust the representation
    double e[2];
    e[0] = static_cast<double>(X);
    e[1] = static_cast<double>(Y);
    self->WidgetRep->WidgetInteraction(e);

    // moving something
    self->EventCallbackCommand->SetAbortFlag(1);
    self->InvokeEvent(vtkCommand::InteractionEvent, nullptr);
    self->Render();
}

//----------------------------------------------------------------------
void ThreeDAxesWidget::EndSelectAction(vtkAbstractWidget *w)
{
    ThreeDAxesWidget *self = reinterpret_cast<ThreeDAxesWidget *>(w);
    if (self->WidgetState == ThreeDAxesWidget::Start)
    {
        return;
    }
    int X = self->Interactor->GetEventPosition()[0];
    int Y = self->Interactor->GetEventPosition()[1];
    double eventPos[2];
    eventPos[0] = static_cast<double>(X);
    eventPos[1] = static_cast<double>(Y);
    self->WidgetRep->EndWidgetInteraction(eventPos);

    // Return state to not active
    self->WidgetState = ThreeDAxesWidget::Start;
    reinterpret_cast<ThreeDAxesRepresentation *>(self->WidgetRep)
        ->SetInteractionState(ThreeDAxesRepresentation::Outside);
    self->ReleaseFocus();

    self->EventCallbackCommand->SetAbortFlag(1);
    self->EndInteraction();
    self->InvokeEvent(vtkCommand::EndInteractionEvent, nullptr);
    self->Render();
}

//----------------------------------------------------------------------
void ThreeDAxesWidget::CreateDefaultRepresentation()
{
    if (!this->WidgetRep)
    {
        this->WidgetRep = ThreeDAxesRepresentation::New();
    }
}
ThreeDAxesRepresentation *ThreeDAxesWidget::GetRepresentation()
{

    return reinterpret_cast<ThreeDAxesRepresentation *>(this->WidgetRep);
}
