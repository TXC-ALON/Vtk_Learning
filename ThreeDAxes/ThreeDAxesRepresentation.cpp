#include "ThreeDAxesRepresentation.h"

#include "vtkActor.h"
#include "vtkAssemblyPath.h"
#include "vtkBox.h"
#include "vtkCallbackCommand.h"
#include "vtkCamera.h"
#include "vtkCellArray.h"
#include "vtkCellPicker.h"
#include "vtkDoubleArray.h"
#include "vtkEventData.h"
#include "vtkInteractorObserver.h"
#include "vtkMath.h"
#include "vtkObjectFactory.h"
#include "vtkPickingManager.h"
#include "vtkPlane.h"
#include "vtkPlanes.h"
#include "vtkPolyData.h"
#include "vtkPolyDataMapper.h"
#include "vtkProperty.h"
#include "vtkQuaternion.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkSphereSource.h"
#include <vtkRegularPolygonSource.h>
#include "vtkTransform.h"
#include "vtkVectorOperators.h"
#include "vtkWindow.h"
#include "vtkArcSource.h"
#include "vtkVectorText.h"
#include "vtkFollower.h"
#include "vtkImplicitPolyDataDistance.h"
#include "vtkPolyLine.h"

#include <assert.h>

#define CIRCLE_RESOLUTION 360

//----------------------------------------------------------------------------
ThreeDAxesRepresentation::ThreeDAxesRepresentation()
{
    // The initial state
    this->InteractionState = ThreeDAxesRepresentation::Outside;

    // Handle size is in pixels for this widget
    this->HandleSize = 5.0;

    // Control orientation of normals
    this->InsideOut = 0;

    // Set up the initial properties
    this->CreateDefaultProperties();
    // Construct initial points
    this->Points = vtkPoints::New(VTK_DOUBLE);
    this->Points->SetNumberOfPoints(21); // 8 corners; 6 faces; 1 center

    this->TotalTransform = vtkTransform::New();

    // Construct connectivity for the faces. These are used to perform
    // the picking.

    // TestArrow
    vtkNew<vtkArrowSource> arrowSource;
    vtkNew<vtkPolyDataMapper> arrowMapper;
    arrowMapper->SetInputConnection(arrowSource->GetOutputPort());

    // Create the Circles
    for (int i = 0; i < 3; i++)
    {
        this->Origin[0] = 0.0;
        this->XDirection[i] = 0.0;
        this->YDirection[i] = 0.0;
        this->ZDirection[i] = 0.0;
        this->ViewDirection[i] = 0.0;
    }
    XDirection[0] = YDirection[1] = ZDirection[2] = 1.0;
    const double units0 = -660.;
    const double units02 = -1000.;
    this->ArrowActor = new vtkActor *[3];
    this->ArrowMapper = new vtkPolyDataMapper *[3];
    this->ArrowGeometry = new vtkPolyData *[3];
    // Create the outline
    this->GenerateOutline(0);
    this->GenerateOutline(1);
    this->GenerateOutline(2);
    for (int i = 0; i < 3; i++)
    {
        this->ArrowGeometry[i] = vtkPolyData::New();
        this->ArrowMapper[i] = vtkPolyDataMapper::New();
        this->ArrowMapper[i]->SetInputData(this->ArrowGeometry[i]);
        this->ArrowMapper[i]->SetResolveCoincidentTopologyToPolygonOffset();
        this->ArrowMapper[i]->SetRelativeCoincidentTopologyLineOffsetParameters(0, units0);
        this->ArrowMapper[i]->SetRelativeCoincidentTopologyPolygonOffsetParameters(0, units0);
        this->ArrowMapper[i]->SetRelativeCoincidentTopologyPointOffsetParameter(units0);
        this->ArrowActor[i] = vtkActor::New();
        this->ArrowActor[i]->SetMapper(this->ArrowMapper[i]);

        if (i == 0)
            this->ArrowActor[i]->SetProperty(this->XArrowProperty);
        else if (i == 1)
            this->ArrowActor[i]->SetProperty(this->YArrowProperty);
        else if (i == 2)
            this->ArrowActor[i]->SetProperty(this->ZArrowProperty);
    }

    this->CircleRadius = 5.;
    this->CircleActor = new vtkActor *[4];
    this->CircleMapper = new vtkPolyDataMapper *[4];
    this->CircleGeometry = new vtkRegularPolygonSource *[4];
    for (int i = 0; i < 4; i++)
    {

        this->CircleGeometry[i] = vtkRegularPolygonSource::New();
        this->CircleGeometry[i]->GeneratePolygonOff(); // Uncomment this line to generate only the outline of the circle
        this->CircleGeometry[i]->SetNumberOfSides(CIRCLE_RESOLUTION);
        this->CircleGeometry[i]->SetRadius(this->CircleRadius);
        this->CircleMapper[i] = vtkPolyDataMapper::New();
        this->CircleMapper[i]->SetInputConnection(this->CircleGeometry[i]->GetOutputPort());
        this->CircleActor[i] = vtkActor::New();
        this->CircleActor[i]->SetMapper(this->CircleMapper[i]);

        if (i == 0)
            this->CircleActor[i]->SetProperty(this->YZCircleProperty);
        else if (i == 1)
            this->CircleActor[i]->SetProperty(this->XZCircleProperty);
        else if (i == 2)
            this->CircleActor[i]->SetProperty(this->XYCircleProperty);
        else if (i == 3)
            this->CircleActor[i]->SetProperty(this->ViewCircleProperty);
    }

    this->HCircleAreaPoints = vtkPoints::New();
    this->HCircleAreaCellArray = vtkCellArray::New();
    this->HCircleAreaCellArray->AllocateEstimate(1, CIRCLE_RESOLUTION);
    this->HCircleArea = vtkPolyData::New();
    this->HCircleArea->SetPoints(this->HCircleAreaPoints);
    this->HCircleArea->SetPolys(this->HCircleAreaCellArray);
    this->HCircleAreaMapper = vtkPolyDataMapper::New();
    this->HCircleAreaMapper->SetInputData(this->HCircleArea);
    this->HCircleAreaMapper->SetResolveCoincidentTopologyToPolygonOffset();
    this->HCircleAreaMapper->SetRelativeCoincidentTopologyLineOffsetParameters(-4.0, -4.0);
    this->HCircleAreaActor = vtkActor::New();
    this->HCircleAreaActor->SetMapper(this->HCircleAreaMapper);
    this->HCircleAreaActor->VisibilityOff();
    this->HCircleAreaActor->SetProperty(this->SelectedHCircleAreaProperty);

    this->HCirclePoints = vtkPoints::New();
    this->HCircleCellArray = vtkCellArray::New();
    this->HCircleCellArray->AllocateEstimate(1, CIRCLE_RESOLUTION + 1);
    this->HCircle = vtkPolyData::New();
    this->HCircle->SetPoints(this->HCirclePoints);
    this->HCircle->SetLines(this->HCircleCellArray);
    this->HCircleMapper = vtkPolyDataMapper::New();
    this->HCircleMapper->SetInputData(this->HCircle);
    this->HCircleMapper->SetResolveCoincidentTopologyToPolygonOffset();
    this->HCircleMapper->SetRelativeCoincidentTopologyLineOffsetParameters(-4.0, -4.0);
    this->HCircleActor = vtkActor::New();
    this->HCircleActor->SetMapper(this->HCircleMapper);
    this->HCircleActor->VisibilityOff();
    this->HCircleActor->SetProperty(this->SelectedHCircleProperty);

    this->TextInput = vtkVectorText::New();
    this->TextInput->SetText("0");
    this->TextMapper = vtkPolyDataMapper::New();
    this->TextMapper->SetInputConnection(this->TextInput->GetOutputPort());
    this->TextActor = vtkFollower::New();
    this->TextActor->SetMapper(this->TextMapper);

    // Create the handles
    this->HandleGeometry = vtkSphereSource::New();
    this->HandleGeometry->SetOutputPointsPrecision(vtkAlgorithm::DOUBLE_PRECISION);
    this->HandleGeometry->SetThetaResolution(16);
    this->HandleGeometry->SetPhiResolution(8);
    this->HandleGeometry->SetRadius(1.0);
    this->HandleMapper = vtkPolyDataMapper::New();

    this->HandleMapper->SetInputConnection(this->HandleGeometry->GetOutputPort());

    this->HandleMapper->SetResolveCoincidentTopologyToPolygonOffset();
    this->HandleMapper->SetRelativeCoincidentTopologyLineOffsetParameters(0, units02);
    this->HandleMapper->SetRelativeCoincidentTopologyPolygonOffsetParameters(0, units02);
    this->HandleMapper->SetRelativeCoincidentTopologyPointOffsetParameter(units02);

    this->Handle = vtkActor::New();
    this->Handle->SetMapper(this->HandleMapper);
    this->Handle->SetProperty(this->HandleProperty);
    // this->Handle->SetDepth(10);
    //  Define the point coordinates
    double bounds[6];
    bounds[0] = -0.5;
    bounds[1] = 0.5;
    bounds[2] = -0.5;
    bounds[3] = 0.5;
    bounds[4] = -0.5;
    bounds[5] = 0.5;
    // Points 8-14 are down by PositionHandles();
    this->BoundingBox = vtkBox::New();
    this->PlaceWidget(bounds);
    // Manage the picking stuff
    this->ArrowPicker = vtkCellPicker::New();
    this->ArrowPicker->SetTolerance(0.001);
    for (int i = 0; i < 3; i++)
    {
        this->ArrowPicker->AddPickList(this->ArrowActor[i]);
    }
    this->ArrowPicker->PickFromListOn();

    // Manage the picking stuff
    this->CirclePicker = vtkCellPicker::New();
    this->CirclePicker->SetTolerance(0.001);
    for (int i = 0; i < 4; i++)
    {
        this->CirclePicker->AddPickList(this->CircleActor[i]);
    }
    this->CirclePicker->PickFromListOn();

    this->HandlePicker = vtkCellPicker::New();
    this->HandlePicker->SetTolerance(0.01);
    this->HandlePicker->AddPickList(this->Handle);
    this->HandlePicker->PickFromListOn();

    this->CurrentHandle = nullptr;

    // Internal data members for performance
    this->Transform = vtkTransform::New();
}

//----------------------------------------------------------------------------
ThreeDAxesRepresentation::~ThreeDAxesRepresentation()
{

    for (int i = 0; i < 4; i++)
    {
        this->ArrowGeometry[i]->Delete();
        this->ArrowMapper[i]->Delete();
        this->ArrowActor[i]->Delete();
    }
    delete[] this->ArrowActor;
    delete[] this->ArrowMapper;
    delete[] this->ArrowGeometry;

    for (int i = 0; i < 4; i++)
    {
        this->CircleGeometry[i]->Delete();
        this->CircleMapper[i]->Delete();
        this->CircleActor[i]->Delete();
    }
    delete[] this->CircleActor;
    delete[] this->CircleMapper;
    delete[] this->CircleGeometry;

    this->HCircleAreaPoints->Delete();
    this->HCircleAreaCellArray->Delete();
    this->HCircleArea->Delete();
    this->HCircleAreaMapper->Delete();
    this->HCircleAreaActor->Delete();

    this->HCirclePoints->Delete();
    this->HCircleCellArray->Delete();
    this->HCircle->Delete();
    this->HCircleMapper->Delete();
    this->HCircleActor->Delete();

    this->TextInput->Delete();
    this->TextMapper->Delete();
    this->TextActor->Delete();

    this->HandleGeometry->Delete();
    this->HandleMapper->Delete();
    this->Handle->Delete();

    this->ArrowPicker->Delete();
    this->CirclePicker->Delete();
    this->HandlePicker->Delete();

    this->Transform->Delete();
    this->TotalTransform->Delete();
    this->BoundingBox->Delete();

    this->YZCircleProperty->Delete();
    this->XZCircleProperty->Delete();
    this->XYCircleProperty->Delete();
    this->ViewCircleProperty->Delete();
    this->SelectedHCircleProperty->Delete();
    this->SelectedHCircleAreaProperty->Delete();

    this->SelectedYZCircleProperty->Delete();
    this->SelectedXZCircleProperty->Delete();
    this->SelectedXYCircleProperty->Delete();
    this->SelectedViewCircleProperty->Delete();

    this->HandleProperty->Delete();
    this->SelectedHandleProperty->Delete();
}

//----------------------------------------------------------------------
void ThreeDAxesRepresentation::StartWidgetInteraction(double e[2])
{
    // Store the start position
    this->StartEventPosition[0] = e[0];
    this->StartEventPosition[1] = e[1];
    this->StartEventPosition[2] = 0.0;

    // Store the start position
    this->LastEventPosition[0] = e[0];
    this->LastEventPosition[1] = e[1];
    this->LastEventPosition[2] = 0.0;

    this->ComputeInteractionState(static_cast<int>(e[0]), static_cast<int>(e[1]), 0);

    // HCircle
    this->HCirclePoints->Reset();
    this->HCircleCellArray->Reset();
    this->HCirclePoints->Modified();

    this->HCircleAreaPoints->Reset();
    this->HCircleAreaCellArray->Reset();
    this->HCircleAreaPoints->Modified();

    char string[512];
    snprintf(string, sizeof(string), "(%1.1f)", 0.);
    this->TextInput->SetText(string);
    this->TextActor->SetCamera(this->Renderer->GetActiveCamera());
}

//----------------------------------------------------------------------
void ThreeDAxesRepresentation::WidgetInteraction(double e[2])
{
    // Convert events to appropriate coordinate systems
    vtkCamera *camera = this->Renderer->GetActiveCamera();
    if (!camera)
    {
        return;
    }
    double focalPoint[4], pickPoint[4], startPickPoint[4], startMovePoint[4];
    double z, vpn[3];
    camera->GetViewPlaneNormal(vpn);
    vpn[0] = -vpn[0];
    vpn[1] = -vpn[1];
    vpn[2] = -vpn[2];

    // Compute the two points defining the motion vector
    double pos[3];
    if (this->LastPicker == this->CirclePicker)
    {
        this->CirclePicker->GetPickPosition(pos);
    }
    else if (this->LastPicker == this->ArrowPicker)
    {
        this->ArrowPicker->GetPickPosition(pos);
    }
    else
    {
        this->HandlePicker->GetPickPosition(pos);
    }
    vtkInteractorObserver::ComputeWorldToDisplay(this->Renderer, pos[0], pos[1], pos[2], focalPoint);
    z = focalPoint[2];
    vtkInteractorObserver::ComputeDisplayToWorld(
        this->Renderer, this->StartEventPosition[0], this->StartEventPosition[1], z, startPickPoint);
    vtkInteractorObserver::ComputeDisplayToWorld(
        this->Renderer, this->LastEventPosition[0], this->LastEventPosition[1], z, startMovePoint);
    vtkInteractorObserver::ComputeDisplayToWorld(this->Renderer, e[0], e[1], z, pickPoint);

    /*double sPt[3] = { startPickPoint[0], startPickPoint[1], startPickPoint[2] };
    double mPt[3] = { startMovePoint[0], startMovePoint[1], startMovePoint[2] };
    double ePt[3] = { pickPoint[0], pickPoint[1], pickPoint[2] };*/

    double sPt[3] = {this->StartEventPosition[0], this->StartEventPosition[1], 0.};
    double mPt[3] = {this->LastEventPosition[0], this->LastEventPosition[1], 0.};
    double ePt[3] = {e[0], e[1], 0.};

    double p0[3], p1[3], p2[3];
    double dist2;
    double polyLineCoords[3], polyLineWeights[CIRCLE_RESOLUTION];

    double focalPoint2[4], pickPoint2[4], prevPickPoint2[4];
    vtkInteractorObserver::ComputeDisplayToWorld(
        this->Renderer, this->LastEventPosition[0], this->LastEventPosition[1], z, prevPickPoint2);
    vtkInteractorObserver::ComputeDisplayToWorld(this->Renderer, e[0], e[1], z, pickPoint2);

    // Process the motion
    if (this->InteractionState == ThreeDAxesRepresentation::RotateYZ)
    {
        // std::cout << "--RotateYZ" << std::endl;
        this->CircleRadius = this->InitialLength / 2.;

        vtkPoints *pts = this->CircleGeometry[0]->GetOutput()->GetPoints();
        vtkIdType npts = pts->GetNumberOfPoints();

        vtkPolyLine *polyLine = vtkPolyLine::New();
        polyLine->GetPointIds()->SetNumberOfIds(npts);
        polyLine->GetPoints()->SetNumberOfPoints(npts);
        for (vtkIdType id = 0; id < npts; id++)
        {
            polyLine->GetPointIds()->SetId(id, id);
            double pt[3], tmpFoclaPoint[4];
            pts->GetPoint(id, pt);
            vtkInteractorObserver::ComputeWorldToDisplay(this->Renderer, pt[0], pt[1], pt[2], tmpFoclaPoint);
            polyLine->GetPoints()->SetPoint(id, tmpFoclaPoint[0], tmpFoclaPoint[1], 0.);
        }
        int subId0, subId1, subId2;
        polyLine->EvaluatePosition(&sPt[0], &p0[0], subId0, &polyLineCoords[0], dist2, &polyLineWeights[0]);
        pts->GetPoint(subId0, p0);
        polyLine->EvaluatePosition(&mPt[0], &p1[0], subId1, &polyLineCoords[0], dist2, &polyLineWeights[0]);
        pts->GetPoint(subId1, p1);
        polyLine->EvaluatePosition(&ePt[0], &p2[0], subId2, &polyLineCoords[0], dist2, &polyLineWeights[0]);
        pts->GetPoint(subId2, p2);
        polyLine->Delete();

        if (subId0 != subId2 && subId1 != subId2)
        {
            this->Rotate(static_cast<int>(e[0]), static_cast<int>(e[1]), p0, p1, p2, this->XDirection);
        }
    }
    else if (this->InteractionState == ThreeDAxesRepresentation::RotateXZ)
    {
        // std::cout << "--RotateXZ" << std::endl;
        this->CircleRadius = this->InitialLength / 2.;
        vtkPoints *pts = this->CircleGeometry[1]->GetOutput()->GetPoints();
        vtkIdType npts = pts->GetNumberOfPoints();

        vtkPolyLine *polyLine = vtkPolyLine::New();
        polyLine->GetPointIds()->SetNumberOfIds(npts);
        polyLine->GetPoints()->SetNumberOfPoints(npts);
        for (vtkIdType id = 0; id < npts; id++)
        {
            polyLine->GetPointIds()->SetId(id, id);
            double pt[3], tmpFoclaPoint[4];
            pts->GetPoint(id, pt);
            vtkInteractorObserver::ComputeWorldToDisplay(this->Renderer, pt[0], pt[1], pt[2], tmpFoclaPoint);
            polyLine->GetPoints()->SetPoint(id, tmpFoclaPoint[0], tmpFoclaPoint[1], 0.);
        }
        int subId0, subId1, subId2;
        polyLine->EvaluatePosition(&sPt[0], &p0[0], subId0, &polyLineCoords[0], dist2, &polyLineWeights[0]);
        pts->GetPoint(subId0, p0);
        polyLine->EvaluatePosition(&mPt[0], &p1[0], subId1, &polyLineCoords[0], dist2, &polyLineWeights[0]);
        pts->GetPoint(subId1, p1);
        polyLine->EvaluatePosition(&ePt[0], &p2[0], subId2, &polyLineCoords[0], dist2, &polyLineWeights[0]);
        pts->GetPoint(subId2, p2);
        polyLine->Delete();

        if (subId0 != subId2 && subId1 != subId2)
        {
            this->Rotate(static_cast<int>(e[0]), static_cast<int>(e[1]), p0, p1, p2, this->YDirection);
        }
    }
    else if (this->InteractionState == ThreeDAxesRepresentation::RotateXY)
    {
        // std::cout << "--RotateXY" << std::endl;

        this->CircleRadius = this->InitialLength / 2.;
        vtkPoints *pts = this->CircleGeometry[2]->GetOutput()->GetPoints();
        vtkIdType npts = pts->GetNumberOfPoints();

        vtkPolyLine *polyLine = vtkPolyLine::New();
        polyLine->GetPointIds()->SetNumberOfIds(npts);
        polyLine->GetPoints()->SetNumberOfPoints(npts);
        for (vtkIdType id = 0; id < npts; id++)
        {
            polyLine->GetPointIds()->SetId(id, id);
            double pt[3], tmpFoclaPoint[4];
            pts->GetPoint(id, pt);
            vtkInteractorObserver::ComputeWorldToDisplay(this->Renderer, pt[0], pt[1], pt[2], tmpFoclaPoint);
            polyLine->GetPoints()->SetPoint(id, tmpFoclaPoint[0], tmpFoclaPoint[1], 0.);
        }
        int subId0, subId1, subId2;
        polyLine->EvaluatePosition(&sPt[0], &p0[0], subId0, &polyLineCoords[0], dist2, &polyLineWeights[0]);
        pts->GetPoint(subId0, p0);
        polyLine->EvaluatePosition(&mPt[0], &p1[0], subId1, &polyLineCoords[0], dist2, &polyLineWeights[0]);
        pts->GetPoint(subId1, p1);
        polyLine->EvaluatePosition(&ePt[0], &p2[0], subId2, &polyLineCoords[0], dist2, &polyLineWeights[0]);
        pts->GetPoint(subId2, p2);
        polyLine->Delete();

        if (subId0 != subId2 && subId1 != subId2)
        {
            this->Rotate(static_cast<int>(e[0]), static_cast<int>(e[1]), p0, p1, p2, this->ZDirection);
        }
    }
    else if (this->InteractionState == ThreeDAxesRepresentation::Translating)
    {
        // std::cout << "--Translating" << std::endl;

        this->Translate(startMovePoint, pickPoint);
    }
    else if (this->InteractionState == ThreeDAxesRepresentation::Rotating)
    {
        // std::cout << "--Rotating" << std::endl;
        this->CircleRadius = 1.5 * this->InitialLength / 2.;
        vtkPoints *pts = this->CircleGeometry[3]->GetOutput()->GetPoints();
        vtkIdType npts = pts->GetNumberOfPoints();

        vtkPolyLine *polyLine = vtkPolyLine::New();
        polyLine->GetPointIds()->SetNumberOfIds(npts);
        polyLine->GetPoints()->SetNumberOfPoints(npts);
        for (vtkIdType id = 0; id < npts; id++)
        {
            polyLine->GetPointIds()->SetId(id, id);
            double pt[3], tmpFoclaPoint[4];
            pts->GetPoint(id, pt);
            vtkInteractorObserver::ComputeWorldToDisplay(this->Renderer, pt[0], pt[1], pt[2], tmpFoclaPoint);
            polyLine->GetPoints()->SetPoint(id, tmpFoclaPoint[0], tmpFoclaPoint[1], 0.);
        }
        int subId0, subId1, subId2;
        polyLine->EvaluatePosition(&sPt[0], &p0[0], subId0, &polyLineCoords[0], dist2, &polyLineWeights[0]);
        pts->GetPoint(subId0, p0);
        polyLine->EvaluatePosition(&mPt[0], &p1[0], subId1, &polyLineCoords[0], dist2, &polyLineWeights[0]);
        pts->GetPoint(subId1, p1);
        polyLine->EvaluatePosition(&ePt[0], &p2[0], subId2, &polyLineCoords[0], dist2, &polyLineWeights[0]);
        pts->GetPoint(subId2, p2);
        polyLine->Delete();

        if (subId0 != subId2 && subId1 != subId2)
        {
            this->Rotate(static_cast<int>(e[0]), static_cast<int>(e[1]), p0, p1, p2, vpn);
        }
    }

    // Process the motion
    else if (this->InteractionState == ThreeDAxesRepresentation::MoveX)
    {
        this->TranslateX(prevPickPoint2, pickPoint2);
    }
    else if (this->InteractionState == ThreeDAxesRepresentation::MoveY)
    {
        this->TranslateY(prevPickPoint2, pickPoint2);
    }
    else if (this->InteractionState == ThreeDAxesRepresentation::MoveZ)
    {
        this->TranslateZ(prevPickPoint2, pickPoint2);
    }

    // Store the start positio
    this->LastEventPosition[0] = e[0];
    this->LastEventPosition[1] = e[1];
    this->LastEventPosition[2] = 0.0;
}

//----------------------------------------------------------------------
void ThreeDAxesRepresentation::EndWidgetInteraction(double vtkNotUsed(e)[2])
{
    // std::cout << "EndWidgetInteraction" << std::endl;
    //  Have to play games here because of the "pipelined" nature of the
    //  transformations.
    this->GetTransform(this->TempTransform);
    this->TotalTransform->SetMatrix(this->TempTransform->GetMatrix());
    vtkMatrix4x4 *m = vtkMatrix4x4::New();
    TotalTransform->GetMatrix(m);
    // Reset the current transformations
    this->Transform->Identity();
}

void ThreeDAxesRepresentation::GenerateRotateCircle()
{
    double center[3] = {this->Origin[0], this->Origin[1], this->Origin[2]};
    // std::cout << "center: " << center[0] << "," << center[1] << "," << center[2] << std::endl;
    for (int i = 0; i < 4; i++)
    {
        this->CircleGeometry[i]->SetRadius(this->InitialLength / 2.);
        this->CircleGeometry[i]->SetCenter(center);
        if (i == 0)
            this->CircleGeometry[i]->SetNormal(this->XDirection);
        else if (i == 1)
            this->CircleGeometry[i]->SetNormal(this->YDirection);
        else if (i == 2)
            this->CircleGeometry[i]->SetNormal(this->ZDirection);
        else
        {
            if (nullptr != this->Renderer)
            {
                vtkCamera *camera = this->Renderer->GetActiveCamera();
                if (nullptr != camera)
                {
                    double vpn[3];
                    camera->GetViewPlaneNormal(vpn);
                    this->CircleGeometry[i]->SetNormal(vpn);
                    this->CircleGeometry[i]->SetRadius(1.5 * this->InitialLength / 2.);
                }
            }
        }
        this->CircleGeometry[i]->Modified();
    }
}

//----------------------------------------------------------------------------
// Loop through all points and translate them
void ThreeDAxesRepresentation::Translate(const double *p1, const double *p2)
{
    // std::cout << "Translate" << std::endl;
    double *pts = static_cast<vtkDoubleArray *>(this->Points->GetData())->GetPointer(0);
    double v[3] = {0, 0, 0};

    v[0] = p2[0] - p1[0];
    v[1] = p2[1] - p1[1];
    v[2] = p2[2] - p1[2];

    // Move the corners
    for (int i = 0; i < 8; i++)
    {
        *pts++ += v[0];
        *pts++ += v[1];
        *pts++ += v[2];
    }

    // Move the origin
    this->Origin[0] += v[0];
    this->Origin[1] += v[1];
    this->Origin[2] += v[2];

    this->PositionHandles();
    this->GenerateRotateCircle();

    this->HandleGeometry->SetCenter(this->Origin);
    this->HandleGeometry->Modified();
    this->Transform->Translate(v);
}
//----------------------------------------------------------------------------
void ThreeDAxesRepresentation::Rotate(
    int X, int Y, const double *p0, const double *p1, const double *p2, const double *vpn)
{

    // std::cout << "Rotate" << std::endl;

    double v0[3], v1[3], v2[3], v[3]; // vector of motion
    double axis[3];                   // axis of rotation
    double theta;                     // rotation angle

    vtkMath::Subtract(p0, this->Origin, v0);
    vtkMath::Subtract(p1, this->Origin, v1);
    vtkMath::Subtract(p2, this->Origin, v2);
    vtkMath::Normalize(v0);
    vtkMath::Normalize(v1);
    vtkMath::Normalize(v2);
    vtkMath::Subtract(p2, p1, v);
    vtkMath::Normalize(v);

    double angle0 = acos(vtkMath::Dot(v1, v2));

    double tempAxis[3];
    vtkMath::Cross(v1, v, tempAxis);

    axis[0] = vpn[0];
    axis[1] = vpn[1];
    axis[2] = vpn[2];
    vtkMath::Normalize(tempAxis);
    vtkMath::Normalize(axis);
    double angle = acos(vtkMath::Dot(tempAxis, axis));
    if (vtkMath::DegreesFromRadians(angle) > 90 || vtkMath::DegreesFromRadians(angle) < -90)
    {
        axis[0] = -axis[0];
        axis[1] = -axis[1];
        axis[2] = -axis[2];
    }
    this->Transform->Translate(this->Origin);
    this->Transform->RotateWXYZ(vtkMath::DegreesFromRadians(angle0), axis);
    this->Transform->Translate(-this->Origin[0], -this->Origin[1], -this->Origin[2]);

    double rotateAngle;
    rotateAngle = acos(vtkMath::Dot(v0, v2)); //

    vtkMath::Subtract(p2, p0, v);
    vtkMath::Normalize(v);
    vtkMath::Cross(v0, v, tempAxis);
    vtkMath::Normalize(tempAxis);
    vtkMath::Cross(v0, tempAxis, v2);
    vtkMath::Normalize(v2);

    constexpr double stepAngle = 2.0 * vtkMath::Pi() / 360;
    int numDivs = static_cast<int>(fabs(rotateAngle) / stepAngle) + 1;
    double p[3], normal[3];

    // Create the arc
    vtkIdType pid, pid2;
    this->HCirclePoints->Reset();
    this->HCircleCellArray->Reset();
    this->HCircleCellArray->InsertNextCell(0);
    pid = this->HCirclePoints->InsertNextPoint(p0);
    this->HCircleCellArray->InsertCellPoint(pid);

    this->HCircleAreaPoints->Reset();
    this->HCircleAreaCellArray->Reset();
    this->HCircleAreaCellArray->InsertNextCell(0);
    pid2 = this->HCircleAreaPoints->InsertNextPoint(p0);
    this->HCircleAreaCellArray->InsertCellPoint(pid2);

    this->HCircleAreaCellArray->InsertNextCell(numDivs + 2);

    for (int id = 0; id <= numDivs; id++)
    {
        theta = id * stepAngle;
        for (int i = 0; i < 3; i++)
        {
            normal[i] = (v0[i]) * cos(theta) - v2[i] * sin(theta);
            p[i] = this->Origin[i] + this->CircleRadius * normal[i];
        }
        pid = this->HCirclePoints->InsertNextPoint(p);
        this->HCircleCellArray->InsertCellPoint(pid);

        pid2 = this->HCircleAreaPoints->InsertNextPoint(p);
        this->HCircleAreaCellArray->InsertCellPoint(pid2);
    }

    pid = this->HCirclePoints->InsertNextPoint(this->Origin);
    this->HCircleCellArray->InsertCellPoint(pid);
    this->HCircleCellArray->InsertCellPoint(0);
    this->HCircleCellArray->UpdateCellCount(this->HCirclePoints->GetNumberOfPoints() + 1);
    this->HCirclePoints->Modified();

    pid2 = this->HCircleAreaPoints->InsertNextPoint(Origin);
    this->HCircleAreaCellArray->InsertCellPoint(pid2);
    this->HCircleAreaPoints->Modified();

    {
        double pos[3];
        const int npoints = this->HCirclePoints->GetNumberOfPoints();
        this->HCirclePoints->GetPoint(npoints / 2, pos);

        char string[512];
        snprintf(string, sizeof(string), "(%1.1f)", vtkMath::DegreesFromRadians(rotateAngle));

        this->TextInput->SetText(string);
        this->TextActor->SetCamera(this->Renderer->GetActiveCamera());
        this->TextActor->SetPosition(pos);
        this->TextActor->SetScale(this->CircleRadius / 10.0, this->CircleRadius / 10.0, this->CircleRadius / 10.0);
    }
}

void ThreeDAxesRepresentation::TranslateX(const double *p1, const double *p2)
{
    // std::cout << "TranslateX" << std::endl;
    double *pts = static_cast<vtkDoubleArray *>(this->Points->GetData())->GetPointer(0);
    double v[3] = {0, 0, 0};

    v[0] = p2[0] - p1[0];
    v[1] = p2[1] - p1[1];
    v[2] = p2[2] - p1[2];

    double axis[3] = {1., 0., 0.}; // axis of rotation
    axis[0] = this->XDirection[0];
    axis[1] = this->XDirection[1];
    axis[2] = this->XDirection[2];

    double dis = vtkMath::Dot(v, axis);
    v[0] = dis * this->XDirection[0];
    v[1] = dis * this->XDirection[1];
    v[2] = dis * this->XDirection[2];

    // Move the corners
    for (int i = 0; i < 8; i++)
    {
        *pts++ += v[0];
        *pts++ += v[1];
        *pts++ += v[2];
    }

    // this->Transform->Identity();
    this->Transform->Translate(v);
    // Move the origin
    this->Origin[0] += v[0];
    this->Origin[1] += v[1];
    this->Origin[2] += v[2];
    this->PositionHandles();
    this->GenerateRotateCircle();
    this->HandleGeometry->SetCenter(this->Origin);
    this->HandleGeometry->Modified();
    if (0) // todo 平移增加文字显示
    {
        double P_O[] = {0, 0, 0};
        char string[512];
        snprintf(string, sizeof(string), "(%1.1f)", Origin[0]);
        this->TextInput->SetText(string);
        this->TextActor->SetCamera(this->Renderer->GetActiveCamera());
        this->TextActor->SetPosition(axis);
        this->TextActor->SetScale(1, 1, 1);
    }
}

void ThreeDAxesRepresentation::TranslateY(const double *p1, const double *p2)
{
    // std::cout << "TranslateY" << std::endl;

    double *pts = static_cast<vtkDoubleArray *>(this->Points->GetData())->GetPointer(0);
    double v[3] = {0, 0, 0};

    v[0] = p2[0] - p1[0];
    v[1] = p2[1] - p1[1];
    v[2] = p2[2] - p1[2];

    double axis[3] = {0., 1., 0.}; // axis of rotation
    axis[0] = this->YDirection[0];
    axis[1] = this->YDirection[1];
    axis[2] = this->YDirection[2];

    double dis = vtkMath::Dot(v, axis);
    v[0] = dis * this->YDirection[0];
    v[1] = dis * this->YDirection[1];
    v[2] = dis * this->YDirection[2];

    // Move the corners
    for (int i = 0; i < 8; i++)
    {
        *pts++ += v[0];
        *pts++ += v[1];
        *pts++ += v[2];
    }
    // this->Transform->Identity();
    this->Transform->Translate(v);
    // Move the origin
    this->Origin[0] += v[0];
    this->Origin[1] += v[1];
    this->Origin[2] += v[2];
    this->PositionHandles();
    this->GenerateRotateCircle();
    this->HandleGeometry->SetCenter(this->Origin);
    this->HandleGeometry->Modified();
}

void ThreeDAxesRepresentation::TranslateZ(const double *p1, const double *p2)
{
    // std::cout << "TranslateZ" << std::endl;

    double *pts = static_cast<vtkDoubleArray *>(this->Points->GetData())->GetPointer(0);
    double v[3] = {0, 0, 0};

    v[0] = p2[0] - p1[0];
    v[1] = p2[1] - p1[1];
    v[2] = p2[2] - p1[2];

    double axis[3] = {0., 0., 1.}; // axis of rotation
    axis[0] = this->ZDirection[0];
    axis[1] = this->ZDirection[1];
    axis[2] = this->ZDirection[2];

    double dis = vtkMath::Dot(v, axis);
    v[0] = dis * this->ZDirection[0];
    v[1] = dis * this->ZDirection[1];
    v[2] = dis * this->ZDirection[2];

    // Move the corners
    for (int i = 0; i < 8; i++)
    {
        *pts++ += v[0];
        *pts++ += v[1];
        *pts++ += v[2];
    }
    // this->Transform->Identity();
    this->Transform->Translate(v);
    // Move the origin
    this->Origin[0] += v[0];
    this->Origin[1] += v[1];
    this->Origin[2] += v[2];
    this->PositionHandles();
    this->GenerateRotateCircle();
    this->HandleGeometry->SetCenter(this->Origin);
    this->HandleGeometry->Modified();
}

void ThreeDAxesRepresentation::UpdatePose(const double *p1, const double *d1, const double *p2, const double *d2)
{
    ;
}

//----------------------------------------------------------------------------
void ThreeDAxesRepresentation::CreateDefaultProperties()
{
    // X Arrow properties
    this->XArrowProperty = vtkProperty::New();
    this->XArrowProperty->SetColor(1, 0, 0);

    // Y Arrow properties
    this->YArrowProperty = vtkProperty::New();
    this->YArrowProperty->SetColor(0, 1, 0);

    // Z Arrow properties
    this->ZArrowProperty = vtkProperty::New();
    this->ZArrowProperty->SetColor(0, 0, 1);

    // YZ Circle properties
    this->YZCircleProperty = vtkProperty::New();
    this->YZCircleProperty->SetColor(1, 0, 0);
    this->YZCircleProperty->SetLineWidth(2.0);

    this->SelectedYZCircleProperty = vtkProperty::New();
    this->SelectedYZCircleProperty->SetColor(1, 0, 0);
    this->SelectedYZCircleProperty->SetLineWidth(4.0);

    // XZ Circle properties
    this->XZCircleProperty = vtkProperty::New();
    this->XZCircleProperty->SetColor(0, 1, 0);
    this->XZCircleProperty->SetLineWidth(2.0);

    this->SelectedXZCircleProperty = vtkProperty::New();
    this->SelectedXZCircleProperty->SetColor(0, 1, 0);
    this->SelectedXZCircleProperty->SetLineWidth(4.0);

    // XY Circle properties
    this->XYCircleProperty = vtkProperty::New();
    this->XYCircleProperty->SetColor(0, 0, 1);
    this->XYCircleProperty->SetLineWidth(2.0);

    this->SelectedXYCircleProperty = vtkProperty::New();
    this->SelectedXYCircleProperty->SetColor(0, 0, 1);
    this->SelectedXYCircleProperty->SetLineWidth(4.0);

    // Z Arrow properties
    this->ViewCircleProperty = vtkProperty::New();
    this->ViewCircleProperty->SetColor(1, 1, 0);
    this->ViewCircleProperty->SetLineWidth(2.0);

    this->SelectedViewCircleProperty = vtkProperty::New();
    this->SelectedViewCircleProperty->SetColor(1, 1, 0);
    this->SelectedViewCircleProperty->SetLineWidth(4.0);

    this->SelectedHCircleProperty = vtkProperty::New();
    this->SelectedHCircleProperty->SetColor(1, 0, 0);
    this->SelectedHCircleProperty->SetLineWidth(1.0);

    this->SelectedHCircleAreaProperty = vtkProperty::New();
    this->SelectedHCircleAreaProperty->SetColor(1, 1, 1);
    this->SelectedHCircleAreaProperty->SetOpacity(0.2);

    // Handle properties
    this->HandleProperty = vtkProperty::New();
    this->HandleProperty->SetColor(1, 1, 0);
    this->HandleProperty->SetOpacity(1);

    this->SelectedHandleProperty = vtkProperty::New();
    this->SelectedHandleProperty->SetColor(1, 0, 0);
}

//----------------------------------------------------------------------------
void ThreeDAxesRepresentation::PlaceWidget(double bds[6])
{
    int i;
    double bounds[6], center[3];

    this->AdjustBounds(bds, bounds, center);

    this->Origin[0] = (bounds[1] + bounds[0]) / 2.0;
    this->Origin[1] = (bounds[3] + bounds[2]) / 2.0;
    this->Origin[2] = (bounds[5] + bounds[4]) / 2.0;

    this->Points->SetPoint(0, bounds[0], bounds[2], bounds[4]);
    this->Points->SetPoint(1, bounds[1], bounds[2], bounds[4]);
    this->Points->SetPoint(2, bounds[1], bounds[3], bounds[4]);
    this->Points->SetPoint(3, bounds[0], bounds[3], bounds[4]);
    this->Points->SetPoint(4, bounds[0], bounds[2], bounds[5]);
    this->Points->SetPoint(5, bounds[1], bounds[2], bounds[5]);
    this->Points->SetPoint(6, bounds[1], bounds[3], bounds[5]);
    this->Points->SetPoint(7, bounds[0], bounds[3], bounds[5]);

    for (i = 0; i < 6; i++)
    {
        this->InitialBounds[i] = bounds[i];
    }
    this->InitialLength = sqrt((bounds[1] - bounds[0]) * (bounds[1] - bounds[0]) +
                               (bounds[3] - bounds[2]) * (bounds[3] - bounds[2]) +
                               (bounds[5] - bounds[4]) * (bounds[5] - bounds[4]));

    this->CircleRadius = InitialLength / 2.;

    this->PositionHandles();
    this->ComputeNormals();
    this->SizeHandles();

    this->ValidPick = 1; // since we have set up widget
    this->GenerateRotateCircle();
    this->HandleGeometry->SetCenter(this->Origin);
    this->HandleGeometry->Modified();
    this->BuildRepresentation();

    this->TotalTransform->Identity();
}

//----------------------------------------------------------------------------
void ThreeDAxesRepresentation::GetTransform(vtkTransform *t)
{
    // std::cout << "GetTransform" << std::endl;
    //  The transformation is relative to the initial bounds.
    //  Initial bounds are set when PlaceWidget() is invoked.
    t->Identity();
    t->DeepCopy(this->Transform);
    t->Concatenate(this->TotalTransform);
}

//----------------------------------------------------------------------------
void ThreeDAxesRepresentation::SetTransform(vtkTransform *t)
{
    if (!t)
    {
        vtkErrorMacro(<< "vtkTransform t must be non-nullptr");
        return;
    }

    double xIn[3];
    // make sure the transform is up-to-date before using it
    t->Update();

    xIn[0] = this->Origin[0];
    xIn[1] = this->Origin[1];
    xIn[2] = this->Origin[2];
    t->InternalTransformPoint(xIn, this->Origin);
}

//----------------------------------------------------------------------------
int ThreeDAxesRepresentation::ComputeInteractionState(int X, int Y, int modify)
{
    // Okay, we can process this. Try to pick handles first;
    // if no handles picked, then pick the bounding box.
    if (!this->Renderer || !this->Renderer->IsInViewport(X, Y))
    {
        this->InteractionState = ThreeDAxesRepresentation::Outside;
        return this->InteractionState;
    }

    // Try and pick a handle first
    this->LastPicker = nullptr;
    this->CurrentHandle = nullptr;

    vtkAssemblyPath *path = this->GetAssemblyPath(X, Y, 0., this->CirclePicker);
    vtkAssemblyPath *arrow_path = this->GetAssemblyPath(X, Y, 0., this->ArrowPicker);
    vtkAssemblyPath *handle_path = this->GetAssemblyPath(X, Y, 0., this->HandlePicker);

    if (path != nullptr)
    {
        //  std::cout << "Circle Path" << std::endl;

        this->ValidPick = 1;
        this->LastPicker = this->CirclePicker;
        this->CurrentHandle = reinterpret_cast<vtkActor *>(path->GetFirstNode()->GetViewProp());
        if (this->CurrentHandle == this->CircleActor[0])
        {
            this->InteractionState = ThreeDAxesRepresentation::RotateYZ;
        }
        else if (this->CurrentHandle == this->CircleActor[1])
        {
            this->InteractionState = ThreeDAxesRepresentation::RotateXZ;
        }
        else if (this->CurrentHandle == this->CircleActor[2])
        {
            this->InteractionState = ThreeDAxesRepresentation::RotateXY;
        }
        else if (this->CurrentHandle == this->CircleActor[3])
        {
            this->InteractionState = ThreeDAxesRepresentation::Rotating;
        }
    }
    else if (arrow_path != nullptr)
    {
        this->ValidPick = 1;
        this->LastPicker = this->HandlePicker;
        this->CurrentHandle = reinterpret_cast<vtkActor *>(arrow_path->GetFirstNode()->GetViewProp());
        if (this->CurrentHandle == this->ArrowActor[0])
        {
            this->InteractionState = ThreeDAxesRepresentation::MoveX;
        }
        else if (this->CurrentHandle == this->ArrowActor[1])
        {
            this->InteractionState = ThreeDAxesRepresentation::MoveY;
        }
        else if (this->CurrentHandle == this->ArrowActor[2])
        {
            this->InteractionState = ThreeDAxesRepresentation::MoveZ;
        }
    }
    else if (handle_path != nullptr)
    {
        //  std::cout << "handle path" << std::endl;
        this->ValidPick = 1;
        this->LastPicker = this->HandlePicker;
        this->CurrentHandle = reinterpret_cast<vtkActor *>(handle_path->GetFirstNode()->GetViewProp());
        this->InteractionState = ThreeDAxesRepresentation::Translating;
    }

    return this->InteractionState;
}

//----------------------------------------------------------------------
void ThreeDAxesRepresentation::SetInteractionState(int state)
{
    // Clamp to allowable values
    state = (state < ThreeDAxesRepresentation::Outside
                 ? ThreeDAxesRepresentation::Outside
                 : state);

    // Depending on state, highlight appropriate parts of representation
    int handle;
    this->InteractionState = state;
    switch (state)
    {
    case ThreeDAxesRepresentation::RotateYZ:
    case ThreeDAxesRepresentation::RotateXZ:
    case ThreeDAxesRepresentation::RotateXY:
        handle = this->HighlightCircle(this->CurrentHandle);
        break;
    case ThreeDAxesRepresentation::MoveX:
    case ThreeDAxesRepresentation::MoveY:
    case ThreeDAxesRepresentation::MoveZ:
        handle = this->HighlightArrow(this->CurrentHandle);
        break;
    case ThreeDAxesRepresentation::Rotating:
        this->HighlightCircle(this->CurrentHandle);
        break;
    case ThreeDAxesRepresentation::Translating:
        this->HighlightHandle(this->Handle);
        break;
    default:
        this->HighlightHandle(nullptr);
        this->HighlightCircle(nullptr);
    }
}

//----------------------------------------------------------------------
double *ThreeDAxesRepresentation::GetBounds()
{
    this->BuildRepresentation();
    this->BoundingBox->SetBounds(this->CircleActor[3]->GetBounds());
    return this->BoundingBox->GetBounds();
}

//----------------------------------------------------------------------------
void ThreeDAxesRepresentation::BuildRepresentation()
{
    // Rebuild only if necessary
    if (this->GetMTime() > this->BuildTime ||
        (this->Renderer && this->Renderer->GetVTKWindow() &&
         (this->Renderer->GetVTKWindow()->GetMTime() > this->BuildTime ||
          this->Renderer->GetActiveCamera()->GetMTime() > this->BuildTime)))
    {
        this->PositionHandles();
        this->GenerateRotateCircle();

        this->HandleGeometry->SetCenter(this->Origin);
        this->HandleGeometry->Modified();

        this->SizeHandles();
        this->BuildTime.Modified();
    }
}

//----------------------------------------------------------------------------
void ThreeDAxesRepresentation::ReleaseGraphicsResources(vtkWindow *w)
{
    this->HCircleActor->ReleaseGraphicsResources(w);
    this->HCircleAreaActor->ReleaseGraphicsResources(w);

    for (int i = 0; i < 3; i++)
    {
        this->ArrowActor[i]->ReleaseGraphicsResources(w);
    }
    for (int i = 0; i < 4; i++)
    {
        this->CircleActor[i]->ReleaseGraphicsResources(w);
    }
    // render the handles
    this->Handle->ReleaseGraphicsResources(w);

    this->TextActor->ReleaseGraphicsResources(w);
}

//----------------------------------------------------------------------------
int ThreeDAxesRepresentation::RenderOpaqueGeometry(vtkViewport *v)
{
    int count = 0;
    this->BuildRepresentation();

    if (this->HCircleActor->GetVisibility())
    {
        count += this->HCircleActor->RenderOpaqueGeometry(v);
    }
    if (this->HCircleAreaActor->GetVisibility())
    {
        count += this->HCircleAreaActor->RenderOpaqueGeometry(v);
    }
    if (this->TextActor->GetVisibility())
    {
        count += this->TextActor->RenderOpaqueGeometry(v);
    }
    for (int i = 0; i < 4; i++)
    {
        if (this->CircleActor[i]->GetVisibility())
        {
            this->CircleActor[i]->SetPropertyKeys(this->GetPropertyKeys());
            count += this->CircleActor[i]->RenderOpaqueGeometry(v);
        }
    }
    for (int i = 0; i < 3; i++)
    {
        if (this->ArrowActor[i]->GetVisibility())
        {
            this->ArrowActor[i]->SetPropertyKeys(this->GetPropertyKeys());
            count += this->ArrowActor[i]->RenderOpaqueGeometry(v);
        }
    }

    if (1)
    {
        this->Handle->SetPropertyKeys(this->GetPropertyKeys());
        count += this->Handle->RenderOpaqueGeometry(v);
    }

    return count;
}

//----------------------------------------------------------------------------
int ThreeDAxesRepresentation::RenderTranslucentPolygonalGeometry(vtkViewport *v)
{
    int count = 0;
    this->BuildRepresentation();

    if (this->HCircleActor->GetVisibility())
    {
        count += this->HCircleActor->RenderTranslucentPolygonalGeometry(v);
    }
    if (this->HCircleAreaActor->GetVisibility())
    {
        count += this->HCircleAreaActor->RenderTranslucentPolygonalGeometry(v);
    }
    if (this->TextActor->GetVisibility())
    {
        count += this->TextActor->RenderTranslucentPolygonalGeometry(v);
    }

    for (int i = 0; i < 4; i++)
    {
        if (this->CircleActor[i]->GetVisibility())
        {
            this->CircleActor[i]->SetPropertyKeys(this->GetPropertyKeys());
            count += this->CircleActor[i]->RenderTranslucentPolygonalGeometry(v);
        }
    }

    for (int i = 0; i < 3; i++)
    {
        if (this->ArrowActor[i]->GetVisibility())
        {
            this->ArrowActor[i]->SetPropertyKeys(this->GetPropertyKeys());
            count += this->ArrowActor[i]->RenderTranslucentPolygonalGeometry(v);
        }
    }
    // render the handles
    if (1)
    {
        this->Handle->SetPropertyKeys(this->GetPropertyKeys());
        count += this->Handle->RenderTranslucentPolygonalGeometry(v);
    }

    return count;
}

//----------------------------------------------------------------------------
vtkTypeBool ThreeDAxesRepresentation::HasTranslucentPolygonalGeometry()
{
    int result = 0;
    this->BuildRepresentation();

    result |= this->HCircleActor->HasTranslucentPolygonalGeometry();
    result |= this->HCircleAreaActor->HasTranslucentPolygonalGeometry();
    result |= this->TextActor->HasTranslucentPolygonalGeometry();

    for (int i = 0; i < 4; i++)
    {
        result |= this->CircleActor[i]->HasTranslucentPolygonalGeometry();
    }

    // render the handles
    result |= this->Handle->HasTranslucentPolygonalGeometry();

    return result;
}

//----------------------------------------------------------------------------
void ThreeDAxesRepresentation::HandlesOn()
{
    this->Handle->VisibilityOn();
}

//----------------------------------------------------------------------------
void ThreeDAxesRepresentation::HandlesOff()
{
    this->Handle->VisibilityOff();
}

//----------------------------------------------------------------------------
void ThreeDAxesRepresentation::SizeHandles()
{
    /*double* center = static_cast<vtkDoubleArray*>(this->Points->GetData())->GetPointer(3 * 14);
    double radius = this->vtkWidgetRepresentation::SizeHandlesInPixels(1.5, center);*/

    double radius = this->vtkWidgetRepresentation::SizeHandlesInPixels(1.5, this->Origin);
    if (radius < 0.05)
    {
        radius = 0.05;
    }
    this->HandleGeometry->SetRadius(radius);
}

//----------------------------------------------------------------------------
int ThreeDAxesRepresentation::HighlightHandle(vtkProp *prop)
{
    // first unhighlight anything picked
    this->ArrowActor[0]->SetProperty(this->XArrowProperty);
    this->ArrowActor[1]->SetProperty(this->YArrowProperty);
    this->ArrowActor[2]->SetProperty(this->ZArrowProperty);
    this->Handle->SetProperty(this->HandleProperty);
    this->GenerateOutline(0);
    this->GenerateOutline(1);
    this->GenerateOutline(2);

    this->CurrentHandle = static_cast<vtkActor *>(prop);

    if (this->CurrentHandle)
    {

        for (int i = 0; i < 3; i++)
        { // find attached face
            if (this->CurrentHandle == this->ArrowActor[i] && i == 0)
            {
                this->GenerateOutline(i, true);
                return i;
            }
            else if (this->CurrentHandle == this->ArrowActor[i] && i == 1)
            {
                this->GenerateOutline(i, true);
                return i;
            }
            else if (this->CurrentHandle == this->ArrowActor[i] && i == 2)
            {
                this->GenerateOutline(i, true);
                return i;
            }
            else if (this->CurrentHandle == this->Handle)
            {
                this->CurrentHandle->SetProperty(this->SelectedHandleProperty);
                return i + 1;
            }
        }
    }

    return -1;
}

//----------------------------------------------------------------------------
int ThreeDAxesRepresentation::HighlightCircle(vtkProp *prop)
{

    // first unhighlight anything picked
    this->CircleActor[0]->SetProperty(this->YZCircleProperty);
    this->CircleActor[1]->SetProperty(this->XZCircleProperty);
    this->CircleActor[2]->SetProperty(this->XYCircleProperty);
    this->CircleActor[3]->SetProperty(this->ViewCircleProperty);

    this->CurrentHandle = static_cast<vtkActor *>(prop);

    if (this->CurrentHandle)
    {
        this->HCircleActor->VisibilityOn();
        this->HCircleAreaActor->VisibilityOn();
        this->TextActor->VisibilityOn();
        if (this->CurrentHandle == this->CircleActor[0])
        {
            this->CurrentHandle->SetProperty(this->SelectedYZCircleProperty);
            return 0;
        }
        else if (this->CurrentHandle == this->CircleActor[1])
        {
            this->CurrentHandle->SetProperty(this->SelectedXZCircleProperty);
            return 1;
        }
        else if (this->CurrentHandle == this->CircleActor[2])
        {
            this->CurrentHandle->SetProperty(this->SelectedXYCircleProperty);
            return 2;
        }
        else if (this->CurrentHandle == this->CircleActor[3])
        {
            this->CurrentHandle->SetProperty(this->SelectedViewCircleProperty);
            return 3;
        }
    }
    else
    {
        this->HCircleActor->VisibilityOff();
        this->HCircleAreaActor->VisibilityOff();
        this->TextActor->VisibilityOff();
    }

    return -1;
}
int ThreeDAxesRepresentation::HighlightArrow(vtkProp *prop)
{

    // first unhighlight anything picked
    this->ArrowActor[0]->SetProperty(this->XArrowProperty);
    this->ArrowActor[1]->SetProperty(this->YArrowProperty);
    this->ArrowActor[2]->SetProperty(this->ZArrowProperty);
    this->GenerateOutline(0);
    this->GenerateOutline(1);
    this->GenerateOutline(2);

    this->CurrentHandle = static_cast<vtkActor *>(prop);

    if (this->CurrentHandle)
    {

        for (int i = 0; i < 4; i++)
        { // find attached face
            if (this->CurrentHandle == this->ArrowActor[i] && i == 0)
            {
                this->GenerateOutline(i, true);
                return i;
            }
            else if (this->CurrentHandle == this->ArrowActor[i] && i == 1)
            {
                this->GenerateOutline(i, true);
                return i;
            }
            else if (this->CurrentHandle == this->ArrowActor[i] && i == 2)
            {
                this->GenerateOutline(i, true);
                return i;
            }
        }
    }

    return -1;
}
//------------------------------------------------------------------------------
void ThreeDAxesRepresentation::RegisterPickers()
{
    vtkPickingManager *pm = this->GetPickingManager();
    if (!pm)
    {
        return;
    }
    pm->AddPicker(this->HandlePicker, this);
    pm->AddPicker(this->ArrowPicker, this);
    pm->AddPicker(this->CirclePicker, this);
}

void ThreeDAxesRepresentation::SetXDirection(double x[3])
{
    if (this->XDirection[0] != x[2] || this->XDirection[1] != x[1] || this->XDirection[2] != x[2])
    {
        this->XDirection[0] = x[0];
        this->XDirection[1] = x[1];
        this->XDirection[2] = x[2];

        this->BuildRepresentation();
        this->Modified();
    }
}

void ThreeDAxesRepresentation::SetYDirection(double x[3])
{
    if (this->YDirection[0] != x[2] || this->YDirection[1] != x[1] || this->YDirection[2] != x[2])
    {
        this->YDirection[0] = x[0];
        this->YDirection[1] = x[1];
        this->YDirection[2] = x[2];

        this->BuildRepresentation();
        this->Modified();
    }
}

void ThreeDAxesRepresentation::SetZDirection(double x[3])
{
    if (this->ZDirection[0] != x[2] || this->ZDirection[1] != x[1] || this->ZDirection[2] != x[2])
    {
        this->ZDirection[0] = x[0];
        this->ZDirection[1] = x[1];
        this->ZDirection[2] = x[2];

        this->BuildRepresentation();
        this->Modified();
    }
}

void ThreeDAxesRepresentation::SetOrigin(double ox, double oy, double oz)
{
    if (this->Origin[0] != ox || this->Origin[1] != oy || this->Origin[2] != oz)
    {
        this->Origin[0] = ox;
        this->Origin[1] = oy;
        this->Origin[2] = oz;

        this->BuildRepresentation();
        this->Modified();
    }
}

void ThreeDAxesRepresentation::GenerateOutline(int i, bool hight)
{

    if (i == 0 && this->InteractionState == MoveX)
    {
        hight = true;
    }
    else if (i == 1 && this->InteractionState == MoveY)
    {
        hight = true;
    }
    else if (i == 2 && this->InteractionState == MoveZ)
    {
        hight = true;
    }
    else
        hight = false;

    double origin[3] = {0, 0, 0}, px[3], py[3], pz[3];
    this->Points->GetPoint(14, origin);
    this->Points->GetPoint(18, px);
    this->Points->GetPoint(19, py);
    this->Points->GetPoint(20, pz);
    double xlength = vtkMath::Distance2BetweenPoints(origin, px);
    double ylength = vtkMath::Distance2BetweenPoints(origin, py);
    double zlength = vtkMath::Distance2BetweenPoints(origin, pz);
    double length = xlength > ylength ? xlength : ylength;
    length = length > zlength ? length : zlength;
    length = std::sqrt(length);

    double p[3];
    this->Points->GetPoint(vtkIdType(18 + i), p);

    double normalizedX[3], normalizedY[3], normalizedZ[3];
    // The X axis is a vector from start to end
    vtkMath::Subtract(p, origin, normalizedX);
    double tmpLength = vtkMath::Norm(normalizedX);
    if (tmpLength < 0.001)
        return;
    vtkMath::Normalize(normalizedX);

    // The Z axis is an arbitrary vector cross X
    double arbitrary[3] = {0, 0, 1};
    vtkMath::Cross(normalizedX, arbitrary, normalizedZ);
    double temp = vtkMath::Norm(normalizedZ);
    if (temp < 0.001)
    {
        arbitrary[0] = 1;
        arbitrary[1] = 0;
        arbitrary[2] = 0;
        vtkMath::Cross(normalizedX, arbitrary, normalizedZ);
    }
    vtkMath::Normalize(normalizedZ);

    // The Y axis is Z cross X
    vtkMath::Cross(normalizedZ, normalizedX, normalizedY);
    vtkSmartPointer<vtkMatrix4x4> matrix =
        vtkSmartPointer<vtkMatrix4x4>::New();

    // Create the direction cosine matrix
    matrix->Identity();
    for (auto i = 0; i < 3; i++)
    {
        matrix->SetElement(i, 0, normalizedX[i]);
        matrix->SetElement(i, 1, normalizedY[i]);
        matrix->SetElement(i, 2, normalizedZ[i]);
    }

    // Apply the transforms
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->Translate(origin);
    transform->Concatenate(matrix);
    transform->Scale(length, length, length);

    // Transform the polydata
    vtkArrowSource *arrowSource = vtkArrowSource::New();
    if (!hight)
    {
        arrowSource->SetShaftRadius(0.01);
        arrowSource->SetShaftResolution(50);
        arrowSource->SetTipRadius(0.05);
        arrowSource->SetTipResolution(30);
    }
    else
    {
        arrowSource->SetShaftRadius(0.02);
        arrowSource->SetShaftResolution(50);
        arrowSource->SetTipRadius(0.1);
        arrowSource->SetTipResolution(30);
    }

    vtkTransformPolyDataFilter *transformPD = vtkTransformPolyDataFilter::New();
    transformPD->SetTransform(transform);
    transformPD->SetInputConnection(arrowSource->GetOutputPort());
    transformPD->Update();

    this->ArrowGeometry[i]->SetPoints(transformPD->GetOutput()->GetPoints());
    this->ArrowGeometry[i]->SetVerts(transformPD->GetOutput()->GetVerts());
    this->ArrowGeometry[i]->SetLines(transformPD->GetOutput()->GetLines());
    this->ArrowGeometry[i]->SetPolys(transformPD->GetOutput()->GetPolys());
    this->ArrowGeometry[i]->SetStrips(transformPD->GetOutput()->GetStrips());
    this->ArrowGeometry[i]->BuildLinks();
    this->ArrowGeometry[i]->Modified();

    arrowSource->Delete();
    transformPD->Delete();

    // this->Renderer->Render();
}
#define VTK_AVERAGE(a, b, c)    \
    c[0] = (a[0] + b[0]) / 2.0; \
    c[1] = (a[1] + b[1]) / 2.0; \
    c[2] = (a[2] + b[2]) / 2.0;

#define ZX_DOUBLELENGTH(a, o, b) \
    b[0] = a[0] + (a[0] - o[0]); \
    b[1] = a[1] + (a[1] - o[1]); \
    b[2] = a[2] + (a[2] - o[2]);

//----------------------------------------------------------------------------
void ThreeDAxesRepresentation::PositionHandles()
{
    double *pts = static_cast<vtkDoubleArray *>(this->Points->GetData())->GetPointer(0);
    double *p0 = pts;
    double *p1 = pts + 3 * 1;
    double *p2 = pts + 3 * 2;
    double *p3 = pts + 3 * 3;
    // double *p4 = pts + 3*4;
    double *p5 = pts + 3 * 5;
    double *p6 = pts + 3 * 6;
    double *p7 = pts + 3 * 7;
    double x8[3], x9[3], x10[3], x11[3], x12[3], x13[3], x14[3];

    VTK_AVERAGE(p0, p7, x8);
    this->Points->SetPoint(8, x8);
    VTK_AVERAGE(p1, p6, x9);
    this->Points->SetPoint(9, x9);
    VTK_AVERAGE(p0, p5, x10);
    this->Points->SetPoint(10, x10);
    VTK_AVERAGE(p2, p7, x11);
    this->Points->SetPoint(11, x11);
    VTK_AVERAGE(p1, p3, x12);
    this->Points->SetPoint(12, x12);
    VTK_AVERAGE(p5, p7, x13);
    this->Points->SetPoint(13, x13);
    VTK_AVERAGE(p0, p6, x14);
    this->Points->SetPoint(14, x14);

    double x15[3], x16[3], x17[3], x18[3], x19[3], x20[3];
    VTK_AVERAGE(p2, p6, x15);
    this->Points->SetPoint(15, x15);
    VTK_AVERAGE(p7, p6, x16);
    this->Points->SetPoint(16, x16);
    VTK_AVERAGE(p5, p6, x17);
    this->Points->SetPoint(17, x17);

    ZX_DOUBLELENGTH(x9, x14, x18);
    ZX_DOUBLELENGTH(x11, x14, x19);
    ZX_DOUBLELENGTH(x13, x14, x20);
    this->Points->SetPoint(18, x18);
    this->Points->SetPoint(19, x19);
    this->Points->SetPoint(20, x20);

    this->HandleGeometry->SetCenter(this->Points->GetPoint(14));

    this->Points->GetData()->Modified();
    this->GenerateOutline(0);
    this->GenerateOutline(1);
    this->GenerateOutline(2);
}

void ThreeDAxesRepresentation::PositionHandles(double *ipts)
{
    double *pts = ipts;
    double *p0 = pts;
    double *p1 = pts + 3 * 1;
    double *p2 = pts + 3 * 2;
    double *p3 = pts + 3 * 3;
    // double *p4 = pts + 3*4;
    double *p5 = pts + 3 * 5;
    double *p6 = pts + 3 * 6;
    double *p7 = pts + 3 * 7;
    double x8[3], x9[3], x10[3], x11[3], x12[3], x13[3], x14[3];

    VTK_AVERAGE(p0, p7, x8);
    this->Points->SetPoint(8, x8);
    VTK_AVERAGE(p1, p6, x9);
    this->Points->SetPoint(9, x9);
    VTK_AVERAGE(p0, p5, x10);
    this->Points->SetPoint(10, x10);
    VTK_AVERAGE(p2, p7, x11);
    this->Points->SetPoint(11, x11);
    VTK_AVERAGE(p1, p3, x12);
    this->Points->SetPoint(12, x12);
    VTK_AVERAGE(p5, p7, x13);
    this->Points->SetPoint(13, x13);
    VTK_AVERAGE(p0, p6, x14);
    this->Points->SetPoint(14, x14);

    double x15[3], x16[3], x17[3], x18[3], x19[3], x20[3];
    VTK_AVERAGE(p2, p6, x15);
    this->Points->SetPoint(15, x15);
    VTK_AVERAGE(p7, p6, x16);
    this->Points->SetPoint(16, x16);
    VTK_AVERAGE(p5, p6, x17);
    this->Points->SetPoint(17, x17);

    ZX_DOUBLELENGTH(x9, x14, x18);
    ZX_DOUBLELENGTH(x11, x14, x19);
    ZX_DOUBLELENGTH(x13, x14, x20);
    this->Points->SetPoint(18, x18);
    this->Points->SetPoint(19, x19);
    this->Points->SetPoint(20, x20);

    this->HandleGeometry->SetCenter(this->Points->GetPoint(14));

    this->Points->GetData()->Modified();
    this->GenerateOutline(0);
    this->GenerateOutline(1);
    this->GenerateOutline(2);
}
#undef VTK_AVERAGE
void ThreeDAxesRepresentation::ComputeNormals()
{
    double *pts = static_cast<vtkDoubleArray *>(this->Points->GetData())->GetPointer(0);
    double *p0 = pts;
    double *px = pts + 3 * 1;
    double *py = pts + 3 * 3;
    double *pz = pts + 3 * 4;
    int i;

    for (i = 0; i < 3; i++)
    {
        this->N[0][i] = p0[i] - px[i];
        this->N[2][i] = p0[i] - py[i];
        this->N[4][i] = p0[i] - pz[i];
    }
    vtkMath::Normalize(this->N[0]);
    vtkMath::Normalize(this->N[2]);
    vtkMath::Normalize(this->N[4]);
    for (i = 0; i < 3; i++)
    {
        this->N[1][i] = -this->N[0][i];
        this->N[3][i] = -this->N[2][i];
        this->N[5][i] = -this->N[4][i];
    }
}
