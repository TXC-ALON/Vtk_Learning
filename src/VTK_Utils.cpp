#include <VTK_Utils.h>

double Deg2Rad(double degrees) { return degrees * 3.1415926 / 180.0; }
vtkSmartPointer<vtkPolyData> ReadPolyData(std::string const &fileName)
{
    vtkSmartPointer<vtkPolyData> polyData;
    std::string extension = "";
    if (fileName.find_last_of(".") != std::string::npos)
    {
        extension = fileName.substr(fileName.find_last_of("."));
    }
    // Make the extension lowercase
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   ::tolower);
    if (extension == ".ply")
    {
        vtkNew<vtkPLYReader> reader;
        reader->SetFileName(fileName.c_str());
        reader->Update();
        polyData = reader->GetOutput();
    }
    else if (extension == ".vtp")
    {
        vtkNew<vtkXMLPolyDataReader> reader;
        reader->SetFileName(fileName.c_str());
        reader->Update();
        polyData = reader->GetOutput();
    }
    else if (extension == ".obj")
    {
        vtkNew<vtkOBJReader> reader;
        reader->SetFileName(fileName.c_str());
        reader->Update();
        polyData = reader->GetOutput();
    }
    else if (extension == ".stl")
    {
        vtkNew<vtkSTLReader> reader;
        reader->SetFileName(fileName.c_str());
        reader->Update();
        polyData = reader->GetOutput();
    }
    else if (extension == ".vtk")
    {
        vtkNew<vtkPolyDataReader> reader;
        reader->SetFileName(fileName.c_str());
        reader->Update();
        polyData = reader->GetOutput();
    }
    else if (extension == ".g")
    {
        vtkNew<vtkBYUReader> reader;
        reader->SetGeometryFileName(fileName.c_str());
        reader->Update();
        polyData = reader->GetOutput();
    }
    else
    {
        // Return a polydata sphere if the extension is unknown.
        vtkNew<vtkSphereSource> source;
        source->SetThetaResolution(20);
        source->SetPhiResolution(11);
        source->Update();
        polyData = source->GetOutput();
    }
    return polyData;
}
vtkSmartPointer<vtkMatrix4x4> GetModelRotationMatrix(double m, double n, double l, double angleX, double angleY, double angleZ)
{

    // 创建一个变换对象
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    // 将模型平移到原点
    std::cout << angleX << " " << angleY << " " << angleZ << " " << std::endl;
    transform->Translate(m, n, l);
    transform->RotateZ(angleZ);
    transform->RotateY(angleY);
    transform->RotateX(angleX);
    transform->Translate(-m, -n, -l);
    // 获取最终的4x4矩阵
    vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();

    transform->GetMatrix(matrix);
    std::cout << "GetModelRotationMatrix\n"
              << *matrix << std::endl;
    return matrix;
}
void SetModelRotationMatrix(vtkActor *actor, double m, double n, double l, double angleX, double angleY, double angleZ)
{
    vtkMatrix4x4 *currentMatrix = actor->GetUserMatrix();
    if (!currentMatrix)
    {
        // 如果当前没有用户矩阵，初始化为单位矩阵
        vtkNew<vtkMatrix4x4> identityMatrix;
        identityMatrix->Identity();
        actor->SetUserMatrix(identityMatrix);
        currentMatrix = actor->GetUserMatrix();
    }

    vtkSmartPointer<vtkMatrix4x4> rotationMatrix = GetModelRotationMatrix(m, n, l, (angleX), (angleY), (angleZ));
    // 将新的平移矩阵与当前矩阵相乘，得到累加平移效果
    vtkNew<vtkMatrix4x4> newMatrix;
    vtkMatrix4x4::Multiply4x4(currentMatrix, rotationMatrix, newMatrix);
    std::cout << "rotationMatrix is \n " << *newMatrix << std::endl;

    // 更新模型的用户矩阵
    actor->SetUserMatrix(newMatrix);
}
void SetModelTranslateMatrix(vtkActor *actor, double m, double n, double l, double dx, double dy, double dz)
{
    vtkMatrix4x4 *currentMatrix = actor->GetUserMatrix();
    if (!currentMatrix)
    {
        // 如果当前没有用户矩阵，初始化为单位矩阵
        vtkNew<vtkMatrix4x4> identityMatrix;
        identityMatrix->Identity();
        actor->SetUserMatrix(identityMatrix);
        currentMatrix = actor->GetUserMatrix();
    }
    // 创建一个变换对象
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    // 将模型平移到原点
    transform->Translate(dx, dy, dz);
    vtkSmartPointer<vtkMatrix4x4> transform_matrix = vtkSmartPointer<vtkMatrix4x4>::New();
    transform->GetMatrix(transform_matrix);
    // 将新的平移矩阵与当前矩阵相乘，得到累加平移效果
    vtkNew<vtkMatrix4x4> newMatrix;
    vtkMatrix4x4::Multiply4x4(currentMatrix, transform_matrix, newMatrix);
    std::cout << "transform_matrix is \n " << *newMatrix << std::endl;
    // 更新模型的用户矩阵
    actor->SetUserMatrix(newMatrix);
}

// 创建箭头并应用变换
vtkSmartPointer<vtkActor> createArrowActor(double scaleFactor, double *color, double *direction)
{
    vtkNew<vtkArrowSource> arrowSource;
    arrowSource->SetTipLength(0.15);
    arrowSource->SetTipRadius(0.05);
    arrowSource->SetTipResolution(50);

    arrowSource->SetShaftResolution(50);
    arrowSource->SetShaftRadius(0.01);
    vtkNew<vtkTransform> transform;
    transform->RotateWXYZ(90, direction);

    vtkNew<vtkTransformPolyDataFilter> transformFilter;
    transformFilter->SetTransform(transform);
    transformFilter->SetInputConnection(arrowSource->GetOutputPort());

    vtkNew<vtkPolyDataMapper> arrowMapper;
    arrowMapper->SetInputConnection(transformFilter->GetOutputPort());

    vtkNew<vtkActor> arrowActor;
    arrowActor->SetMapper(arrowMapper);
    arrowActor->GetProperty()->SetColor(color);
    arrowActor->SetScale(scaleFactor);

    return arrowActor;
}

void Add_Arrow_Axes(vtkSmartPointer<vtkRenderer> renderer, double scaleFactor)
{
    vtkNew<vtkNamedColors> colors;
    // 创建箭头
    auto xArrowActor = createArrowActor(scaleFactor, colors->GetColor3d("Red").GetData(), new double[3]{1, 0, 0});
    xArrowActor->SetPosition(0, 0, 0);

    auto yArrowActor = createArrowActor(scaleFactor, colors->GetColor3d("Yellow").GetData(), new double[3]{0, 0, 1});
    yArrowActor->SetPosition(0, 0, 0);

    auto zArrowActor = createArrowActor(scaleFactor, colors->GetColor3d("Green").GetData(), new double[3]{0, -1, 0});
    zArrowActor->SetPosition(0, 0, 0);

    renderer->AddActor(xArrowActor);
    renderer->AddActor(yArrowActor);
    renderer->AddActor(zArrowActor);
}
void Add_Line_Axes(vtkSmartPointer<vtkRenderer> renderer, double scaleFactor)
{
    vtkNew<vtkAxes> modelAxesSource;
    modelAxesSource->SetScaleFactor(20);
    modelAxesSource->SetOrigin(0, 0, 0);
    vtkNew<vtkTubeFilter> tubeFilter;
    tubeFilter->SetInputConnection(modelAxesSource->GetOutputPort());
    tubeFilter->SetRadius(0.05);
    tubeFilter->SetNumberOfSides(50);

    vtkNew<vtkPolyDataMapper> modelAxesMapper;
    modelAxesMapper->SetInputConnection(tubeFilter->GetOutputPort());

    vtkNew<vtkActor> modelAxes;
    modelAxes->SetMapper(modelAxesMapper);
    renderer->AddActor(modelAxes);
}
