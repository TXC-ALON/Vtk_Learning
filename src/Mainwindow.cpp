#include "Mainwindow.h"
#include <QVBoxLayout>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkActor.h>
#include <vtkDataSetMapper.h>
#include <vtkDoubleArray.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkPointData.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>

#include <cmath>
#include <cstdlib>
#include <random>

MainWindow::~MainWindow()
{
    if (pRotateWidget != nullptr)
    {
        delete pRotateWidget;
        pRotateWidget = nullptr;
    }
}

void MainWindow::RotateSlot()
{
    // std::cout << "RotateSlot" << std::endl;

    pRotateWidget->setVisible(true);
    pRotateWidget->InitOpenPage();
}

void MainWindow::openFileSlot()
{
    std::cout << "openFileSlot" << std::endl;

    // 定义起始目录
    QString selectedDir = recent_skein_directory;
    if (recent_skein_directory.isEmpty() || !QDir().exists(recent_skein_directory))
        selectedDir = QDir::homePath();

    // 定义对话框标题
    QString title = QStringLiteral("选择一个或多个文件");

    // 定义文件过滤器
    QString filter_all_imort = "All import files (*.stl *ipd *.obj *.ptl)"; //*.stl;*.STL;*.ipd;*.IPD;*.obj;*.OBJ;*.ptl;*.PTL
    QString filter_stl = "STL import files (*.stl)";
    QString filter_obj = "OBJ import files (*.obj)";
    QString filter_ipd = "IPD import files (*.ipd)";
    QString filter_ptl = "PTL import files (*.ptl)";
    QString selectedFilter =
        filter_all_imort + ";;" + filter_stl + ";;" + filter_obj + ";;" + filter_ipd + ";;" + filter_ptl;

    QStringList selectedFiles = QFileDialog::getOpenFileNames(
        nullptr,                         // 父窗口
        title,                           // 对话框标题
        selectedDir,                     // 起始目录
        selectedFilter,                  // 文件过滤器
        nullptr,                         // 选择特定的过滤器
        QFileDialog::DontUseNativeDialog // 禁用原生对话框，以支持多选
    );

    if (selectedFiles.isEmpty())
    {
        QMessageBox::warning(nullptr, "Warning", QStringLiteral("没有选中文件!"));
        return;
    }

    loadFileList(selectedFiles);

    // 刷新窗口
    this->ui->vtkWidget.renderer->ResetCamera();
    this->ui->vtkWidget.window->Render();
}
bool MainWindow::loadFile(const QString &filePath)
{
    try
    {
        qDebug() << "P3DSNextGen::loadFile satrt, file is " << filePath;

        Model model;
        FileLoader loader(filePath);

        // 更新recent_skein_directory
        QFileInfo fileinfo(filePath);
        recent_skein_directory = fileinfo.path();

        QTime startTime = QTime::currentTime();
        bool res = loader.loadFile(filePath, &model);
        QTime endTime = QTime::currentTime();
        int elapsed = startTime.msecsTo(endTime); // 以毫秒为单位
        qDebug() << "P3DSNextGen::loadFile loader.loadFile run time:" << elapsed << "ms";

        if (!res)
        {
            qDebug() << "P3DSNextGen::loadFile loader.loadFile failed!";
            return false;
        }

        if (model.objects.size() != 1)
        {
            qDebug() << "P3DSNextGen::loadFile current model: " << filePath << " objects.size() = " << model.objects.size() << ", Error!";
            return false;
        }

        // 导入文件成功
        ModelObject *obj = model.objects.at(0);

        // set group_id，待接入组号
        obj->Set_GroupID(1);

        // 自动修复流程
        {
            qDebug() << "P3DSNextGen::loadFile repair start";

            // 检查是否需要修复
            obj->repair(false);
            // 处理无法修复的模型
            if (obj->UnableRepair())
            {
                qDebug() << "P3DSNextGen::loadFile UnableRepair 1, file is " << filePath;
                backupErrorModel(obj);
                return false;
            }

            // 执行模型修复
            if (repairModel(obj) == false)
            {
                qDebug() << "P3DSNextGen::loadFile repairModel failed, file is " << filePath;
                return false;
            }

            // 再次判断是否修复成功
            if (obj->UnableRepair())
            {
                qDebug() << "P3DSNextGen::loadFile UnableRepair 2, file is " << filePath;
                backupErrorModel(obj);
                return false;
            }
        }

        // 读取的数据添加到MainModel中去
        qDebug() << "P3DSNextGen::loadFile add_object";
        ModelObject *new_obj = VTKDataManager::getInstance()->MainModel.add_object(*obj);
        // 设置基台归属
        new_obj->Set_bed_type("None");
        // 拆分物体
        qDebug() << "P3DSNextGen::loadFile split_volumes";
        new_obj->split_volumes();

        // 再次判断是否修复
        if (new_obj->UnableRepair())
        {
            qDebug() << "P3DSNextGen::loadFile UnableRepair 3, new_obj file is " << filePath;
            backupErrorModel(new_obj);
            size_t idx = VTKDataManager::getInstance()->MainModel.objects.size() - 1;
            VTKDataManager::getInstance()->MainModel.delete_object(idx);
            return false;
        }

        // 新建物体 以物体包围盒中心为mesh原点
        // 创建instance
        qDebug() << "P3DSNextGen::loadFile add instance";

        if (new_obj->instances.size() == 0)
        {
            Vectorf3 trans = new_obj->center_around_origin(false, false, true);
            ModelInstance *instance = new_obj->add_instance(); // 添加一个默认的instance
            instance->offset = Pointf(-trans.x, -trans.y);     // 初始位移设置
            qDebug() << "P3DSNextGen::loadFile instance offset is " << -trans.x << ", " << -trans.y;
        }

        // 添加显示
        qDebug() << "P3DSNextGen::loadFile AddObjectActor";
        if (ui->widgetVTK->AddObjectActor(new_obj))
            qDebug() << "P3DSNextGen::loadFile AddObjectActor completed";
        else
            qDebug() << "P3DSNextGen::loadFile AddObjectActor failed";

        qDebug() << "P3DSNextGen::loadFile AddSupportActor";
        if (ui->widgetVTK->AddSupportActor(new_obj))
            qDebug() << "P3DSNextGen::loadFile AddSupportActor completed";
        else
            qDebug() << "P3DSNextGen::loadFile AddSupportActor failed";

        qDebug() << "P3DSNextGen::loadFile VTKDataManager::getInstance()->MainPrint add_model_object";
        VTKDataManager::getInstance()->MainPrint.add_model_object(new_obj);
        size_t obj_idx = VTKDataManager::getInstance()->MainPrint.objects.size() - 1;

        // 设置默认切片参数，参数模块未集成，待添加

        // 导入物体在盘外依次摆放,ptl文件位置保持不变,plater_shape未赋值，暂不执行
        if (fileinfo.suffix() != ".ptl")
        {
            // VTKDataManager::getInstance()->MainPrint.get_object(obj_idx)->Set_pos_onplater();
        }

        // 更新物体信息
        qDebug() << "P3DSNextGen::loadFile GetObjectInfo";
        ObjectInfo obj_info = GetObjectInfo(new_obj);
        qDebug() << "P3DSNextGen::loadFile GetObjectListInfo";
        ObjectListInfo obj_list_info = GetObjectListInfo(new_obj);

        // 计算二维投影
        qDebug() << "P3DSNextGen::loadFile update_thumbnails";
        QTime thumbnail_startTime = QTime::currentTime();
        new_obj->update_thumbnails();
        // 将投影写入svg文件以供查看
        QString svgPath = FileOperations::getCurrentPath() + "/" + QString::fromStdString(new_obj->name) + "_thumbnail.svg";
        SVG svg1(svgPath.toStdString().c_str());
        svg1.draw(new_obj->thumbnails, "red");
        svg1.Close();

        QTime thumbnail_endTime = QTime::currentTime();
        int thumbnail_elapsed = thumbnail_startTime.msecsTo(thumbnail_endTime); // 以毫秒为单位
        qDebug() << "P3DSNextGen::loadFile update_thumbnails cost time:" << thumbnail_elapsed << "ms";
        qDebug() << "P3DSNextGen::loadFile update_thumbnails successfully, svg file is " << svgPath;

        // 计算激光归属set_model_object_bed_type，待添加

        // 更新列表，待添加

        // 更新按钮状态，待添加
        bObjectLoaded = true;

        qDebug() << "P3DSNextGen::loadFile end, load successfully";
        return true;
    }
    catch (const std::exception &e)
    {
        // 捕获并处理标准异常
        QMessageBox::critical(nullptr, "Error", QString("Caught an exception: ") + e.what());
        return false;
    }
    catch (...)
    {
        // 捕获所有其他类型的异常
        QMessageBox::critical(nullptr, "Error", "An unknown exception occurred!");
        return false;
    }
}

bool MainWindow::loadFileList(const QStringList files)
{
    qDebug() << "P3DSNextGen::loadFileList satrt";
    if (files.empty())
    {
        qDebug() << "P3DSNextGen::loadFileList list is empty";
        return false;
    }

    foreach (const QString &filePath, files)
    {
        qDebug() << "P3DSNextGen::loadFileList start to load" << filePath;
        QTime startTime = QTime::currentTime();

        if (loadFile(filePath))
        {
            qDebug() << "P3DSNextGen::loadFileList " << filePath << "load successfully";
        }
        else
        {
            qDebug() << "P3DSNextGen::loadFileList " << filePath << "load failed";
        }
        QTime endTime = QTime::currentTime();
        int elapsed = startTime.msecsTo(endTime); // 以毫秒为单位
        qDebug() << "P3DSNextGen::loadFileList load a file cost time:" << elapsed << "ms";
    }

    qDebug() << "P3DSNextGen::loadFileList end";

    return true;
}

bool MainWindow::loadDirectory(const QString &dir)
{
    // 定义起始目录
    QString selectedDir = recent_skein_directory;
    if (recent_skein_directory.isEmpty() || !QDir().exists(recent_skein_directory))
        selectedDir = QDir::homePath();

    // 定义对话框标题
    QString title = QStringLiteral("选择文件夹");

    // 获取用户选择的目录路径
    QString directory = QFileDialog::getExistingDirectory(
        nullptr,    // 父窗口，nullptr 表示没有父窗口
        title,      // 对话框标题
        selectedDir // 初始目录，默认打开用户的主目录
    );

    // 检查用户是否取消了操作
    if (directory.isEmpty())
    {
        qDebug() << "用户取消了操作";
        return false;
    }

    // 设置要筛选的文件扩展名列表
    QStringList filters;
    filters << "*.stl *ipd *.obj *.ptl"; // 这里可以添加更多的扩展名，用空格分隔

    // 使用 QDir 遍历目录
    QDir dirinfo(dir);
    dirinfo.setNameFilters(filters); // 设置筛选器
    dirinfo.setFilter(QDir::Files);  // 只筛选文件

    // 遍历目录中的文件
    QStringList list = dirinfo.entryList();
    if (list.isEmpty())
    {
        qDebug() << "文件夹中没有可导入的文件";
        return false;
    }

    foreach (const QString &file, list)
    {
        qDebug() << file;
        // 添加读取文件操作
    }

    return true;
}
void MainWindow::connectSignals(QWidget *widget)
{
    // setAcceptDrops(true);
    connect(ui->openFileButton, &QPushButton::clicked, this, &MainWindow::openFileSlot);
    connect(ui->rotateButton, &QPushButton::clicked, this, &MainWindow::RotateSlot);
}

void MainWindow::setupWidget(QWidget *widget)
{
    // setupWidgetLayoutAndStyle(widget);
    connectSignals(widget);
}