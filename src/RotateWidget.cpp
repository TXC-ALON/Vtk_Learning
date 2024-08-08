#include <RotateWidget.h>

RotateWidget::RotateWidget(QWidget *parent)
	: QWidget(parent),
	  ui(new Ui::RotateWidget)
{
	ui->setupUi(this);
	setupWidget(this);
}

RotateWidget::~RotateWidget()
{
	if (ui != nullptr)
	{
		delete ui;
		ui = nullptr;
	}
}

void RotateWidget::setupWidget(QWidget *widget)
{
	setupWidgetLayoutAndStyle(widget);
	connectSignals(widget);
}

void RotateWidget::setupWidgetLayoutAndStyle(QWidget *widget)
{
	// 设置系统窗口样式
	setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint);
}

void RotateWidget::connectSignals(QWidget *widget)
{
	// 单选框
	connect(ui->radioButtonManual, &QRadioButton::toggled, this, [=](bool checked)
			{
			if (checked)
			{
				ui->groupBoxManual->setEnabled(true);
				ui->groupBoxAuto->setEnabled(false);
			} });

	connect(ui->radioButtonAuto, &QRadioButton::toggled, this, [=](bool checked)
			{
			if (checked)
			{
				ui->groupBoxManual->setEnabled(false);
				ui->groupBoxAuto->setEnabled(true);
			} });
	// 窗口初始化时默认手动姿态
	ui->radioButtonManual->setChecked(true);

	connect(ui->doubleSpinBoxX, SIGNAL(valueChanged(double)), this, SLOT(onValueChangedX(double)));
	connect(ui->doubleSpinBoxY, SIGNAL(valueChanged(double)), this, SLOT(onValueChangedY(double)));
	connect(ui->doubleSpinBoxZ, SIGNAL(valueChanged(double)), this, SLOT(onValueChangedZ(double)));

	// 复选框
	connect(ui->checkBoxFlip, &QCheckBox::stateChanged, this, [=](int state)
			{
				bool isChecked = (state == Qt::Checked);
				// TODO 控制位置翻转的变量
			});
	connect(ui->checkBoxAjust, &QCheckBox::stateChanged, this, [=](int state)
			{
				bool isChecked = (state == Qt::Checked);
				ui->doubleSpinBoxAngle->setEnabled(isChecked);
				ui->doubleSpinBoxCircle->setEnabled(isChecked);
				// TODO 控制姿态微调的变量
			});
	connect(ui->checkBoxAutoAujst, &QCheckBox::stateChanged, this, [=](int state)
			{
				bool isChecked = (state == Qt::Checked);
				// TODO 控制支架自动调整的变量
			});

	connect(ui->pushButtonOK, &QPushButton::clicked, this, &RotateWidget::okSlot);
	connect(ui->pushButtonClose, &QPushButton::clicked, this, &RotateWidget::closeSlot);
	connect(ui->pushButtonApply, &QPushButton::clicked, this, &RotateWidget::applySlot);
}

void RotateWidget::InitOpenPage()
{
	// TODO 打开页面时需要初始化的内容
	ui->pushButtonOK->setEnabled(true);
}

void RotateWidget::onValueChangedX(double value)
{
	temp_x = ui->doubleSpinBoxX->value();
}

void RotateWidget::onValueChangedY(double value)
{
	temp_y = ui->doubleSpinBoxY->value();
}

void RotateWidget::onValueChangedZ(double value)
{
	temp_z = ui->doubleSpinBoxZ->value();
}

void RotateWidget::closeSlot()
{
	close();
}

void RotateWidget::okSlot()
{
	// TODO 获取参数值，调整VTK窗口中零件显示姿态，同时关闭窗口
	// TODO 根据groupBoxManual和groupBoxAuto的状态来确定获取的对应的参数
	if (ui->groupBoxManual->isEnabled())
	{
	}
	else if (ui->groupBoxAuto->isEnabled())
	{
		// TODO 自动姿态姿态生效
		// TODO 获取位置翻转开关状态，调整VTK中的模型翻转
		// TODO 获取姿态微调开关状态和角度阈值、微调补偿，调整VTK中的模型
		// TODO 获取支架自动调整开关状态，调整VTK中的支架模型
	}
	close();
}

void RotateWidget::applySlot()
{
	// TODO 获取参数值，调整VTK窗口中零件显示姿态，不关闭窗口
	// TODO 根据groupBoxManual和groupBoxAuto的状态来确定获取的对应的参数
	if (ui->groupBoxManual->isEnabled())
	{
		// TODO 手动姿态生效

		// TODO 获取X,Y,Z的旋转角度值，调整VTK里的模型角度
	}
	else if (ui->groupBoxAuto->isEnabled())
	{
		// TODO 自动姿态姿态生效
		// TODO 获取位置翻转开关状态，调整VTK中的模型翻转

		// TODO 获取姿态微调开关状态和角度阈值、微调补偿，调整VTK中的模型

		// TODO 获取支架自动调整开关状态，调整VTK中的支架模型
	}

	ui->pushButtonOK->setEnabled(true);
}