#pragma once

#include <QtWidgets\QWidget>
#include <ui_RotateWidget.h>

namespace Ui
{
	class RotateWidget;
}

class RotateWidget : public QWidget
{
	Q_OBJECT

public:
	RotateWidget(QWidget *parent = nullptr);
	~RotateWidget();
	void setupWidget(QWidget *widget);
	void InitOpenPage();

private:
	void setupWidgetLayoutAndStyle(QWidget *widget);
	void connectSignals(QWidget *widget);

private slots:
	// X,Y,Z,dX,dY,dZ值事件
	void onValueChangedX(double value);
	void onValueChangedY(double value);
	void onValueChangedZ(double value);
	void closeSlot();
	void okSlot();
	void applySlot();

private:
	Ui::RotateWidget *ui;
	double temp_x;
	double temp_y;
	double temp_z;
signals:
	void Rotate_update_Signal(); // 自定义信号
	void Rotate_OK_Signal();	 // 自定义信号
};
