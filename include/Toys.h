#pragma once
#include <Vtk_headers.h>

// 自定义的参数化莫比乌斯环类
class CustomMobius : public vtkParametricFunction
{
public:
    vtkTypeMacro(CustomMobius, vtkParametricFunction);

    static CustomMobius *New()
    {
        return new CustomMobius();
    }

    // 构造函数，设置U和V的范围
    CustomMobius()
    {
        this->MinimumU = 0.0;
        this->MaximumU = 2.0 * vtkMath::Pi();
        this->MinimumV = -1.0;
        this->MaximumV = 1.0;
        this->JoinU = 1;
        this->JoinV = 0;
        this->TwistU = 1;
        this->TwistV = 0;
        this->ClockwiseOrdering = 1;
        this->DerivativesAvailable = 1;
    }
    // 实现 GetDimension 方法
    int GetDimension() override
    {
        return 2; // 莫比乌斯环是一个二维曲面
    }

    // 实现 EvaluateScalar 方法
    double EvaluateScalar(double *, double *, double *) override
    {
        return 0.0; //
    }
    // 计算莫比乌斯环的参数化方程
    void Evaluate(double uvw[3], double Pt[3], double Duvw[9]) override
    {
        double u = uvw[0];
        double v = uvw[1];

        // 自定义的莫比乌斯环参数方程
        double R = 1.0 + 0.5 * v * cos(u / 2.0);
        Pt[0] = R * cos(u);             // x坐标
        Pt[1] = R * sin(u);             // y坐标
        Pt[2] = 0.5 * v * sin(u / 2.0); // z坐标

        // 计算一阶导数
        double duR = -0.25 * v * sin(u / 2.0);
        double duX = duR * cos(u) - R * sin(u);
        double duY = duR * sin(u) + R * cos(u);
        double duZ = 0.25 * v * cos(u / 2.0);

        double dvR = 0.5 * cos(u / 2.0);
        double dvX = dvR * cos(u);
        double dvY = dvR * sin(u);
        double dvZ = 0.5 * sin(u / 2.0);

        // 填充导数数组
        Duvw[0] = duX; // du/dx
        Duvw[1] = duY; // du/dy
        Duvw[2] = duZ; // du/dz

        Duvw[3] = dvX; // dv/dx
        Duvw[4] = dvY; // dv/dy
        Duvw[5] = dvZ; // dv/dz

        // 这里我们不计算二阶导数，直接设置为0
        Duvw[6] = Duvw[7] = Duvw[8] = 0.0;
    }
};

class HeartShapeMobius : public vtkParametricFunction
{
public:
    vtkTypeMacro(HeartShapeMobius, vtkParametricFunction);

    static HeartShapeMobius *New()
    {
        return new HeartShapeMobius();
    }

    // 构造函数，设置U和V的范围
    HeartShapeMobius()
    {
        this->MinimumU = 0.0;
        this->MaximumU = 2.0 * vtkMath::Pi();
        this->MinimumV = -1.0;
        this->MaximumV = 1.0;
        this->JoinU = 1;
        this->JoinV = 0;
        this->TwistU = 1;
        this->TwistV = 0;
        this->ClockwiseOrdering = 1;
        this->DerivativesAvailable = 1;
    }

    // 计算心形莫比乌斯环的参数化方程
    void Evaluate(double uvw[3], double Pt[3], double Duvw[9]) override
    {
        double u = uvw[0];
        double v = uvw[1];

        // 心形方程
        double xHeart = 16 * pow(sin(u), 3);
        double yHeart = 13 * cos(u) - 5 * cos(2 * u) - 2 * cos(3 * u) - cos(4 * u);
        double zHeart = sin(u) * pow((1 - abs(-u / PI)), 2) * 8;

        // 自定义的心形莫比乌斯环参数方程
        double R = 1.0 + 0.5 * v * cos(u / 2.0);

        Pt[0] = R * xHeart; // x坐标
        Pt[1] = R * yHeart; // y坐标
        Pt[2] = R * zHeart; // z坐标

        // 计算一阶导数
        double duR = -0.25 * v * sin(u / 2.0);
        double duX = duR * xHeart + R * 48 * pow(sin(u), 2) * cos(u);
        double duY = duR * yHeart - R * (13 * sin(u) - 10 * sin(2 * u) - 6 * sin(3 * u) - 4 * sin(4 * u));
        double abs_u_over_pi = std::fabs(-u / PI);
        double term1 = std::sin(u);
        double term2 = (1 - abs_u_over_pi);
        double term3 = 2 * (1 - abs_u_over_pi) * (-1 / PI);
        double duZ = 8 * (term1 * term2 * term3 + std::cos(u) * term2 * term2);

        double dvR = 0.5 * cos(u / 2.0);
        double dvX = dvR * xHeart;
        double dvY = dvR * yHeart;
        double dvZ = dvR * zHeart;

        // 填充导数数组
        Duvw[0] = duX; // du/dx
        Duvw[1] = duY; // du/dy
        Duvw[2] = duZ; // du/dz

        Duvw[3] = dvX; // dv/dx
        Duvw[4] = dvY; // dv/dy
        Duvw[5] = dvZ; // dv/dz

        // 这里我们不计算二阶导数，直接设置为0
        Duvw[6] = Duvw[7] = Duvw[8] = 0.0;
    }

    // 返回曲面的维度（2D曲面）
    int GetDimension() override
    {
        return 2;
    }

    // 计算标量值
    double EvaluateScalar(double uvw[3], double Pt[3], double Duvw[9]) override
    {
        // 对于这个示例，返回一个常量值
        return 0.0;
    }
};