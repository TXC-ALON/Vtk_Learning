#ifndef JSONHANDLER_H
#define JSONHANDLER_H

#include <QObject>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonValue>
#include <QFile>
#include <QString>
#include <QByteArray>
#include <QTextStream>
#include <qdebug.h>
class JSONHandler : public QObject
{
    Q_OBJECT

public:
    explicit JSONHandler(QObject *parent = nullptr);

    bool readJsonFile(const QString &filePath);
    bool writeJsonFile(const QString &filePath) const;

    QJsonValue getValue(const QStringList &keys) const;              // 查找读取数据
    void setValue(const QStringList &keys, const QJsonValue &value); // 增改数据
    void removeValue(const QStringList &keys);                       // 删除指定条目数据

    bool parseJsonString(const QString &jsonString);                                 // 解析Qstring为QJsonObject
    QString getJsonString() const;                                                   // 将QJsonObject转为Qstring
    QJsonObject getJsonObject() const;                                               // 给出当前的QJsonObject
    void printJsonObject(const QJsonObject &obj, const QString &prefix = QString()); // 打印json

private:
    QJsonObject jsonObject;

private:
    // 设置嵌套键值的函数
    static void setvalue(QJsonObject &jsonObj, const QStringList &keys, const QJsonValue &value);
    // 删除嵌套键值的函数
    static bool removevalue(QJsonObject &jsonObj, const QStringList &keys);
    // 保存JSON到文件
    static bool saveJsonToFile(const QJsonObject &jsonObj, const QString &filePath);
};

#endif // JSONHANDLER_H

/*
    #define TEST_JSON "D:/0Learning/Vtk/0805_qt5_vtk/Test/file.json"
    #define OUTPUT_JSON "D:/0Learning/Vtk/0805_qt5_vtk/Test/output.json"
    // 获取根对象
    JSONHandler testHandler;
    testHandler.readJsonFile(TEST_JSON);
    QStringList keys = {"address", "location", "coussntry", "ss"};
    QStringList keys2 = {"address", "village"};

    QJsonValue newValue = "UU";
    testHandler.setValue(keys, newValue);
    testHandler.removeValue(keys2);
    // 将修改后的QJsonObject转换回QJsonDocument
    // QJsonDocument updatedJsonDoc(rootObj);

    // 将QJsonDocument转换回JSON字符串
    // QString updatedJsonString = QString::fromUtf8(updatedJsonDoc.toJson(QJsonDocument::Indented));

    // 输出修改后的JSON字符串
    testHandler.writeJsonFile(OUTPUT_JSON);

    return 0;
 */