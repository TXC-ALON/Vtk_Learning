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
    bool parseJsonString(const QString &jsonString);
    QString getJsonString() const;
    QJsonObject getJsonObject() const; // 添加此方法
    bool writeJsonFileStream(const QString &filePath, const QJsonObject &jsonObject);
    QJsonValue getValue(const QStringList &keys) const;
    void setValue(const QStringList &keys, const QJsonValue &value);

public:
    QJsonObject jsonObject;
};

#endif // JSONHANDLER_H