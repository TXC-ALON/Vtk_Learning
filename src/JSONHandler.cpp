#include <JsonHandler.h>

JSONHandler::JSONHandler(QObject *parent) : QObject(parent) {}

bool JSONHandler::readJsonFile(const QString &filePath)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        return false;
    }

    QByteArray jsonData = file.readAll();
    file.close();

    QJsonParseError parseError;
    QJsonDocument document = QJsonDocument::fromJson(jsonData, &parseError);
    if (parseError.error != QJsonParseError::NoError)
    {
        return false;
    }

    jsonObject = document.object();
    return true;
}

bool JSONHandler::writeJsonFile(const QString &filePath) const
{
    return saveJsonToFile(jsonObject, filePath);
}
// 定义保存JSON到文件的函数
bool JSONHandler::saveJsonToFile(const QJsonObject &jsonObj, const QString &filePath)
{
    QFile file(filePath);

    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qWarning("Could not open file for writing.");
        return false;
    }

    QJsonDocument jsonDoc(jsonObj);
    file.write(jsonDoc.toJson(QJsonDocument::Indented));
    file.close();

    return true;
}
bool JSONHandler::parseJsonString(const QString &jsonString)
{
    QJsonParseError parseError;
    QJsonDocument document = QJsonDocument::fromJson(jsonString.toUtf8(), &parseError);
    if (parseError.error != QJsonParseError::NoError)
    {
        return false;
    }

    jsonObject = document.object();
    return true;
}
void JSONHandler::setValue(const QStringList &keys, const QJsonValue &value) // 增改数据
{
    QJsonObject rootObj = jsonObject;
    setvalue(rootObj, keys, value);
    jsonObject = rootObj;
}
void JSONHandler::removeValue(const QStringList &keys) // 删除指定条目数据
{
    QJsonObject rootObj = jsonObject;
    removevalue(rootObj, keys);
    jsonObject = rootObj;
}
QString JSONHandler::getJsonString() const
{
    QJsonDocument document(jsonObject);
    return QString::fromUtf8(document.toJson(QJsonDocument::Indented));
}

QJsonObject JSONHandler::getJsonObject() const
{
    return jsonObject;
}

QJsonValue JSONHandler::getValue(const QStringList &keys) const
{
    QJsonValue value = jsonObject;
    for (const QString &key : keys)
    {
        if (value.isObject())
        {
            value = value.toObject().value(key);
        }
        else
        {
            return QJsonValue();
        }
    }
    return value;
}
void JSONHandler::printJsonObject(const QJsonObject &obj, const QString &prefix)
{
    // 遍历QJsonObject中的所有键值对
    for (auto it = obj.begin(); it != obj.end(); ++it)
    {
        QString currentKey = it.key();
        QJsonValue value = it.value();

        // 打印当前键
        qDebug() << prefix + currentKey;

        // 如果当前值是一个嵌套的对象，递归打印
        if (value.isObject())
        {
            printJsonObject(value.toObject(), prefix + currentKey + ".");
        }
        else if (value.isArray())
        {
            // 如果当前值是一个数组，打印数组的每个元素（数组元素不是对象则直接打印）
            const QJsonArray array = value.toArray();
            for (int i = 0; i < array.size(); ++i)
            {
                QJsonValue arrayValue = array.at(i);
                if (arrayValue.isObject())
                {
                    // 嵌套对象，递归打印
                    qDebug() << prefix + currentKey + "[" + QString::number(i) + "] = {...}";
                    printJsonObject(arrayValue.toObject(), prefix + currentKey + "[" + QString::number(i) + "].");
                }
                else
                {
                    // 打印数组元素
                    qDebug() << prefix + currentKey + "[" + QString::number(i) + "] = " + arrayValue.toString();
                }
            }
        }
        else
        {
            // 打印值
            qDebug() << "  " + value.toString();
        }
    }
}
void JSONHandler::setvalue(QJsonObject &jsonObj, const QStringList &keys, const QJsonValue &value)
{
    if (keys.isEmpty())
    {
        return;
    }

    QString key = keys.first();

    if (keys.size() == 1)
    {
        // 最后一级键，直接设置值
        jsonObj[key] = value;
    }
    else
    {
        // 获取下一层对象，如果不存在则创建一个新的 QJsonObject
        QJsonObject nestedObj = jsonObj.value(key).toObject();
        QStringList remainingKeys = keys.mid(1);

        // 递归调用设置值
        setvalue(nestedObj, remainingKeys, value);

        // 更新当前层对象
        jsonObj[key] = nestedObj;
    }
}
// 删除嵌套键值的函数
bool JSONHandler::removevalue(QJsonObject &jsonObj, const QStringList &keys)
{
    if (keys.isEmpty())
    {
        return false;
    }

    QString key = keys.first();
    if (keys.size() == 1)
    {
        // 最后一级键，删除值
        if (jsonObj.contains(key))
        {
            jsonObj.remove(key);
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        // 获取下一层对象
        QJsonObject nestedObj = jsonObj[key].toObject();
        QStringList remainingKeys = keys.mid(1);

        // 递归调用删除值
        bool result = removevalue(nestedObj, remainingKeys);

        // 如果内部键值被成功删除，更新当前层对象
        if (result)
        {
            if (nestedObj.isEmpty())
            {
                jsonObj.remove(key); // 如果删除后对象为空，移除整个对象
            }
            else
            {
                jsonObj[key] = nestedObj; // 否则更新当前层对象
            }
        }

        return result;
    }
}