#include <JSONHandler.h>

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
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        return false;
    }

    QJsonDocument document(jsonObject);
    file.write(document.toJson(QJsonDocument::Indented));
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

void JSONHandler::setValue(const QStringList &keys, const QJsonValue &value)
{
    QJsonObject *currentObject = &jsonObject;
    foreach (const QString &key, currentObject->keys())
    {
        qDebug() << key << ":" << currentObject->value(key);
    }
    for (int i = 0; i < keys.size() - 1; ++i)
    {
        QJsonValue currentValue = currentObject->value(keys[i]);
        if (currentValue.isObject())
        {
            currentObject = const_cast<QJsonObject *>(&(currentValue.toObject()));
        }
        else
        {
            QJsonObject newObject;
            currentObject->insert(keys[i], newObject);
            currentObject = &newObject;
        }
    }
    currentObject->insert(keys.last(), value);
}

bool JSONHandler::writeJsonFileStream(const QString &filePath, const QJsonObject &jsonObject)
{
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "Unable to open file for writing:" << file.errorString();
        return false;
    }

    QTextStream out(&file);
    out.setCodec("UTF-8");
    out << "{\n";

    QStringList keys = jsonObject.keys();
    for (int i = 0; i < keys.size(); ++i)
    {
        QString key = keys[i];
        QJsonValue value = jsonObject.value(key);

        out << "\"" << key << "\": ";

        if (value.isObject())
        {
            out << QString(QJsonDocument(value.toObject()).toJson(QJsonDocument::Compact));
        }
        else if (value.isArray())
        {
            out << QString(QJsonDocument(value.toArray()).toJson(QJsonDocument::Compact));
        }
        else
        {
            out << QString(QJsonDocument(QJsonObject{{key, value}}).toJson(QJsonDocument::Compact)).mid(key.length() + 4);
        }

        if (i < keys.size() - 1)
        {
            out << ",\n";
        }
    }

    out << "\n}\n";
    file.close();
    return true;
}