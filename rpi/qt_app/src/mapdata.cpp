#include "mapdata.h"

#include <QFile>
#include <QRegularExpression>
#include <QTextStream>

static int indentOf(const QString &line)
{
    int n = 0;
    while (n < line.size() && line[n].isSpace()) {
        ++n;
    }
    return n;
}

static QString stripComment(QString line)
{
    const int idx = line.indexOf('#');
    if (idx >= 0) {
        line.truncate(idx);
    }
    return line.trimmed();
}

static bool parseFloatValue(const QString &line, const QString &key, float *out)
{
    const QString prefix = key + ":";
    if (!line.startsWith(prefix)) {
        return false;
    }

    bool ok = false;
    const float value = line.mid(prefix.size()).trimmed().toFloat(&ok);
    if (ok) {
        *out = value;
    }
    return ok;
}

static bool parsePointArray(const QString &text, float *x, float *y)
{
    static const QRegularExpression re(R"(\[\s*([-+]?\d*\.?\d+)\s*,\s*([-+]?\d*\.?\d+)\s*\])");
    const QRegularExpressionMatch match = re.match(text);
    if (!match.hasMatch()) {
        return false;
    }

    bool okX = false;
    bool okY = false;
    const float px = match.captured(1).toFloat(&okX);
    const float py = match.captured(2).toFloat(&okY);
    if (!okX || !okY) {
        return false;
    }

    *x = px;
    *y = py;
    return true;
}

void MapConfig::clear()
{
    m_valid = false;
    m_areas.clear();
    m_obstacles.clear();
    m_starts.clear();
    m_end = {};
    m_hasEnd = false;
    m_minX = 0.0f;
    m_maxX = 0.0f;
    m_minY = 0.0f;
    m_maxY = 0.0f;
    m_haveBounds = false;
}

void MapConfig::includePoint(float x, float y)
{
    if (!m_haveBounds) {
        m_minX = m_maxX = x;
        m_minY = m_maxY = y;
        m_haveBounds = true;
        return;
    }

    m_minX = qMin(m_minX, x);
    m_maxX = qMax(m_maxX, x);
    m_minY = qMin(m_minY, y);
    m_maxY = qMax(m_maxY, y);
}

void MapConfig::includeRect(const MapRect &rect)
{
    includePoint(rect.xMin, rect.yMin);
    includePoint(rect.xMax, rect.yMax);
}

bool MapConfig::loadFromYaml(const QString &path, QString *error)
{
    clear();

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        if (error) {
            *error = QString("cannot open %1").arg(path);
        }
        return false;
    }

    QVector<QString> lines;
    QTextStream in(&file);
    while (!in.atEnd()) {
        lines.append(in.readLine());
    }

    for (int i = 0; i < lines.size(); ++i) {
        const QString raw = lines[i];
        const QString line = stripComment(raw);
        if (line.isEmpty()) {
            continue;
        }

        if (line == "starts:") {
            const int baseIndent = indentOf(raw);
            for (++i; i < lines.size(); ++i) {
                const QString childRaw = lines[i];
                const QString child = stripComment(childRaw);
                if (child.isEmpty()) {
                    continue;
                }
                if (indentOf(childRaw) <= baseIndent) {
                    --i;
                    break;
                }

                const int colon = child.indexOf(':');
                if (colon <= 0) {
                    continue;
                }

                MapPoint pt;
                pt.id = child.left(colon).trimmed();
                if (parsePointArray(child.mid(colon + 1), &pt.x, &pt.y)) {
                    m_starts.append(pt);
                    includePoint(pt.x, pt.y);
                }
            }
            continue;
        }

        if (line.startsWith("end:")) {
            if (parsePointArray(line.mid(QString("end:").size()), &m_end.x, &m_end.y)) {
                m_end.id = "end";
                m_hasEnd = true;
                includePoint(m_end.x, m_end.y);
            }
            continue;
        }

        if (line == "obstacles:") {
            const int baseIndent = indentOf(raw);
            for (++i; i < lines.size(); ++i) {
                QString itemRaw = lines[i];
                QString item = stripComment(itemRaw);
                if (item.isEmpty()) {
                    continue;
                }
                if (indentOf(itemRaw) <= baseIndent) {
                    --i;
                    break;
                }
                if (!item.startsWith("-")) {
                    continue;
                }

                MapRect rect;
                item = item.mid(1).trimmed();
                if (item.startsWith("id:")) {
                    rect.id = item.mid(3).trimmed();
                }

                const int itemIndent = indentOf(itemRaw);
                for (++i; i < lines.size(); ++i) {
                    const QString propRaw = lines[i];
                    const QString prop = stripComment(propRaw);
                    if (prop.isEmpty()) {
                        continue;
                    }
                    if (indentOf(propRaw) <= itemIndent) {
                        --i;
                        break;
                    }

                    if (prop.startsWith("id:")) {
                        rect.id = prop.mid(3).trimmed();
                    }
                    parseFloatValue(prop, "x_min", &rect.xMin);
                    parseFloatValue(prop, "x_max", &rect.xMax);
                    parseFloatValue(prop, "y_min", &rect.yMin);
                    parseFloatValue(prop, "y_max", &rect.yMax);
                    parseFloatValue(prop, "clearance", &rect.clearance);
                    parseFloatValue(prop, "z", &rect.z);
                }

                if (rect.xMax > rect.xMin && rect.yMax > rect.yMin) {
                    m_obstacles.append(rect);
                    includeRect(rect);
                }
            }
            continue;
        }

        if (line.endsWith(":")) {
            const QString key = line.left(line.size() - 1).trimmed();
            if (key == "map" || key == "waypoint_sampling") {
                continue;
            }

            MapRect rect;
            rect.id = key;
            const int baseIndent = indentOf(raw);
            bool hasRectField = false;
            for (++i; i < lines.size(); ++i) {
                const QString propRaw = lines[i];
                const QString prop = stripComment(propRaw);
                if (prop.isEmpty()) {
                    continue;
                }
                if (indentOf(propRaw) <= baseIndent) {
                    --i;
                    break;
                }

                float value = 0.0f;
                if (parseFloatValue(prop, "x_min", &value)) {
                    rect.xMin = value;
                    hasRectField = true;
                } else if (parseFloatValue(prop, "x_max", &value)) {
                    rect.xMax = value;
                    hasRectField = true;
                } else if (parseFloatValue(prop, "y_min", &value)) {
                    rect.yMin = value;
                    hasRectField = true;
                } else if (parseFloatValue(prop, "y_max", &value)) {
                    rect.yMax = value;
                    hasRectField = true;
                } else if (parseFloatValue(prop, "z", &value)) {
                    rect.z = value;
                }
            }

            if (hasRectField && rect.xMax > rect.xMin && rect.yMax > rect.yMin) {
                m_areas.append(rect);
                includeRect(rect);
            }
        }
    }

    m_valid = m_haveBounds;
    if (!m_valid && error) {
        *error = QString("no map bounds found in %1").arg(path);
    }
    return m_valid;
}
