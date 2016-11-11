#ifndef UTIL_H
#define UTIL_H
#include <QDateTime>
#include <QString>

static QString datetimeformat = "dd-MM-yyyy hh:mm";



static QDateTime calculateUTCDateTimeFromTimestamp(long timestamp)
{
    QDateTime dt = QDateTime::fromMSecsSinceEpoch(timestamp,Qt::UTC);

    return dt;

}



#endif // UTIL_H
