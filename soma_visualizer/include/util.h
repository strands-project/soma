#ifndef UTIL_H
#define UTIL_H
#include <QDateTime>
#include <QString>

static QString datetimeformat = "dd-MM-yyyy hh:mm";

class Util
{
public:


    static QDateTime calculateUTCDateTimeFromTimestamp(long timestamp)
    {
        QDateTime dt = QDateTime::fromMSecsSinceEpoch(timestamp,Qt::UTC);

        return dt;

    }

    static long convertSecTimestamptoMSec(long timestamp)
    {
        return timestamp*1000;
    }

};

#endif // UTIL_H
