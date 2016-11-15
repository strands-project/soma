#include "somaobjecttableviewmodel.h"

SomaObjectTableViewModel::SomaObjectTableViewModel(QObject *parent)
                                                   :QAbstractTableModel(parent)
{


}
int SomaObjectTableViewModel::rowCount(const QModelIndex & /*parent*/) const
{
   return this->somaobjects.size();
}

int SomaObjectTableViewModel::columnCount(const QModelIndex & /*parent*/) const
{
    return 3;
}

QVariant SomaObjectTableViewModel::data(const QModelIndex &index, int role) const
{
    if (role == Qt::DisplayRole && index.column() == 0) // id
    {
       return QString("%1")
                   .arg(QString::fromStdString(this->somaobjects[index.row()].id));

    }
    else if (role == Qt::DisplayRole && index.column() == 1) // type
    {
       return QString("%1")
                   .arg(QString::fromStdString(this->somaobjects[index.row()].type));

    }
    else if (role == Qt::DisplayRole && index.column() == 2) // logdate
    {
       // long uppertimestamp = (this->timelimits.mintimestamp+(step)*this->timestep)*1000;

        long timestamp = Util::convertSecTimestamptoMSec(this->somaobjects[index.row()].logtimestamp);

        QDateTime dt = Util::calculateUTCDateTimeFromTimestamp(timestamp);

        QString str = dt.toString(datetimeformat);

        return str;

    }
    return QVariant();
}
void SomaObjectTableViewModel::setSOMAObjects(vector<soma_msgs::SOMAObject> objects)
{
    this->somaobjects = objects;
}
QVariant SomaObjectTableViewModel::headerData(int section, Qt::Orientation orientation, int role) const // Header part of the table
{
    if (role == Qt::DisplayRole)
    {
        if (orientation == Qt::Horizontal) {
            switch (section)
            {
            case 0:
                return QString("id");
            case 1:
                return QString("type");
            case 2:
                return QString("logtime");
            }
        }
    }
    return QVariant();
}

