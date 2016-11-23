#ifndef UTIL_H
#define UTIL_H
#include <QDateTime>
#include <QString>
#include <QListView>
#include <QFile>
#include <QStringListModel>
#include <QTextStream>

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

    static void loadListView(QFile* file,QListView* object)
    {
        if(file->open(QFile::ReadOnly))
        {

            QTextStream stream(file);


            QStringListModel *model = new QStringListModel(object->parent());

            QStringList list;

            // ui->listViewObjectTypes->setmo

            while(!stream.atEnd())
            {
                QString str = stream.readLine();

                list<<str;

               // qDebug()<<str;

                //ui->labelsComboBox->addItem(str);

            }

            model->setStringList(list);

            object->setModel(model);
            object->setSelectionMode(QAbstractItemView::MultiSelection);
            object->setEditTriggers(QAbstractItemView::NoEditTriggers);
        }

    }

};

#endif // UTIL_H
