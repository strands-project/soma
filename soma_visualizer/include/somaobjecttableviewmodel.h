#ifndef SOMAOBJECTTABLEVIEWMODEL_H
#define SOMAOBJECTTABLEVIEWMODEL_H
#include <QAbstractTableModel>
#include <soma_msgs/SOMAObject.h>
#include <vector>
#include "util.h"
#include<QDebug>
using namespace std;

class SomaObjectTableViewModel : public QAbstractTableModel
{
    Q_OBJECT
public:
    SomaObjectTableViewModel(QObject *parent);
    int rowCount(const QModelIndex &parent = QModelIndex()) const Q_DECL_OVERRIDE ;
    int columnCount(const QModelIndex &parent = QModelIndex()) const Q_DECL_OVERRIDE;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const Q_DECL_OVERRIDE;
    QVariant headerData(int section, Qt::Orientation orientation, int role) const Q_DECL_OVERRIDE;
    void setSOMAObjects(vector<soma_msgs::SOMAObject> objects);
private:
    vector<soma_msgs::SOMAObject > somaobjects;

};

#endif // SOMAOBJECTTABLEVIEWMODEL_H

