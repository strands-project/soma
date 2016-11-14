#ifndef SOMAOBJECTDETAILDIALOG_H
#define SOMAOBJECTDETAILDIALOG_H

#include <QDialog>
#include <ros/ros.h>
#include <soma_msgs/SOMAObject.h>
#include <QLabel>
#include <QTextEdit>
#include "util.h"
#include "QImage"
#include <cv_bridge/cv_bridge.h>
namespace Ui {
class SomaObjectDetailDialog;
}

class SomaObjectDetailDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SomaObjectDetailDialog(QWidget *parent = 0);
    SomaObjectDetailDialog(QWidget *parent = 0, soma_msgs::SOMAObject somaobject=soma_msgs::SOMAObject());

    ~SomaObjectDetailDialog();

private slots:
    void on_buttonImageLeft_clicked();

    void on_buttonImageRight_clicked();

private:
    Ui::SomaObjectDetailDialog *ui;
    soma_msgs::SOMAObject somaobject;
    int imageIndex;
    void loadImage();


};

#endif // SOMAOBJECTDETAILDIALOG_H
