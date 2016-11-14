#ifndef SOMAOBJECTDETAILDIALOG_H
#define SOMAOBJECTDETAILDIALOG_H

#include <QDialog>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <ros/ros.h>
#include <QColor>

namespace Ui {
class SomaObjectDetailDialog;
}

class SomaObjectDetailDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SomaObjectDetailDialog(QWidget *parent = 0);
    ~SomaObjectDetailDialog();

private:
    Ui::SomaObjectDetailDialog *ui;
    rviz::VisualizationManager* manager;
    rviz::RenderPanel* renderpanel;
    rviz::Display* grid;

    //manager.createDisplay("rviz/PointCloud2","pointclouds")
};

#endif // SOMAOBJECTDETAILDIALOG_H
