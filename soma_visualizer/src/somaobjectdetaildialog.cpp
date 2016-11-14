#include "somaobjectdetaildialog.h"
#include "ui_somaobjectdetaildialog.h"

SomaObjectDetailDialog::SomaObjectDetailDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SomaObjectDetailDialog)
{
    ui->setupUi(this);

  /*  this->renderpanel  = new rviz::RenderPanel(this);

    manager = new rviz::VisualizationManager( renderpanel );
    renderpanel->initialize( manager->getSceneManager(), manager );
    manager->initialize();
    manager->startUpdate();

    // Create a Grid display.
    grid = manager->createDisplay( "rviz/Grid", "adjustable grid", true );
    ROS_ASSERT( grid_ != NULL );*/

    // Configure the GridDisplay the way we like it.
   // grid->subProp( "Line Style" )->setValue( "Billboards" );
  //  grid->subProp( "Color" )->setValue( "yellow" );




   // ui->pclWidget  = this->renderpanel;

}

SomaObjectDetailDialog::~SomaObjectDetailDialog()
{
    delete ui;
}
