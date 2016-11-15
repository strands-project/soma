#include "../include/mainwindow.h"
#include <QApplication>

using namespace Qt;

int main(int argc, char **argv){

    QApplication app(argc,argv);

    ros::init(argc, argv, "soma_visualizer_node");

    std::string objectsdbname;
    std::string objectscollectionname;
    std::string roidb;

    MainWindow mw;



    std::cout<<"Running the soma_visualizer..."<<std::endl;




    qRegisterMetaType<std::vector<std::string> >();
    qRegisterMetaType<std::vector<SOMAROINameIDConfig> >();


    mw.show();

    return app.exec();




}
