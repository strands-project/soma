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

    if(argc < 2)
    {

        std::cout<<
                    "Running the soma_visualizer with default arguments: MongoDB HostName:localhost, MongoDB Port:62345, ObjectsDBName: somadata, ObjectsCollectionName:object, ROIDBName:roi"
                 <<std::endl;
        // std::cout << "Not enough input arguments!! Quitting..."<<std::endl;

        //  return -1;

    }
    else
    {
        if(argc > 1)
        {
            objectsdbname = argv[1];
            mw.rosthread.setSOMAObjectsDBName(objectsdbname);

        }
        if(argc >2)
        {
            objectscollectionname = argv[2];
            mw.rosthread.setSOMAObjectsCollectionName(objectscollectionname);
        }   
        if(argc >3){
            roidb = argv[3];
            mw.rosthread.setSOMAROIDBName(roidb);
        }
        if(argc > 4)
        {
            std::string str = argv[4];
            mw.rosthread.setSOMAROICollectionName(str);
        }

    }

    qRegisterMetaType<std::vector<std::string> >();
    qRegisterMetaType<std::vector<SOMAROINameIDConfig> >();

    //RosThread thread();


    mw.show();

    return app.exec();




}
