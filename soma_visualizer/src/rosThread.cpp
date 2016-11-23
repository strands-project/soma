#include "rosThread.h"
#include <QDebug>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include<pcl_ros/point_cloud.h>
#include <QStringList>
#include <QJsonObject>
#include <QJsonDocument>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QTextStream>
#include <soma_manager/SOMAQueryObjs.h>
#include <soma_manager/SOMAQueryROIs.h>

#define SOMA_QUERY_ROIS 0
#define SOMA_QUERY_TIMELIMITS 2


typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;




RosThread::RosThread()
{
    shutdown = false;

}


void RosThread::loop()
{


    if(!ros::ok()){

        emit rosStartFailed();

        return;
    }

    emit rosStarted();

    ROS_INFO("Ros Thread is running!!");

    ros::ServiceClient map_client = n.serviceClient<soma_map_manager::MapInfo>("soma/map_info");

    this->object_query_client = n.serviceClient<soma_manager::SOMAQueryObjs>("soma/query_objects");

    this->roi_query_client = n.serviceClient<soma_manager::SOMAQueryROIs>("soma/query_rois");

    this->roi_draw_client = n.serviceClient<soma_roi_manager::DrawROI>("soma/draw_roi");

    soma_map_manager::MapInfo srv;

    ROS_INFO("Waiting for map_info service from soma_map_manager");

    if(!map_client.waitForExistence(ros::Duration(30)))
    {
        ROS_WARN("Warning! SOMA map service is not active!! Quitting...");

        emit rosFinished();

        return;

    }

    if(!object_query_client.waitForExistence(ros::Duration(30)))
    {
        ROS_WARN("Warning! SOMA object query service is not active!! Quitting...");

        emit rosFinished();

        return;

    }

    if(!roi_query_client.waitForExistence(ros::Duration(30)))
    {
        ROS_WARN("Warning! SOMA roi query service is not active!! Quitting...");

        emit rosFinished();

        return;

    }




    if (map_client.call(srv))
    {
        ROS_INFO("Map Info: %s Map Unique ID: %s", srv.response.map_name.data(), srv.response.map_unique_id.data());
        this->map_name = srv.response.map_name.data();
        this->map_unique_id = srv.response.map_unique_id.data();


    }
    else
    {
        ROS_ERROR("Failed to call SOMA map service! Quitting...");

        emit rosFinished();

        return;

    }


    pcp = n.advertise<sensor_msgs::PointCloud2>("soma_visualizer_node/world_state_3d",1);

    ros::Rate loop(10);


    this->fetchDataFromDB();




    while(ros::ok())
    {


        // velocityCommandPublisher.publish(velocityCommand);

        ros::spinOnce();

        loop.sleep();


    }
    ROS_INFO("ROS thread finished!!");

    emit rosFinished();



}
void RosThread::fetchDataFromDB()
{
    this->fetchSOMAROIs();

    this->fetchSOMAObjectTypesIDsConfigs();

    emit mapinfoReceived();



}
void RosThread::drawROIwithID(std::string id)
{

    soma_roi_manager::DrawROI drawroi;

    drawroi.request.map_name = this->map_name;
    drawroi.request.roi_id = id;
    drawroi.request.draw_mostrecent = true;

    this->roi_draw_client.call(drawroi);



}
void RosThread::shutdownROS()
{
    ros::shutdown();


}
void RosThread::fetchSOMAObjectTypesIDsConfigs()
{

    std::vector<std::string> somalabels;

    QString dir = QDir::homePath();

    dir.append("/").append(".soma/");

    QDir typesdir;

    if(!typesdir.exists(dir))
        typesdir.mkdir(dir);

    QString filename = "objecttypes.txt";

    QString filename2 = "objectids.txt";

    QString filename3 = "objectconfigs.txt";

    QString objecttypesdir = dir;
    QString idsdir = dir;
    QString configsdir = dir;

    objecttypesdir.append(filename);

    idsdir.append(filename2);

    configsdir.append(filename3);

    QFile file(objecttypesdir);

    if(!file.open(QFile::WriteOnly))
    {
        qDebug()<<"Cannot Open types file! Returning...";
        return ;


    }

    QFile file2(idsdir);

    if(!file2.open(QFile::WriteOnly))
    {
        qDebug()<<"Cannot Open ids file! Returning...";
        return ;


    }

    QFile file3(configsdir);

    if(!file3.open(QFile::WriteOnly))
    {
        qDebug()<<"Cannot Open configs file! Returning...";
        return ;


    }

    soma_manager::SOMAQueryObjs queryobjs;

    queryobjs.request.query_type = 1;

    if(!this->object_query_client.call(queryobjs))
    {
        ROS_WARN("Warning!! Object query service cannot be called!!");
        file.close();
        file2.close();
        file3.close();
        return;
    }

    // List that stores the object types
    QStringList typesls;

    // List that stores the object ids
    QStringList idsls;

    // List that stores the object ids
    QStringList configsls;




    // If we have any objects
    if(queryobjs.response.types.size()>0)
    {


        for(int i = 0; i < queryobjs.response.types.size(); i++)
        {
            QString str;



            str.append(QString::fromStdString(queryobjs.response.types[i]));



            typesls.append(str);




        }
    }
    else
    {
        file.close();
    }

    // If we have any objects
    if(queryobjs.response.ids.size()>0)
    {


        for(int i = 0; i < queryobjs.response.ids.size(); i++)
        {
            QString str;


            str.append(QString::fromStdString(queryobjs.response.ids[i]));


            idsls.append(str);



        }
    }
    else
    {
        file2.close();
    }
    // If we have any configs
    if(queryobjs.response.configs.size()>0)
    {


        for(int i = 0; i < queryobjs.response.configs.size(); i++)
        {
            QString str;


            str.append(QString::fromStdString(queryobjs.response.configs[i]));


            configsls.append(str);



        }
    }
    else
    {
        file3.close();

    }

    if(file.isOpen())
    {

        QTextStream stream(&file);

        foreach(QString st, typesls)
        {
            stream<<st<<"\n";


        }


        file.close();

    }

    if(file2.isOpen())
    {
        QTextStream stream2(&file2);

        foreach(QString st, idsls)
        {
            stream2<<st<<"\n";


        }


        file2.close();

    }

    if(file3.isOpen())
    {

        QTextStream stream3(&file3);

        foreach(QString st, configsls)
        {
            stream3<<st<<"\n";

        }


        file3.close();
    }

    emit SOMAObjectTypes(somalabels);

    return ;

}

void RosThread::fetchSOMAROIs()
{


    soma_manager::SOMAQueryROIs query_rois;

    query_rois.request.query_type = SOMA_QUERY_ROIS;

    query_rois.request.returnmostrecent = true;

    if(this->roi_query_client.call(query_rois))
    {
        if(query_rois.response.rois.size() > 0)
        {


            for(auto &roi:query_rois.response.rois)
            {
                SOMAROINameIDConfig roinameidconfig;
                roinameidconfig.id = roi.id.data();
                roinameidconfig.name = roi.type.data();
                roinameidconfig.config = roi.config.data();
                this->roinameidconfigs.push_back(roinameidconfig);
                this->roiarray.push_back(roi);
            }

        }


    }

    emit SOMAROINames(this->roinameidconfigs);


}
soma_msgs::SOMAROIObject RosThread::getSOMAROIwithID(int id)
{
    soma_msgs::SOMAROIObject obj;

    for(auto roi:this->roiarray)
    {
        if(id ==  QString::fromStdString(roi.id.data()).toInt())
        {
            //qDebug()<<"ROI found";
            return roi;

        }
    }


    return obj;

}
sensor_msgs::PointCloud2 RosThread::getSOMACombinedObjectCloud(const std::vector<soma_msgs::SOMAObject> &somaobjects)
{
    sensor_msgs::PointCloud2 result;

    Cloud cc;

    // If there are no soma objects send an empty cloud
    if(somaobjects.size() == 0)
    {
        pcl::PointXYZRGB point;
        point.x = 0;
        point.y = 0;
        point.z = 0;

        cc.push_back(point);

        cc.header.frame_id = "/map";
        pcl::toROSMsg(cc,result);

        return result;

    }

    for(int i = 0; i < somaobjects.size(); i++)
    {
        sensor_msgs::PointCloud2 cloud = somaobjects[i].cloud;
        Cloud pclcloud;
        pcl::fromROSMsg(cloud,pclcloud);
        cc.points.resize(cc.width*cc.height + pclcloud.height*pclcloud.width);
        cc += pclcloud;

        if(i == 30) break;


    }
    cc.header.frame_id = "/map";
    pcl::toROSMsg(cc,result);

    return result;
}

std::string RosThread::getMapName()
{
    return this->map_name;

}

SOMATimeLimits RosThread::getSOMACollectionMinMaxTimelimits()
{

    //SOMATimeLimits limits;

    soma_manager::SOMAQueryObjs query_objs;

    query_objs.request.query_type = SOMA_QUERY_TIMELIMITS;

    if(this->object_query_client.call(query_objs))
    {
        limits.mintimestamp =  query_objs.response.timedatelimits[0];

        limits.maxtimestamp = query_objs.response.timedatelimits[1];

        ROS_INFO("Date limits %ld %ld",limits.mintimestamp,limits.maxtimestamp);
    }


    return limits;

}
void RosThread::publishSOMAObjectCloud(sensor_msgs::PointCloud2 msg)
{
    pcp.publish(msg);


}
std::vector<soma_msgs::SOMAObject> RosThread::querySOMAObjects(soma_manager::SOMAQueryObjs& somaquery)
{

    this->object_query_client.call(somaquery);



    return somaquery.response.objects;

}


