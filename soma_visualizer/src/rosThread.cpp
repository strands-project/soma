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
    this->objectsdbname = "somadata";
    this->objectscollectionname = "object";
    this->roibdname = "somadata";

}

void RosThread::setSOMAObjectsDBName(std::string name)
{
    this->objectsdbname = name;


}
void RosThread::setSOMAObjectsCollectionName(std::string name)
{
    this->objectscollectionname = name;


}

void RosThread::setSOMAROIDBName(std::string name)
{
    this->roibdname = name;

}
void RosThread::setSOMAROICollectionName(std::string name)
{
    this->roicollectionname = name;

}

std::string RosThread::getSOMAObjectsDBName()
{
    return this->objectsdbname;
}
std::string RosThread::getSOMAObjectsCollectionName()
{
    return this->objectscollectionname;
}
std::string RosThread::getSOMAROIDBName()
{
    return this->roibdname;
}
std::string RosThread::getSOMAROICollectionName()
{
    return this->roicollectionname;
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

    //srv.request.request = 0;

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


    this->fetchSOMAROIs();

    this->fetchSOMAObjectTypesIDs();

    emit mapinfoReceived();


    while(ros::ok())
    {


        // velocityCommandPublisher.publish(velocityCommand);

        ros::spinOnce();

        loop.sleep();


    }
    ROS_INFO("ROS thread finished!!");

    emit rosFinished();



}
void RosThread::drawROIwithID(std::string id)
{

    soma_roi_manager::DrawROI drawroi;

    drawroi.request.map_name = this->map_name;
    drawroi.request.roi_id = id;

    this->roi_draw_client.call(drawroi);



}
void RosThread::shutdownROS()
{
    ros::shutdown();
    // shutdown = true;


}
void RosThread::fetchSOMAObjectTypesIDs()
{

    std::vector<std::string> somalabels;

    QString dir = QDir::homePath();

    dir.append("/").append(".soma/");

    QDir typesdir;

    if(!typesdir.exists(dir))
        typesdir.mkdir(dir);

    QString filename = "objecttypes.txt";

    QString filename2 = "objectids.txt";

    QString objecttypesdir = dir;
    QString idsdir = dir;

    objecttypesdir.append(filename);

    idsdir.append(filename2);

    QFile file(objecttypesdir);

    if(!file.open(QFile::WriteOnly))
    {
        qDebug()<<"Cannot Open types file! Returning...";
        return ;


    }

    QFile file2(idsdir);

    if(!file2.open(QFile::WriteOnly))
    {
        qDebug()<<"Cannot Open types file! Returning...";
        return ;


    }

    soma_manager::SOMAQueryObjs queryobjs;

    queryobjs.request.query_type = 1;

    if(!this->object_query_client.call(queryobjs))
    {
        ROS_WARN("Warning!! Object query service cannot be called!!");
        file.close();
        file2.close();
        return;
    }

    // List that stores the object types
    QStringList typesls;

    // List that stores the object ids
    QStringList idsls;


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
        file.close();
        file2.close();
        return;
    }



    QTextStream stream(&file);

    foreach(QString st, typesls)
    {
        stream<<st<<"\n";

        // Dump data to array
        // res[count].append(st.toStdString());

        // Then transfer it into vector
        //  somalabels.push_back(res[count]);

        // this->labelnames.push_back(res[count]);

        //  count++;
        // qDebug()<<st;
    }


    file.close();



    QTextStream stream2(&file2);

    foreach(QString st, idsls)
    {
        stream2<<st<<"\n";

        // Dump data to array
        // res[count].append(st.toStdString());

        // Then transfer it into vector
        //  somalabels.push_back(res[count]);

        // this->labelnames.push_back(res[count]);

        //  count++;
        // qDebug()<<st;
    }


    file2.close();


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
std::string RosThread::getSOMAObjectDateWithTimestep(int timestep)
{
    ros::NodeHandle nl;
    mongodb_store::MessageStoreProxy somastore(nl,this->objectscollectionname,this->objectsdbname);

    mongo::BSONObjBuilder builder2;

    sensor_msgs::PointCloud2 result;

    QJsonObject jsonobj;

    jsonobj.insert("timestep",timestep);
    jsonobj.insert("map_name",QString::fromStdString(this->map_name));

    QJsonDocument doc;

    doc.setObject(jsonobj);

    QString str(doc.toJson());

    //  qDebug()<<str;

    std::stringstream ss;
    //  ss<<"{\"timestep\":\""<<timestep<<"\,\"map_name\":\""<<this->map_name<<"\"}";

    builder2.appendElements(mongo::fromjson(str.toStdString()));


    std::vector<boost::shared_ptr<soma_msgs::SOMAObject> > somaobjects;

    somastore.query(somaobjects,builder2.obj());
    nl.shutdown();
    soma_msgs::SOMAObject anobject;
    std::string date;
    if(somaobjects.size() > 0){
        anobject = *somaobjects[0];
        qint64 val = (float)anobject.logtimestamp*1000;
        QDateTime dt = QDateTime::fromMSecsSinceEpoch(val,Qt::UTC);
        //  qDebug()<<dt.toString(Qt::ISODate);
        date = dt.toString(Qt::ISODate).toStdString();
    }


    somaobjects.clear();


    return date;
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

std::vector<soma_msgs::SOMAObject> RosThread::querySOMAObjects(const mongo::BSONObj &queryobj)
{

    ros::NodeHandle nl;

    mongodb_store::MessageStoreProxy somastore(nl,this->objectscollectionname,this->objectsdbname);


    std::vector<soma_msgs::SOMAObject> res;



    std::vector<boost::shared_ptr<soma_msgs::SOMAObject> > somaobjects;



    somastore.query(somaobjects,queryobj,mongo::BSONObj(),mongo::BSONObj(),false,30);


    if(somaobjects.size() > 0)
    {
        for(auto &labelled_object:somaobjects)
        {
            res.push_back(*labelled_object);
            //qDebug()<<labelled_object.use_count;
        }

    }


    qDebug()<<"Query returned"<<res.size()<<"objects";

    return res;


}
std::vector<soma_msgs::SOMAObject> RosThread::querySOMAObjects(soma_manager::SOMAQueryObjs& somaquery)
{
  //  ros::NodeHandle nl;

  //  ros::ServiceClient client = nl.serviceClient("soma/query_objects");

    this->object_query_client.call(somaquery);

    return somaquery.response.objects;

}


