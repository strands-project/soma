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


#define SOMA_QUERY_ROIS 2
#define SOMA_QUERY_TIMELIMITS 3


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

std::string RosThread::getSOMAObjectsDBName()
{
    return this->objectsdbname;
}
std::string RosThread::getSOMAObjectsCollectionName()
{
    return this->objectscollectionname;
}
std::string RosThread::getROIDBName()
{
    return this->roibdname;
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


    this->query_client = n.serviceClient<soma_manager::SOMAQueryObjs>("soma/query_db");


    this->roi_client = n.serviceClient<soma_roi_manager::DrawROI>("soma/draw_roi");

    soma_map_manager::MapInfo srv;

    //srv.request.request = 0;

    ROS_INFO("Waiting for map_info service from soma_map_manager");

    if(!map_client.waitForExistence(ros::Duration(5)))
    {
        ROS_WARN("Warning! SOMA map service is not active!! Quitting...");

        emit rosFinished();

        return;

    }

    if(!query_client.waitForExistence(ros::Duration(5)))
    {
        ROS_WARN("Warning! SOMA query service is not active!! Quitting...");

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

    this->roi_client.call(drawroi);



}
void RosThread::shutdownROS()
{
    ros::shutdown();
    // shutdown = true;


}
void RosThread::fetchSOMAObjectTypesIDs()
{
    ros::NodeHandle nl;

    mongodb_store::MessageStoreProxy somastore(nl,this->objectscollectionname,this->objectsdbname);

    std::vector<boost::shared_ptr<soma_msgs::SOMAObject> >  somaobjects;
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

    mongo::BSONObjBuilder builder;

    builder.append("cloud",0);
    builder.append("images",0);

    // Query all objects,
    somastore.query(somaobjects,mongo::BSONObj(),mongo::BSONObj(),mongo::BSONObj(),builder.obj());

    // List that stores the object types
    QStringList typesls;

    // List that stores the object ids
    QStringList idsls;

    nl.shutdown();

    // If we have any objects
    if(somaobjects.size()>0)
    {

        int maxindex = 0;
        long max = 0;


        int minindex = 0;
        long min = 10000000000;

        for(int i = 0; i < somaobjects.size(); i++)
        {
            QString str;

            QString str2;



            //   spr = somaobjects[i];

            str.append(QString::fromStdString(somaobjects[i]->type));

            str2.append(QString::fromStdString(somaobjects[i]->id));


            typesls.append(str);

            idsls.append(str2);




       /*     if(max < somaobjects[i]->logtimestamp){
                max = somaobjects[i]->logtimestamp;
                maxindex = i;
            }

            if(min > somaobjects[i]->logtimestamp)
            {
                min = somaobjects[i]->logtimestamp;
                minindex = i;
            }




         //   limits.maxtimestep = maxtimestep;//somaobjects[maxindex]->timestep;
            limits.maxtimestamp = somaobjects[maxindex]->logtimestamp;

            limits.mintimestamp = min;


*/

            //std::cout<<soma2objects[i].use_count()<<std::endl;


        }
    }

    //  somaobjects.clear();


    // Remove duplicate names
    typesls.removeDuplicates();

    // Sort the types
    // typesls.sort(Qt::CaseInsensitive);


    QCollator collator;
    collator.setNumericMode(true);

    std::sort(
                typesls.begin(),
                typesls.end(),
                [&collator](const QString &file1, const QString &file2)
    {
        return collator.compare(file1, file2) < 0;
    });


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

    // Remove duplicate names
    idsls.removeDuplicates();

    // Sort the ids
    // idsls.sort(Qt::CaseInsensitive);


    std::sort(
                idsls.begin(),
                idsls.end(),
                [&collator](const QString &file1, const QString &file2)
    {
        return collator.compare(file1, file2) < 0;
    });


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



    /* std::sort(res.begin(), res.end());

    auto last = std::unique(res.begin(), res.end());

    res.erase(last, res.end());*/

    //res.resize( std::distance(res.begin(),it) );


    emit SOMAObjectTypes(somalabels);

    return ;

}

void RosThread::fetchSOMAROIs()
{
    //std::vector<SOMAROINameID> res;

    soma_manager::SOMAQueryObjs query_objs;

    query_objs.request.query_type = SOMA_QUERY_ROIS;

    if(this->query_client.call(query_objs))
    {
        if(query_objs.response.rois.size() > 0)
        {


                for(auto &roi:query_objs.response.rois)
                {
                    // res.push_back(roi->type.data());
                    SOMAROINameID roinameid;
                    roinameid.id = roi.id.data();
                    roinameid.name = roi.type.data();
                    this->roinameids.push_back(roinameid);
                    this->roiarray.push_back(roi);
                }

        }


    }

    emit SOMAROINames(this->roinameids);


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

    if(this->query_client.call(query_objs))
    {
       limits.mintimestamp =  query_objs.response.timedatelimits[0];

       limits.maxtimestamp = query_objs.response.timedatelimits[1];

        ROS_INFO("Date limits %d %d",limits.mintimestamp,limits.maxtimestamp);
    }



    /* somaTimeLimits limits;
    limits.mintimestamp = -1;
    limits.mintimestep = -1;
    limits.maxtimestamp = -1;
    limits.maxtimestep = -1;

    ros::NodeHandle nl;

    mongodb_store::MessageStoreProxy somastore(nl,objectscollectionname,objectsdbname);

    mongo::BSONObjBuilder builder;

    builder.append("$natural",-1);

    std::vector<boost::shared_ptr<soma_msgs::somaObject> > somaobjects;

    somastore.query(somaobjects,mongo::BSONObj(),mongo::BSONObj(),builder.obj(),false);


    if(somaobjects.size() > 0)
    {
        int maxindex = 0;
        long max = 0;
        int minindex = 0;
        long min = 10000000000;
        for(int i = 0; i < somaobjects.size(); i++)
        {
            if(max < somaobjects[i]->logtimestamp){
                max = somaobjects[i]->logtimestamp;
                maxindex = i;
            }

            if(min > somaobjects[i]->logtimestamp)
            {
                min = somaobjects[i]->logtimestamp;
                minindex = i;
            }

        }


        limits.maxtimestep = somaobjects[maxindex]->timestep;
        limits.maxtimestamp = somaobjects[maxindex]->logtimestamp;

        limits.mintimestamp = min;
        limits.mintimestep = somaobjects[minindex]->timestep;
    }
    //std::cout<<somaobjects[0]->timestep<<std::endl;

   /* somaobjects.clear();
    somastore.query(somaobjects,mongo::BSONObj(),mongo::BSONObj(),mongo::BSONObj(),false,1);

    if(somaobjects.size() > 0)
    {


      //  limits.mintimestep = somaobjects[0]->timestep;
       // limits.mintimestamp = somaobjects[0]->logtimestamp;
    }*/



    return limits;

}
void RosThread::publishSOMAObjectCloud(sensor_msgs::PointCloud2 msg)
{
    pcp.publish(msg);


}
/*std::vector<soma_msgs::somaObject> RosThread::querysomaObjectsWithDate(const mongo::BSONObj &queryobj)
{

    ros::NodeHandle nl;
    mongodb_store::MessageStoreProxy somastore(nl,this->objectscollectionname,this->objectsdbname);

    std::vector<soma_msgs::somaObject> res;


    mongo::BSONObjBuilder builder;


    builder.appendElements(queryobj);


    std::vector<boost::shared_ptr<soma_msgs::somaObject> > somaobjects;


    mongo::BSONObj builderobj = builder.obj();

    somastore.query(somaobjects,builderobj);


  //  qDebug()<<QString::fromStdString(builderobj.jsonString());


    if(somaobjects.size() > 0)
    {
        for(auto &labelled_object:somaobjects)
        {
            res.push_back(*labelled_object);
        }

    }


    qDebug()<<"Query returned"<<res.size()<<"objects";

    // std::cout<<QUERY("\"geoloc\""<<stdstr).obj.toString()<<std::endl;

    //  std::cout<< BSON("geoloc"<<stdstr).jsonString();

    return res;

}*/
std::vector<soma_msgs::SOMAObject> RosThread::querySOMAObjects(const mongo::BSONObj &queryobj)
{

    ros::NodeHandle nl;

    mongodb_store::MessageStoreProxy somastore(nl,this->objectscollectionname,this->objectsdbname);


    std::vector<soma_msgs::SOMAObject> res;



    std::vector<boost::shared_ptr<soma_msgs::SOMAObject> > somaobjects;



    somastore.query(somaobjects,queryobj,mongo::BSONObj(),mongo::BSONObj(),mongo::BSONObj(),false,30);


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

