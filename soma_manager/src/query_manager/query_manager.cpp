#include <ros/ros.h>
#include <mongodb_store/message_store.h>
#include "querybuilder.h"
#include <soma_manager/SOMAQueryObjs.h>
#include <soma_map_manager/MapInfo.h>
#include <QCollator>
#include <QDir>
#include <QTextStream>
#include <QDebug>
#include <QString>

std::string objectsdbname="somadata";
std::string roidbname="somadata";
std::string objectscollectionname="soma";
std::string map_name="kthfloor6";

struct SOMATimeLimits{

    int maxtimestep;
    int mintimestep;

    long mintimestamp;
    long maxtimestamp;

};

std::vector<soma_msgs::SOMAROIObject> fetchSOMAROIs()
{
    std::vector<soma_msgs::SOMAROIObject> res;

    ros::NodeHandle nl;

    mongodb_store::MessageStoreProxy somastore(nl,"roi",roidbname);

    mongo::BSONObjBuilder builder;

    builder.append("map_name",map_name);



    // qDebug()<<str;

    // builder.appendElements(mongo::fromjson(str.toStdString()));


    std::vector<boost::shared_ptr<soma_msgs::SOMAROIObject> > rois;


    somastore.query(rois,builder.obj());

    nl.shutdown();

    if(rois.size() > 0)
    {
        for(auto &roi:rois)
        {
            res.push_back(*roi);
        }
    }


    return res;
}

std::vector<std::vector<std::string> > fetchSOMAObjectTypesIDs()
{
    ros::NodeHandle nl;

    std::vector< std::vector<std::string> > result;

    mongodb_store::MessageStoreProxy somastore(nl,objectscollectionname,objectsdbname);

    std::vector<boost::shared_ptr<soma_msgs::SOMAObject> >  somaobjects;

    std::vector<std::string> somatypes;

    std::vector<std::string> somaids;


    // Query all objects,
    somastore.query(somaobjects);

    // List that stores the object types
    QStringList typesls;

    // List that stores the object ids
    QStringList idsls;

    nl.shutdown();

    // If we have any objects
    if(somaobjects.size()>0)
    {


        for(int i = 0; i < somaobjects.size(); i++)
        {
            QString str;

            QString str2;



            //   spr = somaobjects[i];

            str.append(QString::fromStdString(somaobjects[i]->type));

            str2.append(QString::fromStdString(somaobjects[i]->id));


            typesls.append(str);

            idsls.append(str2);



            //std::cout<<somaobjects[i].use_count()<<std::endl;


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
                [&collator](const QString &file1, const QString &file)
    {
        return collator.compare(file1, file) < 0;
    });




    foreach(QString st, typesls)
    {

        somatypes.push_back(st.toStdString());
    }




    // Remove duplicate names
    idsls.removeDuplicates();



    std::sort(
                idsls.begin(),
                idsls.end(),
                [&collator](const QString &file1, const QString &file)
    {
        return collator.compare(file1, file) < 0;
    });


    QString st;


    foreach(st, idsls)
    {
        somaids.push_back(st.toStdString());

    }


    result.push_back(somatypes);
    result.push_back(somaids);

    return result;

}
SOMATimeLimits getSOMACollectionMinMaxTimestep()
{


    SOMATimeLimits limits;
    limits.mintimestamp = -1;
    limits.mintimestep = -1;
    limits.maxtimestamp = -1;
    limits.maxtimestep = -1;

    ros::NodeHandle nl;

    mongodb_store::MessageStoreProxy somastore(nl,objectscollectionname,objectsdbname);

    mongo::BSONObjBuilder builder;

    builder.append("$natural",-1);

    std::vector<boost::shared_ptr<soma_msgs::SOMAObject> > somaobjects;

    somastore.query(somaobjects,mongo::BSONObj(),mongo::BSONObj(),builder.obj(),mongo::BSONObj(),false,1);

  //  somastore.query(somaobjects,mongo::BSONObj(),mongo::BSONObj(),builder.obj(),false,1); // old version


    if(somaobjects.size() > 0){
        limits.maxtimestep = somaobjects[0]->timestep;
        limits.maxtimestamp = somaobjects[0]->logtimestamp;
    }
    //std::cout<<somaobjects[0]->timestep<<std::endl;

    somaobjects.clear();

    somastore.query(somaobjects,mongo::BSONObj(),mongo::BSONObj(),mongo::BSONObj(),mongo::BSONObj(),false,1);

 //   somastore.query(somaobjects,mongo::BSONObj(),mongo::BSONObj(),mongo::BSONObj(),false,1);


    if(somaobjects.size() > 0)
    {

        limits.mintimestep = somaobjects[0]->timestep;
        limits.mintimestamp = somaobjects[0]->logtimestamp;
    }

    return limits;

}
std::vector<soma_msgs::SOMAObject> querySOMAObjects(const mongo::BSONObj &queryobj)
{

    ros::NodeHandle nl;

    mongodb_store::MessageStoreProxy somastore(nl,objectscollectionname,objectsdbname);


    std::vector<soma_msgs::SOMAObject> res;



    std::vector<boost::shared_ptr<soma_msgs::SOMAObject> > somaobjects;



    somastore.query(somaobjects,queryobj);


    if(somaobjects.size() > 0)
    {
        for(auto &labelled_object:somaobjects)
        {
            res.push_back(*labelled_object);
            //qDebug()<<labelled_object.use_count;
        }

    }


    ROS_INFO("Query returned %d objects",res.size());

    return res;


}


bool handleQueryRequests(soma_manager::SOMAQueryObjsRequest & req, soma_manager::SOMAQueryObjsResponse& resp)
{
    mongo::BSONObjBuilder mainbuilder;

    // We are building a SOMAObject Query
    if(req.query_type == 0)
    {
        // If dates are used
        if(req.usedates)
        {

            int mode = 0;

            if(req.lowerdate > 0 && req.upperdate > 0)
            {
                mode = 2;

            }
            else if(req.upperdate > 0)
                mode = 1;

            mongo::BSONObj bsonobj = QueryBuilder::buildSOMADateQuery(req.lowerdate,req.upperdate,mode);

            mainbuilder.appendElements(bsonobj);

        }
        // If timestep is used instead
        else if(req.usetimestep)
        {
           // qDebug()<<"I am here at timestep"<<req.timestep<<req.usetimestep;
            mongo::BSONObj timestepobj = QueryBuilder::buildSOMATimestepQuery(req.timestep);

            mainbuilder.appendElements(timestepobj);


        }
        // If any of the time limits are used
        if(req.uselowertime || req.useuppertime)
        {

            int mode = 0;
            if(req.uselowertime && req.useuppertime)
                mode =2 ;
            else if(req.useuppertime)
                mode=1;

            mongo::BSONObj bsonobj = QueryBuilder::buildSOMATimeQuery(req.lowerhour,req.lowerminutes,req.upperhour,req.upperminutes,mode);

            mainbuilder.appendElements(bsonobj);


        }
        // If weekday limit is used
        if(req.useweekday)
        {
            mongo::BSONObj bsonobj = QueryBuilder::buildSOMAWeekdayQuery(req.weekday);


            mainbuilder.appendElements(bsonobj);
        }

        // If roi is used
        if(req.useroi)
        {

            ros::NodeHandle nl;

            mongodb_store::MessageStoreProxy somastore(nl,"roi",roidbname);

            mongo::BSONObjBuilder builder;

            builder.append("map_name",map_name);


            std::vector<boost::shared_ptr<soma_msgs::SOMAROIObject> > rois;


            somastore.query(rois,builder.obj());

            nl.shutdown();



            for(int i  = 0;i < rois.size(); i++)
            {
                soma_msgs::SOMAROIObject roi = *rois[i];

                if(roi.id == req.roi_id)
                {
                    mongo::BSONObj bsonobj = QueryBuilder::buildSOMAROIWithinQuery(roi);

                    mainbuilder.appendElements(bsonobj);

                    break;

                }

            }


        }
        // If object ids and/or types are used
        if(req.objectids.size()>0 || req.objecttypes.size() > 0)
        {
            if(req.objectids.size() > 0 && req.objecttypes.size() > 0)
            {
              //  qDebug()<<req.objectids.size()<<req.objecttypes.size();
                if(req.objectids[0] != "" && req.objecttypes[0] != "")
                {

                    std::vector<std::string> list;

                    std::vector<std::string> fieldnames;
                    std::vector<int> objectIndexes;
                    fieldnames.push_back("id");
                    fieldnames.push_back("type");



                    list.insert(list.end(),req.objectids.begin(),req.objectids.end());

                    objectIndexes.push_back(req.objectids.size());

                    list.insert(list.end(),req.objecttypes.begin(),req.objecttypes.end());

                    objectIndexes.push_back(req.objecttypes.size());

                    mongo::BSONObj bsonobj = QueryBuilder::buildSOMAStringArrayBasedQuery(list,fieldnames,objectIndexes,"$or");

                    mainbuilder.appendElements(bsonobj);
                }
                else if(req.objecttypes[0] == "")
                {



                    std::vector<std::string> fieldnames;
                    fieldnames.push_back("id");

                    std::vector<int> objectIndexes;
                    objectIndexes.push_back(req.objectids.size());

                    std::vector<std::string> list;

                    //   list.insert(list.end(),req.objectids.data()->begin(),req.objectids.data()->end());


                    mongo::BSONObj bsonobj = QueryBuilder::buildSOMAStringArrayBasedQuery(req.objectids,fieldnames,objectIndexes,"$or");



                    mainbuilder.appendElements(bsonobj);

                }
                else if(req.objectids[0] == "")
                {
                    std::vector<std::string> fieldnames;
                    fieldnames.push_back("type");

                    std::vector<int> objectIndexes;
                    objectIndexes.push_back(req.objecttypes.size());

                    mongo::BSONObj bsonobj = QueryBuilder::buildSOMAStringArrayBasedQuery(req.objecttypes,fieldnames,objectIndexes,"$or");

                    mainbuilder.appendElements(bsonobj);

                }

            }
            else if(req.objectids.size() > 0)
            {
                std::vector<std::string> fieldnames;
                fieldnames.push_back("id");

                std::vector<int> objectIndexes;
                objectIndexes.push_back(req.objectids.size());

                std::vector<std::string> list;

                //   list.insert(list.end(),req.objectids.data()->begin(),req.objectids.data()->end());


                mongo::BSONObj bsonobj = QueryBuilder::buildSOMAStringArrayBasedQuery(req.objectids,fieldnames,objectIndexes,"$or");



                mainbuilder.appendElements(bsonobj);

            }
            else if(req.objecttypes.size() > 0)
            {
                std::vector<std::string> fieldnames;
                fieldnames.push_back("type");

                std::vector<int> objectIndexes;
                objectIndexes.push_back(req.objecttypes.size());

                mongo::BSONObj bsonobj = QueryBuilder::buildSOMAStringArrayBasedQuery(req.objecttypes,fieldnames,objectIndexes,"$or");

                mainbuilder.appendElements(bsonobj);

            }



        }

        if(mainbuilder.len() > 0){

            mongo::BSONObj tempObject = mainbuilder.obj();

           // qDebug()<<QString::fromStdString(tempObject.jsonString());

            std::vector< soma_msgs::SOMAObject > somaobjects =  querySOMAObjects(tempObject);

            resp.objects = somaobjects;
            resp.queryjson = tempObject.jsonString();

        }


    }



    // Handle Query for type and ids
    else if(req.query_type == 1)
    {
        std::vector<std::vector<std::string> > res = fetchSOMAObjectTypesIDs();
        if(res.size()>= 2)
        {
            resp.types = res[0];
            resp.ids = res[1];
        }

    }
    // Handle Query for rois
    else if(req.query_type == 2)
    {
        std::vector<soma_msgs::SOMAROIObject> res = fetchSOMAROIs();

        resp.rois = res;

    }
    // Handle Query for timelimits
    else if(req.query_type == 3)
    {
        SOMATimeLimits res = getSOMACollectionMinMaxTimestep();

        resp.timedatelimits.push_back(res.mintimestep);
        resp.timedatelimits.push_back(res.maxtimestep);
        resp.timedatelimits.push_back(res.mintimestamp);
        resp.timedatelimits.push_back(res.maxtimestamp);

    }



    return true;
}

int main(int argc, char **argv){


    ros::init(argc, argv, "query_manager_node");

    ros::NodeHandle n;


    //std::string mongodbhost;
    //std::string mongodbport;
    std::string roidb;


    if(argc < 2)
    {

        std::cout<<
                    "Running the query_manager_node with default arguments: ObjectsDBName: somadata, ObjectsCollectionName: object, ROICollectionName: roi"
                 <<std::endl;
        // std::cout << "Not enough input arguments!! Quitting..."<<std::endl;

        //  return -1;

    }
    else
    {
        if(argc > 1){
            objectsdbname = argv[1];
            //mw.rosthread.setSOMAObjectsDBName(objectsdbname);

        }
        if(argc > 2){
            objectscollectionname = argv[2];
            // mw.rosthread.setSOMAObjectsCollectionName(objectscollectionname);
        }
        if(argc >3){
            roidb = argv[3];
            //mw.rosthread.setSOMAROIDBName(roidb);
        }

    }

    ros::ServiceClient client = n.serviceClient<soma_map_manager::MapInfo>("soma/map_info");

    ROS_INFO("Waiting for SOMA Map Service...");

    while(!client.exists() && ros::ok());

    if(!ros::ok())
        return 0;

    soma_map_manager::MapInfo srv;

    client.call(srv);
    ROS_INFO("Received map info. Map Name: %s, Map Unique ID: %s",srv.response.map_name.data(),srv.response.map_unique_id.data());

    map_name = srv.response.map_name;

    ros::ServiceServer service = n.advertiseService("soma/query_db", handleQueryRequests);
    ROS_INFO("SOMA Query Service Ready.");

    ros::spin();

    /*ros::NodeHandle n;

    mongodb_store::MessageStoreProxy somastore(n,this->objectscollectionname,this->objectsdbname);*/

    //RosThread thread();



    return 0;


}
