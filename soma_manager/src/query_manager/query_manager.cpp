#include <ros/ros.h>
#include <mongodb_store/message_store.h>
#include "querybuilder.h"
#include <soma_manager/SOMAQueryObjs.h>
#include <soma_manager/SOMAQueryROIs.h>
#include <soma_map_manager/MapInfo.h>
#include <QCollator>
#include <QDir>
#include <QTextStream>
#include <QDebug>
#include <QString>

std::string objectsdbname="somadata";
std::string roidbname="somadata";
std::string objectscollectionname="object";
std::string roiscollectionname="roi";

std::string map_name="kthfloor6";

struct SOMATimeLimits{


    long mintimestamp;
    long maxtimestamp;

};

std::vector<double>  coordsToLngLat(double x, double y){
       std::vector<double> res;
       double earth_radius = 6371000.0; // in meters
       double lng = 90 - acos(float(x) / earth_radius)*180/M_PI;
       double lat = 90 - acos(float(y) / earth_radius)*180/M_PI ;
       res.push_back(lng);
       res.push_back(lat);
       return res;
}

bool is_only_ascii_whitespace( const std::string& str )
{
    auto it = str.begin();
    do {
        if (it == str.end()) return true;
    } while (*it >= 0 && *it <= 0x7f && std::isspace(*(it++)));
             // one of these conditions will be optimized away by the compiler,
             // which one depends on whether char is signed or not
    return false;
}


void fetchSOMAROIConfigsIDs(std::vector<std::string>& configs, std::vector<std::string>& ids)
{

    ros::NodeHandle nl;

    mongodb_store::MessageStoreProxy somastore(nl,roiscollectionname,roidbname);

    mongo::BSONObjBuilder builder;

    builder.append("map_name",map_name);

    // qDebug()<<str;

    // builder.appendElements(mongo::fromjson(str.toStdString()));


    std::vector<boost::shared_ptr<soma_msgs::SOMAROIObject> > rois;


    somastore.query(rois,builder.obj());

    // List that stores the object types
    QStringList configsls;

    // List that stores the object ids
    QStringList idsls;


    if(rois.size() > 0)
    {
        for(auto &roi:rois)
        {
            QString str;

            QString str2;



            //   spr = somaobjects[i];

            str.append(QString::fromStdString(roi->config));

            str2.append(QString::fromStdString(roi->id));


            configsls.append(str);

            idsls.append(str2);
        }



        // Remove duplicate names
        configsls.removeDuplicates();

        // Sort the types
        // typesls.sort(Qt::CaseInsensitive);


        QCollator collator;
        collator.setNumericMode(true);

        std::sort(
                    configsls.begin(),
                    configsls.end(),
                    [&collator](const QString &file1, const QString &file)
        {
            return collator.compare(file1, file) < 0;
        });




        foreach(QString st, configsls)
        {

            configs.push_back(st.toStdString());
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
            ids.push_back(st.toStdString());

        }





    }


    return;
}

void fetchSOMAObjectTypesIDsConfigs(std::vector<std::string>& types, std::vector<std::string>& ids, std::vector<std::string>& configs)
{
    ros::NodeHandle nl;

    std::vector< std::vector<std::string> > result;

    mongodb_store::MessageStoreProxy somastore(nl,objectscollectionname,objectsdbname);

    std::vector<boost::shared_ptr<soma_msgs::SOMAObject> >  somaobjects;

    std::vector<std::string> somatypes;

    std::vector<std::string> somaids;


    mongo::BSONObjBuilder projectionbuilder;

    projectionbuilder.append("cloud",0);
    projectionbuilder.append("images",0);


    somastore.queryWithProjection(somaobjects,mongo::BSONObj(),mongo::BSONObj(),mongo::BSONObj(),projectionbuilder.obj(),false,0);

    // List that stores the object types
    QStringList typesls;

    // List that stores the object ids
    QStringList idsls;

    // List that stores the object configs
    QStringList configsls;



    // If we have any objects
    if(somaobjects.size()>0)
    {


        for(int i = 0; i < somaobjects.size(); i++)
        {
            QString str;

            QString str2;

            QString str3;



            //   spr = somaobjects[i];

            str.append(QString::fromStdString(somaobjects[i]->type));

            str2.append(QString::fromStdString(somaobjects[i]->id));

            str3.append(QString::fromStdString(somaobjects[i]->config));



            typesls.append(str);

            idsls.append(str2);

            configsls.append(str3);



            //std::cout<<somaobjects[i].use_count()<<std::endl;


        }
    }

    //  somaobjects.clear();


    // Remove duplicate names
    typesls.removeDuplicates();

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

        types.push_back(st.toStdString());
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
        ids.push_back(st.toStdString());

    }


    // Remove duplicate names
    configsls.removeDuplicates();



    std::sort(
                configsls.begin(),
                configsls.end(),
                [&collator](const QString &file1, const QString &file)
    {
        return collator.compare(file1, file) < 0;
    });


    foreach(st, configsls)
    {
        configs.push_back(st.toStdString());

    }


    return;

}
SOMATimeLimits getSOMACollectionTimeLimits()
{


    SOMATimeLimits limits;

    limits.mintimestamp = INFINITY;

    limits.maxtimestamp = -1;

    ros::NodeHandle nl;

    mongodb_store::MessageStoreProxy somastore(nl,objectscollectionname,objectsdbname);

    std::vector<boost::shared_ptr<soma_msgs::SOMAObject> > somaobjects;

    mongo::BSONObjBuilder projectionbuilder;

    projectionbuilder.append("cloud",0);
    projectionbuilder.append("images",0);


    somastore.queryWithProjection(somaobjects,mongo::BSONObj(),mongo::BSONObj(),mongo::BSONObj(),projectionbuilder.obj(),false,0);

    if(somaobjects.size() > 0)
    {
        for(auto obj:somaobjects)
        {
            if(obj->logtimestamp > limits.maxtimestamp)
                limits.maxtimestamp = obj->logtimestamp;
            if(obj->logtimestamp < limits.mintimestamp)
                limits.mintimestamp = obj->logtimestamp;
        }


    }


    return limits;

}
std::vector<std::pair< boost::shared_ptr<soma_msgs::SOMAObject>, mongo::BSONObj> > querySOMAObjects(const mongo::BSONObj &queryobj, int limit=0)
{

    ros::NodeHandle nl;

    mongodb_store::MessageStoreProxy somastore(nl,objectscollectionname,objectsdbname);


    std::vector<std::pair< boost::shared_ptr<soma_msgs::SOMAObject>, mongo::BSONObj> > somaobjectsmetas;

    somastore.query(somaobjectsmetas,queryobj,mongo::BSONObj(),mongo::BSONObj(),false,true,limit);




    return somaobjectsmetas;


}
std::vector<std::pair< boost::shared_ptr<soma_msgs::SOMAROIObject>, mongo::BSONObj> > querySOMAROIs(const mongo::BSONObj &queryobj)
{

    ros::NodeHandle nl;

    mongodb_store::MessageStoreProxy somastore(nl,roiscollectionname,roidbname);


    std::vector<soma_msgs::SOMAROIObject> res;


    std::vector<std::pair< boost::shared_ptr<soma_msgs::SOMAROIObject>, mongo::BSONObj> > somaroismetas;

    //std::vector<boost::shared_ptr<soma_msgs::SOMAROIObject> > somarois;

     somastore.query(somaroismetas,queryobj);


   /* if(somarois.size() > 0)
    {
        for(auto &roi:somarois)
        {
            res.push_back(*roi);
            //qDebug()<<labelled_object.use_count;
        }

    }*/




    return somaroismetas;


}


bool handleObjectQueryRequests(soma_manager::SOMAQueryObjsRequest & req, soma_manager::SOMAQueryObjsResponse& resp)
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
        if(req.useroi_id)
        {

            ros::NodeHandle nl;

            mongodb_store::MessageStoreProxy somastore(nl,"roi",roidbname);

            mongo::BSONObjBuilder builder;

            builder.append("map_name",map_name);

            builder.append("id",req.roi_id);

            std::vector<boost::shared_ptr<soma_msgs::SOMAROIObject> > rois;

            mongo::BSONObjBuilder sortquerybuilder;

            sortquerybuilder.append("logtimestamp",-1); // sort descending to get the most recent roi at index 0


            somastore.query(rois,builder.obj(),mongo::BSONObj(),sortquerybuilder.obj());

            nl.shutdown();

            if(rois.size()> 0)
            {
                soma_msgs::SOMAROIObject roi = *rois[0]; // most recent roi
                mongo::BSONObj bsonobj = QueryBuilder::buildSOMAROIWithinQuery(roi);

                mainbuilder.appendElements(bsonobj);
            }





        }
        // If object ids and/or types are used
        if((req.objectids.size()>0 &&  !is_only_ascii_whitespace(req.objectids[0])) ||( req.objecttypes.size() > 0  && !is_only_ascii_whitespace(req.objecttypes[0])) ||( req.configs.size() > 0  && !is_only_ascii_whitespace(req.configs[0])))
        {
            std::vector<std::string> list;

            std::vector<std::string> fieldnames;
            std::vector<int> objectIndexes;

            if(req.objectids.size() > 0 && !is_only_ascii_whitespace(req.objectids[0]))
            {
                 fieldnames.push_back("id");

                 list.insert(list.end(),req.objectids.begin(),req.objectids.end());

                 objectIndexes.push_back(req.objectids.size());


            }
            if(req.objecttypes.size() > 0 && !is_only_ascii_whitespace(req.objecttypes[0]))
            {
                fieldnames.push_back("type");

                list.insert(list.end(),req.objecttypes.begin(),req.objecttypes.end());

                objectIndexes.push_back(req.objecttypes.size());


            }
            if(req.configs.size()> 0 && !is_only_ascii_whitespace(req.configs[0]))
            {
                fieldnames.push_back("config");

                list.insert(list.end(),req.configs.begin(),req.configs.end());

                objectIndexes.push_back(req.configs.size());

            }

            mongo::BSONObj bsonobj = QueryBuilder::buildSOMAStringArrayBasedQuery(list,fieldnames,objectIndexes,"$or");

            mainbuilder.appendElements(bsonobj);
            /*if(req.objectids.size() > 0 && req.objecttypes.size() > 0  )
            {
                //std::cout<<req.objectids.size()<<" "<<req.objecttypes.size()<<" "<<req.objectids[0].data()<<" "<<req.objecttypes[0].data();
                //if()
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

            }*/



        }


        if(!req.useroi_id && req.custom_roi.size() == 4)
        {
            soma_msgs::SOMAROIObject roiobj;

            geometry_msgs::Pose apose ;

            std::vector<double> res = coordsToLngLat(req.custom_roi[0],req.custom_roi[2]);

            apose.position.x = res[0]; //xlower
            apose.position.y = res[1]; //ylower

            roiobj.geoposearray.poses.push_back(apose);

            res = coordsToLngLat(req.custom_roi[0],req.custom_roi[3]);

            apose.position.x = res[0]; //xlower
            apose.position.y = res[1]; //yupper

            roiobj.geoposearray.poses.push_back(apose);


             res = coordsToLngLat(req.custom_roi[1],req.custom_roi[3]);


            apose.position.x = res[0]; //xupper
            apose.position.y = res[1]; //yupper

             roiobj.geoposearray.poses.push_back(apose);

             res = coordsToLngLat(req.custom_roi[1],req.custom_roi[2]);


            apose.position.x = res[0]; //xupper
            apose.position.y = res[1]; //ylower

            roiobj.geoposearray.poses.push_back(apose);
            roiobj.geoposearray.poses.push_back( roiobj.geoposearray.poses[0]); // close the loop

             mongo::BSONObj bsonobj = QueryBuilder::buildSOMAROIWithinQuery(roiobj);

             mainbuilder.appendElements(bsonobj);




        }

        if(mainbuilder.len() > 0)
        {

            mongo::BSONObj tempObject = mainbuilder.obj();

            // qDebug()<<QString::fromStdString(tempObject.jsonString());

           // std::vector< soma_msgs::SOMAObject > somaobjects =  querySOMAObjects(tempObject);

            mongo::BSONObjBuilder projectionbuilder;

            projectionbuilder.append("cloud",0);
            projectionbuilder.append("images",0);


            std::vector<std::pair< boost::shared_ptr<soma_msgs::SOMAObject>, mongo::BSONObj> > tempsomaobjectsmetas;

            std::vector<std::pair< boost::shared_ptr<soma_msgs::SOMAObject>, mongo::BSONObj> > somaobjectsmetas;

            ros::NodeHandle nl;

            mongodb_store::MessageStoreProxy somastore(nl,objectscollectionname,objectsdbname);

            somastore.queryWithProjection(tempsomaobjectsmetas,tempObject,mongo::BSONObj(),mongo::BSONObj(),projectionbuilder.obj());

           // std::cout<<"Objects size "<<tempsomaobjects.size();


            if(tempsomaobjectsmetas.size() > 50)
            {
                  somaobjectsmetas = querySOMAObjects(tempObject,50);

                  std::vector<std::pair< boost::shared_ptr<soma_msgs::SOMAObject>, mongo::BSONObj> >(tempsomaobjectsmetas.begin()+50, tempsomaobjectsmetas.end()).swap(tempsomaobjectsmetas);

                  somaobjectsmetas.insert(somaobjectsmetas.end(),tempsomaobjectsmetas.begin(),tempsomaobjectsmetas.end());

                  ROS_WARN("Query returned %u objects. Cloud,image information of first 50 objects have been fetched",(unsigned int)somaobjectsmetas.size());


            }
            else
            {
                  somaobjectsmetas = querySOMAObjects(tempObject);
                  ROS_INFO("Query returned %u objects.",(unsigned int)somaobjectsmetas.size());

            }


          //  std::vector<std::pair< boost::shared_ptr<soma_msgs::SOMAObject>, mongo::BSONObj> > somaobjectsmetas = querySOMAObjects(tempObject);

            for(size_t i = 0; i < somaobjectsmetas.size(); i++)
            {
                resp.objects.push_back(*somaobjectsmetas[i].first);
                mongo::BSONObj obj = somaobjectsmetas[i].second;
                mongo::BSONElement _idelement = obj.getField("_id");

                std::string _id = _idelement.toString(false);

                /*** Removing the ObjectId(...) part from the unique id.
                 * Only the part within the paranthesis is important for us***/
                _id.erase(_id.begin(),_id.begin()+10);
                _id.erase(_id.end()-2,_id.end());
                /**********************************************************/
                resp.unique_ids.push_back(_id);

            }


            resp.queryjson = tempObject.jsonString();

        }


    }



    // Handle Query for type and ids
    else if(req.query_type == 1)
    {
        std::vector<std::string> types, ids, configs;

        fetchSOMAObjectTypesIDsConfigs(types,  ids, configs);

        resp.types = types;
        resp.ids = ids;
        resp.configs = configs;



    }
    // Handle Query for timelimits
    else if(req.query_type == 2)
    {
        SOMATimeLimits res = getSOMACollectionTimeLimits();

        resp.timedatelimits.push_back(res.mintimestamp);
        resp.timedatelimits.push_back(res.maxtimestamp);

    }



    return true;
}
bool handleROIQueryRequests(soma_manager::SOMAQueryROIsRequest & req, soma_manager::SOMAQueryROIsResponse& resp)
{
    mongo::BSONObjBuilder mainbuilder;

    // We are building a SOMAROI Query
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


        // If object ids and/or types are used
        if(req.roiids.size()>0 || req.roiconfigs.size() > 0)
        {
            if(req.roiids.size() > 0 && req.roiconfigs.size() > 0)
            {
                //  qDebug()<<req.objectids.size()<<req.objecttypes.size();
                if(req.roiids[0] != "" && req.roiconfigs[0] != "")
                {

                    std::vector<std::string> list;

                    std::vector<std::string> fieldnames;
                    std::vector<int> objectIndexes;
                    fieldnames.push_back("id");
                    fieldnames.push_back("config");


                    list.insert(list.end(),req.roiids.begin(),req.roiids.end());

                    objectIndexes.push_back(req.roiids.size());

                    list.insert(list.end(),req.roiconfigs.begin(),req.roiconfigs.end());

                    objectIndexes.push_back(req.roiconfigs.size());

                    mongo::BSONObj bsonobj = QueryBuilder::buildSOMAStringArrayBasedQuery(list,fieldnames,objectIndexes,"$or");

                    mainbuilder.appendElements(bsonobj);
                }
                else if(req.roiconfigs[0] == "")
                {



                    std::vector<std::string> fieldnames;
                    fieldnames.push_back("id");

                    std::vector<int> objectIndexes;
                    objectIndexes.push_back(req.roiids.size());

                    std::vector<std::string> list;

                    //   list.insert(list.end(),req.objectids.data()->begin(),req.objectids.data()->end());


                    mongo::BSONObj bsonobj = QueryBuilder::buildSOMAStringArrayBasedQuery(req.roiids,fieldnames,objectIndexes,"$or");



                    mainbuilder.appendElements(bsonobj);

                }
                else if(req.roiids[0] == "")
                {
                    std::vector<std::string> fieldnames;
                    fieldnames.push_back("config");

                    std::vector<int> objectIndexes;
                    objectIndexes.push_back(req.roiconfigs.size());

                    mongo::BSONObj bsonobj = QueryBuilder::buildSOMAStringArrayBasedQuery(req.roiconfigs,fieldnames,objectIndexes,"$or");

                    mainbuilder.appendElements(bsonobj);

                }

            }
            else if(req.roiids.size() > 0)
            {
                std::vector<std::string> fieldnames;
                fieldnames.push_back("id");

                std::vector<int> objectIndexes;
                objectIndexes.push_back(req.roiids.size());

                std::vector<std::string> list;

                //   list.insert(list.end(),req.objectids.data()->begin(),req.objectids.data()->end());


                mongo::BSONObj bsonobj = QueryBuilder::buildSOMAStringArrayBasedQuery(req.roiids,fieldnames,objectIndexes,"$or");



                mainbuilder.appendElements(bsonobj);

            }
            else if(req.roiconfigs.size() > 0)
            {
                std::vector<std::string> fieldnames;
                fieldnames.push_back("config");

                std::vector<int> objectIndexes;
                objectIndexes.push_back(req.roiconfigs.size());

                mongo::BSONObj bsonobj = QueryBuilder::buildSOMAStringArrayBasedQuery(req.roiconfigs,fieldnames,objectIndexes,"$or");

                mainbuilder.appendElements(bsonobj);

            }



        }

        if(mainbuilder.len() > 0)
        {

            mongo::BSONObj tempObject = mainbuilder.obj();

            // qDebug()<<QString::fromStdString(tempObject.jsonString());

            std::vector<std::pair< boost::shared_ptr<soma_msgs::SOMAROIObject>, mongo::BSONObj> > result = querySOMAROIs(tempObject);

            std::vector<soma_msgs::SOMAROIObject > roiobjects;

            for(int i = 0; i < result.size(); i++)
            {
                roiobjects.push_back(*result[i].first);

            }


            if(req.returnmostrecent)
            {

                /*** Getting the object configurations and ids *******/
                QStringList idlist;
                QStringList configlist;


                for(int i  = 0; i < result.size(); i++)
                {
                    idlist.push_back(QString::fromStdString(roiobjects[i].id));
                    configlist.push_back(QString::fromStdString(roiobjects[i].config));


                }
                /*****************************************************/

                /**** Remove the duplicates ****/
                idlist.removeDuplicates();
                configlist.removeDuplicates();
                /*******************************/

                std::vector< soma_msgs::SOMAROIObject > mostrecentroiobjects;
                std::vector< std::string > unique_ids;


                foreach(QString config,configlist)
                {

                    foreach(QString id,idlist)
                    {
                        int maxtimestamp = 0;
                        int maxid = -1;

                        int count = 0;

                        for(int i  = 0; i < roiobjects.size(); i++)
                        {
                            soma_msgs::SOMAROIObject obj = roiobjects[i];
                            if(obj.id == id.toStdString() && obj.config == config.toStdString())
                            {
                                if(obj.logtimestamp > maxtimestamp)
                                {
                                    maxid = i;
                                    maxtimestamp = obj.logtimestamp;
                                }
                            }


                        }

                        if(maxid >=0)
                        {
                            mostrecentroiobjects.push_back(roiobjects[maxid]);

                            mongo::BSONObj obj = result[maxid].second;
                            mongo::BSONElement _idelement = obj.getField("_id");

                            std::string _id = _idelement.toString(false);

                            /*** Removing the ObjectId(...) part from the unique id.
                             * Only the part within the paranthesis is important for us***/
                            _id.erase(_id.begin(),_id.begin()+10);
                            _id.erase(_id.end()-2,_id.end());
                            /**********************************************************/
                            resp.unique_ids.push_back(_id);

                          //  roiobjects.erase(std::remove(roiobjects.begin(), roiobjects.end(), maxid), roiobjects.end());

                        }


                    }
                }


                resp.rois = mostrecentroiobjects;
                resp.queryjson = tempObject.jsonString();



                return true;


            }

            resp.rois = roiobjects;
            resp.queryjson = tempObject.jsonString();

            ROS_INFO("Query returned %d rois",(int)resp.rois.size());

        }


    }



    // Handle Query for configs and ids
    else if(req.query_type == 1)
    {
        std::vector<std::string> configs, ids;

        fetchSOMAROIConfigsIDs(configs,  ids);

        resp.configs = configs;
        resp.ids = ids;



    }



    return true;
}

int main(int argc, char **argv){


    ros::init(argc, argv, "query_manager_node");

    ros::NodeHandle n;

    if(argc < 2)
    {

        ROS_INFO(
                    "Running the SOMA query_manager_node with default arguments: ObjectsDB: somadata, ObjectsCollection: object, ROIDB: somadata, ROICollection: roi"
                    );



    }
    else
    {
        if(argc > 1){
            objectsdbname = argv[1];


        }
        if(argc > 2){
            objectscollectionname = argv[2];

        }
        if(argc >3){
            roidbname = argv[3];

        }
        if(argc >4){
            roiscollectionname = argv[4];

        }

    }

    ros::ServiceClient client = n.serviceClient<soma_map_manager::MapInfo>("soma/map_info");

    ROS_INFO("SOMA Query Service is waiting for SOMA Map Service...");

    while(!client.exists() && ros::ok());

    if(!ros::ok())
        return -1;

    soma_map_manager::MapInfo srv;

    client.call(srv);
  //  ROS_INFO("Received map info. Map Name: %s, Map Unique ID: %s",srv.response.map_name.data(),srv.response.map_unique_id.data());


    map_name = srv.response.map_name;

    ros::ServiceServer serviceobject = n.advertiseService("soma/query_objects", handleObjectQueryRequests);
    ros::ServiceServer serviceroi = n.advertiseService("soma/query_rois", handleROIQueryRequests);

    ROS_INFO("Running SOMA Query Service (objects db: %s, objects collection: %s, ROI db: %s, ROI collection: %s)",objectsdbname.data(),objectscollectionname.data(),roidbname.data(),roiscollectionname.data());

    ros::spin();




    return 0;


}
