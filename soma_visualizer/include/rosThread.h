#include <QThread>
#include <ros/ros.h>
#include <QVector>
#include <QCollator>
#include <mongodb_store/MongoFind.h>
#include <mongodb_store/message_store.h>
#include <mongo/bson/bson.h>
#include <soma_msgs/SOMAObject.h>
#include <soma_map_manager/MapInfo.h>
#include <soma_msgs/SOMAROIObject.h>
#include <soma_manager/SOMAQueryObjs.h>
#include <soma_roi_manager/DrawROI.h>
#include <algorithm>    // std::unique, std::distance
#include <vector>       // std::vector

struct SOMAROINameIDConfig
{

    std::string id;
    std::string name;
    std::string config;

};

struct SOMATimeLimits
{


    long mintimestamp;
    long maxtimestamp;

};


class RosThread:public QObject
{
    Q_OBJECT

public:

    RosThread();

    sensor_msgs::PointCloud2 worldstate;

     // Shutdown the node
     void shutdownROS();

     // Returns the combined cloud of soma2objects
     sensor_msgs::PointCloud2 getSOMACombinedObjectCloud(const std::vector<soma_msgs::SOMAObject>& somaobjects);

    // sensor_msgs::PointCloud2 getSOMA2ObjectCloudsWithTimestep(int timestep);

     // Publish SOMA combined Object Cloud
     void publishSOMAObjectCloud(sensor_msgs::PointCloud2 msg);

     // Return the name of the current map
     std::string getMapName();

     // Service call for drawing roi with id
     void drawROIwithID(std::string id);

     // Get the SOMA ROI with id
     soma_msgs::SOMAROIObject getSOMAROIwithID(int id);

     // Query the SOMA objects
     std::vector<soma_msgs::SOMAObject> querySOMAObjects(const mongo::BSONObj& queryobj);

     std::vector<soma_msgs::SOMAObject> querySOMAObjects(soma_manager::SOMAQueryObjs &somaquery);

     void fetchDataFromDB();


     // Set the DB name for SOMa objects
     void setSOMAObjectsDBName(std::string name);

     // Set the collection name for SOMa objects
     void setSOMAObjectsCollectionName(std::string name);

     // Set the DB name for SOMa rois
     void setSOMAROIDBName(std::string name);

     // Set the Collection for SOMa rois
     void setSOMAROICollectionName(std::string name);

     // Get the DB name for SOMa objects
     std::string getSOMAObjectsDBName();

     // Get the collection name for SOMa objects
     std::string getSOMAObjectsCollectionName();

     // Get the time limits of SOMa objects
     SOMATimeLimits getSOMACollectionMinMaxTimelimits();

     std::string getSOMAROIDBName();

     std::string getSOMAROICollectionName();


private:
     bool shutdown;
     ros::NodeHandle n;
     ros::Publisher pcp;
     ros::ServiceClient roi_query_client;
     ros::ServiceClient object_query_client;
     ros::ServiceClient roi_draw_client;

     std::string objectsdbname;
     std::string objectscollectionname;
     std::string roibdname;
     std::string roicollectionname;
     SOMATimeLimits limits;

     // Get the ROIs from db
     void fetchSOMAROIs();

     // Get the object labels from db
     void fetchSOMAObjectTypesIDs();

     std::string map_name;
     std::string map_unique_id;
     std::vector<SOMAROINameIDConfig> roinameidconfigs;
     std::vector<std::string> labelnames;
     std::vector <soma_msgs::SOMAROIObject> roiarray;
   //  std::vector<boost::shared_ptr<soma2_msgs::SOMA2Object> >  soma2objects;



signals:

   void  rosStarted();
   void  rosStartFailed();
   void  rosFinished();
   void  mapinfoReceived();
   void  SOMAObjectTypes(std::vector<std::string>);
   void  SOMAObjectIDs(std::vector<std::string>);
   void  SOMAROINames(std::vector<SOMAROINameIDConfig>);

public slots:
   //void getSliderValue(int val);

  // void handleVelocityCommand(QVector<double> velCommand);

   void loop();

};
Q_DECLARE_METATYPE (std::vector<std::string>)
Q_DECLARE_METATYPE (std::vector<SOMAROINameIDConfig>)
