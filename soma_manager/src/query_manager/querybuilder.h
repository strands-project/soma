#ifndef QUERYBUILDER_H
#define QUERYBUILDER_H
#include <mongo/bson/bson.h>
#include <mongo/bson/bsonelement.h>
#include <soma_msgs/SOMAROIObject.h>
#include <mongo/util/time_support.h>
#include <mongo/bson/bsontypes.h>


class QueryBuilder
{
public:
    QueryBuilder();

    static mongo::BSONObj buildSOMAROIWithinQuery(const soma_msgs::SOMAROIObject& roiobj);

    static mongo::BSONObj buildSOMATypeEqualsQuery(const std::vector<std::string>& typelist);

    static mongo::BSONObj buildSOMALabelContainsQuery(const std::string& text);

    static mongo::BSONObj buildSOMAConfigQuery(const std::string& text);

    static mongo::BSONObj buildSOMAWeekdayQuery(int index);

    static mongo::BSONObj buildSOMADateQuery(unsigned long lowerdate, unsigned long upperdate, int mode);

    static mongo::BSONObj buildSOMATimeQuery(int lowerhour,int lowerminute, int upperhour, int upperminute,  int mode);

  //  static mongo::BSONObj buildSOMATimestepQuery(int timestep);

    static mongo::BSONObj buildSOMAStringArrayBasedQuery(const std::vector<std::string>& list, std::vector<std::string> fieldnames, std::vector<int> objectIndexes, std::string arrayOperator);

private:
};

#endif // QUERYBUILDER_H
