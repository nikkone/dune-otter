#include "SearchGridCoverageState.hpp"
#include <iostream>
namespace ENCGIS
{
  bool SearchGridCoverageState::update(float X, float Y, float range, float timeReduction, std::string metric) {
    std::string updateQuery= 
    "update " + getdbGridTable() + " as s set " + metric + " = 1*" + std::to_string(timeReduction) + " from ("
    "select gid from("
    "select * from " + getdbGridTable() + " where ROWID IN ("
    "SELECT ROWID FROM SpatialIndex "
    "        WHERE f_table_name = '" + getdbGridTable() + "' AND "
    "          search_frame = BuildCircleMbr(" + std::to_string(X) + "," + std::to_string(Y) + "," + std::to_string(range) + "," + std::to_string(getSRID()) + ")"
    ")"
    ") where within(geometry, buffer(makepoint(" + std::to_string(X) + "," + std::to_string(Y) + "," + std::to_string(getSRID()) + "), " + std::to_string(range) + "))"
    ") as c where s.gid = c.gid";
    //std::cout << updateQuery << std::endl;
    return getDBconnection()->runNoOutputQuery(updateQuery);
  }

  bool SearchGridCoverageState::updateLogarithmic(float X, float Y, unsigned rpm, float timeReduction, double range, double pCutoff, std::string metric) {
    double b0,b1;
    if(logarithmicModelCoefficients(rpm, b0, b1)) {
      std::string updateQuery = 
      "update " + getdbGridTable() + " as s set " + metric + " = min( (" + metric + "+we*" + std::to_string(timeReduction) + "), 1) from ("
      "select gid, 1/(1+exp(-(" + std::to_string(b0) + " + dist*" + std::to_string(b1) + " ))) as we from ("
        "select gid, distance(c.geometry, makepoint(" + std::to_string(X) + "," + std::to_string(Y) + "," + std::to_string(getSRID()) + ")) as dist from ("
          "select gid, geometry from ("
          "select * from " + getdbGridTable() + " where ROWID IN (SELECT ROWID FROM SpatialIndex "
          "WHERE f_table_name = '" + getdbGridTable() + "' and "
          "search_frame = BuildCircleMbr(" + std::to_string(X) + "," + std::to_string(Y) + "," + std::to_string(range) + "," + std::to_string(getSRID()) + "))"
          ") "
        ") as c"
      ")"
      ") as a where s.gid = a.gid  and we > " + std::to_string(pCutoff);
      //std::cout << updateQuery << std::endl;
      return getDBconnection()->runNoOutputQuery(updateQuery);
    }
    return false;
  }

  bool SearchGridCoverageState::logarithmicModelCoefficients(unsigned rpm, double &b0, double &b1) {
    const double logitCoefzero[2] = {4.748444238767068,-0.011050835615990};
    const double logitCoefonetwenty[2] = {10.510644583981138,-0.122312536613913};
    const double logitCoefoneeighty[2] = {6.172439685021811,-0.033868111399770};
    const double logitCoeftwofourty[2] = {4.762268533530098,-0.028498319426830};
    const double logitCoefthreehundred[2] = {3.888776088643779,-0.030189104248954};

    if(rpm > 330) {
      return false; // No update needed, assume no detections above 300
    } else if(rpm > 270) {
      b0=logitCoefthreehundred[0];
      b1=logitCoefthreehundred[1];
      return true;
    } else if(rpm > 210) {
      b0=logitCoeftwofourty[0];
      b1=logitCoeftwofourty[1];
      return true;
    } else if(rpm > 150) {
      b0=logitCoefoneeighty[0];
      b1=logitCoefoneeighty[1];
      return true;
    } else if(rpm > 30) {
      b0=logitCoefonetwenty[0];
      b1=logitCoefonetwenty[1];
      return true;
    } else {
      b0=logitCoefzero[0];
      b1=logitCoefzero[1];
      return true;
    }
  }

  bool SearchGridCoverageState::decreaseAll(float fixedDecrease, std::string metric)
  {
    std::string decreaseQuery = "update " + getdbGridTable() + " set " + metric + " = max(0, " + metric + "-" + std::to_string(fixedDecrease) + ")";
    return getDBconnection()->runNoOutputQuery(decreaseQuery);
  }
  bool SearchGridCoverageState::decreaseAll(float fixedDecrease, float decreaseFactor, std::string metric)
  {
    std::string decreaseQuery = "update " + getdbGridTable() + " set " + metric + " = max(0, " + metric + "*" + std::to_string(decreaseFactor) + "-" + std::to_string(fixedDecrease) + ")";
    return getDBconnection()->runNoOutputQuery(decreaseQuery);
  }

  bool SearchGridCoverageState::updateDetectionProbability(std::string detProbLayer, std::string detProbMetric, std::string priorMetric, std::string effortLayerMetric) {
    /* Old V1, very slow
    std::string updateQuery = 
    "update " + detProbLayer + " set " + detProbMetric + " = we*1.0 from ("
    "select f.gid as fid, avg(c." + priorMetric + "*(1-c." + effortLayerMetric + ")) as we,f.geometry as rf from " + getdbGridTable() + " as c, " + detProbLayer + " as f where intersects(c.geometry, f.geometry) group by f.gid"
    ") where gid = fid";*/

    std::string updateQuery = 
      "update " + detProbLayer + " as f set " + detProbMetric + " = ("
        "select avg(c." + priorMetric + "*(1-c." + effortLayerMetric + ")) from " + getdbGridTable() + " as c where c.ROWID IN ("
          "SELECT ROWID "
          "FROM SpatialIndex "
          "WHERE f_table_name = '" + getdbGridTable() + "' "
            "AND search_frame = f.geometry"
        ") and intersects(c.geometry, f.geometry)"
      ")";

    //std::cout << updateQuery << std::endl;
    return getDBconnection()->runNoOutputQuery(updateQuery);

    /* Optimizations:
      After initial, only use from coverage != 0.0
      Only include fishsearch where weight is not 0.0
      Instead of full join, find other way
    */
  }
}

