//***************************************************************************
// Copyright 2013-2024 Norwegian University of Science and Technology (NTNU)*
// Department of Engineering Cybernetics (ITK)                              *
//***************************************************************************
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Nikolai Lauvås                                                   *
//***************************************************************************
// Utility for converting lsf logs to spatialite database                   *
//***************************************************************************
#include <fstream>
#include <iostream>
#include <iomanip>
#include <regex>
#include <map>
#include <sqlite3/sqlite3.h>
#include <DUNE/DUNE.hpp>
// ENC database to use with the OMPL integration for DUNE
#include <ENCGIS/DBconnection.hpp>
using DUNE_NAMESPACES;

static
void
usage(void)
{
  std::cerr << "Usage:\n\t lsf2csv [options] host port f1 ... fn\n"
            << "Options:\n"
            << "\t-m msg1,...,msgn: only replay specified messages\n"
            << "\t-S addr: filter using source address"
            << "\t-D addr: filter using destination adreess\n"
            << "\t-v [0-2]: verbosity level\n\n"
            << "f1 ... fn can be:\n"
            << "\t* Gzipped LSF files (.gz extension)\n"
            << "\t* LLF log dir names (will look for Data.lsf.gz in it)\n"
            << "\t* plain LSF files\n";
}

int
main(int argc, char** argv)
{
  std::map<std::string, bool> filter;
  std::map<std::string, bool> tableCreated;
  std::map<std::string, std::vector<std::pair<std::string, std::string>>> generateGeometry;
  filter[std::string("LoggingControl")] = true;
  filter[std::string("Rpm")] = true;
  filter[std::string("TBRFishTag")] = true;
  filter[std::string("TBRSensor")] = true;
  filter[std::string("Reference")] = true;
  filter[std::string("SetThrusterActuation")] = true;
  filter[std::string("EstimatedState")] = true;
  filter[std::string("PathControlState")] = true;
  filter[std::string("RemoteSensorInfo")] = true;
  filter[std::string("otterFormation")] = true;
  filter[std::string("CpuUsage")] = true;
  filter[std::string("SetEntityParameters")] = true;
  filter[std::string("EulerAngles")] = true;
  filter[std::string("GpsFix")] = true;

  int verbose = 0;
  uint16_t src = 0xFFFF, dst = 0xFFFF;

  ++argv; --argc;

  if (!argc)
  {
    usage();
    return 1;
  }

  for (; *argv && **argv == '-'; ++argv, --argc)
  {
    char opt = (*argv)[1];
    ++argv; --argc;

    if (!*argv || **argv == '-')
    {
      std::cerr << "Invalid options\n";
      usage();
      return 1;
    }

    // @todo Use DUNE's OptionParser, too lazy now to do it.
    switch (opt)
    {
      case 'S':
      {
        char* aux;
        src = std::strtol(*argv, &aux, 10);
        if (*aux != 0)
        {
          std::cerr << "Invalid source address: " << *argv << '\n';
          usage();
          return 1;
        }
        break;
      }
      case 'D':
      {
        char* aux;
        dst = std::strtol(*argv, &aux, 10);
        if (*aux != 0)
        {
          std::cerr << "Invalid destination adress: " << *argv << '\n';
          usage();
          return 1;
        }
        break;
      }
      case 'v':
        verbose = std::atoi(*argv);
        break;
      case 'm':
      {
        std::vector<std::string> list;
        DUNE::Utils::String::split(*argv, ",", list);
        for (uint16_t i = 0; i < list.size(); ++i)
          filter[list[i]] = true;
      }
      break;
      default:
        std::cerr << "Invalid option: '-" << opt << "\'\n";
        usage();
        return 1;
    }
  }

  if (argc == 0)
  {
    std::cerr << "Invalid arguments" << std::endl;
    usage();
    return 1;
  }

  std::cout << std::fixed << std::setprecision(4);



    // Establish SQL connection
    ENCGIS::DBconnection* m_writable = new ENCGIS::DBconnection("lsfoutput.db", SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, 32632);
    sqlite3_exec(m_writable->db, "BEGIN", 0, 0, 0);
    const std::string initSpatial = "select InitSpatialMetaData();";
    bool result_c_stmt = m_writable->runNoOutputQuery(initSpatial);
    // Generate template Queries
    const char* createColumns = "select group_concat(key || ' char(50)') from json_each(?1) where rowid >0;";
    const char* insertData = "select group_concat(\"'\" || value || \"'\") from json_each(?1) where rowid >0;";
    //const char* insertData = "select group_concat(value) from json_each(?1);";

    // Create SQL statement 
    sqlite3_stmt *col_stmt = NULL;
    sqlite3_stmt *stmt = NULL;
    // Prepare SQL statements
    sqlite3_prepare_v3(m_writable->db, createColumns, -1, SQLITE_PREPARE_PERSISTENT, &col_stmt, NULL);
    sqlite3_prepare_v3(m_writable->db, insertData, -1, SQLITE_PREPARE_PERSISTENT, &stmt, NULL);
    //
    std::ostream& os = std::cout;
    std::stringstream ss;
    std::stringstream columnss;
    sqlite3_exec(m_writable->db, "COMMIT", 0, 0, 0);
  for (; *argv != 0; argv++)
  {
    Path file(*argv);
    std::istream* is;

    if (file.isDirectory())
    {
      file = file / "Data.lsf";
      if (!file.isFile())
        file += ".gz";
    }

    if (!file.isFile())
    {
      std::cerr << file << " does not exist\n";
      return 1;
    }
    os << "File processing: " << file.c_str() << std::endl;
    Compression::Methods method = Compression::Factory::detect(file.c_str());
    if (method == METHOD_UNKNOWN)
      is = new std::ifstream(file.c_str(), std::ios::binary);
    else
      is = new Compression::FileInput(file.c_str(), method);

    IMC::Message* m;
    try{
      m = IMC::Packet::deserialize(*is);
      if (!m)
      {
        std::cerr << file << " contains no messages\n";
        delete is;
        continue;
      }
    } catch(...) {
      std::cout << "DESERIALIZING ERROR FIRST!" << std::endl;
    }
    sqlite3_exec(m_writable->db, "BEGIN", 0, 0, 0);
    do
    {
      if (
           (src == 0xFFFF || src == m->getSource())
          && (dst == 0xFFFF || dst == m->getDestination())
          && (filter[m->getName()]))
      {
        // Create table if not exists
        if(!tableCreated[m->getName()]) {
          tableCreated[m->getName()] = true;
          m->toJSON(ss);
          std::string columnJSON = ss.str();
          sqlite3_bind_text(col_stmt, 1, columnJSON.c_str(), -1, SQLITE_STATIC);
          if(sqlite3_step(col_stmt) == SQLITE_ROW) {
            if((char*)sqlite3_column_text(col_stmt, 0)) {
              std::string columns = (char*)sqlite3_column_text(col_stmt, 0);
              columnss << "CREATE TABLE IF NOT EXISTS " << m->getName() << "(uid INTEGER PRIMARY KEY, " << columns << ");";
              std::string stri = columnss.str();
              if(sqlite3_exec(m_writable->db, stri.c_str(), NULL, NULL, NULL) == SQLITE_OK) {
                // Add pending geometry adding
                std::string latitude = "lat";
                std::string longitude = "lon";
                if(stri.find(latitude) !=std::string::npos) {
                  std::regex r("\\b" + latitude + "\\b"); // the pattern \b matches a word boundary
                  std::smatch matcher;
                  if (std::regex_search(stri, matcher, r)) { // lat/lon match
                    generateGeometry[m->getName()] = std::vector<std::pair<std::string, std::string>>();
                    //os << "geometry for " << m->getName() << std::endl;
                  } else {
                    // Split string according to comma
                    std::regex r2("_" + latitude + "\\b"); // the pattern \b matches a word boundary
                    std::regex r3("_" + longitude + "\\b"); // the pattern \b matches a word boundary
                    std::vector<std::string> parts;
                    String::split(stri.substr(0, stri.length()), ",", parts);
                    unsigned count = 0;
                    std::string latName;
                    std::string lonName;
                    for(auto part : parts) {
                      if (std::regex_search(part, matcher, r2)) { // lat/lon match
                        //os << "multi geometry for " << m->getName() << "Keyword " << part << std::endl;
                        latName = part;
                        count++;
                      }
                      if (std::regex_search(part, matcher, r3)) { // lat/lon match
                        //os << "multi geometry for " << m->getName() << "Keyword " << part  << std::endl;
                        lonName = part;
                        count++;
                      }
                      if(count >1) {
                        count = 0;
                        std::pair<std::string, std::string> coordName;
                        coordName.first = lonName;
                        coordName.second = latName;
                        //if(generateGeometry[m->getName()].second()){
                        //  generateGeometry[m->getName()] = std::vector<std::pair<std::string, std::string>>();
                        //}
                        generateGeometry[m->getName()].push_back(coordName);
                        //os << "(" << latName << ", " << lonName << ")" << std::endl;
                      }
                    }
                  }
                }
                // Finished adding pending geometry
                os << "SUCESS CREATE" << std::endl;
              }
            }  else {
              os << "sqlite3_exec error" << std::endl;
              break;
            }
            columnss.str(std::string()); // Clear string stream
          } else {
            os << "sqlite3_step error" << std::endl;
            break;
          }
          sqlite3_reset( col_stmt );
        }
        // Insert data
        ss.str(std::string()); // Clear string stream
        m->toJSON(ss);
        std::string dataJSON = ss.str();
        //os << std::endl << dataJSON << std::endl;
        sqlite3_bind_text(stmt, 1, dataJSON.c_str(), -1, SQLITE_STATIC);
        ss.str(std::string()); // Clear string stream
        int rc;
        if((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
          if((char*)sqlite3_column_text(stmt, 0)) {
            std::string columns = (char*)sqlite3_column_text(stmt, 0);
            columnss << "INSERT INTO " << m->getName() << " VALUES (NULL, " << columns << ");";
            std::string stri = columnss.str();
            //os << stri;
            if((rc = sqlite3_exec(m_writable->db, stri.c_str(), NULL, NULL, NULL)) == SQLITE_OK) {
              //os << "SUCESS INSERT" << std::endl;
            }  else {
              os << "sqlite3_exec error insert rc=: " << rc << std::endl;
              break;
            }
          }  else {
            os << "sqlite3_column error insert" <<  std::endl;
            break;
          }
          columnss.str(std::string()); // Clear string stream
        } else {
          os << "sqlite3_step error insert rc=: " << rc << std::endl;
          break;
        }
        sqlite3_reset( stmt );
        //os << "converted\n";
      }
      delete m;
      unsigned tries = 0;
      do{
        try{
          m = IMC::Packet::deserialize(*is);
          break;
        }
        catch (const std::exception &exc)
        {
            os << "DESERIALIZING ERROR!" << exc.what() << std::endl;
            tries++;
        }
      } while(tries < 3);
      if(tries >= 3) {
        break;
      }
    }
    while (m != 0);
    sqlite3_exec(m_writable->db, "COMMIT", 0, 0, 0);
    delete is;
  }

  
  sqlite3_exec(m_writable->db, "BEGIN", 0, 0, 0);
  // Adding geometries
  os << "Generating geometry columns" << std::endl;
  for(auto table : generateGeometry) {
    //os << table.first << std::endl;
    if(table.second.empty())  {
      //os << "single geom" << std::endl;
      // Generate geometry column
      std::string generateGeometryQuery = "select AddGeometryColumn('" + table.first + "', 'geometry', 4326, 'POINT', 'XY');";
      sqlite3_exec(m_writable->db, generateGeometryQuery.c_str(), 0, 0, 0);
      // Add data to geometry columns
      std::string updateGeometryQuery = "update " + table.first + " as t1 set geometry=(" +
      "select makepoint(degrees(cast(lon as real)), degrees(cast(lat as real)), 4326) as geometry from " + table.first + " as t2 where t2.uid = t1.uid);";
      //os << updateGeometryQuery;
      sqlite3_exec(m_writable->db, updateGeometryQuery.c_str(), 0, 0, 0);
      
    } else {
      os << "multi geom" << std::endl;
      for(auto coordNames : table.second) {
        std::vector<std::string> parts;
        String::split(coordNames.first.substr(0, coordNames.first.length()), " ", parts);
        //os << parts[0] << std::endl;
        std::vector<std::string> parts2;
        String::split(coordNames.second.substr(0, coordNames.second.length()), " ", parts2);
        //os << parts2[0] << std::endl;
        // Generate geometry column
        std::string generateGeometryQuery = "select AddGeometryColumn('" + table.first + "', 'geometry_" + parts[0] + "', 4326, 'POINT', 'XY');";
        sqlite3_exec(m_writable->db, generateGeometryQuery.c_str(), 0, 0, 0);
        // Add data to geometry columns
        std::string updateGeometryQuery = "update " + table.first + " as t1 set geometry_" + parts[0] + "=(" +
        "select makepoint(degrees(cast(" + parts[0] + " as real)), degrees(cast(" + parts2[0] + " as real)), 4326) as geometry from " + table.first + " as t2 where t2.uid = t1.uid);";
        //os << updateGeometryQuery;
        sqlite3_exec(m_writable->db, updateGeometryQuery.c_str(), 0, 0, 0);
      }
    }
  }


  // Post prosessing queries
  os << "Post processing" << std::endl;
  // Current estimation by drift
if(filter[std::string("EstimatedState")], filter[std::string("Rpm")]) {
  os << "Calculating distazim" << std::endl;
const char* passivePeriods = "create table passivePeriods as "
"select *, maxtime-mintime as driftTime from ("
"select *, count(*) as cnt, min(timestamp) as mintime, max(timestamp) as maxtime, max(uid), min(uid) as mud "
"from (select p.*,"
"             row_number() over (order by src, timestamp) as seqnum,"
"             row_number() over (partition by value order by src, timestamp) as seqnum_1 "
"      from rpm as p "
"     ) p "
" group by value, (seqnum - seqnum_1), src order by mud) "
" where cnt>3 and value  < 20 and driftTime > 30";
sqlite3_exec(m_writable->db, passivePeriods, 0, 0, 0);
// Create index to significantly decrease time of next query
const char* estimatedStateIndex =  "CREATE INDEX \"timeIndex\" ON \"EstimatedState\" (\"timestamp\"	ASC,\"src\");";
sqlite3_exec(m_writable->db, estimatedStateIndex, 0, 0, 0);
// Query to associate geometries with the drifting periods
const char* distAzim ="create table distazim as "
"select src, cnt, mintime, maxtime, driftTime, cast(distance(minpnt,maxpnt,true)/driftTime as real) as speed, distance(minpnt,maxpnt,true) as dist,degrees(azimuth(minpnt,maxpnt)) as azimDeg, degrees(azimuth(tpnt,maxpnt)) as tazimdeg, minpnt, maxpnt, tpnt, makeline(minpnt, maxpnt) from("
"select *,"
"(select geometry from EstimatedState where src = pp.src and timestamp <=pp.mintime order by timestamp desc limit 1) as minpnt,"
"(select geometry from EstimatedState where src = pp.src and timestamp <=pp.maxtime-10 order by timestamp desc limit 1) as tpnt,"
"(select geometry from EstimatedState where src = pp.src and timestamp <=pp.maxtime order by timestamp desc limit 1) as maxpnt "
" from passivePeriods as pp);";
sqlite3_exec(m_writable->db, distAzim, 0, 0, 0);
}

  // Finishing Cleanup
  sqlite3_exec(m_writable->db, "COMMIT", 0, 0, 0);
  
  sqlite3_finalize( stmt );
  sqlite3_finalize( col_stmt );
  stmt = NULL;
  col_stmt = NULL;
  Memory::clear(m_writable);


  return 0;
}
