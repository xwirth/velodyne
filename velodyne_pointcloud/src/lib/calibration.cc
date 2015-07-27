/**
 * \file  calibration.cc
 * \brief  
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 * Copyright (C) 2012, Austin Robot Technology,
 *                     The University of Texas at Austin
 *
 * License: Modified BSD License
 *
 * $ Id: 02/14/2012 11:36:36 AM piyushk $
 */

#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <limits>
#include <yaml-cpp/yaml.h>

#ifdef HAVE_NEW_YAMLCPP
namespace YAML {

  // The >> operator disappeared in yaml-cpp 0.5, so this function is
  // added to provide support for code written under the yaml-cpp 0.3 API.
  template<typename T>
  void operator >> (const YAML::Node& node, T& i) {
    i = node.as<T>();
  }
} /* YAML */
#endif // HAVE_NEW_YAMLCPP

#include <ros/ros.h>
#include <velodyne_pointcloud/calibration.h>

namespace velodyne_pointcloud 
{

  const std::string NUM_LASERS = "num_lasers";
  const std::string LASERS = "lasers";
  const std::string LASER_ID = "laser_id";
  const std::string VERT_CORRECTION = "vert_correction";

  void operator >> (const YAML::Node& node,
                    std::pair<int, LaserCorrection>& correction)
  {
    node[LASER_ID] >> correction.first;

    node[VERT_CORRECTION] >> correction.second.vert_correction;
   
    // Calculate cached values
    correction.second.cos_vert_correction =
      cosf(correction.second.vert_correction);
    correction.second.sin_vert_correction =
      sinf(correction.second.vert_correction);

    correction.second.laser_ring = 0;   // clear initially (set later)
  }

  void operator >> (const YAML::Node& node, Calibration& calibration) 
  {
    int num_lasers;
    node[NUM_LASERS] >> num_lasers;
    const YAML::Node& lasers = node[LASERS];
    calibration.laser_corrections.clear();
    calibration.num_lasers = num_lasers;
    for (int i = 0; i < num_lasers; i++) {
      std::pair<int, LaserCorrection> correction;
      lasers[i] >> correction;
      calibration.laser_corrections.insert(correction);
    }

    // For each laser ring, find the next-smallest vertical angle.
    //
    // This implementation is simple, but not efficient.  That is OK,
    // since it only runs while starting up.
    double next_angle = -std::numeric_limits<double>::infinity();
    for (int ring = 0; ring < num_lasers; ++ring) {

      // find minimum remaining vertical offset correction
      double min_seen = std::numeric_limits<double>::infinity();
      int next_index = num_lasers;
      for (int j = 0; j < num_lasers; ++j) {

        double angle = calibration.laser_corrections[j].vert_correction;
        if (next_angle < angle && angle < min_seen) {
          min_seen = angle;
          next_index = j;
        }
      }

      if (next_index < num_lasers) {    // anything found in this ring?

        // store this ring number with its corresponding laser number
        calibration.laser_corrections[next_index].laser_ring = ring;
        next_angle = min_seen;
        ROS_INFO_STREAM("laser_ring[" << next_index << "] = " << ring
                         << ", angle = " << next_angle);
      }
    }
  }

  YAML::Emitter& operator << (YAML::Emitter& out,
                              const std::pair<int, LaserCorrection> correction)
  {
    out << YAML::BeginMap;
    out << YAML::Key << LASER_ID << YAML::Value << correction.first;
    out << YAML::Key << VERT_CORRECTION <<
      YAML::Value << correction.second.vert_correction;
    out << YAML::EndMap;
    return out;
  }

  YAML::Emitter& operator <<
  (YAML::Emitter& out, const Calibration& calibration)
  {
    out << YAML::BeginMap;
    out << YAML::Key << NUM_LASERS <<
      YAML::Value << calibration.laser_corrections.size();
    out << YAML::Key << LASERS << YAML::Value << YAML::BeginSeq;
    for (std::map<int, LaserCorrection>::const_iterator
           it = calibration.laser_corrections.begin();
         it != calibration.laser_corrections.end(); it++)
      {
        out << *it; 
      }
    out << YAML::EndSeq;
    out << YAML::EndMap;
    return out;
  }

  void Calibration::read(const std::string& calibration_file) {
    std::ifstream fin(calibration_file.c_str());
    if (!fin.is_open()) {
      initialized = false;
      return;
    }
    initialized = true;
    try {
      YAML::Node doc;
#ifdef HAVE_NEW_YAMLCPP
      fin.close();
      doc = YAML::LoadFile(calibration_file);
#else
      YAML::Parser parser(fin);
      parser.GetNextDocument(doc);
#endif
      doc >> *this;
    } catch (YAML::Exception &e) {
      std::cerr << "YAML Exception: " << e.what() << std::endl;
      initialized = false;
    }
    fin.close();
  }

  void Calibration::write(const std::string& calibration_file) {
    std::ofstream fout(calibration_file.c_str());
    YAML::Emitter out;
    out << *this;
    fout << out.c_str();
    fout.close();
  }
  
} /* velodyne_pointcloud */
