#pragma once

#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

struct CoordSystemConfig {

    Eigen::Vector3f right;
    Eigen::Vector3f up;
    Eigen::Vector3f front;

};

struct LidarBounds {
    float x_min;
    float x_max;
    float y_min; 
    float y_max;
    float z_min; 
    float z_max;
};

struct LidarYamlConfig {
    CoordSystemConfig coord_sys;
    LidarBounds lidar_bounds;
    float lidar_offset;
};


namespace YAML {
template<>
struct convert<Eigen::Vector3f> {
    static Node encode(const Eigen::Vector3f& rhs) {
        Node node;
        node.push_back(rhs.x());
        node.push_back(rhs.y());
        node.push_back(rhs.z());
        return node;
    }

    static bool decode(const Node& node, Eigen::Vector3f& rhs) {
        if(!node.IsSequence() || node.size() != 3) {
            return false;
        }
        rhs << node[0].as<float>(), node[1].as<float>(), node[2].as<float>();
        return true;
    }
};
}

bool loadLidarConfig(const std::string &filename, LidarYamlConfig &cfg){
    YAML::Node root;
    try {
        root = YAML::LoadFile(filename);
    } catch (const YAML::BadFile &e) {
        std::cerr << "Could not open config file: " << e.what() << "\n";
        return false;
    }

    // coord system
    auto cc = root["cloud_coord_system"];
    if (!cc) { std::cerr << "Missing cloud_coord_system section\n"; return false; }

    // coord system
    cfg.coord_sys.right = cc["right"].as<Eigen::Vector3f>();
    cfg.coord_sys.up = cc["up"].as<Eigen::Vector3f>();
    cfg.coord_sys.front = cc["front"].as<Eigen::Vector3f>();

    // bounds
    auto feb = root["feature_extraction"];
    if (!feb) { std::cerr << "Missing feature_extraction section\n"; return false; }

    cfg.lidar_bounds.x_min = feb["x_min"].as<float>();
    cfg.lidar_bounds.x_max = feb["x_max"].as<float>();
    cfg.lidar_bounds.y_min = feb["y_min"].as<float>(); 
    cfg.lidar_bounds.y_max = feb["y_max"].as<float>();
    cfg.lidar_bounds.z_min = feb["z_min"].as<float>(); 
    cfg.lidar_bounds.z_max = feb["z_max"].as<float>();

    //offset
    auto off = root["lidar_offset"];
    if (!off) { std::cerr << "Missing lidar_offset section\n"; return false; }

    cfg.lidar_offset = off.as<float>()/1000.0f;

    return true;
}