#pragma once

#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

struct LidarBoundsCfg{
    float x_min;
    float x_max;
    float y_min;
    float y_max;
    float z_min;
    float z_max;
};


struct CoordSystemConfig {

    Eigen::Vector3f right;
    Eigen::Vector3f up;
    Eigen::Vector3f front;

};


struct LidarYamlConfig {
    CoordSystemConfig coord_sys;
    LidarBoundsCfg lidar_bounds;
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

struct PatternSizeBoard {
    int height;
    int width;
};

struct BoardConfig {
    PatternSizeBoard pattern_size;
    float square_length;
    struct {
        float width;
        float height;
    } board_dimension;
    struct {
        float x;
        float y;
    } translation_error;
};

struct CameraConfig {
    std::string distortion_model;
    struct {
        int width;
        int height;
    } image_size;
    // K is 3x3 matrix
    std::vector<double> K;  
    std::vector<double> D;
};

struct CameraYamlConfig {
    BoardConfig chessboard;
    CameraConfig camera;
};

bool loadCameraConfig(const std::string &filename, CameraYamlConfig &cfg) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(filename);
    } catch (const YAML::BadFile &e) {
        std::cerr << "Could not open config file: " << e.what() << "\n";
        return false;
    }

    // chessboard
    auto cb = root["chessboard"];
    if (!cb) { std::cerr << "Missing chessboard section\n"; return false; }

    // pattern_size
    auto ps = cb["pattern_size"];
    cfg.chessboard.pattern_size.height = ps["height"].as<int>();
    cfg.chessboard.pattern_size.width = ps["width"].as<int>();

    cfg.chessboard.square_length = cb["square_length"].as<float>()/1000.0f;

    auto bd = cb["board_dimension"];
    cfg.chessboard.board_dimension.width = bd["width"].as<float>()/1000.0f;
    cfg.chessboard.board_dimension.height = bd["height"].as<float>()/1000.0f;

    auto te = cb["translation_error"];
    cfg.chessboard.translation_error.x = te["x"].as<float>()/1000.0f;
    cfg.chessboard.translation_error.y = te["y"].as<float>()/1000.0f;

    // camera
    auto cam = root["camera"];
    if (!cam) { std::cerr << "Missing camera section\n"; return false; }

    cfg.camera.distortion_model = cam["distortion_model"].as<std::string>();

    auto imsz = cam["image_size"];
    cfg.camera.image_size.width = imsz["width"].as<int>();
    cfg.camera.image_size.height = imsz["height"].as<int>();

    cfg.camera.K = cam["K"].as<std::vector<double>>();
    cfg.camera.D = cam["D"].as<std::vector<double>>();

    return true;
}

struct TopicsInfo{

    std::string shared_mem_name;
    std::string bbox_shared_mem_name;

    std::string camera_recieve_queue_name;
    std::string lidar_recieve_queue_name;


    std::string lidar_ctrl_mtx_name;
    std::string camera_ctrl_mtx_name;
    std::string bbox_ctrl_mtx_name;
};

bool loadTopicsConfig(const std::string &filename, TopicsInfo &cfg) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(filename);
    } catch (const YAML::BadFile &e) {
        std::cerr << "Could not open config file: " << e.what() << "\n";
        return false;
    }

    
    auto topics = root["topics"];
    if (!topics) { std::cerr << "Missing topics section\n"; return false; }

    
    auto calib = topics["calibrator"];
    if (!calib) { std::cerr << "Missing calibrator section\n"; return false; }

    auto lidar = calib["lidar"];
    if (!lidar) { std::cerr << "Missing lidar section in topics\n"; return false; }

    cfg.lidar_recieve_queue_name = lidar["lidar_recieve_queue_name"].as<std::string>();
    cfg.lidar_ctrl_mtx_name = lidar["lidar_ctrl_mtx_name"].as<std::string>();
    cfg.bbox_ctrl_mtx_name = lidar["bbox_ctrl_mtx_name"].as<std::string>();

    auto camera = calib["camera"];
    if (!camera) { std::cerr << "Missing camera section in topics\n"; return false; }

    cfg.camera_recieve_queue_name = camera["camera_recieve_queue_name"].as<std::string>();
    cfg.camera_ctrl_mtx_name = camera["camera_ctrl_mtx_name"].as<std::string>();

    auto shared = calib["shared"];
    if (!shared) { std::cerr << "Missing shared section in topics\n"; return false; }

    cfg.shared_mem_name = shared["shared_mem_name"].as<std::string>();
    cfg.bbox_shared_mem_name = shared["bbox_shared_mem_name"].as<std::string>();

    return true;
}

struct SimTopicsInfo{

    std::string shared_mem_name;

    std::string camera_recieve_queue_name;
    std::string lidar_recieve_queue_name;

    std::string lidar_ctrl_mtx_name;
    std::string camera_ctrl_mtx_name;
};

bool loadSimTopicsConfig(const std::string &filename, SimTopicsInfo &cfg) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(filename);
    } catch (const YAML::BadFile &e) {
        std::cerr << "Could not open config file: " << e.what() << "\n";
        return false;
    }

    
    auto topics = root["topics"];
    if (!topics) { std::cerr << "Missing topics section\n"; return false; }

    
    auto simul = topics["simulator"];
    if (!simul) { std::cerr << "Missing simulator section\n"; return false; }

    auto lidar = simul["lidar"];
    if (!lidar) { std::cerr << "Missing lidar section in topics\n"; return false; }

    cfg.lidar_recieve_queue_name = lidar["lidar_recieve_queue_name"].as<std::string>();
    cfg.lidar_ctrl_mtx_name = lidar["lidar_ctrl_mtx_name"].as<std::string>();

    auto camera = simul["camera"];
    if (!camera) { std::cerr << "Missing camera section in topics\n"; return false; }

    cfg.camera_recieve_queue_name = camera["camera_recieve_queue_name"].as<std::string>();
    cfg.camera_ctrl_mtx_name = camera["camera_ctrl_mtx_name"].as<std::string>();

    auto shared = simul["shared"];
    if (!shared) { std::cerr << "Missing shared section in topics\n"; return false; }

    cfg.shared_mem_name = shared["shared_mem_name"].as<std::string>();

    return true;
}