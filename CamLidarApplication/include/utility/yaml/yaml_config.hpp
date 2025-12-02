#pragma once

#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>



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

struct YamlConfig {
    BoardConfig chessboard;
    CameraConfig camera;
};

bool loadConfig(const std::string &filename, YamlConfig &cfg) {
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
