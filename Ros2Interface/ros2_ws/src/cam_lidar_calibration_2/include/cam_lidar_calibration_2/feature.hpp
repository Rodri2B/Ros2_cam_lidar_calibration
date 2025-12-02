#pragma once

#include "rclcpp/rclcpp.hpp"

class feature_extractor{

    public:

    feature_extractor();


    private:

    rclcpp::Node::SharedPtr nh_ref;

};