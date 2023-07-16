// Copyright 2023 NITK.K ROS-Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "example_cpp/params_test.hpp"

ParameterExample::ParameterExample(const rclcpp::NodeOptions &options)
    : Node("params_test", options)
{
    using namespace std::chrono_literals; // NOLINT
    this->timer = this->create_wall_timer(0s,
        std::bind(&ParameterExample::onInit, this));
}

void ParameterExample::onInit()
{
    this->timer->cancel();
    this->param_listener_ = std::make_shared<parameter_test::ParamListener>(
        this->get_node_parameters_interface());
    this->params_ = param_listener_->get_params();

// basic --------------------
    RCLCPP_INFO(this->get_logger(), "data_string: %s", this->params_.data_string.c_str());

// map --------------------
    this->params_.test_map.data_double_map["b"].data_double_1 = 0.0;

    for (auto d : this->params_.data_double) {
        RCLCPP_INFO(this->get_logger(), "test_map.data_double_map[%s].data_double_0: %lf",
            d.c_str(),
            this->params_.test_map.data_double_map[d].data_double_0);
        RCLCPP_INFO(this->get_logger(), "test_map.data_double_map[%s].data_double_1: %lf",
            d.c_str(),
            this->params_.test_map.data_double_map[d].data_double_1);
        RCLCPP_INFO(this->get_logger(), "test_map.data_double_map[%s].data_double_2: %lf",
            d.c_str(),
            this->params_.test_map.data_double_map[d].data_double_2);
    }


// fixed --------------------
    RCLCPP_INFO(this->get_logger(),
        "fixed_types.fixed_string: %s",
        std::string{this->params_.fixed_types.fixed_string}.c_str());
    int count = 0;
    for (auto d : this->params_.fixed_types.fixed_double_array) {
        RCLCPP_INFO(this->get_logger(),
            "fixed_types.fixed_double_array[%d]: %lf",count++, d);
    }

// size_comparison --------------------
    RCLCPP_INFO(this->get_logger(),
        "size_comparison.lt_eq_fifteen: %ld",
        this->params_.size_comparison.lt_eq_fifteen);
    RCLCPP_INFO(this->get_logger(),
        "size_comparison.gt_fifteen: %ld",
        this->params_.size_comparison.gt_fifteen);
    RCLCPP_INFO(this->get_logger(),
        "size_comparison.one_number: %ld",
        this->params_.size_comparison.one_number);

// int_array --------------------
    RCLCPP_INFO(this->get_logger(), "root_read.read_only_int: %ld", this->params_.root_read.read_only_int);

    count = 0;
    for (auto d : this->params_.root_read.read_only_int_array) {
        RCLCPP_INFO(this->get_logger(), "root_read.int_array[%d]: %ld", count++, d);
    }

    count = 0;
    for (auto d : this->params_.root_read.three_numbers_of_five) {
        RCLCPP_INFO(this->get_logger(), "root_read.three_numbers_of_five[%d]: %ld", count++, d);
    }

    rclcpp::shutdown();
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParameterExample>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}