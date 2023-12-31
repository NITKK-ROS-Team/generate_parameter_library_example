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

#include "example_cpp/params_test_0.hpp"

ParameterExample::ParameterExample(const rclcpp::NodeOptions &options)
    : Node("params_test", options)
{
    this->param_listener_ = std::make_shared<parameter_test::ParamListener>(
        this->get_node_parameters_interface());
    this->params_ = param_listener_->get_params();

// basic --------------------
    RCLCPP_INFO(this->get_logger(), "data_string: %s", this->params_.data_string.c_str());

    RCLCPP_INFO(this->get_logger(), "camera_params.width: %ld", this->params_.camera_params.width);
    RCLCPP_INFO(this->get_logger(), "camera_params.height: %ld", this->params_.camera_params.height);
    RCLCPP_INFO(this->get_logger(), "camera_params.fps: %ld", this->params_.camera_params.fps);

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
    RCLCPP_INFO(this->get_logger(), "read_only.data_int: %ld", this->params_.read_only.data_int);

    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        [this]() -> void {
            // get data_int, read_only_int
            this->params_ = param_listener_->get_params();
            RCLCPP_INFO(this->get_logger(), "data_int: %ld", this->params_.data_int);
            RCLCPP_INFO(this->get_logger(), "read_only.data_int: %ld", this->params_.read_only.data_int);
        });
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParameterExample>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}