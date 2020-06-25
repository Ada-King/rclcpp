// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include <gtest/gtest.h>

#include <memory>
#include <utility>

#include "rcl/node_options.h"
#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_base.hpp"
#include "rclcpp/node_interfaces/node_graph.hpp"
#include "rclcpp/rclcpp.hpp"
#include "test_msgs/msg/empty.hpp"

TEST(TestNodeGraph, construct_from_node)
{
  rclcpp::init(0, nullptr);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("node", "ns");

  // This dynamic cast is not necessary for the unittest itself, but the coverage utility lcov
  // reports these functions uncovered otherwise.
  const auto * node_graph =
    dynamic_cast<rclcpp::node_interfaces::NodeGraph *>(node->get_node_graph_interface().get());
  ASSERT_NE(nullptr, node_graph);

  auto topic_names_and_types = node_graph->get_topic_names_and_types(false);
  EXPECT_LT(0u, topic_names_and_types.size());

  auto service_names_and_types = node_graph->get_service_names_and_types();
  EXPECT_LT(0u, service_names_and_types.size());

  auto names = node_graph->get_node_names();
  EXPECT_EQ(1u, names.size());

  auto names_and_namespaces = node_graph->get_node_names_and_namespaces();
  EXPECT_EQ(1u, names_and_namespaces.size());

  EXPECT_EQ(0u, node_graph->count_publishers("not_a_topic"));
  EXPECT_EQ(0u, node_graph->count_subscribers("not_a_topic"));

  rclcpp::shutdown();
}

TEST(TestNodeGraph, get_topic_names_and_types)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("node2", "ns");
  const auto * node_graph =
    dynamic_cast<rclcpp::node_interfaces::NodeGraph *>(node->get_node_graph_interface().get());
  ASSERT_NE(nullptr, node_graph);
  auto topic_names_and_types = node_graph->get_topic_names_and_types();
  EXPECT_LT(0u, topic_names_and_types.size());
  rclcpp::shutdown();
}

TEST(TestNodeGraph, get_service_names_and_types)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("node2", "ns");
  const auto * node_graph =
    dynamic_cast<rclcpp::node_interfaces::NodeGraph *>(node->get_node_graph_interface().get());
  ASSERT_NE(nullptr, node_graph);
  auto service_names_and_types = node_graph->get_service_names_and_types();
  EXPECT_LT(0u, service_names_and_types.size());
  rclcpp::shutdown();
}

TEST(TestNodeGraph, get_service_names_and_types_by_node)
{
  rclcpp::init(0, nullptr);
  auto node1 = std::make_shared<rclcpp::Node>("node1", "ns");
  auto node2 = std::make_shared<rclcpp::Node>("node2", "ns");
  const auto * node_graph =
    dynamic_cast<rclcpp::node_interfaces::NodeGraph *>(node1->get_node_graph_interface().get());
  ASSERT_NE(nullptr, node_graph);

  EXPECT_THROW(
    node_graph->get_service_names_and_types_by_node("not_a_node", "not_absolute_namespace"),
    std::runtime_error);
  auto service_names_and_types1 = node_graph->get_service_names_and_types_by_node("node1", "/ns");
  auto service_names_and_types2 = node_graph->get_service_names_and_types_by_node("node2", "/ns");
  EXPECT_EQ(service_names_and_types1.size(), service_names_and_types2.size());
  rclcpp::shutdown();
}

TEST(TestNodeGraph, get_node_names_and_namespaces)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  const auto * node_graph =
    dynamic_cast<rclcpp::node_interfaces::NodeGraph *>(node->get_node_graph_interface().get());
  ASSERT_NE(nullptr, node_graph);

  auto names_and_namespaces = node_graph->get_node_names_and_namespaces();
  EXPECT_EQ(1u, names_and_namespaces.size());
  rclcpp::shutdown();
}

TEST(TestNodeGraph, notify_shutdown)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  auto * node_graph =
    dynamic_cast<rclcpp::node_interfaces::NodeGraph *>(node->get_node_graph_interface().get());
  ASSERT_NE(nullptr, node_graph);

  EXPECT_NO_THROW(node_graph->notify_shutdown());
  rclcpp::shutdown();
}

TEST(TestNodeGraph, wait_for_graph_change)
{
  rclcpp::init(0, nullptr);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("node", "ns");
  auto * node_graph =
    dynamic_cast<rclcpp::node_interfaces::NodeGraph *>(node->get_node_graph_interface().get());
  ASSERT_NE(nullptr, node_graph);

  EXPECT_NO_THROW(node_graph->notify_graph_change());
  EXPECT_THROW(
    node_graph->wait_for_graph_change(nullptr, std::chrono::milliseconds(1)),
    rclcpp::exceptions::InvalidEventError);

  auto event = std::make_shared<rclcpp::Event>();
  EXPECT_THROW(
    node_graph->wait_for_graph_change(event, std::chrono::milliseconds(0)),
    rclcpp::exceptions::EventNotRegisteredError);
  rclcpp::shutdown();
}

TEST(TestNodeGraph, get_info_by_topic)
{
  rclcpp::init(0, nullptr);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("node", "ns");
  auto publisher = node->create_publisher<test_msgs::msg::Empty>("topic", 1);

  auto callback = [](const test_msgs::msg::Empty::SharedPtr) {};
  auto subscription =
    node->create_subscription<test_msgs::msg::Empty>(
    "topic", rclcpp::QoS(10), std::move(callback));
  const auto * node_graph =
    dynamic_cast<rclcpp::node_interfaces::NodeGraph *>(node->get_node_graph_interface().get());
  ASSERT_NE(nullptr, node_graph);

  auto publishers = node_graph->get_publishers_info_by_topic("topic", false);
  auto publisher_endpoint_info = publishers[0];
  const auto const_publisher_endpoint_info = publisher_endpoint_info;
  ASSERT_EQ(1u, publishers.size());
  EXPECT_STREQ("node", publisher_endpoint_info.node_name().c_str());
  EXPECT_STREQ("node", const_publisher_endpoint_info.node_name().c_str());
  EXPECT_STREQ("/ns", publisher_endpoint_info.node_namespace().c_str());
  EXPECT_STREQ("/ns", const_publisher_endpoint_info.node_namespace().c_str());
  EXPECT_STREQ("test_msgs/msg/Empty", publisher_endpoint_info.topic_type().c_str());
  EXPECT_STREQ("test_msgs/msg/Empty", const_publisher_endpoint_info.topic_type().c_str());
  EXPECT_EQ(rclcpp::EndpointType::Publisher, publisher_endpoint_info.endpoint_type());
  EXPECT_EQ(rclcpp::EndpointType::Publisher, const_publisher_endpoint_info.endpoint_type());

  auto endpoint_gid = publisher_endpoint_info.endpoint_gid();
  auto const_endpoint_gid = const_publisher_endpoint_info.endpoint_gid();
  bool endpoint_gid_is_all_zeros = true;
  for (size_t i = 0; i < RMW_GID_STORAGE_SIZE; ++i) {
    endpoint_gid_is_all_zeros &= (endpoint_gid[i] == 0);
    EXPECT_EQ(endpoint_gid[i], const_endpoint_gid[i]);
  }
  EXPECT_FALSE(endpoint_gid_is_all_zeros);
  rclcpp::shutdown();
}