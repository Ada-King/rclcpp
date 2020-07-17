// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/node_options.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <climits>
#include <map>

#include "rclcpp/detail/utilities.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/qos.hpp"

#include "rcutils/get_env.h"

using rclcpp::exceptions::throw_from_rcl_error;

namespace rclcpp
{

namespace detail
{
static
void
rcl_node_options_t_destructor(rcl_node_options_t * node_options)
{
  if (node_options) {
    rcl_ret_t ret = rcl_node_options_fini(node_options);
    if (RCL_RET_OK != ret) {
      // Cannot throw here, as it may be called in the destructor.
      RCLCPP_ERROR(
        rclcpp::get_logger("rclcpp"),
        "failed to finalize rcl node options: %s", rcl_get_error_string().str);
      rcl_reset_error();
    }

    delete node_options;
    node_options = nullptr;
  }
}
}  // namespace detail

NodeOptions::NodeOptions(rcl_allocator_t allocator)
: node_options_(nullptr, detail::rcl_node_options_t_destructor), allocator_(allocator)
{}

NodeOptions::NodeOptions(const NodeOptions & other)
: node_options_(nullptr, detail::rcl_node_options_t_destructor)
{
  *this = other;
}

NodeOptions &
NodeOptions::operator=(const NodeOptions & other)
{
  if (this != &other) {
    this->context_ = other.context_;
    this->arguments_ = other.arguments_;
    this->parameter_overrides_ = other.parameter_overrides_;
    this->use_global_arguments_ = other.use_global_arguments_;
    this->enable_rosout_ = other.enable_rosout_;
    this->use_intra_process_comms_ = other.use_intra_process_comms_;
    this->enable_topic_statistics_ = other.enable_topic_statistics_;
    this->start_parameter_services_ = other.start_parameter_services_;
    this->allocator_ = other.allocator_;
    this->allow_undeclared_parameters_ = other.allow_undeclared_parameters_;
    this->automatically_declare_parameters_from_overrides_ =
      other.automatically_declare_parameters_from_overrides_;
    this->node_options_.reset();
  }
  return *this;
}

const rcl_node_options_t *
NodeOptions::get_rcl_node_options() const
{
  // If it is nullptr, create it on demand.
  if (!node_options_) {
    node_options_.reset(new rcl_node_options_t);
    *node_options_ = rcl_node_get_default_options();
    node_options_->allocator = this->allocator_;
    node_options_->use_global_arguments = this->use_global_arguments_;
    node_options_->enable_rosout = this->enable_rosout_;
	node_options_->rosout_qos = this->get_rosout_qos_profile_from_env();

    int c_argc = 0;
    std::unique_ptr<const char *[]> c_argv;
    if (!this->arguments_.empty()) {
      if (this->arguments_.size() > static_cast<size_t>(std::numeric_limits<int>::max())) {
        throw_from_rcl_error(RCL_RET_INVALID_ARGUMENT, "Too many args");
      }

      c_argc = static_cast<int>(this->arguments_.size());
      c_argv.reset(new const char *[c_argc]);

      for (std::size_t i = 0; i < this->arguments_.size(); ++i) {
        c_argv[i] = this->arguments_[i].c_str();
      }
    }

    rcl_ret_t ret = rcl_parse_arguments(
      c_argc, c_argv.get(), this->allocator_, &(node_options_->arguments));

    if (RCL_RET_OK != ret) {
      throw_from_rcl_error(ret, "failed to parse arguments");
    }

    std::vector<std::string> unparsed_ros_arguments = detail::get_unparsed_ros_arguments(
      c_argc, c_argv.get(), &(node_options_->arguments), this->allocator_);
    if (!unparsed_ros_arguments.empty()) {
      throw exceptions::UnknownROSArgsError(std::move(unparsed_ros_arguments));
    }
  }

  return node_options_.get();
}

rclcpp::Context::SharedPtr
NodeOptions::context() const
{
  return this->context_;
}

NodeOptions &
NodeOptions::context(rclcpp::Context::SharedPtr context)
{
  this->context_ = context;
  return *this;
}

const std::vector<std::string> &
NodeOptions::arguments() const
{
  return this->arguments_;
}

NodeOptions &
NodeOptions::arguments(const std::vector<std::string> & arguments)
{
  this->node_options_.reset();  // reset node options to make it be recreated on next access.
  this->arguments_ = arguments;
  return *this;
}

std::vector<rclcpp::Parameter> &
NodeOptions::parameter_overrides()
{
  return this->parameter_overrides_;
}

const std::vector<rclcpp::Parameter> &
NodeOptions::parameter_overrides() const
{
  return this->parameter_overrides_;
}

NodeOptions &
NodeOptions::parameter_overrides(const std::vector<rclcpp::Parameter> & parameter_overrides)
{
  this->parameter_overrides_ = parameter_overrides;
  return *this;
}

bool
NodeOptions::use_global_arguments() const
{
  return this->use_global_arguments_;
}

NodeOptions &
NodeOptions::use_global_arguments(bool use_global_arguments)
{
  this->node_options_.reset();  // reset node options to make it be recreated on next access.
  this->use_global_arguments_ = use_global_arguments;
  return *this;
}

bool
NodeOptions::enable_rosout() const
{
  return this->enable_rosout_;
}

NodeOptions &
NodeOptions::enable_rosout(bool enable_rosout)
{
  this->node_options_.reset();  // reset node options to make it be recreated on next access.
  this->enable_rosout_ = enable_rosout;
  return *this;
}

bool
NodeOptions::use_intra_process_comms() const
{
  return this->use_intra_process_comms_;
}

NodeOptions &
NodeOptions::use_intra_process_comms(bool use_intra_process_comms)
{
  this->use_intra_process_comms_ = use_intra_process_comms;
  return *this;
}

bool
NodeOptions::enable_topic_statistics() const
{
  return this->enable_topic_statistics_;
}

NodeOptions &
NodeOptions::enable_topic_statistics(bool enable_topic_statistics)
{
  this->enable_topic_statistics_ = enable_topic_statistics;
  return *this;
}

bool
NodeOptions::start_parameter_services() const
{
  return this->start_parameter_services_;
}

NodeOptions &
NodeOptions::start_parameter_services(bool start_parameter_services)
{
  this->start_parameter_services_ = start_parameter_services;
  return *this;
}

bool
NodeOptions::start_parameter_event_publisher() const
{
  return this->start_parameter_event_publisher_;
}

NodeOptions &
NodeOptions::start_parameter_event_publisher(bool start_parameter_event_publisher)
{
  this->start_parameter_event_publisher_ = start_parameter_event_publisher;
  return *this;
}

const rclcpp::QoS &
NodeOptions::parameter_event_qos() const
{
  return this->parameter_event_qos_;
}

NodeOptions &
NodeOptions::parameter_event_qos(const rclcpp::QoS & parameter_event_qos)
{
  this->parameter_event_qos_ = parameter_event_qos;
  return *this;
}

const rclcpp::PublisherOptionsBase &
NodeOptions::parameter_event_publisher_options() const
{
  return parameter_event_publisher_options_;
}

NodeOptions &
NodeOptions::parameter_event_publisher_options(
  const rclcpp::PublisherOptionsBase & parameter_event_publisher_options)
{
  parameter_event_publisher_options_ = parameter_event_publisher_options;
  return *this;
}

bool
NodeOptions::allow_undeclared_parameters() const
{
  return this->allow_undeclared_parameters_;
}

NodeOptions &
NodeOptions::allow_undeclared_parameters(bool allow_undeclared_parameters)
{
  this->allow_undeclared_parameters_ = allow_undeclared_parameters;
  return *this;
}

bool
NodeOptions::automatically_declare_parameters_from_overrides() const
{
  return this->automatically_declare_parameters_from_overrides_;
}

NodeOptions &
NodeOptions::automatically_declare_parameters_from_overrides(
  bool automatically_declare_parameters_from_overrides)
{
  this->automatically_declare_parameters_from_overrides_ =
    automatically_declare_parameters_from_overrides;
  return *this;
}

const rcl_allocator_t &
NodeOptions::allocator() const
{
  return this->allocator_;
}

NodeOptions &
NodeOptions::allocator(rcl_allocator_t allocator)
{
  this->node_options_.reset();  // reset node options to make it be recreated on next access.
  this->allocator_ = allocator;
  return *this;
}

rmw_qos_profile_t 
NodeOptions::get_rosout_qos_profile_from_env() const
{
  // Obtained the rosout qos setting through environment variables.
  rmw_qos_profile_t rosout_qos_profile = rmw_rosout_qos_profile_default;
  const char *get_env_error_str = nullptr;
  const char *qos_depth = nullptr;
  const char *qos_durability = nullptr;
  const char *qos_lifespan = nullptr;
  char *end = nullptr;
  unsigned long number = 0UL;
  uint64_t sec = 0;
  uint64_t nsec = 0;
  constexpr const char *depth_env_var = "ROSOUT_QOS_DEPTH";
  constexpr const char *durability_env_var = "ROSOUT_QOS_DURABILITY";
  constexpr const char *lifespan_env_var = "ROSOUT_QOS_LIFESPAN";

  auto qos_depth_callback = [&](){
	  number = strtoul(qos_depth, &end, 0);
	  if(0 == number && *end != '\0'){
		// TODO(dbt) : Maybe here shouldn't throw exception, if the user's setting is not resonable, 
		// just ignore it. The same below.
		throw std::runtime_error("ROSOUT_QOS_DEPTH is not an integral number");
	  }else if((number == ULONG_MAX && errno == ERANGE) || number > std::numeric_limits<uint32_t>::max()){
	  	throw std::runtime_error("ROSOUT_QOS_DEPTH is not an valid number( > 0)");
	  }else{
	  	rosout_qos_profile.depth = static_cast<size_t>(number);
	  }
  };
  
  auto qos_durability_callback = [&](){
	  std::map<std::string, rmw_qos_durability_policy_t> durability_map{
		  {"ROSOUT_DEFAULT", RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT},
		  {"ROSOUT_TRANSIENT_LOCAL", RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL},
		  {"ROSOUT_VOLATILE", RMW_QOS_POLICY_DURABILITY_VOLATILE}
	  };
	  auto iter = durability_map.find(qos_durability);
	  if(iter != durability_map.end())
		  rosout_qos_profile.durability = iter->second;
  };

  auto qos_lifespan_callback = [&](){
	  char *tmp_nsec = nullptr;
	  char *tmp_sec = strtok(const_cast<char *>(qos_lifespan), ".");
	  if(tmp_sec != NULL)
	  {
	  	sec = atoi(tmp_sec);
		tmp_nsec = strtok(NULL, ".");
	  }
	  if(NULL == tmp_sec || NULL == tmp_nsec)
	  	return;
	  
	  // TODO(dbt) : Here should add a check to user's settings, if it's unresonable, 
	  // then the value will not be set. I will do it later.
	  nsec = atoi(tmp_nsec);
	  rosout_qos_profile.lifespan.sec = sec;
	  rosout_qos_profile.lifespan.nsec = nsec;
  };
  
  get_env_error_str = rcutils_get_env(depth_env_var, &qos_depth);
  if (NULL != get_env_error_str) {
    throw std::runtime_error("failed to interpret ROSOUT_QOS_DEPTH as integral number");
  }
  if(qos_depth && strcmp(qos_depth, "") != 0){
	qos_depth_callback();
  }

  get_env_error_str = rcutils_get_env(durability_env_var, &qos_durability);
  if(get_env_error_str != NULL){
	 throw std::runtime_error("failed to interpret ROSOUT_QOS_DURABILITY as integral number");
  }
  if(qos_durability && strcmp(qos_durability, "") != 0){
	 qos_durability_callback();
  }
  
  get_env_error_str = rcutils_get_env(lifespan_env_var, &qos_lifespan);
  if(get_env_error_str!= NULL){
	 throw std::runtime_error("failed to interpret ROSOUT_QOS_LIFESPAN as integral number");
  }
  if(qos_lifespan && strcmp(qos_lifespan, "") != 0){
	 qos_lifespan_callback();
  }
  
  return rosout_qos_profile;
}


}  // namespace rclcpp
