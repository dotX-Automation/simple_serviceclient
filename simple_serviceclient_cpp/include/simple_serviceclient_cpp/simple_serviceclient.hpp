/**
 * rclcpp service client wrapper implementation.
 *
 * Roberto Masocco <r.masocco@dotxautomation.com>
 *
 * September 23, 2023
 */

/**
 * Copyright 2024 dotX Automation s.r.l.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SIMPLE_SERVICECLIENT_CPP__SIMPLE_SERVICECLIENT_HPP_
#define SIMPLE_SERVICECLIENT_CPP__SIMPLE_SERVICECLIENT_HPP_

#include "visibility_control.h"

#include <chrono>
#include <memory>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

namespace simple_serviceclient
{

/**
 * Wraps a service client providing fully a/synchronous, transparent operation.
 * The request is either handled in a single call that directly returns the final
 * result, but internally runs the back-end by spinning the node when
 * necessary, or in an asynchronous way, returning a Future object.
 */
template<typename ServiceT>
class SIMPLE_SERVICECLIENT_PUBLIC Client final
{
public:
  using RequestT = typename ServiceT::Request;
  using ResponseT = typename ServiceT::Response;

  using SharedPtr = std::shared_ptr<Client<ServiceT>>;
  using WeakPtr = std::weak_ptr<Client<ServiceT>>;
  using UniquePtr = std::unique_ptr<Client<ServiceT>>;
  using ConstSharedPtr = std::shared_ptr<const Client<ServiceT>>;
  using ConstWeakPtr = std::weak_ptr<const Client<ServiceT>>;

  /**
   * @brief Constructor, can wait for the server to become active.
   *
   * @param node The node to be used to manage the client.
   * @param service_name The name of the service to be called.
   * @param wait Whether to wait for the server to become active.
   *
   * @throws RuntimeError if interrupted while waiting.
   */
  explicit Client(
    rclcpp::Node * node,
    const std::string & service_name,
    bool wait = true)
  : node_(node)
  {
    // Create the client
    client_ = node_->create_client<ServiceT>(service_name);

    // Wait for the server to become active
    while (wait && !client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        throw std::runtime_error("Interrupted while waiting for service " + service_name);
      }
      RCLCPP_WARN(
        node_->get_logger(),
        "Service %s not available...",
        service_name.c_str());
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "Initialized client for service %s",
      service_name.c_str());
  }

  /**
   * @brief Destructor.
   */
  ~Client()
  {
    client_.reset();
  }

  /**
   * @brief Calls the service and spins the node while waiting.
   *
   * @param request The request to be sent to the server.
   * @param spin Enables node spinning, else this will block waiting on the future ("get" call).
   * @return The response received from the server.
   *
   * @throws std::runtime_error if interrupted while waiting.
   */
  std::shared_ptr<ResponseT> call_sync(std::shared_ptr<RequestT> request, bool spin = false)
  {
    // Call the service
    auto response_future = client_->async_send_request(request);

    // Wait synchronously if requested
    if (!spin) {
      return response_future.get();
    }

    // Spin the node while waiting
    auto code = rclcpp::spin_until_future_complete(node_->shared_from_this(), response_future);
    if (code == rclcpp::FutureReturnCode::SUCCESS) {
      return response_future.get();
    } else {
      client_->remove_pending_request(response_future);
      throw std::runtime_error(
              "Interrupted while waiting for reponse from service " +
              std::string(client_->get_service_name()));
    }
  }

  /**
   * @brief Calls the service asynchronously
   *
   * @param request The request to be sent to the server.
   * @return A future object that will contain the response.
   */
  typename rclcpp::Client<ServiceT>::FutureAndRequestId call_async(std::shared_ptr<RequestT> request)
  {
    return client_->async_send_request(request);
  }

private:
  rclcpp::Node * node_;
  std::shared_ptr<rclcpp::Client<ServiceT>> client_;
};

} // namespace simple_serviceclient

#endif // SIMPLE_SERVICECLIENT_CPP__SIMPLE_SERVICECLIENT_HPP_
