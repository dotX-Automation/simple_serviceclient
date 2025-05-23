"""
Simple rclpy service client wrapper.

Roberto Masocco <r.masocco@dotxautomation.com>

September 11, 2022
"""

# Copyright 2024 dotX Automation s.r.l.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.task import Future

from typing import TypeVar

ServiceType = TypeVar('ServiceType')
ReqType = TypeVar('ReqType')
RespType = TypeVar('RespType')


class Client():
    """
    Wraps a service client providing fully a/synchronous, transparent operation.
    The request is either handled in a single call that directly returns the final
    result, but internally runs the back-end by spinning the node when
    necessary, or in an asynchronous way, returning a Future object.
    """

    def __init__(
            self,
            node: Node,
            type: ServiceType,
            service_name: str,
            wait: bool = True) -> None:
        """
        Creates a new Client.
        Can wait for the server to become active.

        :param node: Reference to the ROS 2 node to use.
        :param type: Service interface type.
        :param service_name: Name of the service to look for.
        :param wait: Indicates whether to wait for the server immediately.
        """
        # Create the ROS 2 service client and link the provided node
        self._node = node
        self._client = self._node.create_client(type, service_name)

        # Wait for the server to come up
        while wait and not self._client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise RuntimeError(
                    "Interrupted while waiting for service {}".format(self._client.srv_name))
            self._node.get_logger().warn(
                "Service {} not available...".format(self._client.srv_name))

    def call_sync(self, request: ReqType, timeout_sec: float = None) -> RespType:
        """
        Calls the service, returns only when the request has been completed.
        Spins the node while waiting.

        :param request: Service request to send.
        :param timeout_sec: Timeout for the service call (seconds).
        :returns: Service response, or None if the call timed out.
        """
        resp_future = self._client.call_async(request)
        if timeout_sec is None or timeout_sec <= 0.0:
            rclpy.spin_until_future_complete(self._node, resp_future)
        else:
            rclpy.spin_until_future_complete(self._node, resp_future, None, timeout_sec)
        if not rclpy.ok():
            raise RuntimeError(
                "Interrupted while waiting for response from service {}".format(self._client.srv_name))
        if not resp_future.done():
            self._client.remove_pending_request(resp_future)
            return None
        return resp_future.result()

    def call_async(self, request: ReqType) -> Future:
        """
        Calls the service, returns immediately.
        Does not spin the node.

        :param request: Service request to send.
        :returns: Future object for the response.
        """
        return self._client.call_async(request)

    @property
    def service_name(self) -> str:
        return self._client.service_name
