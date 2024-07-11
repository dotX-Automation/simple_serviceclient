# simple_serviceclient_cpp

C++ library that wraps the `rclcpp` service client providing a simpler interface.

## Contents

This package offers the `simple_serviceclient_cpp::Client<ServiceT>` class template, that can be used to create a ROS 2 service client just by passing the node and the service name.

### Features

The service can be called in two different fashions:

- `call_sync`: synchronous call, that blocks until the service response is received but spins the node in the meantime, returning the response.
- `call_async`: asynchronous call, that returns a `FutureAndRequestId` object that can be used to retrieve the response when it is received.

---

## Copyright and License

Copyright 2024 dotX Automation s.r.l.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.

You may obtain a copy of the License at <http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

See the License for the specific language governing permissions and limitations under the License.
