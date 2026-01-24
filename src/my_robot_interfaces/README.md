# my_robot_interfaces

This package provides ROS 2 interface definitions that can be used by other nodes (C++ and Python) to interact with the robot. Currently, it contains a single service interface for sending simple Cartesian target positions.

---

## Provided Interface

### Service: `MoveToXYZ`

**Location:** `srv/MoveToXYZ.srv`

This service is used to send a Cartesian target position (X, Y, Z) to a robot or motion-planning logic (e.g. MoveIt).

```text
float64 x
float64 y
float64 z
---
bool success
string message
```

#### Request

* `x`, `y`, `z` (`float64`): Target position of the end effector in the reference coordinate frame (e.g. `base_link`).

#### Response

* `success` (`bool`): Indicates whether the motion was successful.
* `message` (`string`): Additional status or error information (e.g. timeout, planning failure).

**Design intent:**
The interface is intentionally minimal and abstracts complex motion planning behind a simple service call.

---

## Using the Interface in C++

### Add dependency

In the consuming package’s `CMakeLists.txt`:

```cmake
find_package(my_robot_interfaces REQUIRED)
```

In `package.xml`:

```xml
<depend>my_robot_interfaces</depend>
```

### Using the service (Client)

```cpp
#include "my_robot_interfaces/srv/move_to_xyz.hpp"

using MoveToXYZ = my_robot_interfaces::srv::MoveToXYZ;

auto client = node->create_client<MoveToXYZ>("move_to_xyz");

auto request = std::make_shared<MoveToXYZ::Request>();
request->x = 0.3;
request->y = 0.0;
request->z = 0.2;

client->async_send_request(request);
```

### Implementing the service (Server)

```cpp
auto service = node->create_service<MoveToXYZ>(
  "move_to_xyz",
  [](const std::shared_ptr<MoveToXYZ::Request> request,
     std::shared_ptr<MoveToXYZ::Response> response)
  {
    // Motion logic
    response->success = true;
    response->message = "Goal reached";
  }
);
```

---

## Using the Interface in Python

### Dependency

In the Python package’s `package.xml`:

```xml
<depend>my_robot_interfaces</depend>
```

### Using the service (Client)

```python
from my_robot_interfaces.srv import MoveToXYZ
import rclpy
from rclpy.node import Node

class MoveClient(Node):
    def __init__(self):
        super().__init__('move_client')
        self.client = self.create_client(MoveToXYZ, 'move_to_xyz')

        request = MoveToXYZ.Request()
        request.x = 0.3
        request.y = 0.0
        request.z = 0.2

        self.client.call_async(request)
```

### Implementing the service (Server)

```python
from my_robot_interfaces.srv import MoveToXYZ
import rclpy
from rclpy.node import Node

class MoveServer(Node):
    def __init__(self):
        super().__init__('move_server')
        self.service = self.create_service(
            MoveToXYZ,
            'move_to_xyz',
            self.callback
        )

    def callback(self, request, response):
        # Motion logic
        response.success = True
        response.message = 'Goal reached'
        return response
```

---

## Summary

* `my_robot_interfaces` is a pure **interface package**
* Provides the `MoveToXYZ` service
* Enables simple XYZ target commands via ROS 2 services
* Can be used seamlessly from **C++ and Python**
* Cleanly separates interface definition from implementation
