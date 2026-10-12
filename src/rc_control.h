/*
 * This file is part of the MAVLink Router project
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
#pragma once

#include <functional>
#include <string>

#include "pollable.h"

/*
 * The UDP socket RcGate's mode arrives on: one datagram per mode, as text
 * ("open", "pilot", "local"). The controller re-sends it every second, so a
 * lost datagram or a router restart costs a second at most.
 *
 * No authentication: the port is meant to be reachable from the controller's
 * network only (a compose network, never published on the host).
 */
class RcControl : public Pollable {
public:
    explicit RcControl(std::function<void(const std::string &)> cb);

    /*
     * Binds 0.0.0.0:port. Returns false (and logs why) on failure.
     */
    bool open(unsigned long port);

    int handle_read() override;
    bool handle_canwrite() override { return false; }
    // A bad datagram or a socket error must never take the router down.
    bool is_critical() override { return false; }

private:
    std::function<void(const std::string &)> _cb;
};
