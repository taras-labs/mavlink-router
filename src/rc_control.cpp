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

#include "rc_control.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <utility>

#include <common/log.h>

RcControl::RcControl(std::function<void(const std::string &)> cb)
    : _cb(std::move(cb))
{
}

bool RcControl::open(unsigned long port)
{
    struct sockaddr_in addr = {};
    int val = 1;

    fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (fd < 0) {
        log_error("RC control: could not create socket (%m)");
        return false;
    }
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val));

    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        log_error("RC control: could not bind UDP 0.0.0.0:%lu (%m)", port);
        ::close(fd);
        fd = -1;
        return false;
    }

    log_info("RC control listening on UDP 0.0.0.0:%lu", port);
    return true;
}

int RcControl::handle_read()
{
    char data[64];

    // Drain everything that is queued: only the newest mode matters, but each
    // is applied in order so a log reads as what happened.
    for (;;) {
        ssize_t r = recv(fd, data, sizeof(data) - 1, 0);
        if (r < 0) {
            if (errno == EAGAIN || errno == EINTR) {
                return 0;
            }
            log_error("RC control: recv failed (%m)");
            return -1;
        }
        _cb(std::string(data, r));
    }
}
