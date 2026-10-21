/*
 * This file is part of the MAVLink Router project
 *
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
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
#include "mainloop.h"

#include <assert.h>
#include <signal.h>
#include <sys/epoll.h>
#include <sys/timerfd.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <memory>

#include <common/log.h>
#include <common/util.h>

#include "autolog.h"
#include "tlog.h"

static std::atomic<bool> should_exit{false};

Mainloop Mainloop::_instance{};
bool Mainloop::_initialized = false;

static void exit_signal_handler(int signum)
{
    Mainloop::instance().request_exit(0);
}

static void setup_signal_handlers()
{
    struct sigaction sa = {};

    sa.sa_flags = SA_NOCLDSTOP;
    sa.sa_handler = exit_signal_handler;
    sigaction(SIGTERM, &sa, nullptr);
    sigaction(SIGINT, &sa, nullptr);

    sa.sa_handler = SIG_IGN;
    sigaction(SIGPIPE, &sa, nullptr);
}

Mainloop &Mainloop::init()
{
    assert(_initialized == false);

    _initialized = true;

    return _instance;
}

void Mainloop::teardown()
{
    _initialized = false;
}

Mainloop &Mainloop::instance()
{
    return _instance;
}

void Mainloop::request_exit(int retcode)
{
    _retcode = retcode;
    should_exit.store(true, std::memory_order_relaxed);
}

int Mainloop::open()
{
    _retcode = -1;

    if (epollfd != -1) {
        return -EBUSY;
    }

    epollfd = epoll_create1(EPOLL_CLOEXEC);

    if (epollfd == -1) {
        log_error("%m");
        return -1;
    }

    _retcode = 0;

    return 0;
}

int Mainloop::mod_fd(int fd, void *data, int events) const
{
    struct epoll_event epev = {};

    epev.events = events;
    epev.data.ptr = data;

    if (epoll_ctl(epollfd, EPOLL_CTL_MOD, fd, &epev) < 0) {
        log_error("Could not mod fd (%m)");
        return -1;
    }

    return 0;
}

int Mainloop::add_fd(int fd, void *data, int events) const
{
    struct epoll_event epev = {};

    epev.events = events;
    epev.data.ptr = data;

    if (epoll_ctl(epollfd, EPOLL_CTL_ADD, fd, &epev) < 0) {
        log_error("Could not add fd to epoll (%m)");
        return -1;
    }

    return 0;
}

int Mainloop::remove_fd(int fd) const
{
    if (epoll_ctl(epollfd, EPOLL_CTL_DEL, fd, nullptr) < 0) {
        log_error("Could not remove fd from epoll (%m)");
        return -1;
    }

    return 0;
}

int Mainloop::write_msg(const std::shared_ptr<Endpoint> &e, const struct buffer *buf) const
{
    int r = e->write_msg(buf);

    /*
     * If endpoint would block, add EPOLLOUT event to get notified when it's
     * possible to write again
     */
    if (r == -EAGAIN) {
        mod_fd(e->fd, e.get(), EPOLLIN | EPOLLOUT);
    }

    return r;
}

/*
 * MAVProxy semantics, once any endpoint is a Master: what a master hears goes
 * to every non-master whatever it is addressed to (which is how one GCS reads
 * the autopilot's replies to another), what a non-master hears goes to the
 * masters only, routed by target as usual, and nothing passes between two
 * masters or two non-masters. Log endpoints and messages the router makes
 * itself (no source) keep the normal routing. @uplink_masters is a mask of
 * master ranks a non-master's message may go to (master failover).
 */
Endpoint::AcceptState Mainloop::_hub_accept(const Endpoint &e, const struct buffer *buf,
                                            const Endpoint *source,
                                            uint64_t uplink_masters) const
{
    if (source == nullptr || e.get_type() == ENDPOINT_TYPE_LOG) {
        return e.accept_msg(buf);
    }

    if (&e == source || e.is_master() == source->is_master()) {
        return Endpoint::AcceptState::Rejected;
    }

    if (e.is_master() && ((uplink_masters >> e.get_master_rank()) & 1) == 0) {
        return Endpoint::AcceptState::Rejected;
    }

    return e.accept_msg(buf, !source->is_master());
}

/*
 * Master failover, vehicle side: of the masters a vehicle (source sysid) is
 * heard on, only the best one that delivered within MasterFailoverTimeout is
 * passed on. The others are still recorded, so the next one is ready.
 */
bool Mainloop::_failover_passes(const struct buffer *buf, const Endpoint &source)
{
    const int rank = source.get_master_rank();
    auto v = _master_failover.observe(buf->curr.src_sysid, rank, now_usec());

    if (v.changed) {
        const std::string to = _masters[v.winner]->get_name();
        if (v.previous < 0) {
            log_info("Master failover: sysid %u via %s", buf->curr.src_sysid, to.c_str());
        } else if (v.winner < v.previous) {
            log_info("Master failover: sysid %u back on %s from %s",
                     buf->curr.src_sysid,
                     to.c_str(),
                     _masters[v.previous]->get_name().c_str());
        } else {
            log_warning("Master failover: sysid %u switched to %s, %s silent for %u ms",
                        buf->curr.src_sysid,
                        to.c_str(),
                        _masters[v.previous]->get_name().c_str(),
                        _master_failover.timeout_ms());
        }
    }

    return v.winner == rank;
}

/*
 * Master failover, GCS side: a message for a vehicle goes out on the master
 * that vehicle is heard on now, one with no target on the current master of
 * every autopilot, so a vehicle on two links does not get each command twice.
 * Nothing heard within the timeout: every master, as without failover.
 */
uint64_t Mainloop::_uplink_masters(const struct buffer *buf) const
{
    const usec_t now = now_usec();
    uint64_t mask = 0;

    if (buf->curr.target_sysid > 0) {
        int winner = _master_failover.winner(buf->curr.target_sysid, now);
        if (winner >= 0) {
            mask = (uint64_t)1 << winner;
        }
    } else {
        for (const auto &m : _masters) {
            for (auto sys_comp_id : m->get_autopilots()) {
                int winner = _master_failover.winner(sys_comp_id >> 8, now);
                if (winner >= 0) {
                    mask |= (uint64_t)1 << winner;
                }
            }
        }
    }

    return mask != 0 ? mask : ~(uint64_t)0;
}

void Mainloop::route_msg(struct buffer *buf, const Endpoint *source)
{
    bool unknown = true;
    uint64_t uplink_masters = ~(uint64_t)0;

    if (_message_disable.enabled() && source != nullptr && source->is_master()) {
        _disable_heard(buf, *source);
    }

    if (_rc_gate.enabled() && RcGate::is_rc(buf->curr.msg_id)) {
        // Where it came in, and from whom: what binds a compid to a sender.
        const std::string endpoint = source != nullptr ? source->get_name() : "";
        const std::string peer = source != nullptr ? source->peer_address() : "";
        if (!_rc_gate.check(buf->curr.msg_id,
                            buf->curr.src_compid,
                            buf->curr.payload,
                            buf->curr.payload_len,
                            now_usec(),
                            endpoint,
                            peer)) {
            log_trace("Message %u from %u/%u on %s (%s) dropped by the RC gate (%s)",
                      buf->curr.msg_id,
                      buf->curr.src_sysid,
                      buf->curr.src_compid,
                      endpoint.c_str(),
                      peer.c_str(),
                      RcGate::mode_name(_rc_gate.mode()));
            _log_rc_pass_change();
            return;
        }
        _log_rc_pass_change();
        // TODO(pilot-sysid): remove once every pilot runs v3.7.0+.
        if (_rc_pilot_sysid != 0 && buf->curr.src_sysid != _rc_pilot_sysid
            && endpoint == _rc_gate.pilot_endpoint()) {
            _patch_pilot_sysid(buf);
        }
    }

    if (!_comp_priority.check(buf->curr.msg_id,
                              buf->curr.target_sysid,
                              buf->curr.src_compid,
                              now_usec())) {
        log_trace("Message %u to %d/%d from %u/%u dropped by component priority",
                  buf->curr.msg_id,
                  buf->curr.target_sysid,
                  buf->curr.target_compid,
                  buf->curr.src_sysid,
                  buf->curr.src_compid);
        return;
    }

    if (_master_failover.ranks() > 0 && source != nullptr) {
        if (!source->is_master()) {
            uplink_masters = _uplink_masters(buf);
        } else if (!_failover_passes(buf, *source)) {
            log_trace("Message %u from %u/%u on standby master %s dropped",
                      buf->curr.msg_id,
                      buf->curr.src_sysid,
                      buf->curr.src_compid,
                      source->get_name().c_str());
            return;
        }
    }

    for (const auto &e : this->g_endpoints) {
        auto acceptState = _hub_mode ? _hub_accept(*e, buf, source, uplink_masters)
                                     : e->accept_msg(buf);

        switch (acceptState) {
        case Endpoint::AcceptState::Accepted:
            log_trace("Endpoint [%d] accepted message %u to %d/%d from %u/%u",
                      e->fd,
                      buf->curr.msg_id,
                      buf->curr.target_sysid,
                      buf->curr.target_compid,
                      buf->curr.src_sysid,
                      buf->curr.src_compid);
            if (write_msg(e, buf) == -EPIPE) { // only TCP endpoints should return -EPIPE
                should_process_tcp_hangups = true;
            }
            unknown = false;
            break;
        case Endpoint::AcceptState::Filtered:
            log_trace("Endpoint [%d] filtered out message %u to %d/%d from %u/%u",
                      e->fd,
                      buf->curr.msg_id,
                      buf->curr.target_sysid,
                      buf->curr.target_compid,
                      buf->curr.src_sysid,
                      buf->curr.src_compid);
            unknown = false;
            break;
        case Endpoint::AcceptState::Rejected:
            // fall through
        default:
            break; // do nothing (will count as unknown)
        }
    }

    if (unknown) {
        _errors_aggregate.msg_to_unknown++;
        log_trace("Message %u to unknown sysid/compid: %d/%d",
                  buf->curr.msg_id,
                  buf->curr.target_sysid,
                  buf->curr.target_compid);
    }
}

void Mainloop::process_tcp_hangups()
{
    // Remove endpoints, which are invalid
    for (auto it = g_endpoints.begin(); it != g_endpoints.end();) {
        if (it->get()->get_type() == ENDPOINT_TYPE_TCP) {
            auto *tcp_endpoint = static_cast<TcpEndpoint *>(it->get());
            if (!tcp_endpoint->is_valid()) {
                it = g_endpoints.erase(it);
            } else {
                ++it;
            }
        } else {
            ++it;
        }
    }

    should_process_tcp_hangups = false;
}

void Mainloop::handle_tcp_connection()
{
    log_debug("TCP Server: New client");

    auto *tcp = new TcpEndpoint{"dynamic"};

    int fd = tcp->accept(g_tcp_fd);
    if (fd == -1) {
        goto accept_error;
    }

    g_endpoints.emplace_back(tcp);
    this->add_fd(g_endpoints.back()->fd, g_endpoints.back().get(), EPOLLIN);

    return;

accept_error:
    log_error("TCP Server: Could not accept TCP connection (%m)");
    delete tcp;
}

int Mainloop::loop()
{
    const int max_events = 8;
    struct epoll_event events[max_events];
    int r;

    if (epollfd < 0) {
        return -EINVAL;
    }

    setup_signal_handlers();

    add_timeout(LOG_AGGREGATE_INTERVAL_SEC * MSEC_PER_SEC,
                std::bind(&Mainloop::_log_aggregate_timeout, this, std::placeholders::_1),
                this);

    while (!should_exit.load(std::memory_order_relaxed)) {
        int i;

        r = epoll_wait(epollfd, events, max_events, -1);
        if (r < 0 && errno == EINTR) {
            continue;
        }

        for (i = 0; i < r; i++) {
            if (events[i].data.ptr == &g_tcp_fd) {
                handle_tcp_connection();
                continue;
            }

            auto *p = static_cast<Pollable *>(events[i].data.ptr);

            if (events[i].events & EPOLLIN) {
                int rd = p->handle_read();
                if (rd < 0 && !p->is_valid()) {
                    // Only TcpEndpoint may become invalid after a read
                    should_process_tcp_hangups = true;
                }
            }

            if (events[i].events & EPOLLOUT) {
                if (!p->handle_canwrite()) {
                    mod_fd(p->fd, p, EPOLLIN);
                }
            }

            if (events[i].events & EPOLLERR) {
                log_error("poll error for fd %i", p->fd);

                if (p->is_critical()) {
                    log_error("Critical fd %i got error, exiting", p->fd);
                    request_exit(EXIT_FAILURE);
                } else {
                    log_debug("Non-critical fd %i, error is okay.", p->fd);
                }
            }
        }

        if (should_process_tcp_hangups) {
            process_tcp_hangups();
        }

        _del_timeouts();
    }

    if (_log_endpoint != nullptr) {
        _log_endpoint->stop();
    }

    clear_endpoints();

    if (_rc_control != nullptr) {
        remove_fd(_rc_control->fd);
        _rc_control.reset();
    }

    // free all remaning Timeouts
    while (_timeouts != nullptr) {
        Timeout *current = _timeouts;
        _timeouts = current->next;
        remove_fd(current->fd);
        delete current;
    }

    return _retcode;
}

bool Mainloop::_log_aggregate_timeout(void *data)
{
    if (_errors_aggregate.msg_to_unknown > 0) {
        log_warning("%u messages to unknown endpoints in the last %d seconds",
                    _errors_aggregate.msg_to_unknown,
                    LOG_AGGREGATE_INTERVAL_SEC);
        _errors_aggregate.msg_to_unknown = 0;
    }

    for (const auto &e : g_endpoints) {
        e->log_aggregate(LOG_AGGREGATE_INTERVAL_SEC);
    }
    return true;
}

void Mainloop::print_statistics()
{
    for (const auto &e : g_endpoints) {
        e->print_statistics();
    }
}

static bool _print_statistics_timeout_cb(void *data)
{
    auto *mainloop = static_cast<Mainloop *>(data);
    mainloop->print_statistics();
    return true;
}

bool Mainloop::dedup_check_msg(const buffer *buf)
{
    return _msg_dedup.check_packet(buf->data, buf->len)
        == Dedup::PacketStatus::NEW_PACKET_OR_TIMED_OUT;
}

void Mainloop::_write_to(const std::shared_ptr<Endpoint> &e, const mavlink_message_t *msg)
{
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    struct buffer buf = {};

    buf.len = mavlink_msg_to_send_buffer(data, msg);
    buf.data = data;

    if (write_msg(e, &buf) == -EPIPE) {
        should_process_tcp_hangups = true;
    }
}

bool Mainloop::_gcs_heartbeat_timeout_cb(void *data)
{
    mavlink_message_t msg;

    mavlink_msg_heartbeat_pack(_gcs.sysid,
                               _gcs.compid,
                               &msg,
                               MAV_TYPE_GCS,
                               MAV_AUTOPILOT_INVALID,
                               0,
                               0,
                               0);

    for (const auto &e : g_endpoints) {
        if (e->is_master()) {
            _write_to(e, &msg);
        }
    }

    return true;
}

void Mainloop::_request_streams(const std::shared_ptr<Endpoint> &e, uint16_t sys_comp_id)
{
    mavlink_message_t msg;

    mavlink_msg_request_data_stream_pack(_gcs.sysid,
                                         _gcs.compid,
                                         &msg,
                                         sys_comp_id >> 8,
                                         sys_comp_id & 0xff,
                                         MAV_DATA_STREAM_ALL,
                                         _gcs.stream_rate,
                                         1);
    _write_to(e, &msg);
}

void Mainloop::request_streams(const Endpoint *master, uint16_t sys_comp_id)
{
    if (_gcs.stream_rate == 0) {
        return;
    }

    for (const auto &e : g_endpoints) {
        if (e.get() == master) {
            _request_streams(e, sys_comp_id);
            return;
        }
    }
}

// MAVProxy re-sends this every 15 s, so a vehicle that rebooted, or a rate
// another GCS changed, comes back on its own.
bool Mainloop::_stream_rate_timeout_cb(void *data)
{
    for (const auto &e : g_endpoints) {
        if (!e->is_master()) {
            continue;
        }
        for (auto sys_comp_id : e->get_autopilots()) {
            _request_streams(e, sys_comp_id);
        }
    }

    return true;
}

// A listed message heard from an autopilot on a master is switched off on that
// link, the one the autopilot keeps the setting for -- the standby of a
// failover pair included, whose data is dropped only after this, so a switch
// to it does not bring the messages back.
void Mainloop::_disable_heard(const struct buffer *buf, const Endpoint &source)
{
    const uint32_t msg_id = buf->curr.msg_id;
    if (!_message_disable.listed(msg_id)) {
        return;
    }

    const uint16_t sys_comp_id = ((uint16_t)buf->curr.src_sysid << 8) | buf->curr.src_compid;
    if (!vector_contains(source.get_autopilots(), sys_comp_id)
        || !_message_disable.heard((uintptr_t)&source, sys_comp_id, msg_id, now_usec())) {
        return;
    }

    for (const auto &e : g_endpoints) {
        if (e.get() == &source) {
            mavlink_message_t msg;
            MessageDisable::pack(_gcs.sysid, _gcs.compid, sys_comp_id, msg_id, &msg);
            log_debug("DisableMessages: message %u heard from %u/%u on %s, switching it off",
                      msg_id,
                      buf->curr.src_sysid,
                      buf->curr.src_compid,
                      source.get_name().c_str());
            _write_to(e, &msg);
            return;
        }
    }
}

// RcPilotSysid, a transitional measure: a pilot up to 3.6.12 sends its RC as
// sysid 254, which the FC drops, so a station can be updated ahead of its
// pilots. TODO(pilot-sysid): remove once every pilot runs v3.7.0+.
void Mainloop::_patch_pilot_sysid(struct buffer *buf)
{
    const uint8_t was = buf->curr.src_sysid;
    if (!RcGate::patch_sysid(buf->data, buf->len, _rc_pilot_sysid)) {
        if (!_rc_pilot_sysid_warned) {
            log_warning("RC gate: a pilot frame as sysid %u is signed or malformed, passed on "
                        "as it is (RcPilotSysid %u)",
                        was,
                        _rc_pilot_sysid);
            _rc_pilot_sysid_warned = true;
        }
        return;
    }
    buf->curr.src_sysid = _rc_pilot_sysid;
    if (was != _rc_pilot_sysid_seen) {
        log_info("RC gate: the pilot sends RC as sysid %u, passed on as %u (RcPilotSysid)",
                 was,
                 _rc_pilot_sysid);
        _rc_pilot_sysid_seen = was;
    }
}

void Mainloop::_send_rc_override(const uint16_t ch[RcGate::CHANNELS])
{
    mavlink_message_t msg;

    for (const auto &e : g_endpoints) {
        if (!e->is_master()) {
            continue;
        }
        for (auto sys_comp_id : e->get_autopilots()) {
            mavlink_msg_rc_channels_override_pack(_gcs.sysid,
                                                  _gcs.compid,
                                                  &msg,
                                                  sys_comp_id >> 8,
                                                  sys_comp_id & 0xff,
                                                  ch[0],
                                                  ch[1],
                                                  ch[2],
                                                  ch[3],
                                                  ch[4],
                                                  ch[5],
                                                  ch[6],
                                                  ch[7],
                                                  ch[8],
                                                  ch[9],
                                                  ch[10],
                                                  ch[11],
                                                  ch[12],
                                                  ch[13],
                                                  ch[14],
                                                  ch[15],
                                                  ch[16],
                                                  ch[17]);
            _write_to(e, &msg);
        }
    }
}

void Mainloop::_log_rc_pass_change()
{
    bool blocked;

    if (_rc_gate.take_pass_change(&blocked)) {
        if (blocked) {
            log_warning("RC gate: no stick source, holding back the always-pass frames so the "
                        "FC sees the transmitter gone");
        } else {
            log_info("RC gate: a stick source is back, always-pass frames pass again");
        }
    }
}

void Mainloop::_rc_control_cb(const std::string &text)
{
    RcGate::Mode mode;
    std::string pilot_ip;

    if (!RcGate::parse_mode(text, &mode, &pilot_ip)) {
        // debug, not warning: the controller repeats itself every second
        log_debug("RC control: '%s' is not open, pilot [ip] or local", text.c_str());
        return;
    }

    const RcGate::Mode was = _rc_gate.mode();
    const std::string was_ip = _rc_gate.pilot_ip();
    if (!_rc_gate.set_mode(mode, now_usec(), pilot_ip)) {
        if (pilot_ip != was_ip) {
            log_info("RC gate: pilot's frames now from %s",
                     pilot_ip.empty() ? "any address" : pilot_ip.c_str());
        }
        return;
    }
    log_info("RC gate: %s -> %s%s%s",
             RcGate::mode_name(was),
             RcGate::mode_name(mode),
             pilot_ip.empty() ? "" : ", pilot at ",
             pilot_ip.c_str());

    if (was == RcGate::Mode::Pilot) {
        // The pilot's last frame, or the failsafe one, would otherwise go on
        // holding the sticks for the FC's RC_OVERRIDE_TIME after the pilot.
        uint16_t ch[RcGate::CHANNELS];
        if (_rc_gate.release_channels(_rc_failsafe_sent, ch)) {
            _send_rc_override(ch);
        }
        _rc_failsafe_sent = false;
    }
}

bool Mainloop::_rc_failsafe_timeout_cb(void *data)
{
    if (_rc_gate.failsafe_due(now_usec())) {
        if (!_rc_failsafe_sent) {
            log_warning("RC gate: remote pilot silent for %u ms, holding the FC in RC failsafe",
                        _rc_gate.failsafe_timeout_ms());
            _rc_failsafe_sent = true;
        }
        uint16_t ch[RcGate::CHANNELS];
        _rc_gate.failsafe_channels(ch);
        _send_rc_override(ch);
    } else if (_rc_failsafe_sent && _rc_gate.mode() == RcGate::Mode::Pilot) {
        log_info("RC gate: remote pilot is back, failsafe frame stopped");
        _rc_failsafe_sent = false;
    }

    return true;
}

bool Mainloop::add_endpoints(const Configuration &config)
{
    // Create UART and UDP endpoints
    if (config.sniffer_sysid != 0) {
        Endpoint::sniffer_sysid = config.sniffer_sysid;
        log_info("An endpoint with sysid %u on it will sniff all messages",
                 Endpoint::sniffer_sysid);
    }
    for (const auto &conf : config.uart_configs) {
        auto uart = std::make_shared<UartEndpoint>(conf.name);

        if (!uart->setup(conf)) {
            return false;
        }

        g_endpoints.push_back(uart);
        auto endpoint = g_endpoints.back();
        this->add_fd(endpoint->fd, endpoint.get(), EPOLLIN);
    }

    for (const auto &conf : config.udp_configs) {
        auto udp = std::make_shared<UdpEndpoint>(conf.name);

        if (!udp->setup(conf)) {
            return false;
        }

        g_endpoints.emplace_back(udp);
        auto endpoint = g_endpoints.back();
        this->add_fd(endpoint->fd, endpoint.get(), EPOLLIN);
    }

    // Create TCP endpoints
    for (const auto &conf : config.tcp_configs) {
        auto tcp = std::make_shared<TcpEndpoint>(conf.name);

        if (!tcp->setup(conf)) { // handles reconnect and add_fd
            return false;        // only on fatal errors
        }

        g_endpoints.emplace_back(tcp);
    }

    // Link grouped endpoints together
    for (auto e : g_endpoints) {
        if (e->get_group_name().empty()) {
            continue;
        }

        for (auto other : g_endpoints) { // find other endpoints in group
            if (other != e && e->get_group_name() == e->get_group_name()) {
                e->link_group_member(other);
            }
        }
    }

    // Create TCP server
    if (config.tcp_port != 0u) {
        g_tcp_fd = tcp_open(config.tcp_port);
    }

    // Create Log endpoint
    auto conf = config.log_config;
    if (!conf.logs_dir.empty()) {
        switch (conf.mavlink_dialect) {
        case LogOptions::MavDialect::Ardupilotmega:
            this->_log_endpoint = std::make_shared<BinLog>(conf);
            break;

        case LogOptions::MavDialect::Common:
            this->_log_endpoint = std::make_shared<ULog>(conf);
            break;

        case LogOptions::MavDialect::Auto:
            this->_log_endpoint = std::make_shared<AutoLog>(conf);
            break;

            // no default case on purpose
        }
        this->_log_endpoint->mark_unfinished_logs();
        g_endpoints.emplace_back(this->_log_endpoint);

        if (conf.log_telemetry) {
            auto tlog_endpoint = std::make_shared<TLog>(conf);
            tlog_endpoint->mark_unfinished_logs();
            g_endpoints.emplace_back(tlog_endpoint);
        }
    }

    // Apply other options
    if (config.report_msg_statistics) {
        add_timeout(MSEC_PER_SEC, _print_statistics_timeout_cb, this);
    }

    if (config.dedup_period_ms > 0) {
        log_info("Message de-duplication enabled: %ld ms period", config.dedup_period_ms);
        _msg_dedup.set_dedup_period(config.dedup_period_ms);
    }

    if (!config.comp_priority.empty()) {
        std::vector<uint32_t> msg_ids = config.comp_priority_msg_ids;
        if (msg_ids.empty()) {
            msg_ids = {MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, MAVLINK_MSG_ID_MANUAL_CONTROL};
        }

        std::string order, ids;
        for (auto compid : config.comp_priority) {
            order += (order.empty() ? "" : " > ") + std::to_string(compid);
        }
        for (auto msg_id : msg_ids) {
            ids += (ids.empty() ? "" : ",") + std::to_string(msg_id);
        }
        log_info("Component priority enabled: %s for messages %s, %lu ms timeout",
                 order.c_str(),
                 ids.c_str(),
                 config.comp_priority_timeout_ms);
        _comp_priority.configure(config.comp_priority, msg_ids, config.comp_priority_timeout_ms);
    }

    for (const auto &e : g_endpoints) {
        if (e->is_master()) {
            _masters.push_back(e);
        }
    }
    std::stable_sort(_masters.begin(),
                     _masters.end(),
                     [](const std::shared_ptr<Endpoint> &a, const std::shared_ptr<Endpoint> &b) {
                         return a->get_master_priority() < b->get_master_priority();
                     });
    for (size_t i = 0; i < _masters.size(); i++) {
        _masters[i]->set_master_rank(i);
    }
    _hub_mode = !_masters.empty();
    if (_hub_mode) {
        log_info("MAVProxy-style routing: %zu master endpoint(s)", _masters.size());
    }

    if (config.master_failover_timeout_ms > 0) {
        if (_masters.size() < 2) {
            log_warning("MasterFailoverTimeout needs two or more Master endpoints: failover off");
        } else if (_masters.size() > 64) {
            log_error("Master failover handles up to 64 Master endpoints");
            return false;
        } else {
            std::string order;
            for (const auto &m : _masters) {
                order += (order.empty() ? "" : " > ") + m->get_name();
            }
            log_info("Master failover enabled: %s, %lu ms timeout",
                     order.c_str(),
                     config.master_failover_timeout_ms);
            _master_failover.configure(_masters.size(), config.master_failover_timeout_ms);
        }
    }

    _gcs.sysid = config.gcs_sysid;
    _gcs.compid = config.gcs_compid;
    _gcs.stream_rate = config.stream_rate;

    _message_disable.configure(config.disable_msg_ids);

    if ((config.gcs_heartbeat_rate > 0 || config.stream_rate > 0 || _message_disable.enabled())
        && !_hub_mode) {
        log_error("GcsHeartbeatRate, StreamRate and DisableMessages are sent to Master "
                  "endpoints, and none is configured");
        return false;
    }

    if (config.gcs_heartbeat_rate > 0) {
        log_info("Sending GCS heartbeat as %u/%u at %lu Hz to masters",
                 _gcs.sysid,
                 _gcs.compid,
                 config.gcs_heartbeat_rate);
        add_timeout(MSEC_PER_SEC / config.gcs_heartbeat_rate,
                    std::bind(&Mainloop::_gcs_heartbeat_timeout_cb, this, std::placeholders::_1),
                    this);
    }

    if (config.stream_rate > 0) {
        log_info("Requesting all streams at %lu Hz from autopilots on masters", config.stream_rate);
        add_timeout(15 * MSEC_PER_SEC,
                    std::bind(&Mainloop::_stream_rate_timeout_cb, this, std::placeholders::_1),
                    this);
    }

    if (config.rc_control_port != 0) {
        RcGate::Template failsafe;
        if (!RcGate::parse_failsafe(config.rc_failsafe, &failsafe)) {
            log_error("RcFailsafe: '%s' is not up to 18 comma-separated pulse widths, "
                      "'last' or 'ignore'",
                      config.rc_failsafe.c_str());
            return false;
        }
        if (config.rc_failsafe_timeout_ms > 0 && !_hub_mode) {
            log_error("RcFailsafeTimeout sends to Master endpoints, and none is configured");
            return false;
        }
        // The pilot is whoever sends on its endpoint: without one the gate
        // could tell nobody apart.
        if (config.rc_pilot_endpoint.empty()) {
            log_error("RcPilotEndpoint is required with RcControlPort");
            return false;
        }
        const std::string &pilot_ep = config.rc_pilot_endpoint;
        const std::string &local_ep = config.rc_local_endpoint;
        const std::string &priority_ep = config.rc_priority_endpoint;
        // One endpoint is one source: the pilot's and the local one are
        // matched before any compid, so sharing would hand one's frames to
        // the other, or the priority component's to either.
        if (pilot_ep == local_ep || pilot_ep == priority_ep
            || (!local_ep.empty() && local_ep == priority_ep)) {
            log_error("RcPilotEndpoint, RcLocalEndpoint and RcPriorityEndpoint must differ");
            return false;
        }
        _rc_gate.configure(config.rc_pass_compids, config.rc_failsafe_timeout_ms, failsafe);
        for (const auto &name : {pilot_ep, local_ep, priority_ep}) {
            if (name.empty()) {
                continue;
            }
            bool found = false;
            for (const auto &e : g_endpoints) {
                found = found || e->get_name() == name;
            }
            if (!found) {
                log_error("RcPilotEndpoint / RcLocalEndpoint / RcPriorityEndpoint: no endpoint "
                          "named '%s'",
                          name.c_str());
                return false;
            }
        }
        _rc_gate.bind_endpoints(pilot_ep, local_ep, priority_ep);
        // TODO(pilot-sysid): remove once every pilot runs v3.7.0+.
        _rc_pilot_sysid = (uint8_t)config.rc_pilot_sysid;
        if (_rc_pilot_sysid != 0) {
            log_info("RC gate: RC from '%s' passed on as sysid %u, whatever the pilot sent "
                     "(RcPilotSysid, transitional: for pilots up to 3.6.12)",
                     pilot_ep.c_str(),
                     _rc_pilot_sysid);
        }
        _rc_gate.set_pass_timeout(config.rc_pass_timeout_ms);
        if (config.rc_pass_timeout_ms > 0) {
            log_info("RC gate: always-pass frames need a stick source within %lu ms",
                     config.rc_pass_timeout_ms);
        }
        log_info("RC gate: the pilot is '%s', the local transmitter '%s', priority frames only "
                 "on '%s' (compids name, endpoints identify)",
                 pilot_ep.c_str(),
                 local_ep.empty() ? "everyone but the pilot" : local_ep.c_str(),
                 priority_ep.empty() ? "any" : priority_ep.c_str());
        if (config.rc_priority_compid <= 255) {
            _rc_gate.set_priority(config.rc_priority_compid, config.rc_priority_timeout_ms);
            log_info("RC gate: compid %lu above all, in every mode, for %lu ms after each frame",
                     config.rc_priority_compid,
                     config.rc_priority_timeout_ms);
        }

        _rc_control = std::make_unique<RcControl>(
            std::bind(&Mainloop::_rc_control_cb, this, std::placeholders::_1));
        if (!_rc_control->open(config.rc_control_port)) {
            return false;
        }
        add_fd(_rc_control->fd, _rc_control.get(), EPOLLIN);

        std::string pass;
        for (auto compid : config.rc_pass_compids) {
            pass += (pass.empty() ? "" : ",") + std::to_string(compid);
        }
        log_info("RC gate: always passing [%s], starts open", pass.c_str());
        if (config.rc_failsafe_timeout_ms > 0) {
            log_info("RC gate: failsafe frame [%s] after %lu ms of pilot silence, at %lu Hz",
                     config.rc_failsafe.c_str(),
                     config.rc_failsafe_timeout_ms,
                     config.rc_failsafe_rate);
            add_timeout(MSEC_PER_SEC / config.rc_failsafe_rate,
                        std::bind(&Mainloop::_rc_failsafe_timeout_cb, this, std::placeholders::_1),
                        this);
        }
    }

    if (_message_disable.enabled()) {
        log_info("Keeping %zu message(s) off on autopilots on masters: SET_MESSAGE_INTERVAL "
                 "-1 when one is heard, at most one a second per link",
                 _message_disable.size());
        if (config.stream_rate > 0) {
            log_warning("StreamRate turns DisableMessages' messages back on every 15 s, "
                        "and each costs a command again");
        }
    }

    return true;
}

void Mainloop::clear_endpoints()
{
    g_endpoints.clear();
}

int Mainloop::tcp_open(unsigned long tcp_port)
{
    int fd;
    struct sockaddr_in6 sockaddr = {};
    int val = 1;

    fd = socket(AF_INET6, SOCK_STREAM | SOCK_NONBLOCK, 0);
    if (fd == -1) {
        log_error("TCP Server: Could not create tcp socket (%m)");
        return -1;
    }

    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val));

    sockaddr.sin6_family = AF_INET6;
    sockaddr.sin6_port = htons(tcp_port);
    sockaddr.sin6_addr = in6addr_any;

    if (bind(fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) < 0) {
        log_error("TCP Server: Could not bind to tcp socket (%m)");
        close(fd);
        return -1;
    }

    if (listen(fd, SOMAXCONN) < 0) {
        log_error("TCP Server: Could not listen on tcp socket (%m)");
        close(fd);
        return -1;
    }

    add_fd(fd, &g_tcp_fd, EPOLLIN);

    log_info("Opened TCP Server [%d] [::]:%lu", fd, tcp_port);

    return fd;
}

Timeout *Mainloop::add_timeout(uint32_t timeout_msec, std::function<bool(void *)> cb,
                               const void *data)
{
    auto *t = new Timeout(cb, data);

    assert_or_return(t, nullptr);

    t->fd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
    if (t->fd < 0) {
        log_error("Unable to create timerfd: %m");
        goto error;
    }

    mod_timeout(t, timeout_msec);

    if (add_fd(t->fd, t, EPOLLIN) < 0) {
        goto error;
    }

    t->next = _timeouts;
    _timeouts = t;

    return t;

error:
    delete t;
    return nullptr;
}

void Mainloop::del_timeout(Timeout *t)
{
    t->remove_me = true;
}

void Mainloop::mod_timeout(Timeout *t, uint32_t timeout_msec)
{
    struct itimerspec ts;

    ts.it_interval.tv_sec = timeout_msec / MSEC_PER_SEC;
    ts.it_interval.tv_nsec = (timeout_msec % MSEC_PER_SEC) * NSEC_PER_MSEC;
    ts.it_value.tv_sec = ts.it_interval.tv_sec;
    ts.it_value.tv_nsec = ts.it_interval.tv_nsec;

    timerfd_settime(t->fd, 0, &ts, nullptr);
}

void Mainloop::_del_timeouts()
{
    // Guarantee one valid Timeout on the beginning of the list
    while ((_timeouts != nullptr) && _timeouts->remove_me) {
        Timeout *next = _timeouts->next;
        remove_fd(_timeouts->fd);
        delete _timeouts;
        _timeouts = next;
    }

    // Remove all other Timeouts
    if (_timeouts != nullptr) {
        Timeout *prev = _timeouts;
        Timeout *current = _timeouts->next;
        while (current != nullptr) {
            if (current->remove_me) {
                prev->next = current->next;
                remove_fd(current->fd);
                delete current;
                current = prev->next;
            } else {
                prev = current;
                current = current->next;
            }
        }
    }
}
