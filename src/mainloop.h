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
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "common/log.h"

#include "binlog.h"
#include "comm.h"
#include "comp_priority.h"
#include "priority_arbiter.h"
#include "dedup.h"
#include "endpoint.h"
#include "message_disable.h"
#include "rc_control.h"
#include "rc_gate.h"
#include "timeout.h"
#include "ulog.h"

struct Configuration {
    std::string conf_file_name;        ///< CLI "conf-file" only!
    std::string conf_dir;              ///< CLI "conf-dir" only!
    unsigned long tcp_port{5760};      ///< conf "TcpServerPort" or CLI "tcp-port"
    bool report_msg_statistics{false}; ///< conf "ReportStats" or CLI "report_msg_statistics"
    Log::Level debug_log_level{Log::Level::INFO}; ///< conf "DebugLogLevel" or CLI "debug-log-level"
    Log::Backend log_backend{Log::Backend::STDERR}; ///< CLI "syslog"
    unsigned long dedup_period_ms;                  ///< conf "DeduplicationPeriod"

    LogOptions log_config; ///< logging is in General config section, but internally an endpoint
    std::vector<UartEndpointConfig> uart_configs;
    std::vector<UdpEndpointConfig> udp_configs;
    std::vector<TcpEndpointConfig> tcp_configs;
    unsigned long sniffer_sysid;

    std::vector<uint8_t> comp_priority;           ///< conf "CompPriority", highest first
    std::vector<uint32_t> comp_priority_msg_ids;  ///< conf "CompPriorityMsgIds"
    unsigned long comp_priority_timeout_ms{500};  ///< conf "CompPriorityTimeout"
    unsigned long gcs_heartbeat_rate{0};          ///< conf "GcsHeartbeatRate", Hz
    unsigned long stream_rate{0};                 ///< conf "StreamRate", Hz
    std::vector<uint32_t> disable_msg_ids;        ///< conf "DisableMessages"
    unsigned long gcs_sysid{255};                 ///< conf "GcsSysid"
    unsigned long gcs_compid{230};                ///< conf "GcsCompid"
    unsigned long master_failover_timeout_ms{0};  ///< conf "MasterFailoverTimeout"
    unsigned long rc_control_port{0};             ///< conf "RcControlPort", 0: no RC gate
    std::vector<uint8_t> rc_pass_compids;         ///< conf "RcPassCompids"
    unsigned long rc_failsafe_timeout_ms{500};    ///< conf "RcFailsafeTimeout", 0: never
    std::string rc_failsafe{"1500,1500,900,1500"}; ///< conf "RcFailsafe"
    unsigned long rc_failsafe_rate{10};           ///< conf "RcFailsafeRate", Hz
    unsigned long rc_priority_compid{256};        ///< conf "RcPriorityCompid", 256: none
    unsigned long rc_priority_timeout_ms{1500};   ///< conf "RcPriorityTimeout"
    std::string rc_pilot_endpoint;                ///< conf "RcPilotEndpoint", required with the gate
    std::string rc_local_endpoint;                ///< conf "RcLocalEndpoint", empty: all but the pilot
    std::string rc_priority_endpoint;             ///< conf "RcPriorityEndpoint", empty: any
    unsigned long rc_pass_timeout_ms{0};          ///< conf "RcPassTimeout", 0: always pass
    // TODO(pilot-sysid): remove once every pilot runs v3.7.0+.
    unsigned long rc_pilot_sysid{0};              ///< conf "RcPilotSysid", 0: as sent
};

struct endpoint_entry {
    struct endpoint_entry *next;
    TcpEndpoint *endpoint;
};

class Mainloop {
public:
    int open();
    int add_fd(int fd, void *data, int events) const;
    int mod_fd(int fd, void *data, int events) const;
    int remove_fd(int fd) const;
    int loop();
    void route_msg(struct buffer *buf, const Endpoint *source = nullptr);
    void handle_tcp_connection();
    int write_msg(const std::shared_ptr<Endpoint> &e, const struct buffer *buf) const;
    void process_tcp_hangups();
    Timeout *add_timeout(uint32_t timeout_msec, std::function<bool(void *)> cb, const void *data);
    void del_timeout(Timeout *t);
    void mod_timeout(Timeout *t, uint32_t timeout_msec);

    bool add_endpoints(const Configuration &config);
    void clear_endpoints();

    /*
     * Returns true, if the message was already received earlier
     */
    bool dedup_check_msg(const buffer *buf);

    void print_statistics();

    /*
     * Ask a newly seen autopilot on a master endpoint for its streams now,
     * rather than on the next StreamRate period.
     */
    void request_streams(const Endpoint *master, uint16_t sys_comp_id);

    int epollfd = -1;
    bool should_process_tcp_hangups = false;

    /*
     * Return singleton for this class, tied to the main thread. It needds to
     * be called after a call to Mainloop::init().
     */
    static Mainloop &get_instance()
    {
        assert(_initialized);
        return _instance;
    }

    /*
     * Initialize and return singleton.
     */
    static Mainloop &init();

    /*
     * De-initialize singleton so we can start a fresh on the same
     * thread
     */
    static void teardown();

    static Mainloop &instance();

    /*
     * Request that loop exits on next iteration.
     */
    void request_exit(int retcode);

private:
    static const unsigned int LOG_AGGREGATE_INTERVAL_SEC = 5;

    std::vector<std::shared_ptr<Endpoint>> g_endpoints{};
    int g_tcp_fd = -1; ///< for TCP server
    std::shared_ptr<LogEndpoint> _log_endpoint{nullptr};

    Timeout *_timeouts = nullptr;

    Dedup _msg_dedup{0}; // disabled by default
    CompPriority _comp_priority{}; // disabled by default

    bool _hub_mode = false; // any endpoint is a Master
    std::vector<std::shared_ptr<Endpoint>> _masters{}; // best MasterPriority first
    PriorityArbiter _master_failover{};                 // disabled: 0 ranks
    MessageDisable _message_disable{};                  // disabled by default
    RcGate _rc_gate{};                                  // disabled by default
    std::unique_ptr<RcControl> _rc_control{};
    bool _rc_failsafe_sent = false; // the failsafe frame holds the sticks
    // TODO(pilot-sysid): remove once every pilot runs v3.7.0+.
    uint8_t _rc_pilot_sysid = 0;     // RcPilotSysid, 0: off
    int _rc_pilot_sysid_seen = -1;   // the pilot's own sysid last logged
    bool _rc_pilot_sysid_warned = false;
    struct {
        uint8_t sysid = 255;
        uint8_t compid = 230;
        unsigned long stream_rate = 0;
    } _gcs;

    struct {
        uint32_t msg_to_unknown = 0;
    } _errors_aggregate;

    int _retcode;

    int tcp_open(unsigned long tcp_port);
    void _del_timeouts();
    bool _retry_timeout_cb(void *data);
    bool _log_aggregate_timeout(void *data);
    Endpoint::AcceptState _hub_accept(const Endpoint &e, const struct buffer *buf,
                                      const Endpoint *source, uint64_t uplink_masters) const;
    bool _failover_passes(const struct buffer *buf, const Endpoint &source);
    uint64_t _uplink_masters(const struct buffer *buf) const;
    void _write_to(const std::shared_ptr<Endpoint> &e, const mavlink_message_t *msg);
    void _request_streams(const std::shared_ptr<Endpoint> &e, uint16_t sys_comp_id);
    bool _gcs_heartbeat_timeout_cb(void *data);
    bool _stream_rate_timeout_cb(void *data);
    void _disable_heard(const struct buffer *buf, const Endpoint &source);
    void _rc_control_cb(const std::string &text);
    void _log_rc_pass_change();
    void _patch_pilot_sysid(struct buffer *buf); // TODO(pilot-sysid): remove once every pilot runs v3.7.0+.
    bool _rc_failsafe_timeout_cb(void *data);
    void _send_rc_override(const uint16_t channels[RcGate::CHANNELS]);

    Mainloop() = default;
    Mainloop(const Mainloop &) = delete;
    Mainloop &operator=(const Mainloop &) = delete;

    static Mainloop _instance;
    static bool _initialized;
};
