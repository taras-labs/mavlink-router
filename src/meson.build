subdir('common')

exe_mavlink_router = executable('mavlink-routerd',
        [
         'autolog.cpp',
         'binlog.cpp',
         'comp_priority.cpp',
         'dedup.cpp',
         'endpoint.cpp',
         'git_version.cpp',
         'logendpoint.cpp',
         'main.cpp',
         'mainloop.cpp',
         'message_disable.cpp',
         'msg_rate.cpp',
         'pollable.cpp',
         'priority_arbiter.cpp',
         'rc_control.cpp',
         'rc_gate.cpp',
         'timeout.cpp',
         'ulog.cpp',
         'tlog.cpp',
         version_h,
        ],
        include_directories: [src_inc, mavlink_inc],
        dependencies: [
                dep_math,
                dep_rt,
                dep_thread,
        ],
        link_with: libcommon_private,
        install: true,
)

if dep_gtest.found()
  exe_mainloop_test = executable('mainloop_test',
          [
            'autolog.cpp',
            'binlog.cpp',
            'comp_priority.cpp',
            'dedup.cpp',
            'endpoint.cpp',
            'logendpoint.cpp',
            'mainloop.cpp',
            'message_disable.cpp',
            'msg_rate.cpp',
            'mainloop_test.cpp',
            'pollable.cpp',
            'priority_arbiter.cpp',
            'rc_control.cpp',
            'rc_gate.cpp',
            'timeout.cpp',
            'ulog.cpp',
            'tlog.cpp',
            version_h,
          ],
          include_directories: [src_inc, mavlink_inc],
          dependencies : [
            dep_gtest,
            dep_rt,
          ],
          link_with: libcommon_private,
          install: false,
  )
  test('mainloop', exe_mainloop_test)

  exe_endpoints_test = executable('endpoints_test',
          [
            'autolog.cpp',
            'binlog.cpp',
            'comp_priority.cpp',
            'dedup.cpp',
            'endpoint.cpp',
            'endpoints_test.cpp',
            'logendpoint.cpp',
            'mainloop.cpp',
            'message_disable.cpp',
            'msg_rate.cpp',
            'pollable.cpp',
            'priority_arbiter.cpp',
            'rc_control.cpp',
            'rc_gate.cpp',
            'timeout.cpp',
            'ulog.cpp',
            'tlog.cpp',
            version_h,
          ],
          include_directories: [src_inc, mavlink_inc],
          dependencies : [
            dep_gtest,
            dep_rt,
          ],
          link_with: libcommon_private,
          install: false,
  )
  test('endpoints', exe_endpoints_test)

  exe_comp_priority_test = executable('comp_priority_test',
          [
            'comp_priority.cpp',
            'comp_priority_test.cpp',
            'priority_arbiter.cpp',
          ],
          include_directories: [src_inc, mavlink_inc],
          dependencies : [
            dep_gtest,
            dep_rt,
          ],
          link_with: libcommon_private,
          install: false,
  )
  test('comp_priority', exe_comp_priority_test)

  exe_message_disable_test = executable('message_disable_test',
          [
            'message_disable.cpp',
            'message_disable_test.cpp',
          ],
          include_directories: [src_inc, mavlink_inc],
          dependencies : [
            dep_gtest,
            dep_rt,
          ],
          link_with: libcommon_private,
          install: false,
  )
  test('message_disable', exe_message_disable_test)

  exe_msg_rate_test = executable('msg_rate_test',
          [
            'msg_rate.cpp',
            'msg_rate_test.cpp',
          ],
          include_directories: [src_inc, mavlink_inc],
          dependencies : [
            dep_gtest,
            dep_rt,
          ],
          link_with: libcommon_private,
          install: false,
  )
  test('msg_rate', exe_msg_rate_test)

  exe_rc_gate_test = executable('rc_gate_test',
          [
            'rc_gate.cpp',
            'rc_gate_test.cpp',
          ],
          include_directories: [src_inc, mavlink_inc],
          dependencies : [
            dep_gtest,
            dep_rt,
          ],
          link_with: libcommon_private,
          install: false,
  )
  test('rc_gate', exe_rc_gate_test)
endif
