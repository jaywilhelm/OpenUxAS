srcs_pixhawk = [
  'PixhawkService.cpp'
]

incs_pixhawk = [
  include_directories(
    '../../src/Utilities',
    '../../resources/AutomationDiagramDataService',
    '../../src/Communications',
    '../../src/Includes',
    '../../src/Services',
    '../../src/VisilibityLib',
    '../../src/Plans',
    '../../src/DPSS',
    '../../src/Tasks',
    '../../3rd/zyre-2.0.0/include',
    '../../3rd/cppzmq-4.2.2',
    '../../3rd/czmq-4.0.2/include',
    '../../3rd/zeromq-4.2.3/include',
    '../../3rd/PugiXML/src',
    '../../3rd/SQLiteCpp-1.3.1/include',
    '../../3rd/sqlite-amalgamation-3120200',
    '../../3rd/serial-1.2.1/include',
    '../../3rd/TinyGPS',
    '../../3rd/zlib-1.2.8/contrib/minizip',
    '../../3rd/zlib-1.2.8',
  ),
  incs_lmcp,
]


cpp_args = []
cpp = meson.get_compiler('cpp')
cpp_args_services = cpp_args

if cpp.get_id() == 'msvc'
  cpp_args_services += [
    '/std:c++14',
	'-DZMQ_DEFINED_STDINT',
        '-DZMQ_STATIC',
	'-DLIBCZMQ_STATIC',
  ]
endif

lib_pixhawk = static_library(
  'pixhawk',
  srcs_pixhawk,
  dependencies: [
    dep_boost,
  ],
  cpp_args: cpp_args_services,
  include_directories: incs_pixhawk,
)
