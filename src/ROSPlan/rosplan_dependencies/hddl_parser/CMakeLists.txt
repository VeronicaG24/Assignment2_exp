# hierarchical task network .hddl parser based on flex and bison
project(hddl_parser)

# use C++ 14 (modern C++)
add_compile_options(-std=c++14)

cmake_minimum_required(VERSION 3.1)

find_package(BISON REQUIRED) # tested with VERSION 3.0.4
find_package(FLEX REQUIRED) # tested with VERSION 2.6.0

# convert hddl_parser.yacc into hddl_parser.tab.cpp & hddl_parser.tab.hpp (yacc into cpp code) using bison
BISON_TARGET(hddl_parser src/hddl_parser.yacc ${CMAKE_CURRENT_BINARY_DIR}/hddl_parser.tab.cpp)

# convert hddl_scanner.lex into hddl_scanner.yy.cpp (lex into cpp code) using flex
FLEX_TARGET(hddl_scanner src/hddl_scanner.lex ${CMAKE_CURRENT_BINARY_DIR}/hddl_scanner.yacc.cpp)

# bind flex and bison output
ADD_FLEX_BISON_DEPENDENCY(hddl_scanner hddl_parser)

# include automatically generated header files (by flex and bison) and also custom header files
include_directories(include ${CMAKE_CURRENT_BINARY_DIR})

add_library(hddl_parser src/hddl_parser.cpp ${FLEX_hddl_scanner_OUTPUTS} ${BISON_hddl_parser_OUTPUTS})

# make executable as an example of how to use the parser
add_executable(parser_example_usage src/parser_example_usage.cpp)
target_link_libraries(parser_example_usage hddl_parser)
