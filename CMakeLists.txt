cmake_minimum_required(VERSION 3.17.1)

set(CONF_FILE prj.conf)
find_package(Zephyr)

project(z_quad_rotor)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Application sources
target_sources(app PRIVATE src/main.c)