# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)

# Firmware version (esp_app_desc_t) is NOT set here on purpose: ESP-IDF
# falls back to `git describe --tags`, so a build from a release tag gets
# exactly that tag (e.g. "v0.1.0") — which is what the Android app compares
# with the GitHub release tag when checking for updates.

project(ago_slider)
