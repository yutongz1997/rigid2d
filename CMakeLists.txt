cmake_minimum_required(VERSION 3.14)
project(rigid2d)

set(CMAKE_CXX_STANDARD 17)

# Configure the vcpkg toolchain file
if (DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")
endif()
# Configure the triplet in vcpkg
if (DEFINED ENV{VCPKG_DEFAULT_TRIPLET} AND NOT DEFINED VCPKG_TARGET_TRIPLET)
    set(VCPKG_TARGET_TRIPLET "$ENV{VCPKG_DEFAULT_TRIPLET}" CACHE STRING "")
endif()


find_package(fmt CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)

# Enable math constants when compiling with Microsoft Visual C++
if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    add_compile_definitions(_USE_MATH_DEFINES)
endif()

# List source files to be compiled
set(PROJECT_HEADER_DIR "${CMAKE_SOURCE_DIR}/include")
set(PROJECT_SOURCE_DIR "${CMAKE_SOURCE_DIR}/src")
set(SOURCE_FILES
        "src/body.cpp"
        "src/bodysystem.cpp"
        "src/bvh.cpp"
        "src/contact.cpp"
        "src/forces.cpp"
        "src/geometry.cpp"
        "src/scene.cpp"
        "src/shader.cpp"
        "src/simulator.cpp"
        "src/main.cpp"
)

add_executable(rigid2d ${SOURCE_FILES})
target_include_directories(rigid2d PRIVATE ${PROJECT_HEADER_DIR})
target_link_libraries(rigid2d PRIVATE fmt::fmt
                                      Eigen3::Eigen
                                      nlohmann_json::nlohmann_json
                                      glad::glad
                                      glfw
                                      imgui::imgui
                                      glm::glm)
