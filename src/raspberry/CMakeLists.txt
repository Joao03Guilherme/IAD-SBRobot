cmake_minimum_required(VERSION 3.13)

# Include the Pico SDK cmake file
include(pico_sdk_import.cmake)

# Set project name and language
project(sbrobot C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize Pico SDK
pico_sdk_init()

# Add your Python files to be copied to the Pico
# Order matters - list dependencies first, then files that depend on them
set(PYTHON_FILES
    controllers/gyroscope_controller.py
    controllers/motor_controller.py
    bluethooth/BLEEmiter.py 
    bluethooth/BLEReceiver.py
    bluethooth/BLEParameters.py
    accelerometer.py
    directions_finder.py
    drive.py
    full_bluethooth_pc.py
    full_prototype.py
)

# Create target for Python files
add_custom_target(copy_python_files ALL
    DEPENDS ${PYTHON_FILES}
)

# Copy Python files to the build directory in order
foreach(PYTHON_FILE ${PYTHON_FILES})
    add_custom_command(
        TARGET copy_python_files
        COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_CURRENT_SOURCE_DIR}/${PYTHON_FILE}
            ${CMAKE_CURRENT_BINARY_DIR}/${PYTHON_FILE}
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${PYTHON_FILE}
    )
endforeach()