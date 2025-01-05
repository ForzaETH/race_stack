
set(TEST_DIR "${CMAKE_CURRENT_LIST_DIR}")

#
# MIP
#

macro(add_mip_test name sources command)

    add_executable(${name} ${sources})

    target_link_libraries(${name} mip)

    add_test(${name} ${command} ${ARGN})

endmacro()

add_mip_test(TestMipPacketBuilding "${TEST_DIR}/mip/test_mip_packet_builder.c" TestMipPacketBuilding)
add_mip_test(TestMipParsing        "${TEST_DIR}/mip/test_mip_parser.c" TestMipParsing "${TEST_DIR}/data/mip_data.bin")
add_mip_test(TestMipRandom         "${TEST_DIR}/mip/test_mip_random.c" TestMipRandom)
add_mip_test(TestMipFields         "${TEST_DIR}/mip/test_mip_fields.c" TestMipFields)
add_mip_test(TestMipCpp            "${TEST_DIR}/mip/test_mip.cpp" TestMipCpp)
add_mip_test(TestMipPerf           "${TEST_DIR}/mip/mip_parser_performance.cpp" TestMipPerf)

if(WITH_SERIAL)
    add_executable(TestSerial "${TEST_DIR}/test_serial.cpp")
    target_include_directories(TestSerial PUBLIC "${SERIAL_INCLUDE_DIRS}")
    target_link_libraries(TestSerial PUBLIC "serial")
    add_test(TestSerial TestSerial)
endif()
