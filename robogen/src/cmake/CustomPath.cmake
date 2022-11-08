# This file is generally not needed and is included only if libraries are installed in non-standard locations
# Uncomment and set appropriately the following paths

# Set the path to your utils folder
if (WIN32)
	set(UTILS_PATH "D:/robogen/utils")
	#set(QT_PATH "D:/qt5-5.5.0-vs2010/qt5-x86-shared-release")
endif()

####################################################################
# Do not edit under this line
####################################################################

if (UNIX)
if ("${CMAKE_SYSTEM}" MATCHES "Linux")
         set(UTILS_PATH "/mnt/lustre/groups/CSCI1142/RoboGen")
    # ODE
	set (ODE_INCLUDE_PATH "/mnt/lustre/groups/CSCI1142/RoboGen/include/ode")
        set (ODE_LIBRARIES "/mnt/lustre/groups/CSCI1142/RoboGen/lib/libode.a")
    # PNG
	set (PNG_PNG_INCLUDE_DIR "/mnt/lustre/groups/CSCI1142/RoboGen/include/libpng16")
        set (PNG_LIBRARY "/mnt/lustre/groups/CSCI1142/RoboGen/lib/libpng.a")

    # OSG
	set(ENV{OSG_DIR} "/mnt/lustre/groups/CSCI1142/RoboGen/bin" )
        set(ENV{OSG_ROOT} "${UTILS_PATH}")
    # Protobuf
	set (Protobuf_INCLUDE_DIR "${UTILS_PATH}/include")
        set (Protobuf_LIBRARY "${UTILS_PATH}/lib/libprotobuf.a")
        set (PROTOBUF_PROTOC_EXECUTABLE "${UTILS_PATH}/bin/protoc")
#-DProtobuf_LIBRARY=/mnt/lustre/groups/CSCI1142/RoboGen/lib/libprotobuf.so -DProtobuf_INCLUDE_DIR=/mnt/lustre/groups/CSCI1142/RoboGen/include
    # BOOST
	set (BOOST_ROOT "${UTILS_PATH}")
        set (BOOST_LIB "${UTILS_PATH}/lib")
        set (BOOST_LIB_DIR "${UTILS_PATH}/lib")
        set (Boost_USE_STATIC_LIBS ON)
        set (BOOST_IGNORE_SYSTEM_PATHS=1)
    # JANSSON
#	set (JANSSON_LIBRARIES="${UTILS_PATH}/lib/libjansson.a")
#	set (JANSSON_INCLUDE_DIR="${UTILS_PATH}/include")

#-DJANSSON_LIBRARIES=/mnt/lustre/groups/CSCI1142/RoboGen/lib/libjansson.so -DJANSSON_INCLUDE_DIR=/mnt/lustre/groups/CSCI1142/RoboGen/include/
        set (JANSSON_LIBRARIES "${UTILS_PATH}/lib/libjansson.so")
        set (JANSSON_INCLUDE_DIR "${UTILS_PATH}/include/")
#        set (PNG_PNG_INCLUDE_DIR "/apps/chpc/bio/lib/png/1.6.21/include/libpng16")
#        set (PNG_LIBRARY "/apps/chpc/bio/lib/png/1.6.21/lib/libpng.a")

endif ()
endif (UNIX)

if (APPLE)
    set(UTILS_PATH "/usr/local")

	# ZLIB
	set (ZLIB_ROOT ${UTILS_PATH})

	# PNG
	set (PNG_PNG_INCLUDE_DIR ${UTILS_PATH})
	set (PNG_LIBRARY "${UTILS_PATH}/lib/libpng.a")

	# OSG
	set(ENV{OSG_DIR} ${UTILS_PATH} )

	# Protobuf
	set (PROTOBUF_INCLUDE_DIR "${UTILS_PATH}/include")
	set (PROTOBUF_LIBRARY "${UTILS_PATH}/lib/libprotobuf.a")
	set (PROTOBUF_PROTOC_EXECUTABLE "${UTILS_PATH}/bin/protoc")

	# BOOST
	set (BOOST_ROOT ${UTILS_PATH})
	set (Boost_USE_STATIC_LIBS ON)

	# ODE
	set (ODE_INCLUDE_PATH "${UTILS_PATH}/include")
	set (ODE_LIBRARIES "${UTILS_PATH}/lib/libode.a")

endif()

if (WIN32)

	# ZLIB
	set (ZLIB_ROOT ${UTILS_PATH})

	# PNG
	set (PNG_PNG_INCLUDE_DIR ${UTILS_PATH})
	set (PNG_LIBRARY "${UTILS_PATH}/lib/libpng.lib")

	# OSG
	set(ENV{OSG_DIR} ${UTILS_PATH} )

	# Protobuf
	set (PROTOBUF_INCLUDE_DIR "${UTILS_PATH}/include")
	set (PROTOBUF_LIBRARY "${UTILS_PATH}/lib/libprotobuf.lib")
	set (PROTOBUF_LIBRARY_DEBUG "${UTILS_PATH}/lib/libprotobufd.lib")
	set (PROTOBUF_PROTOC_EXECUTABLE "${UTILS_PATH}/bin/protoc.exe")

	# BOOST
	set (BOOST_ROOT ${UTILS_PATH})
	set (Boost_USE_STATIC_LIBS ON)

	# ODE
	set (ODE_INCLUDE_PATH "${UTILS_PATH}/include")
	set (ODE_LIBRARIES "${UTILS_PATH}/lib/ode_double.lib")
	
	# JANSSON
	set (JANSSON_INCLUDE_DIRS "${UTILS_PATH}/include")
	set (JANSSON_LIBRARIES "${UTILS_PATH}/lib/jansson.lib")

endif()
