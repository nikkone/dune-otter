dune_add_tasks(${PROJECT_SOURCE_DIR}/user/src)
#dune_add_tasks(${PROJECT_SOURCE_DIR}/private/src)

dune_add_task(${PROJECT_SOURCE_DIR}/src Transports/Announce/Task.cmake)
dune_add_task(${PROJECT_SOURCE_DIR}/src Transports/Discovery/Task.cmake)
dune_add_task(${PROJECT_SOURCE_DIR}/src Transports/Logging/Task.cmake)
dune_add_task(${PROJECT_SOURCE_DIR}/src Transports/UDP/Task.cmake)
dune_add_task(${PROJECT_SOURCE_DIR}/src Transports/HTTP/Task.cmake)
dune_add_task(${PROJECT_SOURCE_DIR}/src Transports/Cache/Task.cmake)
dune_add_task(${PROJECT_SOURCE_DIR}/src Transports/LogBook/Task.cmake)
dune_add_task(${PROJECT_SOURCE_DIR}/src Transports/FTP/Task.cmake)
dune_add_task(${PROJECT_SOURCE_DIR}/src Transports/Fragments/Task.cmake)

dune_add_task(${PROJECT_SOURCE_DIR}/src Simulators/GPS/Task.cmake)
dune_add_task(${PROJECT_SOURCE_DIR}/src Simulators/Motor/Task.cmake)
dune_add_task(${PROJECT_SOURCE_DIR}/src Simulators/VSIM/Task.cmake)

dune_add_task(${PROJECT_SOURCE_DIR}/src Sensors/GPS/Task.cmake)
dune_add_task(${PROJECT_SOURCE_DIR}/src Sensors/ThermalZone/Task.cmake)

dune_add_task(${PROJECT_SOURCE_DIR}/src UserInterfaces/LEDs/Task.cmake)

dune_add_task(${PROJECT_SOURCE_DIR}/src Supervisors/Vehicle/Task.cmake)

dune_add_task(${PROJECT_SOURCE_DIR}/src Monitors/Clock/Task.cmake)
dune_add_task(${PROJECT_SOURCE_DIR}/src Monitors/Entities/Task.cmake)

dune_add_task(${PROJECT_SOURCE_DIR}/src Maneuver/RowsCoverage/Task.cmake)
dune_add_task(${PROJECT_SOURCE_DIR}/src Maneuver/FollowReference/AUV/Task.cmake)
dune_add_task(${PROJECT_SOURCE_DIR}/src Maneuver/FollowSystem/Task.cmake)
dune_add_task(${PROJECT_SOURCE_DIR}/src Maneuver/Multiplexer/Task.cmake)
dune_add_task(${PROJECT_SOURCE_DIR}/src Maneuver/Teleoperation/Task.cmake)
dune_add_task(${PROJECT_SOURCE_DIR}/src Maneuver/CoverArea/Task.cmake)


dune_add_task(${PROJECT_SOURCE_DIR}/src Control/ASV/HeadingAndSpeed/Task.cmake)
dune_add_task(${PROJECT_SOURCE_DIR}/src Control/Path/ILOS/Task.cmake)
dune_add_task(${PROJECT_SOURCE_DIR}/src Control/ASV/RemoteOperation/Task.cmake)
dune_add_task(${PROJECT_SOURCE_DIR}/src Control/Path/VectorField/Task.cmake)
dune_add_task(${PROJECT_SOURCE_DIR}/src Control/Path/PurePursuit/Task.cmake)

dune_add_task(${PROJECT_SOURCE_DIR}/src Navigation/General/GPSNavigation/Task.cmake)

dune_add_task(${PROJECT_SOURCE_DIR}/src Plan/Engine/Task.cmake)
dune_add_task(${PROJECT_SOURCE_DIR}/src Plan/DB/Task.cmake)
dune_add_task(${PROJECT_SOURCE_DIR}/src Plan/Generator/Task.cmake)
dune_add_task(${PROJECT_SOURCE_DIR}/src Autonomy/TextActions/Task.cmake)

dune_add_task(${PROJECT_SOURCE_DIR}/src Supervisors/AUV/LostComms/Task.cmake)