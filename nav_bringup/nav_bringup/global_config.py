FRAMES = {
    "base_frame": "base_link",
    "imu_frame": "imu_link",
    "gps_frame": "gps_link",
    "odom_frame": "odom",
    "map_frame": "map",
}

# GPS origin in WGS84 degrees used for simulation. All map-frame coordinates are relative to this point.
# Used by the localization simulator and other packages (e.g. course creation tools) that simulate a field.
GPS_ORIGIN_SIM = {
    "latitude": 42.294621,
    "longitude": -83.708112,
}
