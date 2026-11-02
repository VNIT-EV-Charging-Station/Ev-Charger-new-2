idf_component_register(
    SRCS
        "algorithm.c"
        "main.c"
        "current_sensor_sct013.c"
        "inmp441_fft.c"
        "lora_sender.c"
        "pt1000_max31865.c"
        "sensor_telemetry.c"
        "sht30_sensor.c"
        "thermal_model.c"
    INCLUDE_DIRS
        "."
)

