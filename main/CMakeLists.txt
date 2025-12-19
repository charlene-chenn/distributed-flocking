idf_component_register(
  SRCS "main.cpp" "kinematics.c" "neighbor_table.c" "radio.c" "sntp_time.c" "wifi_connect.c"
  INCLUDE_DIRS "."
  REQUIRES esp_driver_gpio esp_timer mbedtls jgromes__radiolib nvs_flash esp_netif esp_event esp_wifi mqtt lwip wpa_supplicant
)


