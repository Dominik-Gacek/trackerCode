idf_component_register(SRCS "new_proj.c" test.c
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES bt nvs_flash fatfs driver)

littlefs_create_partition_image(mystore mystore FLASH_IN_PROJECT)
