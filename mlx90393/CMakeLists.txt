add_executable(mlx90393
        mlx90393demo.cpp
        )

target_sources(mlx90393 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/../lib/arduino-MLX90393/MLX90393.cpp
)


target_include_directories(mlx90393 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/../lib/arduino-MLX90393
)

add_compile_options(-Wall
        -Wno-narrowing
                )


# pull in common dependencies
target_link_libraries(mlx90393 pico_stdlib hardware_i2c)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(mlx90393)
