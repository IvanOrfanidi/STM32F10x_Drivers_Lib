## STM32F10x Drivers Lib
Библиотека драйверов периферии микроконтроллеров STM32F1xx.

### Сборка с использование CMake
Для подключения сборки в проект с использованием CMake указать функции в `CMakeLists.txt`.

* Указать вложенную директорию
```
add_subdirectory(stm32f10x_drivers_lib)
```

* Добавить цель
```
target_link_libraries(${PROJECT_NAME} PRIVATE
 STM32F10x_Drivers_Lib
)
```

* Для доступа к инклюдам по `<>` указать цель
```
target_include_directories(stm32f10x_drivers_lib INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}
)
```
