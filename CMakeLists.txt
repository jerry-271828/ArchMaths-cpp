cmake_minimum_required(VERSION 3.16)
project(ArchMaths VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# 检测Android/Termux环境
if(ANDROID OR CMAKE_SYSTEM_NAME STREQUAL "Android" OR EXISTS "/data/data/com.termux")
    set(ON_ANDROID TRUE)
    message(STATUS "Detected Android/Termux environment")
else()
    set(ON_ANDROID FALSE)
endif()

# WebAssembly/Emscripten detection
if(EMSCRIPTEN)
    message(STATUS "Building for WebAssembly (WebGL 2.0)")
    set(CMAKE_EXECUTABLE_SUFFIX ".html")
    add_definitions(-DUSE_GLES -DWASM_BUILD)
    set(USE_GLES TRUE)
    set(GL_LIBRARIES "")
endif()

# Allow forcing OpenGL ES from command line
if(USE_GLES AND NOT EMSCRIPTEN)
    message(STATUS "OpenGL ES forced via USE_GLES flag")
    find_library(GLES2_LIBRARY NAMES GLESv2 libGLESv2)
    find_library(EGL_LIBRARY NAMES EGL libEGL)
    if(GLES2_LIBRARY AND EGL_LIBRARY)
        message(STATUS "Found OpenGL ES: ${GLES2_LIBRARY}")
        message(STATUS "Found EGL: ${EGL_LIBRARY}")
        set(GL_LIBRARIES ${GLES2_LIBRARY} ${EGL_LIBRARY})
        add_definitions(-DUSE_GLES)
    else()
        message(STATUS "OpenGL ES libraries not found, using stub")
        set(GL_LIBRARIES "")
        add_definitions(-DUSE_GLES)
    endif()
elseif(NOT EMSCRIPTEN)
find_library(GLES2_LIBRARY
    NAMES GLESv2 libGLESv2
    PATHS
        /system/lib64
        /system/lib
        /data/data/com.termux/files/usr/lib
        /usr/lib
        /usr/lib64
    NO_DEFAULT_PATH
)
find_library(GLES2_LIBRARY NAMES GLESv2 libGLESv2)

find_library(EGL_LIBRARY
    NAMES EGL libEGL
    PATHS
        /system/lib64
        /system/lib
        /data/data/com.termux/files/usr/lib
        /usr/lib
        /usr/lib64
    NO_DEFAULT_PATH
)
find_library(EGL_LIBRARY NAMES EGL libEGL)

if(GLES2_LIBRARY AND EGL_LIBRARY)
    message(STATUS "Found OpenGL ES: ${GLES2_LIBRARY}")
    message(STATUS "Found EGL: ${EGL_LIBRARY}")
    set(GL_LIBRARIES ${GLES2_LIBRARY} ${EGL_LIBRARY})
    add_definitions(-DUSE_GLES)
    set(USE_GLES TRUE)
elseif(ON_ANDROID)
    # Android上必须有OpenGL ES
    message(FATAL_ERROR "OpenGL ES libraries not found on Android/Termux. Please install: pkg install mesa")
else()
    # 桌面环境：查找标准OpenGL
    find_package(OpenGL REQUIRED)
    set(GL_LIBRARIES OpenGL::GL)
    set(USE_GLES FALSE)
endif()
endif()  # Close the outer if(USE_GLES)/elseif(NOT EMSCRIPTEN) block

# Qt查找 - 优先使用Qt6，回退到Qt5
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE NEVER)
find_package(Qt6 QUIET COMPONENTS Core Gui Widgets OpenGLWidgets)
if(Qt6_FOUND)
    message(STATUS "Using Qt6")
    set(QT_VERSION 6)
    set(QT_LIBS Qt6::Core Qt6::Gui Qt6::Widgets Qt6::OpenGLWidgets)
else()
    find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
    message(STATUS "Using Qt5")
    set(QT_VERSION 5)
    set(QT_LIBS Qt5::Core Qt5::Gui Qt5::Widgets)
endif()
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "/usr/lib/aarch64-linux-gnu:/lib/aarch64-linux-gnu")

# Remove unsupported compiler flags on ARM64
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64" AND Qt6_FOUND)
    foreach(_qt_target Qt6::Core Qt6::Gui Qt6::Widgets Qt6::OpenGLWidgets Qt6::OpenGL Qt6::Platform)
        if(TARGET ${_qt_target})
            get_target_property(_opts ${_qt_target} INTERFACE_COMPILE_OPTIONS)
            if(_opts)
                list(FILTER _opts EXCLUDE REGEX "-mno-direct-extern-access")
                set_target_properties(${_qt_target} PROPERTIES INTERFACE_COMPILE_OPTIONS "${_opts}")
            endif()
        endif()
    endforeach()
endif()

# 源文件
set(SOURCES
    src/main.cpp
    src/math/ExpressionParser.cpp
    src/math/ExpressionEvaluator.cpp
    src/math/Tokenizer.cpp
    src/geometry/Point.cpp
    src/geometry/Line.cpp
    src/geometry/Circle.cpp
    src/geometry/GeometryManager.cpp
    src/rendering/GLCanvas.cpp
    src/ui/MainWindow.cpp
    src/ui/SidePanel.cpp
    src/ui/EntryWidget.cpp
    src/ui/ParameterSlider.cpp
)

# 头文件
set(HEADERS
    include/math/ExpressionParser.h
    include/math/ExpressionEvaluator.h
    include/math/Tokenizer.h
    include/math/MathTypes.h
    include/geometry/Point.h
    include/geometry/Line.h
    include/geometry/Circle.h
    include/geometry/GeometryManager.h
    include/geometry/GeometryObject.h
    include/rendering/GLCanvas.h
    include/ui/MainWindow.h
    include/ui/SidePanel.h
    include/ui/EntryWidget.h
    include/ui/ParameterSlider.h
)

# 资源文件
set(RESOURCES
    resources/resources.qrc
)

# Emscripten linker flags for WebAssembly
if(EMSCRIPTEN)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} \
        -s USE_WEBGL2=1 \
        -s FULL_ES3=1 \
        -s ALLOW_MEMORY_GROWTH=1 \
        -s ASYNCIFY=1 \
        -s EXPORTED_FUNCTIONS=['_main'] \
        --shell-file ${CMAKE_SOURCE_DIR}/web/shell.html")
endif()

# 创建可执行文件
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS} ${RESOURCES})

# 包含目录
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 链接库
target_link_libraries(${PROJECT_NAME} PRIVATE
    ${QT_LIBS}
    ${GL_LIBRARIES}
)

# 编译优化选项
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_options(${PROJECT_NAME} PRIVATE
        -O3
        -ffast-math
    )
endif()

# 安装规则
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
)
