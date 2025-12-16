set(TARGET_LANGUAGE c++)

add_subdirectory(c++)
# Relies on SWIG support, which was removed from coda-oss in version 2
# if (BUILD_PYTHON_MODULES)
#     add_subdirectory(python)
# endif()
