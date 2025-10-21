# Copyright (c) 2023 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

# m2b
get_filename_component(M2B_ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/../.. ABSOLUTE)

# trolly
if (EXISTS ${M2B_ROOT_DIR}/..)
    get_filename_component(M2B_REPO_TROLLY ${M2B_ROOT_DIR}/../trolly ABSOLUTE)
endif()
