# CAELUS FDM Dependency
FetchContent_Declare(
        caelus_fdm
        GIT_REPOSITORY "https://github.com/strath-ace-labs/UAV-6dof-GNC.git"
        GIT_TAG "803676b2a7603330d128edcc3e0babe4e744736b"
        SOURCE_SUBDIR CAELUS_FDM
)
add_subdirectory(caelus_fdm)