# CAELUS FDM Dependency
FetchContent_Declare(
        caelus_fdm
        GIT_REPOSITORY "https://github.com/strath-ace-labs/UAV-6dof-GNC.git"
        GIT_TAG "39646780bc20f18e6d5cee626594b36cdf7b1c95"
        SOURCE_SUBDIR CAELUS_FDM
)
add_subdirectory(caelus_fdm)