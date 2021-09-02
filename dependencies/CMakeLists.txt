# CAELUS FDM Dependency
FetchContent_Declare(
        caelus_fdm
        GIT_REPOSITORY "https://github.com/strath-ace-labs/UAV-6dof-GNC.git"
        GIT_TAG "22c0ec7327375054944ddda162c492a1e12cb122"
        SOURCE_SUBDIR CAELUS_FDM
)
add_subdirectory(caelus_fdm)