# CAELUS FDM Dependency
FetchContent_Declare(
        caelus_fdm
        GIT_REPOSITORY "https://github.com/strath-ace-labs/UAV-6dof-GNC.git"
        GIT_TAG "a406ceb622a116354b48cdcec9628a53bab0b5c8"
        SOURCE_SUBDIR CAELUS_FDM
)
add_subdirectory(caelus_fdm)