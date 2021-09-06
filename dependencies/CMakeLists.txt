# CAELUS FDM Dependency
FetchContent_Declare(
        caelus_fdm
        GIT_REPOSITORY "https://github.com/strath-ace-labs/UAV-6dof-GNC.git"
        GIT_TAG "206f0c046e2fa8a633f67efee6f4632811bbfe58"
        SOURCE_SUBDIR CAELUS_FDM
)
add_subdirectory(caelus_fdm)