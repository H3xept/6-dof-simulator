# CAELUS FDM Dependency
FetchContent_Declare(
        caelus_fdm
        GIT_REPOSITORY "https://github.com/strath-ace-labs/UAV-6dof-GNC.git"
        GIT_TAG "0a62d2928d95576eedf03468a77ed91674d934e4"
        SOURCE_SUBDIR CAELUS_FDM
)
add_subdirectory(caelus_fdm)