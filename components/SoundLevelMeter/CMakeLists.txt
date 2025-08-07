idf_component_register(
    SRCS "SoundLevelMeter.cpp"
    INCLUDE_DIRS "include"
    REQUIRES AWeighting CWeighting TimeWeighting FilterCoeffs
)

