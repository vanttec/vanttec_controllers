# VantTec Controllers

VantTec controllers library intended to be used in all vehicles.

### Dynamic models

The library includes dynamic models for ground vehicles and marine vehicles.

    Ground vehicles
        Car-like robots
            Base 3DOF model
            VTEC SDC1

    Marine vehicles
        Underwater vehicles
            Base 6DOF model
            VTEC U4 6DOF model
        Surface vehicles
            Base 3DOF model (pending)
            VTEC S3 3DOF model (pending)

### Controllers

The library includes guidance and control algorithms for ground and marine vehicles.

    Guidance laws
        LOS
        Stanley

    Classic Control Laws
        PID
        Sliding Mode Control (SMC)
            Adaptive SMC - ASMC
            Adaptive Integral Terminal SMC - AITSMC
            Adaptive Non-Singular Terminal SMC - ANTSMC (pending)
    
    Feedback Linearization
        PID
        ASMC
        AITSMC

        Model-based Feedback Linearization
            Underwater vehicles
                VTEC U4 6DOF PID
            Self-driving cars
                VTEC SDC1 PID
                VTEC SDC1 ASMC
                VTEC SDC1 AITSMC

### Unit tests

This package includes unit tests for some controllers.
- VTEC SDC1 PID (pending to implement)