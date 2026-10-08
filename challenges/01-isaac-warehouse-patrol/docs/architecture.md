# Execution and evidence ownership

```mermaid
flowchart TD
    U[One natural-language task] --> A[Native Agent · existing configured model]
    R[Read-only ROS Expert probe] --> A
    A --> P[Agentd · one site proposal]
    P --> O[operatord · Body-bound SIM card]
    O --> D[rosclawd ActionChannel]
    D --> N[Daemon single-site executor]
    N --> V[Official Nav2 NavigateToPose]
    V --> I[Isaac official warehouse and Nova Carter]
    I --> X[Independent PhysX pose and contact observer]
    I --> L[Actual LiDAR witness]
    X --> E[Pure verifier · hash-bound canonical receipt]
    L --> E
    V --> E
    E --> A
    A --> M[patrol.verify_and_remember]
    M --> C[Recheck every receipt and independent visit]
    C --> PM[Existing RosPracticeAdapter · Memory · PracticeRecorder]
    PM --> T[TaskKernel closes only after final receipt]
```

The Agent cannot execute motion through its MCP process. Its action declarations
are materialized as guarded proposals. Only the daemon registers the Nav2 executor.
That executor resolves one measured site ID and never knows or executes a patrol
sequence. Expected order is held for final acceptance, not sent as a navigation
program. SIM receipts explicitly remain unusable as real-hardware verification.

Per-run source manifests bind the public source snapshots, configuration, compiled
Body, pinned upstream commit and Docker image ID. Successful navigation receipts
bind the raw trajectory artifact SHA-256. Independent evaluation repeats the
arrival/contact/LiDAR calculations and rejects early TaskKernel closure. Public
summaries expose tool calls and final answers, never raw private model reasoning.
