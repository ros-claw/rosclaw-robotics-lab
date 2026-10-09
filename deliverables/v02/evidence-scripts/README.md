# Execution audit copy

`standard-appendix-supervisor.py` is the exact source used for the single predeclared `06-standard` reset, SHA256-bound by `../plan-amendment.json`. It derives from frozen runtime `scripts/lab.py` and only selects repetition 6 and the standard condition, adjusts its one-attempt summary/plan, and locates the original frozen checkout explicitly. Production runtime, Body, evaluator, image, model, task and environment settings stay pinned.

It contains original machine paths as audit provenance. It is not a portable application or a second robot runtime. Reproduction uses the public challenge CLI and one's own configuration; do not run this audit copy against someone else's paths.
