# S4.5 Adapter coverage

`scripts/audit_adapter_coverage.py` audits all 18 manifest entrypoints, including internal capabilities. Each row has an explicit `supported`, `plan_only`, or `unsupported` disposition and mode-specific support. The current coverage is 11 `plan_only`, 0 `supported`, and 7 `unsupported`; 11 core capabilities produce complete dry-run plans. Hardware `live=supported` is deliberately zero.

`plan_only` means that a fixed InvocationPlan can be constructed and reviewed. It does not mean the entrypoint has been executed, validated on hardware, or authorized for live use. Unsupported status is retained wherever a fixed binding would require inventing CLI parameters or accepting an arbitrary path.
