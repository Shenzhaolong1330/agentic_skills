# 仓库拓扑基线

## 外层仓库

- root: `/Users/key_z/Documents/github/agentic_skills`
- branch: `develop/gen_agent`
- base SHA: `c454585fe6a4cfd5ad18d3743fde5a514a380f03`
- 当前基线工作区：干净
- 外层 Git metadata：`/Users/key_z/Documents/github/agentic_skills/.git`

## 内层仓库和 gitlink

| 路径 | 形态 | 基线状态 | 所属与提交规则 |
| --- | --- | --- | --- |
| `atomic_skills/object_locator` | 外层 index 中的 gitlink | SHA `55410ac2f0e9cb3e2559051e61a4b9a47d9721f2`；当前工作区未 checkout | 由独立 skill project 管理；外层只记录指针 |

`git submodule status --recursive` 在基线采集时失败，因为仓库没有 `.gitmodules`，但 index 中存在上述 gitlink。这一事实被保留为已知拓扑限制；本阶段不删除 gitlink、不创建虚假 `.gitmodules`、不扁平化内层项目，也不修改外层历史。

除上述 gitlink 外，基线扫描未发现其他内层 `.git` 目录、gitfile 或 gitlink。外层仓库管理 Harness、manifest、schemas、task/procedure 文档和本阶段新增验收文件；任何独立 skill project 的源码、标定和模型仍应在其自身仓库独立提交。

## artifact 约束

运行 trace、图像、深度数组、模型权重、设备日志、私密标定和临时服务 socket 必须写入 `/tmp/agentic_skills_runs/` 或用户显式指定的外部 artifact 目录，不得写入 Git 跟踪目录。S0/S1 原始 baseline 和 final 验收日志位于 `/tmp/agentic_skills_gen_agent/S00_S01/`，不提交。
