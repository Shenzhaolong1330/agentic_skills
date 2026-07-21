# S0/S1 自动验收策略

## 基线

修改前在外层仓库固定 branch 和 base SHA，运行可用的离线测试发现器、语法检查和 `git diff --check`。原始输出保存到 `/tmp/agentic_skills_gen_agent/S00_S01/baseline/`，不提交。系统没有 `pytest` 时使用等价的 `unittest discover` 并记录环境限制。

## Final 必检项

`scripts/run_gen_agent_acceptance.py --phase s0-s1` 必须验证 branch、SHA、diff whitespace、manifest JSON、敏感 credential gate 零引用、公开 live 示例的授权参数、inventory 分类、硬件调用拦截、相关测试、根目录测试、Python/Shell 语法和 artifact 输出目录约束，并生成 JSON/Markdown 报告。

新增授权测试、静态验收测试和 wrapper 负向测试必须全部通过。相对 baseline 不得新增失败；baseline 已存在的环境 collection error 只能在报告中保留，不能伪装成通过。

不得删除失败测试、降低断言、添加无理由 skip/xfail、捕获所有异常返回成功、改变默认授权值，或用测试专用绕过开关替代生产 Gate。本阶段没有真机人工验收；自动验收通过即为 PASS。

