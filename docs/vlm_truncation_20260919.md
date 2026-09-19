# Demo 4 第二支管盖识别 JSON 截断

现场记录：`/tmp/agentic_skills_runs/demo_4_vial_extract_20260919_131746_667285`。
第一支已经执行拔出与落料；第二支完成观察和标定绑定，管盖识别失败，未进入第二支开爪/拔出。

`extract_cap_2_vlm.json` 的两次回复均为 `finish_reason=length`。
首轮 completion_tokens=4081、reasoning_tokens=3928；第二轮分别为 4092、3928。
请求上限 4096。首轮包含未闭合的定位 JSON；旧逻辑将这种截断当成格式不兼容，
取消 JSON Schema 重试，第二轮输出分析文字并再次截断。不能据这些残缺坐标驱动机械臂。

[OpenRouter 官方说明](https://openrouter.ai/docs/guides/best-practices/reasoning-tokens)
明确指出多数提供方将推理与可见输出计入同一个 max_tokens 额度，建议降低 reasoning.effort
或增加总额度。文档列出的 Gemini 3.5 Flash 支持 low 推理强度；实际预算分配仍由提供方决定。

修改内容：

- Demo 4 增加 `vlm_reasoning_effort: low`；配置经定位 config、CLI runtime 传至请求 reasoning.effort。
- 长度截断单独处理：用同一张图片和当前格式约束补试一次，4096 提升至 8192。
  通用逻辑为加倍、最低 4096、上限 8192；没有增长空间或已补试则停止。
- 长度截断不触发去除 Schema 的回退；非长度解析错误仍保留原回退逻辑。
- 截断结果即使局部可解析也不接受。逐次保存 finish_reason、usage、预算及原始内容。
- 简化短标签与 notes 的输出要求，最终报错提示预算与结构化输出问题。

52 项离线检查通过，覆盖截断补试上限、同图/同 Schema、配置贯通与原有网络重试。
未重跑在线识别、未执行任何真机动作；完整 JSON 不等于孔位选择已被现场验证。
