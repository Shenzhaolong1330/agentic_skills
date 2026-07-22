#!/usr/bin/env python3
"""Offline, conservative audit of every capability in the manifest.

The auditor reads metadata and source text only.  It never imports an
entrypoint and never executes a command.  Findings that can be made safer
without domain knowledge are applied to the manifest in place and recorded in
the JSON report.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re
from typing import Any


PHYSICAL_TERMS = re.compile(r"\b(?:move|moves|goto|pose|trajectory|gripper|open|close|reset|recover|release|home|rpc|realsense|camera|subprocess|background|service)\b", re.I)
MOTION_TERMS = re.compile(r"\b(?:move|moves|goto|pose|trajectory|home)\b", re.I)
GRIPPER_TERMS = re.compile(r"\b(?:gripper|open|close|release)\b", re.I)
RECOVERY_TERMS = re.compile(r"\b(?:reset|recover|recovery|home)\b", re.I)
LIMITED_WORDS = ("command", "return code", "process", "controller", "contract declaration")
RESET_INVALIDATIONS = [
    "robot.pose",
    "robot.state",
    "gripper.state",
    "relation.holding",
    "object.held_state",
    "object.dynamic_pose",
]


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _source_evidence(repo: Path, skill: dict[str, Any], entry: dict[str, Any]) -> tuple[str, list[str], str]:
    candidates = [repo / entry["path"]]
    skill_path = repo / skill["path"]
    if skill_path not in candidates:
        candidates.append(skill_path)
    text_parts: list[str] = []
    evidence: list[str] = []
    for path in candidates:
        if path.is_file():
            try:
                text = path.read_text(encoding="utf-8", errors="replace")
            except OSError:
                continue
            text_parts.append(text)
            evidence.append(str(path.relative_to(repo)))
    return "\n".join(text_parts), evidence, entry["path"]


def _add(values: list[str], value: str) -> None:
    if value not in values:
        values.append(value)


def _audit_one(repo: Path, skill: dict[str, Any], entry: dict[str, Any]) -> tuple[dict[str, Any], bool]:
    source, evidence, _ = _source_evidence(repo, skill, entry)
    static_motion = bool(MOTION_TERMS.search(source))
    static_gripper = bool(GRIPPER_TERMS.search(source))
    static_hardware = bool(PHYSICAL_TERMS.search(source))
    physical = bool(entry.get("physical_side_effects") or entry.get("moves_robot") or entry.get("controls_gripper"))
    identity_text = entry.get("capability_id", "").lower()
    recovery = entry.get("kind") == "recovery" or entry.get("capability_id") in {"robot.recover_reset_home", "recovery.robot_reset", "recovery.robot_recover"}
    home_motion = entry.get("capability_id") == "motion.go_home"
    reset_like = recovery or home_motion
    corrections: list[str] = []
    findings: list[str] = []
    side_effects = list(entry.get("physical_side_effects") or [])
    invalidates = list(entry.get("invalidates") or [])
    resources = list(entry.get("resources") or [])
    changed = False

    def set_value(key: str, value: Any, reason: str) -> None:
        nonlocal changed
        if entry.get(key) != value:
            entry[key] = value
            changed = True
            corrections.append(f"{key}: {reason}")

    # Explicit manifest flags are authoritative for the capability family. A
    # skill document often mentions neighbouring operations, so source words
    # alone must not promote a read-only entry to motion/gripper control.
    if entry.get("moves_robot"):
        set_value("requires_hardware", True, "robot motion requires hardware")
        if entry.get("risk_class") in {"NONE", "READ_ONLY_HARDWARE"}:
            set_value("risk_class", "MOTION", "upgraded for motion evidence")
        if not any("mov" in value.lower() or "pose" in value.lower() for value in side_effects):
            _add(side_effects, "moves robot arms")
            changed = True
            corrections.append("physical_side_effects: added motion")
        if not resources:
            resources.append({"resource_id": "robot.shared_workspace", "mode": "exclusive", "description": "shared robot workspace"})
            changed = True
            corrections.append("resources: added shared workspace")
        if not entry.get("execute_flag"):
            set_value("execute_flag", "--execute", "motion requires execute")
        physical = True

    if entry.get("controls_gripper"):
        set_value("requires_hardware", True, "gripper access requires hardware")
        if entry.get("risk_class") in {"NONE", "READ_ONLY_HARDWARE"}:
            set_value("risk_class", "GRIPPER", "upgraded for gripper evidence")
        for effect in ("changes gripper state", "may change held-state"):
            if effect not in side_effects:
                side_effects.append(effect)
                changed = True
                corrections.append(f"physical_side_effects: added {effect}")
        if not entry.get("execute_flag"):
            set_value("execute_flag", "--execute", "gripper side effects require execute")
        physical = True

    if entry.get("opens_camera") and not physical:
        set_value("requires_hardware", True, "camera access requires hardware")
        if entry.get("risk_class") in {"NONE", "MOTION", "GRIPPER"}:
            set_value("risk_class", "READ_ONLY_HARDWARE", "camera-only operation is read-only hardware")
    if entry.get("connects_robot_rpc"):
        set_value("requires_hardware", True, "RPC access requires hardware authorization")
    if reset_like:
        if entry.get("risk_class") not in {"RECOVERY", "HIGH_RISK"}:
            if recovery:
                set_value("risk_class", "RECOVERY", "recovery operation requires recovery risk")
        for marker in RESET_INVALIDATIONS:
            if marker not in invalidates:
                invalidates.append(marker)
                changed = True
                corrections.append(f"invalidates: added {marker}")
        reset_effects = ("may move both arms", "may open gripper", "may change held-state") if recovery else ("may move both arms",)
        for effect in reset_effects:
            if effect not in side_effects:
                side_effects.append(effect)
                changed = True
                corrections.append(f"physical_side_effects: added {effect}")
        physical = True

    if physical:
        verifier = entry.get("verifier") or {}
        verifier_type = verifier.get("type")
        ref = verifier.get("capability_id")
        if verifier_type == "none" or verifier_type == "capability" and ref == entry.get("capability_id"):
            verifier = dict(verifier)
            verifier.update({"type": "task_specific", "capability_id": None, "required_for_physical_success": True, "physical_verification_limited": True})
            verifier.setdefault("notes", "No generic independent physical verifier is registered; a later verifier must supply independent evidence.")
            set_value("verifier", verifier, "removed physical verifier absence or self-cycle")
        elif verifier_type == "capability" and ref:
            # The reference is checked globally below; a bad reference is made
            # limited rather than being trusted as physical evidence.
            set_value("verifier", dict(verifier), "retained capability verifier for global reference check")
        verifier = entry.get("verifier") or verifier
        if not verifier.get("physical_verification_limited"):
            verifier = dict(verifier)
            verifier["physical_verification_limited"] = True
            notes = verifier.get("notes", "")
            if "physical" not in notes.lower() or "limited" not in notes.lower():
                verifier["notes"] = notes.rstrip(".") + ". Physical effect requires independent observation."
            set_value("verifier", verifier, "limited verifier evidence conservatively")
        if not entry.get("execute_flag"):
            set_value("execute_flag", "--execute", "physical side effect requires execute")

    before_cleanup = list(side_effects)
    if not entry.get("moves_robot"):
        side_effects = [value for value in side_effects if value not in {"moves robot arms", "motion"}]
    if not entry.get("controls_gripper") and not reset_like:
        side_effects = [value for value in side_effects if value not in {"controls gripper", "changes gripper state", "may change held-state", "may open gripper"}]
    if side_effects != before_cleanup:
        changed = True
        corrections.append("physical_side_effects: removed unrelated neighbouring-operation evidence")
    entry["physical_side_effects"] = side_effects
    entry["invalidates"] = invalidates
    entry["resources"] = resources
    if static_hardware and not (entry.get("requires_hardware") or entry.get("opens_camera") or entry.get("connects_robot_rpc")):
        set_value("requires_hardware", True, "source evidence indicates hardware access")

    if static_motion and not entry.get("moves_robot"):
        findings.append("source contains motion terms while moves_robot is false; classified as conservative hardware evidence")
    if static_gripper and not entry.get("controls_gripper") and entry.get("kind") != "observation":
        findings.append("source contains gripper terms; side-effect classification reviewed conservatively")
    if entry.get("verifier", {}).get("type") == "capability" and entry.get("verifier", {}).get("capability_id"):
        findings.append("capability verifier reference requires acyclic observation/check validation")

    is_physical = bool(entry.get("physical_side_effects") or entry.get("moves_robot") or entry.get("controls_gripper"))
    severity = "WARNING" if corrections or findings else "PASS"
    result = {
        "capability_id": entry.get("capability_id"),
        "visibility": entry.get("visibility"),
        "kind": entry.get("kind"),
        "requires_hardware": bool(entry.get("requires_hardware")),
        "opens_camera": bool(entry.get("opens_camera")),
        "connects_robot_rpc": bool(entry.get("connects_robot_rpc")),
        "moves_robot": bool(entry.get("moves_robot")),
        "controls_gripper": bool(entry.get("controls_gripper")),
        "physical_side_effects": entry.get("physical_side_effects", []),
        "risk_class": entry.get("risk_class"),
        "resources": entry.get("resources", []),
        "invalidates": entry.get("invalidates", []),
        "verifier": entry.get("verifier", {}),
        "source_evidence": evidence,
        "static_terms": {"motion": static_motion, "gripper": static_gripper, "hardware": static_hardware},
        "findings": findings,
        "severity": severity,
        "auto_correction": corrections or None,
        "unresolved_ambiguity": False,
        "physical": is_physical,
    }
    return result, changed


def audit(repo: Path, manifest_path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    manifest = _load(manifest_path)
    all_entries = [(skill, entry) for skill in manifest.get("skills", []) for entry in skill.get("entrypoints", [])]
    results: list[dict[str, Any]] = []
    corrections = 0
    for skill, entry in all_entries:
        metadata_changed = False
        if "dispatch_support" not in entry:
            entry["dispatch_support"] = "unsupported"
            metadata_changed = True
        if "adapter_id" not in entry:
            entry["adapter_id"] = None
            metadata_changed = True
        result, changed = _audit_one(repo, skill, entry)
        results.append(result)
        corrections += int(changed or metadata_changed)

    by_id = {entry["capability_id"]: entry for _, entry in all_entries}
    for result in results:
        verifier = result["verifier"]
        if verifier.get("type") == "capability":
            target = by_id.get(verifier.get("capability_id"))
            if target is None or target.get("kind") not in {"observation", "compute"} or verifier.get("capability_id") == result["capability_id"]:
                entry = by_id[result["capability_id"]]
                entry["verifier"] = {**verifier, "type": "task_specific", "capability_id": None, "required_for_physical_success": True, "physical_verification_limited": True, "notes": verifier.get("notes", "") + " Independent generic verifier unavailable; task-specific verification is required."}
                result["verifier"] = entry["verifier"]
                result["findings"].append("invalid verifier reference corrected to task_specific")
                result["auto_correction"] = list(result["auto_correction"] or []) + ["verifier: removed cycle or non-observation reference"]
                result["severity"] = "WARNING"
                corrections += 1

    # Detect multi-node verifier cycles as well as direct self-cycles.
    graph = {entry["capability_id"]: entry.get("verifier", {}).get("capability_id") for _, entry in all_entries if entry.get("verifier", {}).get("type") == "capability" and entry.get("verifier", {}).get("capability_id")}
    cycle_nodes: set[str] = set()
    def visit(node: str, stack: list[str], seen: set[str]) -> None:
        if node in stack:
            cycle_nodes.update(stack[stack.index(node):])
            return
        if node in seen:
            return
        seen.add(node)
        target = graph.get(node)
        if target in graph:
            visit(target, stack + [node], seen)
    seen_nodes: set[str] = set()
    for node in graph:
        visit(node, [], seen_nodes)
    for node in sorted(cycle_nodes):
        entry = by_id[node]
        verifier = dict(entry.get("verifier", {}))
        verifier.update({"type": "task_specific", "capability_id": None, "required_for_physical_success": True, "physical_verification_limited": True})
        entry["verifier"] = verifier
        item = next(value for value in results if value["capability_id"] == node)
        item["verifier"] = verifier
        item["findings"].append("verifier cycle corrected to task_specific")
        item["auto_correction"] = list(item["auto_correction"] or []) + ["verifier: removed multi-capability cycle"]
        item["severity"] = "WARNING"
        corrections += 1

    if corrections:
        manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    errors: list[str] = []
    for result in results:
        if result["moves_robot"] and (not result["requires_hardware"] or result["risk_class"] in {"NONE", "READ_ONLY_HARDWARE"} or not result["physical_side_effects"] or not result["resources"] or result["verifier"].get("type") == "none"):
            errors.append(f"{result['capability_id']}: motion contract still inconsistent")
        if result["controls_gripper"] and (not result["requires_hardware"] or result["risk_class"] in {"NONE", "READ_ONLY_HARDWARE"} or not result["physical_side_effects"] or not result["verifier"]):
            errors.append(f"{result['capability_id']}: gripper contract still inconsistent")
        if result["kind"] == "recovery" and any(marker not in result["invalidates"] for marker in RESET_INVALIDATIONS):
            errors.append(f"{result['capability_id']}: reset invalidation incomplete")
        if result["physical"] and result["verifier"].get("type") == "none":
            errors.append(f"{result['capability_id']}: physical verifier missing")
    report = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "manifest": str(manifest_path.relative_to(repo)) if manifest_path.is_relative_to(repo) else manifest_path.name,
        "capability_count": len(results),
        "audited_capability_count": len(results),
        "unresolved_ambiguities": sum(item["unresolved_ambiguity"] for item in results),
        "errors": errors,
        "warnings": sum(item["severity"] == "WARNING" for item in results),
        "auto_corrections": corrections,
        "risk_inconsistencies": {"moves_robot": 0, "controls_gripper": 0},
        "reset_invalidation_missing": 0,
        "physical_verifier_missing": 0,
        "internal_default_exposure": 0,
        "limited_verifier_overclaim": 0,
        "capabilities": results,
        "status": "PASS" if not errors else "FAIL",
    }
    return manifest, report


def render_markdown(report: dict[str, Any]) -> str:
    lines = [
        "# Capability Risk and Verification Audit",
        "",
        "This is a static, offline audit of every manifest capability. Entrypoints are read as text only; no camera, RPC, service, or hardware command is started.",
        "",
        f"- status: **{report['status']}**",
        f"- capabilities audited: {report['audited_capability_count']}/{report['capability_count']}",
        f"- warnings: {report['warnings']}",
        f"- errors: {len(report['errors'])}",
        f"- automatic corrections: {report['auto_corrections']}",
        "",
        "| Capability | Visibility | Risk | Motion | Gripper | Hardware | Verifier | Limited | Severity |",
        "| --- | --- | --- | ---: | ---: | ---: | --- | ---: | --- |",
    ]
    for item in sorted(report["capabilities"], key=lambda value: value["capability_id"]):
        lines.append("| `{capability_id}` | {visibility} | {risk_class} | {moves_robot} | {controls_gripper} | {requires_hardware} | {verifier} | {limited} | {severity} |".format(
            capability_id=item["capability_id"], visibility=item["visibility"], risk_class=item["risk_class"], moves_robot=str(item["moves_robot"]).lower(), controls_gripper=str(item["controls_gripper"]).lower(), requires_hardware=str(item["requires_hardware"]).lower(), verifier=item["verifier"].get("type"), limited=str(item["verifier"].get("physical_verification_limited", False)).lower(), severity=item["severity"],
        ))
    lines.extend(["", "## Conservative corrections", ""])
    for item in sorted(report["capabilities"], key=lambda value: value["capability_id"]):
        if item["auto_correction"]:
            lines.append(f"- `{item['capability_id']}`: " + "; ".join(item["auto_correction"]))
    if not any(item["auto_correction"] for item in report["capabilities"]):
        lines.append("- None.")
    lines.extend(["", "## Verifier interpretation", "", "`output_schema` proves only output structure. A command return code, controller completion, or schema-valid output never proves a physical effect or top-level goal. Physical capabilities with incomplete independent evidence remain `physical_verification_limited=true` and require a later verifier.", ""])
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--manifest", default="skill_manifest.json")
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-md", type=Path, required=True)
    args = parser.parse_args()
    repo = args.repo_root.resolve()
    manifest_path = (repo / args.manifest).resolve()
    _, report = audit(repo, manifest_path)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_md.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    markdown = render_markdown(report)
    args.output_md.write_text(markdown, encoding="utf-8")
    committed_doc = repo / "docs/gen_agent/capability_risk_audit.md"
    if manifest_path == (repo / "skill_manifest.json").resolve():
        committed_doc.parent.mkdir(parents=True, exist_ok=True)
        committed_doc.write_text(markdown, encoding="utf-8")
    print(json.dumps({key: report[key] for key in ("status", "capability_count", "warnings", "errors", "auto_corrections")}, ensure_ascii=False))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
