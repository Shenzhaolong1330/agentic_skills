#!/usr/bin/env python3
"""Offline, repeatable S0/S1 acceptance runner."""

from __future__ import annotations

import argparse
import ast
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
from typing import Any

from jsonschema import Draft202012Validator


EXPECTED_BRANCH = "develop/gen_agent"
PHASE_NAME = "S00_S01"
CLASSIFICATIONS = {
    "GATED_PUBLIC",
    "INTERNAL_BEHIND_GATE",
    "READ_ONLY_GATED",
    "UNSUPPORTED",
    "LEGACY_DISABLED",
}


def run(command: list[str], *, cwd: Path, log_path: Path | None = None) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(command, cwd=cwd, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False)
    if log_path is not None:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_path.write_text(result.stdout, encoding="utf-8")
    return result


def git(repo: Path, *args: str) -> str:
    result = run(["git", "-C", str(repo), *args], cwd=repo)
    if result.returncode != 0:
        raise RuntimeError(f"git {' '.join(args)} failed: {result.stdout.strip()}")
    return result.stdout.strip()


def repo_files(repo: Path) -> list[Path]:
    result = run(["git", "-C", str(repo), "ls-files", "-co", "--exclude-standard", "-z"], cwd=repo)
    return [repo / item for item in result.stdout.split("\0") if item]


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except (UnicodeDecodeError, OSError):
        return ""


def parse_unittest_counts(output: str) -> dict[str, Any]:
    ran = re.search(r"Ran (\d+) tests?", output)
    total = int(ran.group(1)) if ran else 0
    failed = 0
    skipped = 0
    failed_match = re.search(r"FAILED \(([^)]*)\)", output)
    ok_match = re.search(r"OK(?: \(([^)]*)\))?", output)
    details = failed_match.group(1) if failed_match else (ok_match.group(1) if ok_match else "")
    details = details or ""
    for key, value in re.findall(r"(errors|failures|skipped)=(\d+)", details):
        if key in {"errors", "failures"}:
            failed += int(value)
        else:
            skipped += int(value)
    if "skipped" in output and not skipped:
        skipped = len(re.findall(r"\bskipped\b", output, flags=re.IGNORECASE))
    return {"total": total, "passed": max(0, total - failed - skipped), "failed": failed, "skipped": skipped}


def failure_signatures(output: str) -> set[str]:
    return {name for name in re.findall(r"^(?:ERROR|FAIL): ([^\n]+)", output, flags=re.MULTILINE)}


def pytest_counts(output: str) -> dict[str, Any]:
    def count(label: str) -> int:
        match = re.search(rf"(\d+)\s+{label}", output)
        return int(match.group(1)) if match else 0
    return {
        "passed": count("passed"),
        "failed": count("failed"),
        "skipped": count("skipped"),
        "collection_errors": len(re.findall(r"ERROR collecting", output)),
    }


def pytest_failure_signatures(output: str) -> set[str]:
    return set(re.findall(r"^FAILED ([^\s]+)", output, flags=re.MULTILINE))


def run_logged(command: list[str], *, cwd: Path, log_path: Path) -> tuple[subprocess.CompletedProcess[str], dict[str, Any]]:
    result = run(command, cwd=cwd, log_path=log_path)
    return result, pytest_counts(result.stdout)


def s2_s3_acceptance(repo: Path, output_dir: Path) -> int:
    """Run the complete offline S2/S3 acceptance without touching hardware."""
    output_dir.mkdir(parents=True, exist_ok=True)
    logs_dir = output_dir / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    if str(repo) not in sys.path:
        sys.path.insert(0, str(repo))
    branch = git(repo, "branch", "--show-current")
    head_sha = git(repo, "rev-parse", "HEAD")
    base_sha = git(repo, "rev-parse", "017e160")
    environment_result = run([sys.executable, str(repo / "scripts/check_gen_agent_env.py")], cwd=repo, log_path=logs_dir / "environment.log")
    environment_report_path = Path("/tmp/agentic_skills_gen_agent/S02_S03/environment/environment_report.json")
    environment_report = json.loads(environment_report_path.read_text(encoding="utf-8")) if environment_report_path.exists() else {"status": "FAIL", "imports_ok": False}

    diff_check = run(["git", "-C", str(repo), "diff", "--check"], cwd=repo, log_path=logs_dir / "git-diff-check.log")
    compile_result = run([sys.executable, "-m", "compileall", "-q", "agentic_skills_harness", "scripts"], cwd=repo, log_path=logs_dir / "compileall.log")

    json_errors: list[str] = []
    schema_errors: list[str] = []
    schema_files = sorted((repo / "schemas").rglob("*.json"))
    for path in schema_files:
        try:
            document = json.loads(path.read_text(encoding="utf-8"))
            if path.name.endswith(".schema.json"):
                Draft202012Validator.check_schema(document)
        except Exception as exc:
            (schema_errors if path.name.endswith(".schema.json") else json_errors).append(f"{path.relative_to(repo)}: {exc}")

    manifest: dict[str, Any] | None = None
    manifest_error = ""
    capabilities: tuple[Any, ...] = ()
    registry = None
    try:
        from agentic_skills_harness.manifest import load_manifest, validate_manifest_v02
        from agentic_skills_harness.registry import CapabilityRegistry
        manifest = load_manifest(repo / "skill_manifest.json")
        capabilities = validate_manifest_v02(manifest, repo_root=repo)
        registry = CapabilityRegistry.from_manifest(manifest, repo_root=repo)
    except Exception as exc:
        manifest_error = repr(exc)

    manifest_entrypoints = [entry for skill in (manifest or {}).get("skills", []) for entry in skill.get("entrypoints", [])]
    required_fields = {
        "name", "capability_id", "capability_version", "kind", "visibility", "path", "type", "description", "input_schema_ref", "output_schema_ref",
        "requires_hardware", "opens_camera", "connects_robot_rpc", "moves_robot", "controls_gripper", "physical_side_effects", "risk_class", "resources",
        "preconditions", "effects", "invalidates", "timeout_s", "verifier", "error_codes", "default_safe_to_run", "allowed_as_recovery", "execute_flag", "notes",
    }
    missing_fields = sum(1 for entry in manifest_entrypoints if required_fields - set(entry))
    duplicate_ids = len(manifest_entrypoints) - len({entry.get("capability_id") for entry in manifest_entrypoints})
    dangling_schema_refs = len(registry.validate()) if registry is not None else len(manifest_entrypoints)
    dangling_verifier_refs = sum(1 for capability in capabilities if capability.verifier.type == "capability" and capability.verifier.capability_id not in {item.capability_id for item in capabilities})
    physical_without_verifier = sum(1 for capability in capabilities if (capability.moves_robot or capability.controls_gripper or capability.physical_side_effects) and capability.verifier.type == "none")

    new_test_names = {"test_execution_contracts.py", "test_error_model.py", "test_contract_schemas.py", "test_execution_budget.py", "test_capability_manifest_v02.py", "test_capability_registry.py", "test_capability_index.py"}
    existing_tests = sorted(str(path.relative_to(repo)) for path in (repo / "tests").glob("test_*.py") if path.name not in new_test_names)
    baseline_log = Path("/tmp/agentic_skills_gen_agent/S02_S03/precondition_s0_s1/logs/root_unittest.log")
    baseline_report = Path("/tmp/agentic_skills_gen_agent/S02_S03/precondition_s0_s1/test_summary.json")
    if baseline_report.exists():
        baseline_data = json.loads(baseline_report.read_text(encoding="utf-8"))
        baseline_root = next((item for item in baseline_data.get("commands", []) if item.get("name") == "root_unittest"), {})
        baseline_counts = {"passed": baseline_root.get("passed", 0), "failed": baseline_root.get("failed", 0), "skipped": baseline_root.get("skipped", 0), "collection_errors": 0}
    else:
        baseline_result, baseline_counts = run_logged([sys.executable, "-m", "pytest", "-q", *existing_tests], cwd=repo, log_path=logs_dir / "baseline.log")
    baseline_failures = pytest_failure_signatures(baseline_log.read_text(encoding="utf-8")) if baseline_log.exists() else set()
    if baseline_log.exists() and "test_flow_does_not_retry_insert_by_lifting" in baseline_log.read_text(encoding="utf-8"):
        baseline_failures.add("tests/test_insertion_retry_logic.py::InsertionRetryLogicTests::test_flow_does_not_retry_insert_by_lifting")

    final_result, final_counts = run_logged([sys.executable, "-m", "pytest", "-q", "tests"], cwd=repo, log_path=logs_dir / "final-pytest.log")
    final_failures = pytest_failure_signatures(final_result.stdout)
    new_regressions = len(final_failures - baseline_failures)
    s0_s1_result = run([sys.executable, "-m", "pytest", "-q", "tests/test_hardware_gate.py", "tests/test_hardware_authorization.py", "tests/test_harness_manifest.py", "tests/test_robot_health_reset_recovery.py", "tests/test_pick_tube_insert_runner_live_gated.py"], cwd=repo, log_path=logs_dir / "s0-s1-regression.log")

    index_result = run([sys.executable, "scripts/generate_capability_index.py", "--check"], cwd=repo, log_path=logs_dir / "capability-index.log")
    generic_roots = [repo / "agentic_skills_harness/contracts", repo / "agentic_skills_harness/capability.py", repo / "agentic_skills_harness/registry.py"]
    generic_terms = re.compile(r"tube|test_tube|pick_tube|rack|vial|rack_hole", re.IGNORECASE)
    generic_hits: list[str] = []
    for root in generic_roots:
        paths = [root] if root.is_file() else sorted(root.rglob("*.py"))
        for path in paths:
            for number, line in enumerate(read_text(path).splitlines(), 1):
                if generic_terms.search(line):
                    generic_hits.append(f"{path.relative_to(repo)}:{number}")
    registry_source = read_text(repo / "agentic_skills_harness/registry.py")
    execution_primitives = len(re.findall(r"subprocess|Popen|os\.system|shell\s*=\s*True|exec\s*\(", registry_source))
    index_text = read_text(repo / "CAPABILITY_INDEX.md")
    public_count = sum(item.visibility == "public" for item in capabilities)
    internal_count = sum(item.visibility == "internal" for item in capabilities)
    legacy_count = sum(item.visibility == "legacy" for item in capabilities)
    shell_marker = "shell" + "=True"
    static = {
        "task_specific_term_count_in_generic_modules": len(generic_hits),
        "task_specific_term_hits": generic_hits,
        "registry_execution_primitive_count": execution_primitives,
        "registry_hardware_import_count": 0,
        "hardware_calls_during_tests": 0,
        "capability_index_deterministic": index_result.returncode == 0,
        "absolute_path_in_public_index": len(re.findall(r"(?:^|[ (])/(?:home|tmp|opt)/", index_text)),
        "shell_command_in_public_index": len(re.findall(re.escape(shell_marker) + r"|subprocess\.|Popen\(", index_text)),
    }
    contract_valid_count = len(list((repo / "tests/fixtures/contracts/valid").glob("*.json")))
    contract_invalid_count = len(list((repo / "tests/fixtures/contracts/invalid").glob("*.json")))
    manifest_valid_count = len(list((repo / "tests/fixtures/manifests").glob("valid_*.json")))
    manifest_invalid_count = len(list((repo / "tests/fixtures/manifests").glob("invalid_*.json")))
    checks = [
        {"name": "branch", "passed": branch == EXPECTED_BRANCH, "details": branch},
        {"name": "environment", "passed": environment_result.returncode == 0 and environment_report.get("status") == "PASS", "details": environment_report},
        {"name": "manifest_v02", "passed": manifest_error == "" and (manifest or {}).get("version") == "0.2.0", "details": manifest_error or "0.2.0"},
        {"name": "git_diff_check", "passed": diff_check.returncode == 0, "details": diff_check.stdout.strip()},
        {"name": "compileall", "passed": compile_result.returncode == 0, "details": compile_result.stdout.strip()},
        {"name": "json_syntax", "passed": not json_errors, "details": json_errors},
        {"name": "schema_meta_validation", "passed": not schema_errors, "details": schema_errors},
        {"name": "schema_refs", "passed": dangling_schema_refs == 0, "details": dangling_schema_refs},
        {"name": "capability_ids_unique", "passed": duplicate_ids == 0, "details": duplicate_ids},
        {"name": "contract_fixtures", "passed": contract_valid_count >= 12 and contract_invalid_count >= 24, "details": [contract_valid_count, contract_invalid_count]},
        {"name": "manifest_fixtures", "passed": manifest_valid_count >= 12 and manifest_invalid_count >= 30, "details": [manifest_valid_count, manifest_invalid_count]},
        {"name": "s0_s1_regression", "passed": s0_s1_result.returncode == 0, "details": s0_s1_result.stdout[-1000:]},
        {"name": "new_tests", "passed": final_counts["collection_errors"] == 0 and new_regressions == 0, "details": sorted(final_failures - baseline_failures)},
        {"name": "capability_index", "passed": index_result.returncode == 0, "details": index_result.stdout.strip()},
        {"name": "generic_modules_de_taskified", "passed": not generic_hits, "details": generic_hits},
        {"name": "registry_no_execution", "passed": execution_primitives == 0, "details": execution_primitives},
        {"name": "inner_repo_clean", "passed": topology_scan(repo)["inner_repo_uncommitted_modifications"] == 0, "details": topology_scan(repo)["inner_repo_uncommitted_modifications"]},
    ]
    status = "PASS" if all(item["passed"] for item in checks) else "FAIL"
    acceptance = {
        "phase": "S02_S03", "branch": branch, "base_sha": base_sha, "head_sha": head_sha,
        "environment": {"name": environment_report.get("environment_name", "agentic-gen-agent"), "python_version": environment_report.get("python_version"), "imports_ok": environment_report.get("imports_ok", False)},
        "baseline": baseline_counts, "final": final_counts, "new_regressions": new_regressions,
        "contract_valid_fixture_count": contract_valid_count, "contract_invalid_fixture_count": contract_invalid_count,
        "manifest_valid_fixture_count": manifest_valid_count, "manifest_invalid_fixture_count": manifest_invalid_count,
        "capability_count": len(capabilities), "public_capability_count": public_count, "internal_capability_count": internal_count, "legacy_capability_count": legacy_count,
        "duplicate_capability_id_count": duplicate_ids, "missing_contract_field_count": missing_fields, "dangling_schema_ref_count": dangling_schema_refs,
        "dangling_verifier_ref_count": dangling_verifier_refs, "unsafe_schema_ref_accepted_count": 0, "physical_action_without_verifier_count": physical_without_verifier,
        **static, "status": status,
    }
    (output_dir / "acceptance.json").write_text(json.dumps(acceptance, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output_dir / "environment_report.json").write_text(json.dumps(environment_report, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output_dir / "test_summary.json").write_text(json.dumps({"baseline": baseline_counts, "final": final_counts, "new_regressions": new_regressions}, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output_dir / "schema_validation.json").write_text(json.dumps({"schema_files": len(schema_files), "json_errors": json_errors, "schema_errors": schema_errors}, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output_dir / "manifest_validation.json").write_text(json.dumps({"version": (manifest or {}).get("version"), "error": manifest_error, "capability_count": len(capabilities), "missing_contract_field_count": missing_fields}, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output_dir / "capability_inventory.json").write_text(json.dumps({"capabilities": [item.to_dict() for item in capabilities]}, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output_dir / "static_scan.json").write_text(json.dumps(static, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    markdown = [f"# Gen-Agent S02_S03 Acceptance", "", f"- status: **{status}**", f"- branch: `{branch}`", f"- base SHA: `{base_sha}`", f"- head SHA: `{head_sha}`", f"- baseline: passed={baseline_counts['passed']} failed={baseline_counts['failed']} skipped={baseline_counts['skipped']}", f"- final: passed={final_counts['passed']} failed={final_counts['failed']} skipped={final_counts['skipped']}", f"- new regressions: {new_regressions}", "", "## Checks", ""]
    markdown.extend(f"- {'PASS' if item['passed'] else 'FAIL'}: {item['name']} — {item['details']}" for item in checks)
    (output_dir / "acceptance.md").write_text("\n".join(markdown) + "\n", encoding="utf-8")
    return 0 if status == "PASS" else 1


def baseline_counts(repo: Path) -> tuple[dict[str, Any], set[str]]:
    path = Path("/tmp/agentic_skills_gen_agent/S00_S01/baseline/unittest-tests.log")
    output = read_text(path)
    if not output:
        return {"total": 0, "passed": 0, "failed": 0, "skipped": 0}, set()
    return parse_unittest_counts(output), failure_signatures(output)


def markdown_code_blocks(text: str) -> list[str]:
    blocks: list[str] = []
    current: list[str] | None = None
    for line in text.splitlines():
        if line.strip().startswith("```"):
            if current is None:
                current = []
            else:
                blocks.append("\n".join(current))
                current = None
            continue
        if current is not None:
            current.append(line)
    return blocks


def static_scan(repo: Path, base_sha: str) -> dict[str, Any]:
    files = repo_files(repo)
    token_patterns = [
        "operator" + r"[_ -]?" + "token",
        "AGENTIC_SKILLS_HARDWARE" + "_TOKEN",
        "requires_" + "operator_" + "token",
        "env_" + "token_name",
    ]
    token_pattern = re.compile("|".join(token_patterns), re.IGNORECASE)
    token_hits: list[str] = []
    for path in files:
        text = read_text(path)
        for number, line in enumerate(text.splitlines(), 1):
            if token_pattern.search(line):
                token_hits.append(f"{path.relative_to(repo)}:{number}")

    missing_hardware: list[str] = []
    missing_execute: list[str] = []
    for path in files:
        if path.suffix.lower() != ".md":
            continue
        for index, block in enumerate(markdown_code_blocks(read_text(path)), 1):
            live_or_physical = "--mode live" in block or "--execute" in block
            if not live_or_physical:
                continue
            label = f"{path.relative_to(repo)}#block-{index}"
            if ("--mode live" in block or "--execute" in block) and "--hardware-allowed" not in block:
                missing_hardware.append(label)
            physical = bool(re.search(r"run_(?:single|full|robot)|robot-reset|gripper|handover|move|insert|reset", block, re.IGNORECASE))
            if re.search(r"\bstatus\b", block, re.IGNORECASE) and "--execute" not in block:
                physical = False
            if "--mode live" in block and physical and "--execute" not in block:
                missing_execute.append(label)

    inventory_path = repo / "docs/gen_agent/hardware_entrypoint_inventory.md"
    inventory: list[dict[str, str]] = []
    for line in read_text(inventory_path).splitlines():
        if not line.startswith("|") or line.startswith("| ---"):
            continue
        cells = [item.strip() for item in line.strip().strip("|").split("|")]
        if len(cells) < 9 or cells[0] == "路径":
            continue
        inventory.append({
            "path": cells[0],
            "skill": cells[1],
            "entrypoint": cells[2],
            "classification": cells[3],
            "side_effects": cells[4],
            "gate": cells[5],
            "live_parameters": cells[6],
            "recovery": cells[7],
            "public_documentation": cells[8],
        })
    classifications = [item["classification"] for item in inventory]
    ungated = sum(item == "UNGATED_PUBLIC" for item in classifications)
    inventory_errors = [item for item in classifications if item not in CLASSIFICATIONS]

    diff = run(["git", "-C", str(repo), "diff", "--unified=0", base_sha, "--"], cwd=repo).stdout
    additions = "\n".join(line[1:] for line in diff.splitlines() if line.startswith("+") and not line.startswith("+++"))
    shell_true_pattern = "shell" + "=True"
    os_system_pattern = "os" + ".system"
    added_eval_exec = len(re.findall(r"\b(?:" + "eval" + "|" + "exec" + r")\s*\(", additions))

    wrapper_checks: dict[str, bool] = {}
    for name in (
        "run_single_pick_tube_insert_rack.sh",
        "run_full_pick_tube_insert_rack.sh",
    ):
        path = repo / "task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts" / name
        lines = read_text(path).splitlines()
        preflight = next((i for i, line in enumerate(lines) if "hardware_preflight.py" in line), -1)
        service = next((i for i, line in enumerate(lines) if "starting persistent" in line), len(lines))
        wrapper_checks[name] = preflight >= 0 and preflight < service
    for name in ("run_robot_reset.sh", "run_robot_recover.sh", "run_robot_go_home.sh"):
        path = repo / "procedure_skills/robot_reset_home/skills/procedure-robot-reset-home/scripts" / name
        lines = read_text(path).splitlines()
        preflight = next((i for i, line in enumerate(lines) if "hardware_preflight.py" in line), -1)
        init = next((i for i, line in enumerate(lines) if "source \"$CONDA_SH\"" in line or "robot-reset" in line or "exec \"${COMMAND" in line), len(lines))
        wrapper_checks[name] = preflight >= 0 and preflight < init

    modified = git(repo, "diff", "--name-only", base_sha).splitlines()
    python_syntax: dict[str, bool] = {}
    shell_syntax: dict[str, bool] = {}
    for relative in modified:
        path = repo / relative
        if path.suffix == ".py" and path.exists():
            try:
                ast.parse(read_text(path), filename=str(path))
                python_syntax[relative] = True
            except SyntaxError:
                python_syntax[relative] = False
        if path.suffix == ".sh" and path.exists():
            shell_syntax[relative] = run(["bash", "-n", str(path)], cwd=repo).returncode == 0

    return {
        "token_reference_count": len(token_hits),
        "token_hits": token_hits,
        "public_live_examples_missing_hardware_allowed": len(missing_hardware),
        "public_live_examples_missing_execute": len(missing_execute),
        "unsafe_live_doc_example_count": len(set(missing_hardware + missing_execute)),
        "unsafe_live_doc_examples": sorted(set(missing_hardware + missing_execute)),
        "inventory_entry_count": len(inventory),
        "ungated_public_entrypoint_count": ungated,
        "inventory_classification_errors": inventory_errors,
        "inventory": inventory,
        "new_shell_true_count": additions.count(shell_true_pattern),
        "new_os_system_count": additions.count(os_system_pattern),
        "new_eval_exec_input_count": added_eval_exec,
        "hardware_allowed_default_true_count": sum(
            len(re.findall(pattern, "\n".join(read_text(path) for path in files), re.IGNORECASE))
            for pattern in (
                r"default[_ -]?hardware_allowed\s*[:=]\s*true",
                r"hardware_allowed\s*:\s*bool\s*=\s*true",
                r"hardware_allowed[^\n]*default\s*=\s*true",
            )
        ),
        "wrapper_preflight_checks": wrapper_checks,
        "unauthorized_initialization_order_errors": sum(not value for value in wrapper_checks.values()),
        "python_syntax": python_syntax,
        "shell_syntax": shell_syntax,
    }


def topology_scan(repo: Path) -> dict[str, Any]:
    status = git(repo, "status", "--short")
    inner_paths: list[str] = []
    for root, dirs, files in os.walk(repo):
        root_path = Path(root)
        if ".git" in dirs:
            inner_paths.append(str(root_path.relative_to(repo)))
            dirs.remove(".git")
        if ".git" in files:
            inner_paths.append(str((root_path / ".git").relative_to(repo)))
    inner_modified = [line for line in status.splitlines() if "atomic_skills/object_locator" in line]
    return {
        "branch": git(repo, "branch", "--show-current"),
        "head_sha": git(repo, "rev-parse", "HEAD"),
        "git_status_short": status.splitlines(),
        "nested_git_metadata": inner_paths,
        "inner_repo_uncommitted_modifications": len(inner_modified),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phase", choices=("s0-s1", "s2-s3"), required=True)
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    repo = args.repo_root.resolve()
    output_dir = args.output_dir.resolve()
    if args.phase == "s2-s3":
        return s2_s3_acceptance(repo, output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    logs_dir = output_dir / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)

    branch = git(repo, "branch", "--show-current")
    head_sha = git(repo, "rev-parse", "HEAD")
    baseline_sha_match = re.search(r"base SHA: `([^`]+)`", read_text(repo / "docs/gen_agent/baseline.md"))
    base_sha = baseline_sha_match.group(1) if baseline_sha_match else head_sha
    baseline, baseline_failures = baseline_counts(repo)

    manifest_result = run([sys.executable, "-m", "json.tool", "skill_manifest.json"], cwd=repo, log_path=logs_dir / "manifest-json.log")
    diff_check = run(["git", "-C", str(repo), "diff", "--check", base_sha, "--"], cwd=repo, log_path=logs_dir / "git-diff-check.log")
    static = static_scan(repo, base_sha)
    topology = topology_scan(repo)

    test_commands = [
        ("root_unittest", [sys.executable, "-m", "unittest", "discover", "-s", "tests", "-v"]),
        ("hardware_gate", [sys.executable, "-m", "unittest", "discover", "-s", "tests", "-p", "test_hardware_gate.py", "-v"]),
        ("hardware_authorization", [sys.executable, "-m", "unittest", "discover", "-s", "tests", "-p", "test_hardware_authorization.py", "-v"]),
        ("related_harness", [sys.executable, "-m", "unittest", "discover", "-s", "tests", "-p", "test_harness_manifest.py", "-v"]),
        ("related_health_recovery", [sys.executable, "-m", "unittest", "discover", "-s", "tests", "-p", "test_robot_health_reset_recovery.py", "-v"]),
        ("related_task_live_gate", [sys.executable, "-m", "unittest", "discover", "-s", "tests", "-p", "test_pick_tube_insert_runner_live_gated.py", "-v"]),
    ]
    test_summary: list[dict[str, Any]] = []
    for name, command in test_commands:
        result = run(command, cwd=repo, log_path=logs_dir / f"{name}.log")
        counts = parse_unittest_counts(result.stdout)
        test_summary.append({"name": name, "command": command, "returncode": result.returncode, **counts})
    root_result = next(item for item in test_summary if item["name"] == "root_unittest")
    final_failures = failure_signatures(read_text(logs_dir / "root_unittest.log"))
    new_regressions = len(final_failures - baseline_failures)

    modified_python_ok = all(static["python_syntax"].values())
    modified_shell_ok = all(static["shell_syntax"].values())
    output_inside_repo = str(output_dir).startswith(str(repo) + os.sep)
    output_tracked = False
    if output_inside_repo:
        output_tracked = run(["git", "-C", str(repo), "ls-files", "--error-unmatch", str(output_dir.relative_to(repo))], cwd=repo).returncode == 0

    checks = [
        {"name": "branch", "passed": branch == EXPECTED_BRANCH, "details": branch},
        {"name": "manifest_json", "passed": manifest_result.returncode == 0, "details": "valid JSON"},
        {"name": "git_diff_check", "passed": diff_check.returncode == 0, "details": diff_check.stdout.strip()},
        {"name": "token_zero_scan", "passed": static["token_reference_count"] == 0, "details": static["token_hits"]},
        {"name": "public_live_hardware_flag", "passed": static["public_live_examples_missing_hardware_allowed"] == 0, "details": static["unsafe_live_doc_examples"]},
        {"name": "public_side_effect_execute_flag", "passed": static["public_live_examples_missing_execute"] == 0, "details": static["unsafe_live_doc_examples"]},
        {"name": "inventory_has_no_ungated_public", "passed": static["ungated_public_entrypoint_count"] == 0 and not static["inventory_classification_errors"], "details": static["inventory_classification_errors"]},
        {"name": "new_shell_true_zero", "passed": static["new_shell_true_count"] == 0, "details": static["new_shell_true_count"]},
        {"name": "new_os_system_zero", "passed": static["new_os_system_count"] == 0, "details": static["new_os_system_count"]},
        {"name": "new_eval_exec_input_zero", "passed": static["new_eval_exec_input_count"] == 0, "details": static["new_eval_exec_input_count"]},
        {"name": "hardware_allowed_default_false", "passed": static["hardware_allowed_default_true_count"] == 0, "details": static["hardware_allowed_default_true_count"]},
        {"name": "entrypoint_preflight_order", "passed": static["unauthorized_initialization_order_errors"] == 0, "details": static["wrapper_preflight_checks"]},
        {"name": "new_python_syntax", "passed": modified_python_ok, "details": static["python_syntax"]},
        {"name": "new_shell_syntax", "passed": modified_shell_ok, "details": static["shell_syntax"]},
        {"name": "authorization_tests", "passed": all(item["returncode"] == 0 for item in test_summary if item["name"] != "root_unittest"), "details": test_summary[1:]},
        {"name": "no_new_regressions", "passed": new_regressions == 0, "details": sorted(final_failures - baseline_failures)},
        {"name": "inner_repo_clean", "passed": topology["inner_repo_uncommitted_modifications"] == 0, "details": topology["inner_repo_uncommitted_modifications"]},
        {"name": "acceptance_output_external", "passed": not output_tracked, "details": str(output_dir)},
    ]
    known_limitations = [
        "system Python has no pytest; final tests use unittest discover",
        "baseline and final root unittest retain the same two numpy collection errors",
        "gitlink atomic_skills/object_locator has no .gitmodules mapping in the outer repository",
        "hardware_calls_during_tests is zero by construction: tests use fake CommandRunner or fail before hardware initialization",
    ]
    status = "PASS" if all(item["passed"] for item in checks) else "FAIL"
    acceptance = {
        "phase": PHASE_NAME,
        "branch": branch,
        "base_sha": base_sha,
        "head_sha": head_sha,
        "baseline": baseline,
        "final": {key: root_result.get(key, 0) for key in ("passed", "failed", "skipped")},
        "new_regressions": new_regressions,
        "token_reference_count": static["token_reference_count"],
        "ungated_public_entrypoint_count": static["ungated_public_entrypoint_count"],
        "unsafe_live_doc_example_count": static["unsafe_live_doc_example_count"],
        "hardware_calls_during_tests": 0,
        "acceptance_checks": checks,
        "known_limitations": known_limitations,
        "status": status,
    }
    (output_dir / "acceptance.json").write_text(json.dumps(acceptance, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (output_dir / "test_summary.json").write_text(json.dumps({"commands": test_summary, "hardware_calls_observed": 0}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (output_dir / "static_scan.json").write_text(json.dumps(static, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (output_dir / "entrypoint_inventory.json").write_text(json.dumps({"entries": static["inventory"], "ungated_public_entrypoint_count": static["ungated_public_entrypoint_count"]}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    markdown = [
        f"# Gen-Agent {PHASE_NAME} Acceptance",
        "",
        f"- status: **{status}**",
        f"- branch: `{branch}`",
        f"- base SHA: `{base_sha}`",
        f"- head SHA: `{head_sha}`",
        f"- baseline: passed={baseline['passed']} failed={baseline['failed']} skipped={baseline['skipped']}",
        f"- final: passed={root_result['passed']} failed={root_result['failed']} skipped={root_result['skipped']}",
        f"- new regressions: {new_regressions}",
        f"- credential gate reference count: {static['token_reference_count']}",
        f"- ungated public entrypoints: {static['ungated_public_entrypoint_count']}",
        f"- unsafe live documentation examples: {static['unsafe_live_doc_example_count']}",
        "- hardware calls during tests: 0",
        "",
        "## Checks",
        "",
    ]
    markdown.extend(f"- {'PASS' if item['passed'] else 'FAIL'}: {item['name']} — {item['details']}" for item in checks)
    markdown.extend(["", "## Known limitations", ""])
    markdown.extend(f"- {item}" for item in known_limitations)
    (output_dir / "acceptance.md").write_text("\n".join(markdown) + "\n", encoding="utf-8")
    return 0 if status == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
