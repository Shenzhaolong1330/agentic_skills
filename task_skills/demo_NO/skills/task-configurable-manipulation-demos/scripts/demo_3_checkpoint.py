"""Persistent safe tube-boundary checkpoints for the legacy Demo 3 shell flow."""
import hashlib
import json
from pathlib import Path
import sys


def read(path):
    return json.loads(path.read_text())


def write(path, data):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2))
    temporary.replace(path)


def fingerprints(paths):
    return {str(Path(p).resolve()): hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in paths}


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    action, directory = args[:2]
    if action == 'resolve':
        if directory == 'auto':
            candidates = [p.parent for p in Path('/tmp/agentic_skills_runs').glob(
                'demo_3_all_vials_to_rack_*/demo3_state.json') if not read(p).get('finished')]
            if not candidates:
                raise ValueError('没有带恢复检查点的未完成 Demo 3；旧运行目录不支持恢复')
            directory = max(candidates, key=lambda p: p.stat().st_mtime)
        print(Path(directory).resolve())
        return
    root = Path(directory)
    path = root / 'demo3_state.json'
    if action == 'prepare':
        mode = args[2]
        split = args.index('--signature')
        snapshot = {'files': fingerprints(args[4:split]), 'arguments': args[split + 1:]}
        if mode == 'new':
            if root.exists() and any(root.iterdir()):
                raise ValueError('新运行 artifact-dir 必须为空；恢复请使用 --resume')
            root.mkdir(parents=True, exist_ok=True)
            write(path, {'snapshot': snapshot, 'next_tube': 1, 'active': None,
                         'failed': False, 'finished': False})
            print(1)
            return
        state = read(path)
        if state['snapshot'] != snapshot:
            raise ValueError('当前配置/参数与失败运行不一致；拒绝恢复')
        if state['finished'] or state['active'] is not None or state['failed']:
            raise ValueError('运行已完成或抓取/插入/复位状态不明确，不能自动恢复；请人工处置后新建运行')
        marker = root / 'inventory_complete.json'
        if marker.exists() and fingerprints(read(marker).keys()) != read(marker):
            raise ValueError('缓存 inventory/rack/tube 文件已变化；拒绝恢复')
        print(state['next_tube'])
        return
    state = read(path)
    index = int(args[2]) if len(args) > 2 else 0
    if action == 'inventory':
        # These paths are part of the effective CLI snapshot.
        inventory, rack = state['snapshot']['arguments'][3:5]
        data = read(Path(inventory))
        tubes = data.get('tubes', [])
        if not tubes:
            raise ValueError('inventory contains no executable tubes')
        write(root / 'inventory_complete.json', fingerprints(
            [inventory, rack] + [t['result_json'] for t in tubes]))
    elif action == 'begin':
        state['active'] = index
    elif action == 'failed':
        state['failed'] = True
    elif action == 'complete':
        if state['active'] != index:
            raise ValueError('tube checkpoint mismatch')
        state.update(active=None, next_tube=index + 1)
    elif action == 'finish':
        if state['active'] is not None or state['failed']:
            raise ValueError('cannot finish an incomplete run')
        state['finished'] = True
    else:
        raise ValueError(f'unknown checkpoint action: {action}')
    write(path, state)


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError, KeyError) as exc:
        sys.exit(f'ERROR: {exc}')
