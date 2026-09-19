#!/usr/bin/env python3
"""Trace one command and the remote ZeroRPC IO thread; never retry the command."""
from __future__ import annotations
import argparse
import json
import re
import select
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

# Only the ZeroMQ IO thread is traced; no controller thread or process restart.
REMOTE = r'''
import json, os, select, signal, subprocess, sys, tempfile, time
from pathlib import Path
matches = []
for entry in Path('/proc').iterdir():
    if not entry.name.isdigit(): continue
    try:
        args = (entry / 'cmdline').read_bytes().split(b'\0')
        if len(args)>1 and Path(os.fsdecode(args[1])).name == 'dual_franka_robotiq_rpc_server':
            matches.append(entry)
    except (OSError, ValueError): pass
if len(matches) != 1: raise RuntimeError('Expected exactly one Franka RPC server, found %s' % len(matches))
pid = matches[0]
tids = [p for p in (pid/'task').iterdir() if (p/'comm').read_text().strip().startswith('ZMQbg/IO/')]
if not tids: raise RuntimeError('No ZeroMQ IO thread found')
folder = Path(tempfile.mkdtemp(prefix='franka_rpc_wire_'))
trace = folder/'server_wire.txt'
with (folder/'strace_stderr.txt').open('w+') as errors:
    proc = subprocess.Popen(['strace','-qq','-ttt','-T','-s','256','-e',
          'trace=sendto,recvfrom,connect,close','-o',str(trace),
          *[arg for p in tids for arg in ('-p',p.name)]], stderr=errors)
    try:
        deadline = time.monotonic()+8
        while True:
            if proc.poll() is not None:
                errors.seek(0); raise RuntimeError('strace failed: '+errors.read())
            attached = all(('TracerPid:\t%d' % proc.pid) in (p/'status').read_text() for p in tids)
            if attached: break
            if time.monotonic()>deadline: raise RuntimeError('strace attach timed out')
            time.sleep(.05)
        print(json.dumps({'ready':True,'server_pid':int(pid.name),'io_threads':[int(p.name) for p in tids],
                          'remote_trace':str(trace),'capture_start_unix':time.time()}),flush=True)
        # Tied to this command's SSH stdin; EOF also detaches after client exit.
        # A hard limit avoids an orphan capture if a caller leaves stdin open.
        ready,_,_ = select.select([sys.stdin],[],[],1200)
        if ready: sys.stdin.readline()
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try: proc.wait(timeout=5)
            except subprocess.TimeoutExpired: proc.kill(); proc.wait()
        if trace.exists():
            sys.stdout.write(trace.read_text()); sys.stdout.flush()
'''


def run(host, output_dir, argv):
    if not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.@-]*', host):
        raise ValueError('Invalid SSH host alias')
    if not argv:
        raise ValueError('Missing command')
    if not shutil.which('strace') or not shutil.which('ssh'):
        raise RuntimeError('RPC tracing requires local strace and ssh; command not started')
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=False)
    command = 'python3 -u -c ' + shlex.quote(REMOTE)
    with (out/'ssh_stderr.txt').open('w') as errors:
        ssh = subprocess.Popen(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=5',host,command],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=errors, text=True)
        try:
            ready, _, _ = select.select([ssh.stdout], [], [], 15)
            if not ready:
                raise RuntimeError('Remote trace readiness timed out; command not started')
            first = ssh.stdout.readline()
            try:
                status = json.loads(first)
            except (ValueError, TypeError):
                raise RuntimeError(f'Remote trace failed; command not started; see {out / "ssh_stderr.txt"}')
            if status.get('ready') is not True:
                raise RuntimeError('Remote trace not ready; command not started')
            (out/'capture.json').write_text(json.dumps(status, indent=2)+'\n')
            print(f'[rpc_trace] ready: server PID {status["server_pid"]}; artifacts {out}',
                  file=sys.stderr, flush=True)
            # No shell and exactly one invocation of the original argv.
            return subprocess.call(['strace','-f','-qq','-ttt','-T','-s','256','-e',
                                    'trace=sendto,recvfrom,connect,close','-o',str(out/'client_wire.txt'),
                                    '--',*argv])
        finally:
            try:
                captured, _ = ssh.communicate('stop\n', timeout=10)
                (out/'server_wire.txt').write_text(captured)
            except (subprocess.TimeoutExpired, BrokenPipeError, OSError) as exc:
                # Never change a motion result or rerun it due to capture cleanup.
                print(f'[rpc_trace] collection incomplete: {exc}; remote path in capture.json', file=sys.stderr)
                ssh.kill()
                ssh.communicate()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    argv = args.command[1:] if args.command[:1] == ['--'] else args.command
    try:
        return run(args.host, args.output_dir, argv)
    except (Exception, KeyboardInterrupt) as exc:
        print(f'RPC trace failed: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
