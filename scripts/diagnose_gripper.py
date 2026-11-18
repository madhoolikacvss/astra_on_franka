#!/usr/bin/env python3
"""Read-only Dynamixel recording and offline comparison; no arm/camera connections."""
import argparse
import importlib.metadata
import json
import math
from pathlib import Path
import platform
import sys
import time

import numpy as np
import yaml
from record_observations import ROOT,create_run,write_json
from astra_on_franka.hardware.managed_gripper import ManagedGripper


class ReadOnlyGripper(ManagedGripper):
    def write_register(self,*args,**kwargs):
        raise RuntimeError('Diagnostic prohibits all servo register writes')

    def close(self):
        # Diagnostic never enables/disables torque or changes existing mode.
        self.port.closePort()


def inspect_transport(config, sys_root=Path('/sys')):
    """Read OS metadata only: never open the serial port or change settings."""
    device=Path(config['device'])
    report={'configured_device':str(device),'baud_rate':config['baud_rate'],
            'device_exists':device.exists(),'platform':platform.platform(),
            'latency_timer_ms':None,'latency_timer_path':None,'usb_devices':[],
            'errors':[]}
    if not device.exists():
        report['errors'].append('Configured serial device does not exist on this host')
        return report
    resolved=device.resolve()
    report['resolved_device']=str(resolved)
    tty=sys_root/'class/tty'/resolved.name/'device'
    latency=sys_root/'bus/usb-serial/devices'/resolved.name/'latency_timer'
    if latency.exists():
        report['latency_timer_path']=str(latency)
        try:report['latency_timer_ms']=int(latency.read_text().strip())
        except (OSError,ValueError) as exc:report['errors'].append(str(exc))
    else:
        report['errors'].append('USB serial latency_timer not available')
    if tty.exists():
        node=tty.resolve()
        report['sysfs_device']=str(node)
        for ancestor in (node,*node.parents):
            if not (ancestor/'idVendor').exists():continue
            usb={'path':str(ancestor)}
            for name in ('idVendor','idProduct','manufacturer','product','serial','speed',
                         'power/control','power/autosuspend_delay_ms'):
                path=ancestor/name
                if path.exists():
                    try:usb[name]=path.read_text().strip()
                    except OSError as exc:report['errors'].append(str(exc))
            report['usb_devices'].append(usb)
    report['note']='Read-only OS inspection. USB latency is distinct from SDK packet timeout and the 100 ms observation deadline.'
    return report


def summarize(path):
    path=Path(path)
    if path.is_dir():path=path/'dynamixel_reads.jsonl'
    rows=[json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    result={'source':str(path.resolve()),'read_count':len(rows),'registers':{}}
    for address in sorted({row['register'] for row in rows}):
        selected=[row for row in rows if row['register']==address]
        failures=sum(bool(row.get('error')) or row.get('communication_result')!=0 or row.get('packet_error')!=0 for row in selected)
        result['registers'][str(address)]={'read_count':len(selected),'failure_count':failures,
            'failure_rate':failures/len(selected),
            'elapsed_ms':dict(zip(('median','p95','p99','max'),np.percentile([r['elapsed_ms'] for r in selected],[50,95,99,100]).tolist())),
            'thread_cpu_ms_median':float(np.median([r['thread_cpu_ms'] for r in selected]))}
    return result


def compare(baseline,standalone):
    first,second=summarize(baseline),summarize(standalone)
    differences={}
    for address in first['registers'].keys() & second['registers'].keys():
        a,b=first['registers'][address],second['registers'][address]
        differences[address]={'failure_rate_change_percentage_points':100*(b['failure_rate']-a['failure_rate']),
            'median_read_ms_change':b['elapsed_ms']['median']-a['elapsed_ms']['median']}
    return {'combined_baseline':first,'gripper_only':second,'changes':differences,
        'interpretation':'Compare similar durations and unchanged wiring/settings. A difference suggests association with workload, not proof of USB contention; repeat to assess variability.'}


def record(args):
    if not math.isfinite(args.seconds) or args.seconds<=0 or not math.isfinite(args.interval) or args.interval<=0:
        raise ValueError('seconds and interval must be finite and positive')
    if args.baseline:summarize(args.baseline)  # Validate input before opening hardware.
    config=yaml.safe_load((ROOT/'config/gripper.yaml').read_text())
    run=create_run(ROOT/'data/gripper_diagnostics',prefix='gripper')
    write_json(run/'metadata.json',{'config':config,'seconds':args.seconds,'gap_after_read_s':args.interval,
        'python':sys.version,'platform':platform.platform(),
        'dynamixel_sdk_version':importlib.metadata.version('dynamixel-sdk'),
        'read_only':True,'registers':[132,70],'transport':inspect_transport(config),
        'note':'Same sequential reads as bridge. No torque/mode/current/position writes, no hardware reset, no arm/camera connections.'})
    print(f'Recording to {run}',flush=True)
    gripper=None;attempts=successes=0;interrupted=False;terminal_error=None
    started=time.monotonic()
    try:
        gripper=ReadOnlyGripper(config)
        with (run/'dynamixel_reads.jsonl').open('x') as reads,(run/'observations.jsonl').open('x') as observations:
            while time.monotonic()-started<args.seconds:
                attempts+=1
                try:
                    row={'observation':gripper.read(),'valid':True};successes+=1
                except Exception as exc:
                    # No immediate retries. Record failure, then wait the normal gap.
                    row={'valid':False,'error':f'{type(exc).__name__}: {exc}'}
                row.update(attempt=attempts,unix_ns=time.time_ns())
                observations.write(json.dumps(row,allow_nan=False)+'\n')
                for event in gripper.drain_read_events():reads.write(json.dumps(event,allow_nan=False)+'\n')
                reads.flush();observations.flush()
                time.sleep(min(args.interval,max(0,args.seconds-(time.monotonic()-started))))
    except KeyboardInterrupt:interrupted=True
    except Exception as exc:
        terminal_error=f'{type(exc).__name__}: {exc}'
        raise
    finally:
        cleanup_error=None
        if gripper:
            try:gripper.close()
            except Exception as exc:cleanup_error=str(exc)
        write_json(run/'summary.json',{'attempts':attempts,'successful_observations':successes,
            'failed_observations':attempts-successes,'elapsed_s':time.monotonic()-started,
            'interrupted':interrupted,'terminal_error':terminal_error,'cleanup_error':cleanup_error})
    if args.baseline:
        report=compare(args.baseline,run);write_json(run/'comparison.json',report)
        print(json.dumps(report,indent=2))
    else:print(json.dumps(summarize(run),indent=2))
    return 0


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    commands=parser.add_subparsers(dest='action',required=True)
    commands.add_parser('inspect',help='Read USB latency/topology from OS; no serial connection or setting changes')
    capture=commands.add_parser('record',help='Operator only: stop bridge and all gripper owners first')
    capture.add_argument('--seconds',type=float,default=260)
    capture.add_argument('--interval',type=float,default=.1,help='Gap after each read, matching bridge sleep')
    capture.add_argument('--baseline',type=Path)
    comparison=commands.add_parser('compare',help='Offline; does not import SDK or open devices')
    comparison.add_argument('baseline',type=Path);comparison.add_argument('standalone',type=Path)
    args=parser.parse_args(argv)
    if args.action=='inspect':
        config=yaml.safe_load((ROOT/'config/gripper.yaml').read_text())
        print(json.dumps(inspect_transport(config),indent=2));return 0
    if args.action=='compare':
        print(json.dumps(compare(args.baseline,args.standalone),indent=2));return 0
    return record(args)


if __name__=='__main__':raise SystemExit(main())
