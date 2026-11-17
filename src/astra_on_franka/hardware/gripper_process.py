"""Exclusive Dynamixel owner in a spawned process; bounded, serialized RPCs."""
from collections import deque
import multiprocessing as mp
import os
import threading
import time

from .managed_gripper import ManagedGripper


def gripper_worker(connection,config,execute,factory=ManagedGripper):
    gripper=None
    try:
        gripper=factory(config)
        connection.send({'ready':True,'pid':os.getpid()})
        while True:
            if not connection.poll(.1):
                parent=mp.parent_process()
                if parent is not None and not parent.is_alive():break
                continue
            request=connection.recv()
            operation=request['operation']
            if operation=='close':
                closing=gripper;gripper=None
                try:closing.close();connection.send({'id':request['id'],'result':None,'events':[]})
                except Exception as exc:connection.send({'id':request['id'],'error':str(exc),'events':[]})
                break
            response={'id':request['id']}
            try:
                if time.monotonic_ns()>request['deadline_ns']:
                    raise RuntimeError('Gripper request expired before execution')
                if operation not in ('read','enable_at_current_position','command','disable'):
                    raise ValueError('Unsupported gripper operation')
                if not execute and operation in ('enable_at_current_position','command'):
                    raise RuntimeError('Preview worker prohibits torque enable and motion')
                if operation=='command':response['result']=gripper.command(request['fraction'])
                else:response['result']=getattr(gripper,operation)()
            except Exception as exc:
                response['error']=f'{type(exc).__name__}: {exc}'
                if operation in ('enable_at_current_position','command'):
                    try:gripper.disable()
                    except Exception as cleanup:response['disable_error']=str(cleanup)
            response['events']=gripper.drain_read_events()
            response['events_dropped']=gripper.read_events_dropped
            connection.send(response)
    except (EOFError,BrokenPipeError,ConnectionResetError):
        pass
    except Exception as exc:
        try:connection.send({'error':f'{type(exc).__name__}: {exc}'})
        except (OSError,EOFError):pass
    finally:
        if gripper is not None:
            try:gripper.close()
            except Exception as exc:
                # A broken pipe cannot deliver cleanup errors; retain stderr evidence.
                print(f'Gripper worker shutdown failed: {exc}',flush=True)
        connection.close()


class GripperProcess:
    """Same narrow interface as ManagedGripper; no SDK objects in the parent."""
    def __init__(self,config,*,execute=False,worker_factory=ManagedGripper):
        self._lock=threading.Lock();self._events=deque(maxlen=512)
        self.read_events_dropped=0;self._worker_dropped=0;self._request_id=0;self._fault=None;self._closed=False
        context=mp.get_context('spawn')
        self._connection,child=context.Pipe()
        self._process=context.Process(target=gripper_worker,args=(child,config,execute,worker_factory),
                                      name='astra-dynamixel',daemon=True)
        self._process.start();child.close()
        try:
            if not self._connection.poll(10):raise RuntimeError('Gripper worker startup timeout')
            ready=self._connection.recv()
            if not ready.get('ready'):raise RuntimeError(ready.get('error','Gripper startup failed'))
            self.worker_pid=ready['pid']
        except BaseException:
            self._connection.close();self._process.join(2)
            raise

    def _call(self,operation,*,fraction=None,timeout=.25):
        started=time.monotonic_ns()
        with self._lock:
            if self._closed or self._fault:raise RuntimeError(self._fault or 'Gripper process closed')
            self._request_id+=1
            request={'id':self._request_id,'operation':operation,
                     'deadline_ns':started+int(timeout*1e9)}
            if fraction is not None:request['fraction']=fraction
            try:
                self._connection.send(request)
                remaining=max(0,(request['deadline_ns']-time.monotonic_ns())/1e9)
                if not self._connection.poll(remaining):raise TimeoutError(f'Gripper worker {operation} timeout')
                response=self._connection.recv()
                if response.get('id')!=request['id']:raise RuntimeError('Gripper response ID mismatch')
            except (OSError,EOFError,RuntimeError) as exc:
                self._fault=f'Gripper IPC fault: {exc}; no retries, worker shutdown requested'
                self._connection.close()  # Worker sees EOF and disables its enabled torque.
                raise RuntimeError(self._fault) from exc
            for event in response.get('events',[]):
                if len(self._events)==self._events.maxlen:self.read_events_dropped+=1
                self._events.append({**event,'worker_pid':self.worker_pid})
            dropped=response.get('events_dropped',0)
            self.read_events_dropped+=max(0,dropped-self._worker_dropped)
            self._worker_dropped=dropped
            if response.get('error'):
                raise RuntimeError(response['error']+(f'; disable_error={response["disable_error"]}' if response.get('disable_error') else ''))
            return response.get('result'),(time.monotonic_ns()-started)/1e6

    def read(self):
        result,elapsed=self._call('read')
        age=(time.monotonic_ns()-result['host_received_monotonic_ns'])/1e6
        if elapsed>100 or age>100:
            raise RuntimeError(f'Slow gripper observation including IPC: round_trip_ms={elapsed:.3f}, age_ms={age:.3f}, limit_ms=100')
        return {**result,'ipc_round_trip_ms':elapsed,'worker_pid':self.worker_pid}

    def enable_at_current_position(self):self._call('enable_at_current_position')
    def command(self,fraction):self._call('command',fraction=fraction)
    def disable(self):self._call('disable')

    def drain_read_events(self):
        events=[]
        while self._events:events.append(self._events.popleft())
        return events

    def close(self):
        if self._closed:return
        error=None
        try:
            if not self._fault:self._call('close',timeout=2)
            else:error=self._fault
        except Exception as exc:error=str(exc)
        finally:
            self._closed=True;self._connection.close();self._process.join(3)
        if self._process.is_alive():raise RuntimeError('Gripper worker did not exit; torque-disable unconfirmed; operator intervention required')
        if error:raise RuntimeError(error)
