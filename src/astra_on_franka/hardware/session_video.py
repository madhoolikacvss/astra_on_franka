"""Bounded-memory side-view video recorder using the existing camera feed."""
import json
import threading
import time


class SessionVideo:
    def __init__(self,run,fps=10.):
        self.run=run;self.fps=fps;self.latest=None;self.error=None;self.frames=0
        self.done=threading.Event();self.thread=threading.Thread(target=self.work,daemon=True)
        self.thread.start()

    def submit(self,frame):
        # One latest immutable frame reference: no unbounded encoder queue.
        self.latest=(frame['rgb'],frame['host_received_unix_ns'],frame['host_received_monotonic_ns'])

    def work(self):
        writer=None;start=None;last_source=None;repeats=0
        try:
            import cv2
            import numpy as np
            with (self.run/'side_video_frames.jsonl').open('x') as log:
                while not self.done.is_set():
                    frame=self.latest
                    if frame is None:self.done.wait(.01);continue
                    now=time.monotonic()
                    if start is None:
                        start=now;h,w=frame[0].shape[:2]
                        writer=cv2.VideoWriter(str(self.run/'side_view.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),self.fps,(w,h))
                        if not writer.isOpened():raise RuntimeError('Cannot open side_view.mp4 encoder (mp4v)')
                    delay=start+self.frames/self.fps-now
                    if delay>0:self.done.wait(min(delay,.1));continue
                    repeated=frame[2]==last_source;repeats+=int(repeated)
                    writer.write(np.ascontiguousarray(frame[0][:,:,::-1]))
                    log.write(json.dumps({'video_frame':self.frames,'video_time_s':self.frames/self.fps,
                        'encoded_unix_ns':time.time_ns(),'source_unix_ns':frame[1],
                        'source_age_ms':(time.monotonic_ns()-frame[2])/1e6,'repeated_source':repeated})+'\n')
                    self.frames+=1;last_source=frame[2]
        except Exception as exc:self.error=f'{type(exc).__name__}: {exc}'
        finally:
            if writer is not None:writer.release()
            with (self.run/'side_video_summary.json').open('w') as f:
                json.dump({'file':'side_view.mp4','fps':self.fps,'frames':self.frames,
                    'repeated_frames':repeats,'error':self.error,
                    'source':'Side RGB frames sampled by existing bridge, resampled to 10 fps; not all 30 fps camera frames'},f,indent=2)

    def close(self):
        self.done.set();self.thread.join(10)
        if self.thread.is_alive():raise RuntimeError('Side video writer did not stop')
        if self.error:raise RuntimeError(self.error)
