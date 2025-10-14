#!/usr/bin/env python3
import asyncio, signal, threading, time, sys
import numpy as np, cv2
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.media import MediaBlackhole
from av import VideoFrame
from ultralytics import YOLO
from picamera2 import Picamera2
from libcamera import Transform

# ---------- hardcoded config ----------
HOST, PORT    = "0.0.0.0", 8080
W, H, FPS     = 1280, 720, 30
MODEL_PATH    = "yolov8n-pose.pt"
IMG_SIZE      = 640
CONF          = 0.25

INDEX_HTML = """<!doctype html><html><body>
<video id="v" autoplay playsinline muted style="width:80vw;background:#000;border-radius:8px"></video>
<pre id="log" style="white-space:pre-wrap;font:12px system-ui;color:#555"></pre>
<script>
const log=(...a)=>document.getElementById('log').textContent+=a.join(' ')+"\\n";
const pc=new RTCPeerConnection({iceServers:[{urls:"stun:stun.l.google.com:19302"}]});
pc.oniceconnectionstatechange=()=>log("ICE:",pc.iceConnectionState);
pc.onconnectionstatechange=()=>log("PC:",pc.connectionState);
pc.ontrack=e=>{document.getElementById('v').srcObject=e.streams[0];};
pc.addTransceiver("video",{direction:"recvonly"});
async function waitIce(pc){ if(pc.iceGatheringState==="complete") return;
  await new Promise(r=>{ function c(){ if(pc.iceGatheringState==="complete"){pc.removeEventListener("icegatheringstatechange",c); r();}}
    pc.addEventListener("icegatheringstatechange",c);
  });
}
(async ()=>{
  const offer=await pc.createOffer(); await pc.setLocalDescription(offer); await waitIce(pc);
  const res=await fetch("/offer",{method:"POST",headers:{"Content-Type":"application/json"},
    body:JSON.stringify({sdp:pc.localDescription.sdp,type:pc.localDescription.type})});
  const ans=await res.json(); await pc.setRemoteDescription(ans);
  log("Answer set");
})().catch(err=>log("ERR:",err));
</script></body></html>"""

# -------- camera wrapper (Picamera2) --------
class PiCam:
    def __init__(self, w, h, fps):
        self.cam = Picamera2()
        self.cam.configure(self.cam.create_video_configuration(
            main={"size": (w, h), "format": "RGB888"},
            transform=Transform(hflip=False, vflip=False),
            buffer_count=4))
        self.cam.set_controls({"FrameRate": fps})
        self.cam.start()

        self._lock = threading.Lock()
        self._frame = None
        self._stop = False

        self._thr = threading.Thread(target=self._loop, daemon=True)
        self._thr.start()

    def _loop(self):
        while not self._stop:
            f = self.cam.capture_array()  # RGB HxWx3
            with self._lock:
                self._frame = f

    def read(self):
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def stop(self):
        self._stop = True
        try: self._thr.join(timeout=1.0)
        except: pass
        try: self.cam.stop()
        except: pass

# -------- WebRTC video track with YOLO pose --------
class YoloPoseTrack(VideoStreamTrack):
    kind = "video"
    def __init__(self, cam: PiCam):
        super().__init__()
        print(f"[YOLO] Loading {MODEL_PATH} ...")
        self.model = YOLO(MODEL_PATH)
        print("[YOLO] Ready.")
        self.cam = cam
        self.interval = 1.0 / max(FPS, 1)
        self.last = time.time()
        # fps log
        self._n = 0
        self._t0 = time.time()

    async def recv(self):
        # timestamps for this frame (critical for playback)
        pts, time_base = await self.next_timestamp()

        # pace
        wait = self.interval - (time.time() - self.last)
        if wait > 0:
            await asyncio.sleep(wait)
        self.last = time.time()

        # fetch frame
        rgb = self.cam.read()
        if rgb is None:
            frame = np.zeros((H, W, 3), np.uint8)
        else:
            frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        # run pose + draw skeleton
        res = self.model.predict(frame, imgsz=IMG_SIZE, conf=CONF, verbose=False)
        annotated = res[0].plot()  # BGR

        # wrap
        vf = VideoFrame.from_ndarray(annotated, format="bgr24")
        vf.pts, vf.time_base = pts, time_base

        # light fps log
        self._n += 1
        if self._n % 60 == 0:
            dt = time.time() - self._t0
            print(f"[POSE] ~{self._n/dt:.1f} fps")
            self._t0 = time.time()
            self._n = 0

        return vf

    def close(self):
        self.cam.stop()

# -------- aiohttp app / signaling --------
pcs = set()
track = None

async def index(_):
    return web.Response(content_type="text/html", text=INDEX_HTML)

async def offer(req):
    global track
    params = await req.json()
    pc = RTCPeerConnection()
    pcs.add(pc)
    print("[SIG] /offer received")

    if track is None:
        cam = PiCam(W, H, FPS)
        track = YoloPoseTrack(cam)
    pc.addTrack(track)

    @pc.on("connectionstatechange")
    async def _():
        print(f"[PC] {pc.connectionState}")
        if pc.connectionState in ("failed", "closed", "disconnected"):
            await pc.close()
            pcs.discard(pc)

    # (no audio; keep a sink to appease some browsers)
    bh = MediaBlackhole()
    await bh.start()

    await pc.setRemoteDescription(RTCSessionDescription(sdp=params["sdp"], type=params["type"]))
    ans = await pc.createAnswer()
    await pc.setLocalDescription(ans)
    return web.json_response({"sdp": pc.localDescription.sdp, "type": pc.localDescription.type})

async def on_shutdown(_):
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros, return_exceptions=True)
    pcs.clear()
    if track is not None:
        track.close()

def main():
    app = web.Application()
    app.on_shutdown.append(on_shutdown)
    app.router.add_get("/", index)
    app.router.add_post("/offer", offer)
    print(f"Open http://{HOST}:{PORT}/")
    web.run_app(app, host=HOST, port=PORT)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    main()
