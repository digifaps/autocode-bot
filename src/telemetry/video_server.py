#!/usr/bin/env python3
"""MJPEG video streaming server with proper threading."""

import cv2
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn
import time
import sys

sys.stdout = sys.stderr

def gst_pipeline(sensor_id, width=640, height=480, fps=15):
    return (
        f'nvarguscamerasrc sensor-id={sensor_id} ! '
        f'video/x-raw(memory:NVMM), width={width}, height={height}, '
        f'framerate={fps}/1 ! '
        'nvvidconv ! video/x-raw, format=BGRx ! '
        'videoconvert ! video/x-raw, format=BGR ! appsink drop=1'
    )

cap_left = None
cap_right = None
frame_left = None
frame_right = None
lock_left = threading.Lock()
lock_right = threading.Lock()

def init_cameras():
    global cap_left, cap_right
    
    print('[CAM] Init left...', flush=True)
    cap_left = cv2.VideoCapture(gst_pipeline(0), cv2.CAP_GSTREAMER)
    if cap_left.isOpened():
        ret, f = cap_left.read()
        print(f'[CAM] Left: {"OK" if ret else "FAIL"}', flush=True)
    else:
        cap_left = None
        print('[CAM] Left: FAILED', flush=True)
    
    print('[CAM] Init right...', flush=True)
    cap_right = cv2.VideoCapture(gst_pipeline(1), cv2.CAP_GSTREAMER)
    if cap_right.isOpened():
        ret, f = cap_right.read()
        print(f'[CAM] Right: {"OK" if ret else "FAIL"}', flush=True)
    else:
        cap_right = None
        print('[CAM] Right: FAILED', flush=True)

def capture_left():
    global frame_left
    while True:
        if cap_left and cap_left.isOpened():
            ret, frame = cap_left.read()
            if ret:
                with lock_left:
                    frame_left = frame.copy()
        time.sleep(0.033)

def capture_right():
    global frame_right
    while True:
        if cap_right and cap_right.isOpened():
            ret, frame = cap_right.read()
            if ret:
                with lock_right:
                    frame_right = frame.copy()
        time.sleep(0.033)

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True

class StreamHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass
    
    def do_GET(self):
        if self.path == '/left':
            self.stream_camera('left')
        elif self.path == '/right':
            self.stream_camera('right')
        elif self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            html = '<html><body style="background:#000;margin:0;display:flex;">'
            html += '<img src="/left" style="width:50%">'
            html += '<img src="/right" style="width:50%">'
            html += '</body></html>'
            self.wfile.write(html.encode())
        else:
            self.send_error(404)
    
    def stream_camera(self, cam):
        self.send_response(200)
        self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        
        lock = lock_left if cam == 'left' else lock_right
        
        try:
            while True:
                with lock:
                    frame = frame_left.copy() if cam == 'left' and frame_left is not None else None
                    if cam == 'right' and frame_right is not None:
                        frame = frame_right.copy()
                
                if frame is not None:
                    _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
                    self.wfile.write(b'--frame\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                    self.wfile.write(jpeg.tobytes())
                    self.wfile.write(b'\r\n')
                
                time.sleep(0.1)  # 10 fps per stream
        except:
            pass

if __name__ == '__main__':
    print('[VIDEO] Starting...', flush=True)
    init_cameras()
    
    # Separate capture threads for each camera
    if cap_left:
        t1 = threading.Thread(target=capture_left, daemon=True)
        t1.start()
        print('[VIDEO] Left capture thread started', flush=True)
    
    if cap_right:
        t2 = threading.Thread(target=capture_right, daemon=True)
        t2.start()
        print('[VIDEO] Right capture thread started', flush=True)
    
    if cap_left or cap_right:
        print('[VIDEO] Threaded server on http://0.0.0.0:8081', flush=True)
        server = ThreadedHTTPServer(('0.0.0.0', 8081), StreamHandler)
        server.serve_forever()
    else:
        print('[VIDEO] No cameras!', flush=True)
