#!/usr/bin/env python3
"""Rosbridge WebSocket server with correct ROS1 message format."""

import asyncio
import json
import math
import random
import time
import websockets

imu_counter = 0

def get_imu_data():
    """Fake IMU: steady gravity + small noise (no intentional oscillation)."""
    global imu_counter
    imu_counter += 1
    now = time.time()
    # Very small random noise (heavily filtered look) so demo stays steady
    r = lambda scale: (random.random() - 0.5) * scale
    return {
        'header': {
            'seq': imu_counter,
            'stamp': {'secs': int(now), 'nsecs': int((now % 1) * 1e9)},
            'frame_id': 'imu_link'
        },
        'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
        'orientation_covariance': [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        'angular_velocity': {
            'x': r(0.005), 'y': r(0.005), 'z': r(0.005)
        },
        'angular_velocity_covariance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        'linear_acceleration': {
            'x': r(0.02), 'y': r(0.02), 'z': 9.81 + r(0.01)
        },
        'linear_acceleration_covariance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    }


def get_fake_scan():
    """Fake LaserScan for demo when no real /scan is available (e.g. mock server)."""
    n = 360
    angle_min = -math.pi
    angle_max = math.pi
    angle_increment = (angle_max - angle_min) / n
    t = time.time()
    ranges = []
    for i in range(n):
        angle = angle_min + i * angle_increment
        # Fake "room": walls at ~3–5 m with a gap, plus some noise
        base = 4.0 + 0.5 * math.sin(angle * 2) + 0.3 * math.sin(t + angle * 3)
        r = base + (0.2 * (hash(str(i) + str(int(t))) % 100) / 100.0)
        ranges.append(round(r, 3))
    return {
        'header': {'stamp': {'secs': int(t), 'nsecs': int((t % 1) * 1e9)}, 'frame_id': 'base_laser'},
        'angle_min': angle_min,
        'angle_max': angle_max,
        'angle_increment': angle_increment,
        'range_min': 0.1,
        'range_max': 12.0,
        'ranges': ranges,
        'intensities': [],
    }

# ROS1-style message definitions
IMU_TYPEDEF = '''std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
================================================================================
MSG: std_msgs/Header
uint32 seq
time stamp
string frame_id
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z'''

async def handler(websocket):
    print(f'[CONN] Client: {websocket.remote_address}')
    stream_task = None
    
    async def stream_data():
        print('[STREAM] Starting (IMU + fake /scan)')
        try:
            scan_interval = 0
            while True:
                msg = {'op': 'publish', 'topic': '/imu/data_raw', 'msg': get_imu_data()}
                await websocket.send(json.dumps(msg))
                scan_interval += 1
                if scan_interval >= 2:  # ~5 Hz fake /scan
                    scan_interval = 0
                    await websocket.send(json.dumps({
                        'op': 'publish', 'topic': '/scan', 'msg': get_fake_scan()
                    }))
                await asyncio.sleep(0.1)
        except Exception as e:
            print(f'[STREAM] Stopped: {e}')
    
    try:
        async for message in websocket:
            data = json.loads(message)
            op = data.get('op', '')
            
            if op == 'call_service':
                service = data.get('service', '')
                call_id = data.get('id', '')
                print(f'[SVC] {service}')
                
                response = {
                    'op': 'service_response',
                    'id': call_id,
                    'service': service,
                    'result': True,
                    'values': {
                        'topics': ['/imu/data_raw'],
                        'types': ['sensor_msgs/Imu'],
                        'typedefs_full_text': [IMU_TYPEDEF]
                    }
                }
                await websocket.send(json.dumps(response))
                
                if stream_task is None:
                    stream_task = asyncio.create_task(stream_data())
                    
            elif op == 'subscribe':
                print(f'[SUB] {data.get("topic", "")}')
                if stream_task is None:
                    stream_task = asyncio.create_task(stream_data())
                    
    except websockets.exceptions.ConnectionClosed:
        print('[CONN] Disconnected')
    finally:
        if stream_task:
            stream_task.cancel()

async def main():
    print('Rosbridge Server on ws://0.0.0.0:9090')
    async with websockets.serve(handler, '0.0.0.0', 9090):
        await asyncio.Future()

if __name__ == '__main__':
    asyncio.run(main())
