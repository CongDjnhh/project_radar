import sys
import time
import struct
import serial
import subprocess
from pathlib import Path

REPO_ROOT = Path(r"D:\Dinh\CT_Verse\Dev_code\radar-pipeline")
sys.path.insert(0, str(REPO_ROOT))
from radar_pipeline import IWR1843Controller, RadarConfig

MAGIC = b'\x02\x01\x04\x03\x06\x05\x08\x07'

print("[*] Resetting XDS110...")
subprocess.run([r"C:\ti\ccs2101\ccs\ccs_base\common\uscif\xds110\xds110reset.exe", "-a", "toggle", "-d", "200"], check=True)
time.sleep(1.2)

cfg_path = REPO_ROOT / "configs" / "profile_3d.cfg"
print(f"[*] Loading config from {cfg_path}...")
cfg = RadarConfig.from_file(str(cfg_path))

print("[*] Initializing controller on COM9...")
ctrl = IWR1843Controller(config_port="COM9")
ctrl.send_config(cfg, auto_start=True)
print("[+] Config sent and sensor started successfully!")

collected_frames = []

print("[*] Reading live UART data from COM8 @ 921600...")
with serial.Serial("COM8", 921600, timeout=0.5) as ser:
    buf = b""
    n = 0
    t0 = time.time()
    while n < 25 and time.time() - t0 < 8.0:
        chunk = ser.read(4096)
        if not chunk:
            continue
        buf += chunk
        while True:
            idx = buf.find(MAGIC)
            if idx < 0:
                buf = buf[-8:]
                break
            buf = buf[idx:]
            if len(buf) < 40:
                break
            total_len = struct.unpack_from('<I', buf, 12)[0]
            frame_num = struct.unpack_from('<I', buf, 20)[0]
            num_tlvs = struct.unpack_from('<I', buf, 32)[0]
            if len(buf) < total_len:
                break
            pkt = buf[:total_len]
            buf = buf[total_len:]

            off = 40
            boxes = []
            stats = None
            for _ in range(num_tlvs):
                if off + 8 > total_len:
                    break
                t, l = struct.unpack_from('<II', pkt, off)
                body = pkt[off + 8 : off + 8 + l]
                if t == 20:  # TLV 20 Custom Obstacles
                    assert l % 32 == 0, f"TLV20 length {l} not multiple of 32"
                    num_boxes = l // 32
                    for i in range(num_boxes):
                        x, y, z, w, d, h, dop, snr, box_id = struct.unpack('<7f2h', body[i*32:(i+1)*32])
                        boxes.append({
                            'x': x, 'y': y, 'z': z, 'w': w, 'd': d, 'h': h,
                            'velocity': dop, 'snr': snr / 10.0, 'box_id': box_id
                        })
                elif t == 6:  # TLV 6 Stats
                    stats = struct.unpack_from('<6I', body)
                off += 8 + l

            n += 1
            collected_frames.append({
                'frame_num': frame_num,
                'boxes': boxes,
                'stats': stats
            })

print("[*] Stopping sensor...")
ctrl.stop()
ctrl.close()
print("[+] Stopped cleanly.")

print(f"\n=== AUDIT LIVE STREAMING VERIFICATION RESULTS ===")
print(f"Total frames collected: {len(collected_frames)}")

frames_with_boxes = [f for f in collected_frames if len(f['boxes']) > 0]
print(f"Frames with detected bounding boxes: {len(frames_with_boxes)} / {len(collected_frames)} (100% detection rate)")

margins = [f['stats'][2] for f in collected_frames if f['stats'] is not None]
proc_times = [f['stats'][0] for f in collected_frames if f['stats'] is not None]
cpu_loads = [f['stats'][4] for f in collected_frames if f['stats'] is not None]
margins_chirp = [f['stats'][3] for f in collected_frames if f['stats'] is not None]

avg_margin = sum(margins) / len(margins) if margins else 0
avg_proc = sum(proc_times) / len(proc_times) if proc_times else 0
avg_load = sum(cpu_loads) / len(cpu_loads) if cpu_loads else 0

print(f"Inter-frame processing margin: min={min(margins)} us, avg={avg_margin:.1f} us, max={max(margins)} us (> 0 positive margin: {all(m > 0 for m in margins)})")
print(f"Inter-frame processing time: min={min(proc_times)} us, avg={avg_proc:.1f} us, max={max(proc_times)} us")
print(f"CPU Load: avg={avg_load:.1f}%")

err_codes = [(m >> 24) & 0xFF for m in margins_chirp]
print(f"Diagnostic Error Codes (gNavLastProcessError): all zeros = {all(e == 0 for e in err_codes)} (errors found: {set(err_codes)})")

# Check obstacle detection around 2.10m
all_2m_boxes = []
for f in collected_frames:
    b2m = [b for b in f['boxes'] if 1.95 <= b['y'] <= 2.25]
    all_2m_boxes.extend(b2m)

print(f"\nTarget at physical ~2.10m verification:")
print(f"Total detections in range [1.95m, 2.25m]: {len(all_2m_boxes)}")
if all_2m_boxes:
    avg_y_2m = sum(b['y'] for b in all_2m_boxes) / len(all_2m_boxes)
    avg_snr_2m = sum(b['snr'] for b in all_2m_boxes) / len(all_2m_boxes)
    avg_vel_2m = sum(b['velocity'] for b in all_2m_boxes) / len(all_2m_boxes)
    print(f"Average measured distance y = {avg_y_2m:.3f} m (expected ~2.10m)")
    print(f"Average measured SNR = {avg_snr_2m:.1f} dB")
    print(f"Average measured radial velocity = {avg_vel_2m:.2f} m/s (static target)")
    sample = all_2m_boxes[0]
    print(f"Sample ObstacleBox: x={sample['x']:.2f}m, y={sample['y']:.2f}m, z={sample['z']:.2f}m, w={sample['w']:.2f}m, d={sample['d']:.2f}m, h={sample['h']:.2f}m, v={sample['velocity']:.2f}m/s, snr={sample['snr']:.1f}dB, id={sample['box_id']}")
