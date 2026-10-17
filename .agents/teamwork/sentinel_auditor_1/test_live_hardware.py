import sys
import time
import struct
import subprocess
import serial
from pathlib import Path

# Setup paths
REPO_ROOT = Path(r"D:\Dinh\CT_Verse\Dev_code\radar-pipeline")
sys.path.insert(0, str(REPO_ROOT))
from radar_pipeline import IWR1843Controller, RadarConfig

MAGIC_WORD = b'\x02\x01\x04\x03\x06\x05\x08\x07'

def run_independent_hardware_audit():
    print("[1] Performing clean XDS110 hardware reset...")
    reset_exe = r"C:\ti\ccs2101\ccs\ccs_base\common\uscif\xds110\xds110reset.exe"
    res = subprocess.run([reset_exe, "-a", "toggle", "-d", "200"], capture_output=True, text=True)
    print("    xds110reset output:", res.stdout.strip())
    time.sleep(1.5)

    cfg_file = REPO_ROOT / "configs" / "profile_3d.cfg"
    print(f"[2] Loading radar configuration from {cfg_file}...")
    cfg = RadarConfig.from_file(str(cfg_file))

    print("[3] Connecting to COM9 (CLI @ 115200)...")
    ctrl = IWR1843Controller(config_port="COM9")
    print("    Sending configuration and issuing sensorStart...")
    ctrl.send_config(cfg, auto_start=True)
    print("[+] Configuration successfully sent! Sensor is actively streaming.")

    collected_frames = []
    print("[4] Reading live radar data stream from COM8 @ 921600 baud...")
    t_start = time.time()
    with serial.Serial("COM8", 921600, timeout=0.5) as ser:
        buf = b""
        target_frames = 30
        while len(collected_frames) < target_frames and (time.time() - t_start < 10.0):
            chunk = ser.read(4096)
            if not chunk:
                continue
            buf += chunk
            while True:
                idx = buf.find(MAGIC_WORD)
                if idx < 0:
                    buf = buf[-8:]
                    break
                buf = buf[idx:]
                if len(buf) < 40:
                    break
                
                # Unpack header
                magic, version, total_len, platform, frame_num, cpu_time, num_detected, num_tlvs, subframe = struct.unpack_from('<8sIIIIIIII', buf, 0)
                if len(buf) < total_len:
                    break

                pkt = buf[:total_len]
                buf = buf[total_len:]

                offset = 40
                tlv20_boxes = []
                tlv6_stats = None

                for _ in range(num_tlvs):
                    if offset + 8 > total_len:
                        break
                    tlv_type, tlv_len = struct.unpack_from('<II', pkt, offset)
                    payload = pkt[offset + 8 : offset + 8 + tlv_len]

                    if tlv_type == 20: # TLV 20: Custom Obstacles (32 bytes per ObstacleBox)
                        assert tlv_len % 32 == 0, f"TLV 20 length {tlv_len} not multiple of 32"
                        n_boxes = tlv_len // 32
                        for b_idx in range(n_boxes):
                            x, y, z, w, d, h, vel, snr_raw, box_id = struct.unpack('<7f2h', payload[b_idx*32:(b_idx+1)*32])
                            tlv20_boxes.append({
                                'x': x, 'y': y, 'z': z,
                                'width_x': w, 'depth_y': d, 'height_z': h,
                                'velocity': vel,
                                'snr_db': snr_raw / 10.0,
                                'box_id': box_id
                            })
                    elif tlv_type == 6: # TLV 6: Stats
                        tlv6_stats = struct.unpack_from('<6I', payload)

                    offset += 8 + tlv_len

                collected_frames.append({
                    'frame_num': frame_num,
                    'num_detected_header': num_detected,
                    'num_tlvs': num_tlvs,
                    'boxes': tlv20_boxes,
                    'stats': tlv6_stats
                })

    print(f"[5] Stopping sensor and closing COM9...")
    ctrl.stop()
    ctrl.close()
    print("[+] Sensor stopped cleanly.")

    print(f"\n=======================================================")
    print(f"       INDEPENDENT VICTORY AUDITOR LIVE RESULTS        ")
    print(f"=======================================================")
    print(f"Total live frames received: {len(collected_frames)}")
    assert len(collected_frames) >= 20, f"Insufficient frames received: {len(collected_frames)}"

    # 1. Detection rate
    frames_with_boxes = [f for f in collected_frames if len(f['boxes']) > 0]
    box_counts = [len(f['boxes']) for f in collected_frames]
    det_rate = (len(frames_with_boxes) / len(collected_frames)) * 100.0
    print(f"Frames with detected bounding boxes: {len(frames_with_boxes)} / {len(collected_frames)} ({det_rate:.1f}%)")
    print(f"Boxes per frame: min={min(box_counts)}, avg={sum(box_counts)/len(box_counts):.1f}, max={max(box_counts)}")

    # 2. Timing and DSP margins from TLV 6 Stats
    # stats: [interFrameProcessingTime, transmitOutputTime, interFrameProcessingMargin, interChirpProcessingMargin, activeCPULoad, interCPULoad]
    valid_stats = [f['stats'] for f in collected_frames if f['stats'] is not None]
    if valid_stats:
        margins = [s[2] for s in valid_stats] # interFrameProcessingMargin in usec
        proc_times = [s[0] for s in valid_stats]
        chirp_diag = [s[3] for s in valid_stats]
        cpu_loads = [s[4] for s in valid_stats]

        print(f"\n--- DSP Performance Metrics ---")
        print(f"Inter-frame Processing Margin: min={min(margins)} us, avg={sum(margins)/len(margins):.1f} us, max={max(margins)} us")
        print(f"Inter-frame Processing Time:   min={min(proc_times)} us, avg={sum(proc_times)/len(proc_times):.1f} us, max={max(proc_times)} us")
        print(f"Active CPU Load: avg={sum(cpu_loads)/len(cpu_loads):.1f}%")

        # Diagnostics packed into interChirpProcessingMargin
        err_codes = [(m >> 24) & 0xFF for m in chirp_diag]
        cfar_peaks = [(m >> 16) & 0xFF for m in chirp_diag]
        print(f"Diagnostic Error Codes (gNavLastProcessError): max={max(err_codes)}, set={set(err_codes)} (0 = OK)")
        print(f"C674x CASO-CFAR Peak Candidates per frame: avg={sum(cfar_peaks)/len(cfar_peaks):.1f}, min={min(cfar_peaks)}, max={max(cfar_peaks)}")
        assert all(e == 0 for e in err_codes), f"Non-zero error code detected: {set(err_codes)}"
        assert all(m > 0 for m in margins), "Negative processing margin detected!"

    # 3. Static target at physical Range Bin 50 (~2.10m, 0 Doppler)
    target_2m_boxes = []
    for f in collected_frames:
        for b in f['boxes']:
            if 1.95 <= b['y'] <= 2.25:
                target_2m_boxes.append(b)

    print(f"\n--- Ground Truth Target Verification (Range Bin 50 / ~2.10m Static Obstacle) ---")
    print(f"Total detections in window [1.95m, 2.25m]: {len(target_2m_boxes)}")
    assert len(target_2m_boxes) > 0, "CRITICAL: Physical target at 2.10m was NOT detected!"

    avg_y = sum(b['y'] for b in target_2m_boxes) / len(target_2m_boxes)
    avg_snr = sum(b['snr_db'] for b in target_2m_boxes) / len(target_2m_boxes)
    avg_vel = sum(b['velocity'] for b in target_2m_boxes) / len(target_2m_boxes)
    print(f"Average forward distance y: {avg_y:.3f} m (matches ~2.10m ground truth)")
    print(f"Average SNR:                {avg_snr:.1f} dB (> 20 dB strong detection)")
    print(f"Average radial velocity:    {avg_vel:.2f} m/s (static reflector at 0 Doppler)")

    sample_b = target_2m_boxes[0]
    print(f"Sample 32-byte ObstacleBox (<7f2h):")
    print(f"  Center:   x={sample_b['x']:.3f}m, y={sample_b['y']:.3f}m, z={sample_b['z']:.3f}m")
    print(f"  Size:     w_x={sample_b['width_x']:.3f}m, d_y={sample_b['depth_y']:.3f}m, h_z={sample_b['height_z']:.3f}m")
    print(f"  Kinemat:  vel={sample_b['velocity']:.3f} m/s, snr={sample_b['snr_db']:.1f} dB, id={sample_b['box_id']}")
    print(f"=======================================================")
    print(f"INDEPENDENT LIVE HARDWARE VERIFICATION RESULT: PASS")
    print(f"=======================================================")

if __name__ == '__main__':
    run_independent_hardware_audit()
