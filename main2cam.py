import cv2
import numpy as np
import time
import serial

# =========================================================
# KONFIGURASI
# =========================================================
# --- Kamera ---
CAMERA_INDEX = 1                 # 0 = webcam laptop, 1 = USB eksternal (tergantung perangkat)
FRAME_W, FRAME_H = 640, 480      # spek kamera

# --- Serial ke Arduino MASTER ---
USE_SERIAL = True                # False -> uji deteksi saja tanpa Arduino
SERIAL_PORT = "COM3"
BAUDRATE = 115200
SEND_INTERVAL = 0.1              # detik (10 Hz)

# --- ROI ---
#   "none"    : tanpa ROI, seluruh frame dipakai
#   "horizon" : ROI persegi sederhana, bagian atas (langit/gedung) dibuang
#   "polygon" : ROI trapesium (proporsional terhadap ukuran frame)
ROI_MODE = "polygon"
HORIZON_RATIO = 0.5              # mode "horizon": buang 50% bagian atas frame
POLYGON_RATIO = [                # mode "polygon": (x/lebar, y/tinggi), urutan: kiri-bawah, kanan-bawah, kanan-atas, kiri-atas
    (-0.20, 1.00),               # x negatif = di luar frame (sengaja, supaya marka kiri tidak terpotong)
    (1.10, 1.00),
    (0.65, 0.45),
    (0.37, 0.45),
]
ROI_FALLBACK = True              # True -> jika ROI menghasilkan <2 garis, coba ulang hanya dengan membuang langit
                                 #         (supaya roda tidak mati total saat ROI meleset)

# --- Deteksi tepi & garis ---
HOUGH_THRESHOLD = 50
HOUGH_MIN_LINE = 15
HOUGH_MAX_GAP = 20               # 60 untuk 1280 px -> 30 untuk 640 px
MIN_ABS_SLOPE = 0.5              # buang garis yang terlalu landai (horizontal)
LANE_END_RATIO = 0.6             # ujung atas garis hijau berada di 60% tinggi frame

# --- Aturan keputusan & power (px untuk lebar 640) ---
CENTER_OFFSET = 0                # geser titik tengah kamera (px), + ke kanan
DEADBAND = 9                     # |error| <= 5 px -> STRAIGHT (setara ±9 px pada 1280)
MAX_ERROR = 40                   # error >= 40 px -> tikungan tajam (setara 80 px pada 1280)
MIN_POWER = 128
MAX_POWER = 255
SMOOTH_ALPHA = 0.5               # 1.0 = tanpa smoothing

SWAP_LEFT_RIGHT = False          # True kalau arah roda terbalik
SHOW_DEBUG_WINDOWS = False       # True -> tampilkan juga window Grayscale, Blur, Canny, ROI
PRINT_MASTER_REPLY = False       # True -> tampilkan balasan master di terminal

CMD_LEFT = '2' if SWAP_LEFT_RIGHT else '1'
CMD_RIGHT = '1' if SWAP_LEFT_RIGHT else '2'
LABELS = {'0': "LURUS", '1': "KIRI", '2': "KANAN"}


# =========================================================
# SERIAL
# =========================================================
ser = None
if USE_SERIAL:
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0)
        time.sleep(2)   # tunggu Arduino reset
        print(f"Serial terhubung: {SERIAL_PORT}")
    except serial.SerialException as e:
        print(f"[PERINGATAN] Gagal membuka {SERIAL_PORT}: {e}")
        print("Lanjut tanpa kirim perintah ke Arduino.")
        ser = None

last_send_time = 0.0
last_sent = "-"


def send_command(cmd, power):
    """Kirim '<cmd>,<power>\\n' ke master (dibatasi SEND_INTERVAL)."""
    global last_send_time, last_sent
    if ser is None:
        return
    now = time.time()
    if now - last_send_time >= SEND_INTERVAL:
        try:
            ser.write(f"{cmd},{int(power)}\n".encode())
            last_send_time = now
            last_sent = f"{cmd},{int(power)}"
        except serial.SerialException:
            pass


def send_now(cmd, power):
    """Kirim langsung tanpa pembatasan interval (pause / keluar)."""
    if ser is None:
        return
    try:
        ser.write(f"{cmd},{int(power)}\n".encode())
    except serial.SerialException:
        pass


def drain_serial():
    """Kosongkan buffer balasan master."""
    if ser is None:
        return
    try:
        if ser.in_waiting:
            msg = ser.read(ser.in_waiting).decode(errors="ignore")
            if PRINT_MASTER_REPLY:
                print("[MASTER]", msg.strip())
    except serial.SerialException:
        pass


# =========================================================
# KEPUTUSAN: ERROR (px) -> ARAH + POWER
# =========================================================
def compute_command(lane_error):
    a = abs(lane_error)
    if a <= DEADBAND:
        return '0', 0, "STRAIGHT"

    ratio = (min(a, MAX_ERROR) - DEADBAND) / float(MAX_ERROR - DEADBAND)
    power = int(MIN_POWER + ratio * (MAX_POWER - MIN_POWER))
    power = max(MIN_POWER, min(MAX_POWER, power))

    if lane_error < 0:
        return CMD_LEFT, power, "TURN LEFT"
    return CMD_RIGHT, power, "TURN RIGHT"


# =========================================================
# DETEKSI MARKA
# =========================================================
def apply_roi(edges):
    """Terapkan ROI sesuai ROI_MODE. Mode 'none' mengembalikan citra apa adanya."""
    if ROI_MODE == "none":
        return edges

    h, w = edges.shape[:2]
    mask = np.zeros_like(edges)

    if ROI_MODE == "horizon":
        mask[int(h * HORIZON_RATIO):, :] = 255
    elif ROI_MODE == "polygon":
        pts = np.array([[(int(x * w), int(y * h)) for x, y in POLYGON_RATIO]], np.int32)
        cv2.fillPoly(mask, pts, 255)
    else:
        return edges

    return cv2.bitwise_and(edges, mask)


def draw_roi_outline(image):
    """Gambar garis kuning putus-putus batas ROI supaya mudah disetel."""
    h, w = image.shape[:2]
    if ROI_MODE == "polygon":
        pts = np.array([(int(x * w), int(y * h)) for x, y in POLYGON_RATIO], np.int32)
        cv2.polylines(image, [pts], True, (0, 255, 255), 1)
    elif ROI_MODE == "horizon":
        y = int(h * HORIZON_RATIO)
        cv2.line(image, (0, y), (w, y), (0, 255, 255), 1)


def make_coordinates(image, line_parameters):
    slope, intercept = line_parameters
    y1 = image.shape[0]
    y2 = int(y1 * LANE_END_RATIO)
    if slope == 0:
        slope = 0.1
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return [x1, y1, x2, y2]


def average_slope_intercept(image, lines):
    left_fit = []
    right_fit = []

    if lines is None:
        return []

    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)
        if x1 == x2:
            continue          # garis vertikal sempurna, polyfit tidak stabil
        parameters = np.polyfit((x1, x2), (y1, y2), 1)
        slope = parameters[0]
        intercept = parameters[1]

        if abs(slope) < MIN_ABS_SLOPE:
            continue

        if slope < 0 and x1 < image.shape[1] / 2:
            left_fit.append((slope, intercept))
        elif slope > 0 and x1 > image.shape[1] / 2:
            right_fit.append((slope, intercept))

    lane_lines = []
    if len(left_fit) > 0:
        lane_lines.append(make_coordinates(image, np.average(left_fit, axis=0)))
    if len(right_fit) > 0:
        lane_lines.append(make_coordinates(image, np.average(right_fit, axis=0)))

    return lane_lines


# =========================================================
# BUKA KAMERA
# =========================================================
cap = None
for api, api_name in ((cv2.CAP_DSHOW, "DSHOW"), (cv2.CAP_MSMF, "MSMF"), (cv2.CAP_ANY, "ANY")):
    trial = cv2.VideoCapture(CAMERA_INDEX, api)
    if trial.isOpened():
        ok, test_frame = trial.read()          # sama seperti camera_scan.py: baca dulu
        if ok:
            cap = trial
            print(f"Kamera index {CAMERA_INDEX} terbuka (backend {api_name}), "
                  f"resolusi asli {test_frame.shape[1]}x{test_frame.shape[0]}.")
            break
    trial.release()

if cap is None:
    print(f"[ERROR] Kamera index {CAMERA_INDEX} tidak bisa dibuka dengan backend apa pun.")
    print("Jalankan camera_scan.py untuk mencari index kamera yang benar.")
    if ser is not None:
        ser.close()
    raise SystemExit

print("\n--- Kontrol ---")
print("[Spasi] Pause / Resume (roda dikembalikan lurus saat pause)")
print("[s]     Tampilkan / Sembunyikan garis panduan")
print("[r]     Tampilkan / Sembunyikan garis batas ROI")
print("[q]     Keluar")
print(f"ROI_MODE = {ROI_MODE}\n")

paused = False
show_guides = True
show_roi_outline = True
last_combo = None

smoothed_error = 0.0
prev_cmd = None
prev_power = 0

while True:
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord(' '):
        paused = not paused
        if paused:
            send_now('0', 0)
    elif key == ord('s'):
        show_guides = not show_guides
    elif key == ord('r'):
        show_roi_outline = not show_roi_outline

    if paused:
        if last_combo is not None:
            cv2.imshow("Lane Following", last_combo)
        drain_serial()
        continue

    start_time = time.time()

    ret, frame = cap.read()
    if not ret:
        print("\nGagal membaca frame dari kamera (kamera terputus?).")
        send_now('0', 0)     # kamera putus -> roda langsung lurus
        break

    image = cv2.resize(frame, (FRAME_W, FRAME_H))
    h, w = image.shape[:2]

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (9, 9), 0)

    high_thresh, _ = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    batas_atas = high_thresh * 2.0
    low_thresh = 0.5 * high_thresh
    batas_bawah = low_thresh * 1.5
    canny = cv2.Canny(blur, batas_bawah, batas_atas)

    roi = apply_roi(canny)
    lines = cv2.HoughLinesP(roi, 2, np.pi / 180, HOUGH_THRESHOLD,
                            minLineLength=HOUGH_MIN_LINE, maxLineGap=HOUGH_MAX_GAP)
    averaged_lines = average_slope_intercept(image, lines)

    used_fallback = False
    if ROI_FALLBACK and ROI_MODE != "none" and len(averaged_lines) < 2:
        edges_fb = canny.copy()
        edges_fb[:int(h * HORIZON_RATIO), :] = 0       # buang bagian langit saja
        lines_fb = cv2.HoughLinesP(edges_fb, 1, np.pi / 180, HOUGH_THRESHOLD,
                                   minLineLength=HOUGH_MIN_LINE, maxLineGap=HOUGH_MAX_GAP)
        fb_lines = average_slope_intercept(image, lines_fb)
        if len(fb_lines) == 2:
            averaged_lines = fb_lines
            used_fallback = True

    line_image = np.zeros_like(image)
    center_x = (w // 2) + CENTER_OFFSET

    cmd, power, text = '0', 0, f"NO LANE ({len(averaged_lines)}/2 garis)"
    lane_error = 0

    for line in averaged_lines:
        x1, y1, x2, y2 = [int(v) for v in line]
        cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 3)

    if len(averaged_lines) == 2:
        left_x2 = averaged_lines[0][2]
        right_x2 = averaged_lines[1][2]
        lane_center = int((left_x2 + right_x2) / 2)
        raw_error = lane_center - center_x

        smoothed_error = SMOOTH_ALPHA * raw_error + (1 - SMOOTH_ALPHA) * smoothed_error
        lane_error = int(round(smoothed_error))

        if show_guides:
            y_top = int(h * 0.55)
            y_mid = int(h * 0.76)
            cv2.line(line_image, (lane_center, y_top), (lane_center, h), (255, 0, 0), 3)
            cv2.line(line_image, (center_x, y_top), (center_x, h), (0, 0, 255), 3)
            cv2.line(line_image, (center_x, y_mid), (lane_center, y_mid), (0, 255, 255), 2)

        cmd, power, text = compute_command(lane_error)
    else:
        smoothed_error = 0.0

    # --- Kirim ke Arduino master ---
    send_command(cmd, power)
    drain_serial()

    # --- Terminal: cetak hanya saat arah berubah / power berubah >= 20 ---
    if cmd != prev_cmd or abs(power - prev_power) >= 20:
        print(f"CMD {cmd} ({LABELS[cmd]}) | power {power} | {text} | error {lane_error} px")
        prev_cmd = cmd
        prev_power = power

    # --- Tampilan ---
    proc_time = (time.time() - start_time) * 1000
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(image, text, (15, 30), font, 0.8, (0, 255, 255), 2)
    cv2.putText(image, f"Error: {lane_error} px", (15, 58), font, 0.5, (255, 255, 255), 1)
    cv2.putText(image, f"Power: {power} / 255", (15, 80), font, 0.5, (0, 255, 0), 1)
    cv2.putText(image, f"Proc: {proc_time:.1f} ms", (15, 102), font, 0.5, (255, 150, 0), 1)
    serial_status = "ON" if ser is not None else "OFF"
    cv2.putText(image, f"Serial: {serial_status} | TX: {last_sent} | ROI: {ROI_MODE}{' (FALLBACK)' if used_fallback else ''}", (15, 124),
                font, 0.45, (200, 200, 200), 1)

    if show_roi_outline:
        draw_roi_outline(image)

    combo = cv2.addWeighted(image, 0.8, line_image, 1, 1)
    last_combo = combo
    cv2.imshow("Lane Following", combo)

    if SHOW_DEBUG_WINDOWS:
        cv2.imshow("Grayscale", gray)
        cv2.imshow("Gaussian Blur", blur)
        cv2.imshow("Canny Edge", canny)
        cv2.imshow("ROI / Mask", roi)

# =========================================================
# BERSIH-BERSIH
# =========================================================
cap.release()
cv2.destroyAllWindows()

if ser is not None:
    send_now('0', 0)         # roda kembali lurus, power 0
    time.sleep(0.2)
    drain_serial()
    try:
        ser.close()
    except serial.SerialException:
        pass

print("Selesai.")