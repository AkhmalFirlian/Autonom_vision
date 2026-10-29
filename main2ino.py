import cv2
import numpy as np
import time
import serial

# =========================================================
# KONFIGURASI
# =========================================================
VIDEO_PATH = r"D:\College\Autonom Car\Road\POV11.mp4"   # ganti sesuai video / 0 untuk webcam
LOOP_VIDEO = True            # True -> video diulang dari awal saat selesai

USE_SERIAL = True            # False -> uji deteksi saja tanpa Arduino
SERIAL_PORT = "COM3"         # sesuaikan dengan port Arduino master (tutup Serial Monitor Arduino IDE!)
BAUDRATE = 115200            # harus sama dengan Serial.begin() di master
SEND_INTERVAL = 0.1          # detik (10 Hz)

# Protokol ke Arduino: tiap SEND_INTERVAL dikirim satu baris berisi nilai error (px), mis. "-45\n"
# Arduino yang memutuskan: error > 15 -> kanan (2), error < -15 -> kiri (1), selain itu lurus (0)
TURN_THRESHOLD = 15          # px, hanya untuk teks di layar (keputusan sebenarnya ada di Arduino)
ERROR_LIMIT = 999            # px, error dipotong ke rentang -999..999 sebelum dikirim
CMD_MAP = {"STRAIGHT": '0', "TURN LEFT": '1', "TURN RIGHT": '2'}   # hanya untuk tampilan/terminal

PRINT_MASTER_REPLY = True    # True -> tampilkan balasan master ("Kirim: error ...") di terminal

# --- Geometri ---
FRAME_W, FRAME_H = 1280, 720
Y_TOP = 0                    # ujung atas garis yang digambar (0 = sampai batas atas layar)
Y_REF = int(FRAME_H * 0.6)   # baris acuan untuk menghitung lane_center & error

# --- ROI (koordinat ternormalisasi: x/FRAME_W, y/FRAME_H) ---
ROI_POLYGON = [(0.0, 1.0), (1.0, 1.0), (0.78, 0.30), (0.12, 0.30)]

# =========================================================
# SUMBER TEPI (EDGE)
# =========================================================
#   "gray"          -> Canny pada citra grayscale (generik)
#   "color_mask"    -> segmentasi warna (HSV) dulu, lalu Canny pada mask (stabil untuk lakban kuning)
#   "gray_in_color" -> Canny grayscale, hanya tepi di sekitar area berwarna
EDGE_SOURCE = "color_mask"

# --- Canny (dipakai untuk "gray" dan "gray_in_color") ---
CANNY_AUTO = True
OTSU_LOW_RATIO = 0.5
OTSU_HIGH_RATIO = 1.0
CANNY_SMOOTH = 0.2
CANNY_LOW = 50
CANNY_HIGH = 150
BLUR_KERNEL = 9              # harus ganjil (1 = tanpa blur)

# --- Warna (HSV, OpenCV: H 0-179, S/V 0-255). Default: kuning ---
HSV_LOW = (15, 80, 80)
HSV_HIGH = (40, 255, 255)

# --- Parameter Hough ---
HOUGH_THRESHOLD = 30
MIN_LINE_LENGTH = 50
MAX_LINE_GAP = 40
MIN_ABS_SLOPE = 0.5

# --- Penolakan outlier per sisi (kiri/kanan) ---
OUTLIER_PX = 80
OUTLIER_SLOPE = 0.5

# --- Debug window ---
SHOW_DEBUG_WINDOWS = True    # toggle dengan [d]
SHOW_EXTRA_DEBUG = False
DEBUG_SCALE = 0.5

DEBUG_WINDOWS = ["Canny Edge", "Hough Lines (raw)", "Color Mask", "Grayscale", "Gaussian Blur", "ROI / Mask"]
TUNING_WINDOW = "Tuning"

P = {
    "auto": CANNY_AUTO,
    "canny_low": CANNY_LOW,
    "canny_high": CANNY_HIGH,
    "blur_k": BLUR_KERNEL,
    "hough_thr": HOUGH_THRESHOLD,
    "min_len": MIN_LINE_LENGTH,
    "max_gap": MAX_LINE_GAP,
    "outlier_px": OUTLIER_PX,
    "h_lo": HSV_LOW[0], "s_lo": HSV_LOW[1], "v_lo": HSV_LOW[2],
    "h_hi": HSV_HIGH[0], "s_hi": HSV_HIGH[1], "v_hi": HSV_HIGH[2],
}


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
last_sent_error = None


def send_command(error):
    """Kirim nilai error (px) ke master sebagai satu baris, mis. '-45\\n' (dibatasi SEND_INTERVAL)."""
    global last_send_time, last_sent_error
    if ser is None:
        return
    now = time.time()
    if now - last_send_time >= SEND_INTERVAL:
        e = int(max(-ERROR_LIMIT, min(ERROR_LIMIT, error)))
        try:
            ser.write(f"{e}\n".encode())
            last_send_time = now
            last_sent_error = e
        except serial.SerialException:
            pass


def drain_serial():
    """Kosongkan buffer balasan master supaya tidak penuh."""
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
# TRACKBAR TUNING
# =========================================================
TRACKBARS = [
    ("Canny Auto (0/1)", "auto", 1),
    ("Canny Low", "canny_low", 255),
    ("Canny High", "canny_high", 255),
    ("Blur (k=2n+1)", "blur_k", 15),
    ("Hough Thr", "hough_thr", 150),
    ("Min Len", "min_len", 300),
    ("Max Gap", "max_gap", 200),
    ("Outlier px", "outlier_px", 300),
    ("H low", "h_lo", 179), ("S low", "s_lo", 255), ("V low", "v_lo", 255),
    ("H high", "h_hi", 179), ("S high", "s_hi", 255), ("V high", "v_hi", 255),
]


def open_tuning():
    cv2.namedWindow(TUNING_WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(TUNING_WINDOW, 480, 520)
    for label, key, vmax in TRACKBARS:
        if key == "blur_k":
            init = (P["blur_k"] - 1) // 2
        else:
            init = int(P[key])
        cv2.createTrackbar(label, TUNING_WINDOW, init, vmax, lambda v: None)


def read_tuning():
    for label, key, vmax in TRACKBARS:
        pos = cv2.getTrackbarPos(label, TUNING_WINDOW)
        if key == "auto":
            P[key] = bool(pos)
        elif key == "blur_k":
            P[key] = 2 * pos + 1
        elif key == "hough_thr":
            P[key] = max(1, pos)
        else:
            P[key] = pos


def close_tuning():
    try:
        cv2.destroyWindow(TUNING_WINDOW)
    except cv2.error:
        pass


def print_params(last_thr):
    print("\n# --- Parameter saat ini (salin ke bagian KONFIGURASI) ---")
    print(f"CANNY_AUTO = {P['auto']}")
    print(f"CANNY_LOW = {P['canny_low']}")
    print(f"CANNY_HIGH = {P['canny_high']}")
    if last_thr is not None:
        print(f"# ambang Canny terakhir yang dipakai: low={last_thr[0]:.0f}, high={last_thr[1]:.0f}")
    print(f"BLUR_KERNEL = {P['blur_k']}")
    print(f"HSV_LOW = ({P['h_lo']}, {P['s_lo']}, {P['v_lo']})")
    print(f"HSV_HIGH = ({P['h_hi']}, {P['s_hi']}, {P['v_hi']})")
    print(f"HOUGH_THRESHOLD = {P['hough_thr']}")
    print(f"MIN_LINE_LENGTH = {P['min_len']}")
    print(f"MAX_LINE_GAP = {P['max_gap']}")
    print(f"OUTLIER_PX = {P['outlier_px']}\n")


# =========================================================
# DETEKSI TEPI
# =========================================================
def roi_points():
    return np.array([[(int(x * FRAME_W), int(y * FRAME_H)) for x, y in ROI_POLYGON]], np.int32)


ROI_MASK = np.zeros((FRAME_H, FRAME_W), np.uint8)
cv2.fillPoly(ROI_MASK, roi_points(), 255)

MORPH_KERNEL = np.ones((5, 5), np.uint8)
DILATE_KERNEL = np.ones((15, 15), np.uint8)
thr_state = {"low": None, "high": None}


def get_canny_thresholds(blur):
    """Ambang Canny: otomatis (Otsu di dalam ROI + smoothing antar frame) atau manual."""
    if not P["auto"]:
        return float(P["canny_low"]), float(P["canny_high"])

    pixels = blur[ROI_MASK > 0].reshape(-1, 1)
    otsu, _ = cv2.threshold(pixels, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    high = min(255.0, otsu * OTSU_HIGH_RATIO)
    low = min(high, otsu * OTSU_LOW_RATIO)

    if CANNY_SMOOTH > 0 and thr_state["low"] is not None:
        a = CANNY_SMOOTH
        low = a * low + (1 - a) * thr_state["low"]
        high = a * high + (1 - a) * thr_state["high"]
    thr_state["low"], thr_state["high"] = low, high
    return low, high


def detect_edges(image):
    """Kembalikan (canny, gray, blur, color_mask | None, (low, high) | None)."""
    k = P["blur_k"] if P["blur_k"] % 2 == 1 else P["blur_k"] + 1
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = gray if k <= 1 else cv2.GaussianBlur(gray, (k, k), 0)

    color_mask = None
    if EDGE_SOURCE in ("color_mask", "gray_in_color"):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        color_mask = cv2.inRange(hsv, (P["h_lo"], P["s_lo"], P["v_lo"]), (P["h_hi"], P["s_hi"], P["v_hi"]))
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, MORPH_KERNEL)
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, MORPH_KERNEL)

    if EDGE_SOURCE == "color_mask":
        canny = cv2.Canny(cv2.GaussianBlur(color_mask, (5, 5), 0), 50, 150)
        return canny, gray, blur, color_mask, None

    low, high = get_canny_thresholds(blur)
    canny = cv2.Canny(blur, low, high)

    if EDGE_SOURCE == "gray_in_color":
        canny = cv2.bitwise_and(canny, cv2.dilate(color_mask, DILATE_KERNEL))

    return canny, gray, blur, color_mask, (low, high)


# =========================================================
# DETEKSI MARKA
# =========================================================
def region_of_interest(image):
    return cv2.bitwise_and(image, ROI_MASK)


def make_coordinates(image, line_parameters, y_top=Y_TOP):
    """Garis dari dasar frame (y1) sampai y_top."""
    slope, intercept = line_parameters
    y1 = image.shape[0]
    y2 = y_top
    if slope == 0:
        slope = 0.1
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return [x1, y1, x2, y2]


def x_at(line, y):
    """x pada ketinggian y untuk garis [x1, y1, x2, y2]."""
    x1, y1, x2, y2 = line
    if y2 == y1:
        return x1
    return x1 + (x2 - x1) * (y - y1) / (y2 - y1)


def classify_segment(x1, y1, x2, y2, width):
    """Kembalikan 'left' / 'right' / None. Memakai titik tengah segmen."""
    dx = x2 - x1
    if dx == 0:
        return None
    slope = (y2 - y1) / dx
    if abs(slope) < MIN_ABS_SLOPE:
        return None
    x_mid = (x1 + x2) / 2
    if slope < 0 and x_mid < width / 2:
        return "left"
    if slope > 0 and x_mid > width / 2:
        return "right"
    return None


def weighted_median(values, weights):
    order = np.argsort(values)
    v = np.asarray(values)[order]
    w = np.asarray(weights)[order]
    cum = np.cumsum(w)
    return v[np.searchsorted(cum, cum[-1] / 2.0)]


def fit_side(items):
    """
    items: list of (slope, intercept, length, x_ref, segment)
    Kembalikan ((slope, intercept) | None, inliers, outliers).
    """
    if not items:
        return None, [], []

    ms = np.array([it[0] for it in items])
    ls = np.array([it[2] for it in items])
    xs = np.array([it[3] for it in items])

    med_x = weighted_median(xs, ls)
    med_m = weighted_median(ms, ls)

    mask = (np.abs(xs - med_x) <= P["outlier_px"]) & (np.abs(ms - med_m) <= OUTLIER_SLOPE)
    if not mask.any():
        mask = np.abs(xs - med_x) <= P["outlier_px"]

    inliers = [it for it, ok in zip(items, mask) if ok]
    outliers = [it for it, ok in zip(items, mask) if not ok]

    m = np.average(ms[mask], weights=ls[mask])
    x_ref = np.average(xs[mask], weights=ls[mask])
    b = Y_REF - m * x_ref
    return (m, b), inliers, outliers


def average_slope_intercept(image, lines):
    """Kembalikan (lane_lines, debug)."""
    debug = {"left_in": [], "right_in": [], "outliers": []}
    if lines is None:
        return [], debug

    width = image.shape[1]
    left_items, right_items = [], []

    for line in lines:
        x1, y1, x2, y2 = [int(v) for v in line.reshape(4)]
        side = classify_segment(x1, y1, x2, y2, width)
        if side is None:
            continue
        slope = (y2 - y1) / (x2 - x1)
        intercept = y1 - slope * x1
        length = float(np.hypot(x2 - x1, y2 - y1))
        x_ref = (Y_REF - intercept) / slope
        item = (slope, intercept, length, x_ref, (x1, y1, x2, y2))
        (left_items if side == "left" else right_items).append(item)

    lane_lines = []
    for side, items in (("left", left_items), ("right", right_items)):
        params, inliers, outliers = fit_side(items)
        if params is not None:
            lane_lines.append(make_coordinates(image, params))
        debug[f"{side}_in"] = [it[4] for it in inliers]
        debug["outliers"] += [it[4] for it in outliers]

    return lane_lines, debug


def draw_raw_hough(clean_image, lines, debug):
    """
    Semua segmen mentah HoughLinesP.
      Biru = dipakai (kiri), Oranye = dipakai (kanan), Merah = outlier, Abu = ditolak filter.
      Garis putih tipis = batas ROI.
    """
    out = clean_image.copy()
    cv2.polylines(out, roi_points(), True, (255, 255, 255), 2)

    used = set(debug["left_in"]) | set(debug["right_in"]) | set(debug["outliers"])
    count = 0
    if lines is not None:
        count = len(lines)
        for line in lines:
            seg = tuple(int(v) for v in line.reshape(4))
            if seg not in used:
                cv2.line(out, seg[:2], seg[2:], (150, 150, 150), 1)

    for seg in debug["outliers"]:
        cv2.line(out, seg[:2], seg[2:], (0, 0, 255), 2)
    for seg in debug["left_in"]:
        cv2.line(out, seg[:2], seg[2:], (255, 100, 0), 3)
    for seg in debug["right_in"]:
        cv2.line(out, seg[:2], seg[2:], (0, 140, 255), 3)

    cv2.putText(out, f"Segmen Hough: {count}", (30, FRAME_H - 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
    return out


def show_scaled(name, img):
    if DEBUG_SCALE != 1.0:
        img = cv2.resize(img, None, fx=DEBUG_SCALE, fy=DEBUG_SCALE, interpolation=cv2.INTER_AREA)
    cv2.imshow(name, img)


def close_debug_windows():
    for name in DEBUG_WINDOWS:
        try:
            cv2.destroyWindow(name)
        except cv2.error:
            pass


# =========================================================
# MAIN PROGRAM
# =========================================================
cap = cv2.VideoCapture(VIDEO_PATH)

print("\n--- Kontrol ---")
print("[Spasi] Pause / Resume (roda dikembalikan lurus saat pause)")
print("[s]     Tampilkan / Sembunyikan garis panduan")
print("[d]     Tampilkan / Sembunyikan window debug (Canny, Hough raw, Color Mask)")
print("[t]     Buka / Tutup window Tuning (trackbar Canny, Hough, HSV)")
print("[p]     Cetak parameter saat ini ke terminal")
print("[q]     Keluar\n")

paused = False
show_guides = True
show_debug = SHOW_DEBUG_WINDOWS
tuning_open = False
last_combo = None
last_thr = None
prev_cmd = None   # untuk mencetak ke terminal hanya saat keputusan berubah

LABELS = {'0': "LURUS", '1': "KIRI", '2': "KANAN"}

while True:
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord(' '):
        paused = not paused
        if paused:
            send_command(0)
    elif key == ord('s'):
        show_guides = not show_guides
    elif key == ord('d'):
        show_debug = not show_debug
        if not show_debug:
            close_debug_windows()
    elif key == ord('t'):
        tuning_open = not tuning_open
        if tuning_open:
            open_tuning()
        else:
            close_tuning()
    elif key == ord('p'):
        print_params(last_thr)

    if tuning_open:
        read_tuning()

    if paused:
        if last_combo is not None:
            cv2.imshow("Lane Following", last_combo)
        drain_serial()
        continue

    start_time = time.time()

    ret, frame = cap.read()
    if not ret:
        if LOOP_VIDEO and isinstance(VIDEO_PATH, str):
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)   # kembali ke frame pertama
            prev_cmd = None
            ret, frame = cap.read()
            if not ret:
                # fallback: buka ulang video jika cap.set() tidak didukung codec
                cap.release()
                cap = cv2.VideoCapture(VIDEO_PATH)
                ret, frame = cap.read()
                if not ret:
                    print("\nGagal memutar ulang video.")
                    break
        else:
            print("\nVideo selesai diputar.")
            break

    image = cv2.resize(frame, (FRAME_W, FRAME_H))
    clean = image.copy()          # salinan tanpa teks, untuk window debug

    canny, gray, blur, color_mask, thr = detect_edges(image)
    last_thr = thr if thr is not None else last_thr

    roi = region_of_interest(canny)
    lines = cv2.HoughLinesP(roi, 1, np.pi / 180, P["hough_thr"],
                            minLineLength=P["min_len"], maxLineGap=P["max_gap"])
    averaged_lines, hough_debug = average_slope_intercept(image, lines)

    line_image = np.zeros_like(image)
    center_x = (image.shape[1] // 2) - 50

    text = "NO LANE"
    lane_error = 0       # 0 juga dikirim saat garis tidak terdeteksi -> roda lurus

    for line in averaged_lines:
        x1, y1, x2, y2 = [int(v) for v in line]
        cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 4)

    if len(averaged_lines) == 2:
        left_x = x_at(averaged_lines[0], Y_REF)
        right_x = x_at(averaged_lines[1], Y_REF)
        lane_center = int((left_x + right_x) / 2)
        lane_error = lane_center - center_x

        if show_guides:
            cv2.line(line_image, (lane_center, Y_TOP), (lane_center, FRAME_H), (255, 0, 0), 4)   # biru: pusat jalur
            cv2.line(line_image, (center_x, Y_TOP), (center_x, FRAME_H), (0, 0, 255), 4)         # merah: pusat kamera
            cv2.line(line_image, (center_x, Y_REF), (lane_center, Y_REF), (0, 255, 255), 3)      # kuning: error

        if lane_error < -TURN_THRESHOLD:
            text = "TURN LEFT"
        elif lane_error > TURN_THRESHOLD:
            text = "TURN RIGHT"
        else:
            text = "STRAIGHT"

    cmd = CMD_MAP.get(text, '0')   # hanya untuk tampilan terminal

    # --- Kirim error ke Arduino master ---
    send_command(lane_error)
    drain_serial()

    # --- Terminal: cetak hanya saat keputusan berubah ---
    if cmd != prev_cmd:
        print(f"{LABELS[cmd]} | {text} | error {lane_error} px")
        prev_cmd = cmd

    # --- Tampilan ---
    proc_time = (time.time() - start_time) * 1000
    cv2.putText(image, text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
    cv2.putText(image, f"Error: {lane_error} px", (50, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(image, f"Proc: {proc_time:.1f} ms", (50, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 150, 0), 2)
    serial_status = "ON" if ser is not None else "OFF"
    sent = "-" if last_sent_error is None else str(last_sent_error)
    cv2.putText(image, f"Serial: {serial_status} | Sent: {sent}", (50, 160),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)

    combo = cv2.addWeighted(image, 0.8, line_image, 1, 1)
    last_combo = combo
    cv2.imshow("Lane Following", combo)

    if show_debug:
        canny_view = canny.copy()
        if thr is not None:
            label = f"Canny {'AUTO' if P['auto'] else 'MANUAL'}: low={thr[0]:.0f} high={thr[1]:.0f}"
        else:
            label = "Canny pada Color Mask"
        cv2.putText(canny_view, label, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)
        show_scaled("Canny Edge", canny_view)
        show_scaled("Hough Lines (raw)", draw_raw_hough(clean, lines, hough_debug))
        if color_mask is not None:
            show_scaled("Color Mask", color_mask)
        if SHOW_EXTRA_DEBUG:
            show_scaled("Grayscale", gray)
            show_scaled("Gaussian Blur", blur)
            show_scaled("ROI / Mask", roi)

# =========================================================
# BERSIH-BERSIH
# =========================================================
cap.release()
cv2.destroyAllWindows()

if ser is not None:
    try:
        ser.write(b"0\n")      # error 0 -> roda kembali lurus
        time.sleep(0.2)
        drain_serial()
        ser.close()
    except serial.SerialException:
        pass

print("Selesai.")
