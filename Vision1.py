import cv2
import numpy as np

# =========================================================
# KONFIGURASI
# =========================================================
VIDEO_PATH = r"D:\College\Autonom Car\Database\road5.mp4"

N_SAMPLES = 10                  # jumlah titik yg disampling sepanjang tiap garis hijau
SEARCH_RADIUS = 4              # toleransi geser kiri-kanan (px) saat mencocokkan ke marka
FP_MISMATCH_THRESHOLD = 0.650    # garis dianggap "tidak pas" jika >90% titiknya meleset dari marka

TOPHAT_KERNEL_SIZE = 8         # ukuran kernel top-hat (px). Naikkan kalau marka jauh/tipis
                                 # masih belum tertangkap (coba 35, 45, dst). Terlalu besar
                                 # bisa membuat rumput/kerikil ikut terdeteksi sebagai marka.

DILATE_KERNEL_SIZE = 5          # toleransi lebar marka & geseran piksel. Naikkan kalau garis
                                 # hijau dianggap salah padahal cukup dekat dengan marka.

SHOW_DEBUG = True               # tampilkan window debug (mask marka + garis hijau) saat berjalan


def region_of_interest(image):
    mask = np.zeros_like(image)
    polygon = np.array([[
        # road3 (samakan dgn ROI di lane_detection.py)
        [158, 678],
        [926, 684],
        [639, 413],
        [545, 410],
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image


def make_coordinates(image, line_parameters):
    slope, intercept = line_parameters

    y1 = image.shape[0]
    y2 = int(y1 * 0.6)

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
        parameters = np.polyfit((x1, x2), (y1, y2), 1)
        slope = parameters[0]
        intercept = parameters[1]

        if abs(slope) < 0.5:
            continue

        if slope < 0 and x1 < image.shape[1] / 2:
            left_fit.append((slope, intercept))
        elif slope > 0 and x1 > image.shape[1] / 2:
            right_fit.append((slope, intercept))

    lane_lines = []

    if len(left_fit) > 0:
        left_fit_average = np.average(left_fit, axis=0)
        lane_lines.append(make_coordinates(image, left_fit_average))

    if len(right_fit) > 0:
        right_fit_average = np.average(right_fit, axis=0)
        lane_lines.append(make_coordinates(image, right_fit_average))

    return lane_lines


def get_marking_mask(image):
    """
    Mendeteksi piksel marka jalan (garis putih & kuning) sebagai acuan
    'ground truth otomatis' untuk mengevaluasi apakah garis hijau hasil
    deteksi benar-benar berada di atas marka jalan.

    Marka putih dideteksi dengan TOP-HAT TRANSFORM (bukan threshold warna
    absolut). Top-hat menonjolkan objek yang LEBIH TERANG DARI SEKITARNYA
    SECARA LOKAL -> marka yang tipis/jauh/redup (mis. di sisi kanan yang
    perspektifnya jauh) tetap tertangkap, karena yang dinilai bukan nilai
    brightness absolut tapi kontrasnya terhadap aspal di sekelilingnya.

    Marka kuning tetap dideteksi via filter warna HSV karena warnanya
    cukup khas dan jarang bermasalah dengan jarak/pudar.
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # --- Marka putih: top-hat transform ---
    # kernel elips cukup besar agar garis tipis "lolos" sebagai foreground,
    # sementara area aspal luas dianggap background dan dihilangkan.
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (TOPHAT_KERNEL_SIZE, TOPHAT_KERNEL_SIZE))
    tophat = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, kernel)

    # threshold adaptif (Otsu) terhadap hasil top-hat, bukan nilai absolut
    _, mask_white = cv2.threshold(tophat, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # --- Marka kuning: filter warna HSV ---
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([15, 60, 80])
    upper_yellow = np.array([35, 255, 255])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    mask = cv2.bitwise_or(mask_white, mask_yellow)

    # batasi hanya pada area jalan (ROI) supaya objek lain tidak ikut terhitung
    mask = region_of_interest(mask)

    # dilasi tipis -> beri toleransi lebar marka & sedikit pergeseran piksel
    kernel_dilate = np.ones((DILATE_KERNEL_SIZE, DILATE_KERNEL_SIZE), np.uint8)
    mask = cv2.dilate(mask, kernel_dilate, iterations=1)

    return mask


def line_match_ratio(line, mask, n_samples=N_SAMPLES, search_radius=SEARCH_RADIUS):
    """
    Menghitung berapa persen titik sepanjang 'line' yang berhimpitan dengan mask marka.
    line: [x1, y1, x2, y2]  -> (x1,y1) = titik bawah, (x2,y2) = titik atas
    """
    x1, y1, x2, y2 = line
    h, w = mask.shape[:2]

    matched = 0
    total = 0

    for i in range(n_samples):
        t = i / (n_samples - 1)
        x = int(x1 + (x2 - x1) * t)
        y = int(y1 + (y2 - y1) * t)

        if y < 0 or y >= h:
            continue

        x_min = max(0, x - search_radius)
        x_max = min(w, x + search_radius + 1)

        total += 1
        if np.any(mask[y, x_min:x_max] > 0):
            matched += 1

    if total == 0:
        return 0.0

    return matched / total


def classify_frame(averaged_lines, mask):
    """
    Klasifikasi tiap frame menjadi TP / FN / FP:
    - FN : tidak ada 2 garis hijau (deteksi gagal)
    - FP : ada 2 garis hijau, tapi salah satu garis >90% titiknya TIDAK pas dgn marka
    - TP : ada 2 garis hijau, dan keduanya pas dengan marka
    """
    if len(averaged_lines) < 2:
        return "FN", []

    ratios = [line_match_ratio(line, mask) for line in averaged_lines]

    for ratio in ratios:
        mismatch = 1 - ratio
        if mismatch > FP_MISMATCH_THRESHOLD:
            return "FP", ratios

    return "TP", ratios


def draw_debug(image, mask, averaged_lines, label, ratios):
    debug = image.copy()
    mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    debug = cv2.addWeighted(debug, 0.7, mask_colored, 0.6, 0)

    for line in averaged_lines:
        x1, y1, x2, y2 = line
        cv2.line(debug, (x1, y1), (x2, y2), (0, 255, 0), 4)

    color = {"TP": (0, 255, 0), "FP": (0, 0, 255), "FN": (0, 165, 255)}[label]
    cv2.putText(debug, label, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)

    if ratios:
        for i, r in enumerate(ratios):
            cv2.putText(debug, f"match[{i}]: {r*100:.1f}%", (50, 90 + i * 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    return debug


def main():
    cap = cv2.VideoCapture(VIDEO_PATH)

    TP = FP = FN = 0
    frame_idx = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_idx += 1
        image = cv2.resize(frame, (1280, 720))

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        high_thresh, _ = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        batas_atas = high_thresh * 2.2
        low_thresh = 0.5 * high_thresh
        batas_bawah = low_thresh * 1.5
        canny = cv2.Canny(blur, batas_bawah, batas_atas)
        canny_display = cv2.cvtColor(canny, cv2.COLOR_GRAY2BGR)

        roi = region_of_interest(canny)
        lines = cv2.HoughLinesP(roi, 1, np.pi / 180, 20, minLineLength=10, maxLineGap=60)
        averaged_lines = average_slope_intercept(image, lines)

        mask = get_marking_mask(image)

        label, ratios = classify_frame(averaged_lines, mask)

        if label == "TP":
            TP += 1
        elif label == "FP":
            FP += 1
        else:
            FN += 1

        print(f"Frame {frame_idx:5d} -> {label}  match_ratio={['%.2f' % r for r in ratios]}")

        if SHOW_DEBUG:
            cv2.imshow("1. Grayscale", gray)
            cv2.imshow("2. Gaussian Blur", blur)
            cv2.imshow("3. Canny Edge", canny_display)
            cv2.imshow("4. ROI / Mask", roi)
            cv2.imshow("5. Mask Marka (Top-hat + Kuning)", mask)

            debug_img = draw_debug(image, mask, averaged_lines, label, ratios)
            cv2.imshow("6. Debug: Mask Marka + Garis Hijau", debug_img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()

    precision = TP / (TP + FP) if (TP + FP) > 0 else 0
    recall = TP / (TP + FN) if (TP + FN) > 0 else 0
    f1 = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0

    print("\n=== Hasil Evaluasi Deteksi Garis Jalan ===")
    print(f"Total frame diproses : {frame_idx}")
    print(f"TP : {TP}")
    print(f"FP : {FP}")
    print(f"FN : {FN}")
    print(f"Precision : {precision:.3f}")
    print(f"Recall    : {recall:.3f}")
    print(f"F1-Score  : {f1:.3f}")


if __name__ == "__main__":
    main()
