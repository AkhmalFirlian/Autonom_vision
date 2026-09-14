import cv2

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Gagal: Webcam tidak terdeteksi")
    exit(1)

print("Webcam aktif!")
print(f"Resolusi: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
print(f"FPS: {cap.get(cv2.CAP_PROP_FPS)}")

ret, frame = cap.read()

if ret:
    print("Frame berhasil dibaca ✅")
    cv2.imwrite("webcam_test.png", frame)
    print("Gambar tersimpan: webcam_test.png")
else:
    print("Gagal membaca frame ❌")

cap.release()   
