import numpy as np
import cv2

# ============================================================
# 1. HARİTA VE ENGEL TANIMLAMA (Robotanik Sera - Kapalı Çevre)
# ============================================================
res = 0.05          # Çözünürlük: 1 piksel = 5 cm (0.05 metre)
width_m = 11        # X yönü (metre)
height_m = 50       # Y yönü (metre)

width_px = int(width_m / res)
height_px = int(height_m / res)

# Boş (Beyaz) Harita
harita = np.ones((height_px, width_px), dtype=np.uint8) * 255

# --- DIŞ DUVARLAR (ÇEVRE KORUMASI) ---
# 4 piksel = 20 cm kalınlığında kapalı bir kutu çiziyoruz
d_kalinlik = 4 
harita[0:d_kalinlik, :] = 0          # Alt Duvar
harita[-d_kalinlik:, :] = 0          # Üst Duvar
harita[:, 0:d_kalinlik] = 0          # Sol Duvar
harita[:, -d_kalinlik:] = 0          # Sağ Duvar

# --- İÇ ENGELLER (Dikey Bitki Sıraları) ---
y_bas = int(2 / res)   # 1. metre
y_bit = int(48 / res)  # 49. metre
obstacle_cols_m = [1, 3, 5, 7, 9]

for x in obstacle_cols_m:
    x_bas = int(x / res)
    x_bit = int((x + 1) / res)
    harita[y_bas:y_bit, x_bas:x_bit] = 0

harita = np.flipud(harita)
cv2.imwrite('robotanik_sera.pgm', harita)
print("Dış duvarları kapalı robotanik_sera.pgm başarıyla oluşturuldu!")
