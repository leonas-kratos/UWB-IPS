import os
import numpy as np
import matplotlib.pyplot as plt

# ================= CONFIG =================
DELIMITER = ","
VALUE_COL = 1   # lấy số sau dấu phẩy
# =========================================

# ====== 1. GOM TOÀN BỘ SAMPLE (LOS) ======
x_all = []   # measured
y_all = []   # ground truth

for file in sorted(os.listdir(".")):
    if file.endswith(".csv"):
        gt = float(os.path.splitext(file)[0])   # GT từ tên file
        measured = np.loadtxt(
            file,
            delimiter=DELIMITER,
            usecols=VALUE_COL
        )
        x_all.append(measured)
        y_all.append(np.full_like(measured, gt))

x_all = np.concatenate(x_all)
y_all = np.concatenate(y_all)

# ====== 2. FIT POLYNOMIAL BẬC 2: y = a x² + b x + c ======
A = np.vstack([x_all**2, x_all, np.ones(len(x_all))]).T
coeffs = np.linalg.lstsq(A, y_all, rcond=None)[0]
a, b, c = coeffs

print("===== CALIBRATION MODEL (Bậc 2) =====")
print(f"y = {a:.8e} * x² + {b:.8f} * x + {c:.3f}")
print("=====================================")

# ====== 3. TÍNH RMSE TRƯỚC / SAU CALI ======
gt_vals = []
rmse_before = []
rmse_after = []

for file in sorted(os.listdir(".")):
    if file.endswith(".csv"):
        gt = float(os.path.splitext(file)[0])
        measured = np.loadtxt(file, delimiter=DELIMITER, usecols=VALUE_COL)
        
        # RMSE trước cali
        rmse_b = np.sqrt(np.mean((measured - gt) ** 2))
        
        # RMSE sau cali (bậc 2)
        calibrated = a * measured**2 + b * measured + c
        rmse_a = np.sqrt(np.mean((calibrated - gt) ** 2))
        
        gt_vals.append(gt)
        rmse_before.append(rmse_b)
        rmse_after.append(rmse_a)
        print(f"{file}: RMSE before = {rmse_b:.3f}, after = {rmse_a:.3f}")

# ====== 4. VẼ ĐỒ THỊ SO SÁNH ======
plt.figure(figsize=(10, 6))
plt.plot(gt_vals, rmse_before, 'o-', label="Before calibration", linewidth=2)
plt.plot(gt_vals, rmse_after, 's-', label="After calibration (Bậc 2)", linewidth=2)
plt.xlabel("Ground truth")
plt.ylabel("RMSE")
plt.title("RMSE before / after calibration (Polynomial bậc 2)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
