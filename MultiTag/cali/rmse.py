import os
import numpy as np
import matplotlib.pyplot as plt

rmse_list = []
gt_list = []

for file in sorted(os.listdir(".")):
    if file.endswith(".csv"):
        # ground truth = tên file
        gt = float(os.path.splitext(file)[0])

        # CHỈ đọc số sau dấu phẩy
        measured = np.loadtxt(
            file,
            delimiter=",",
            usecols=1
        )

        rmse = np.sqrt(np.mean((measured - gt) ** 2))

        gt_list.append(gt)
        rmse_list.append(rmse)

        print(f"{file}: RMSE = {rmse:.4f}")

# ===== VẼ RMSE =====
plt.figure()
plt.plot(gt_list, rmse_list, marker='o')
plt.xlabel("Ground truth (từ tên file)")
plt.ylabel("RMSE")
plt.title("RMSE so với ground truth")
plt.grid(True)
plt.tight_layout()
plt.show()

