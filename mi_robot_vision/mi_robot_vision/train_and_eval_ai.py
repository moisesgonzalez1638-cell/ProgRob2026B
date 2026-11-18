#!/usr/bin/env python3
import os
import time
import numpy as np
import cv2
import joblib
from sklearn.neural_network import MLPClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score, classification_report

CLASSES = ['red', 'green', 'blue']

def generate_synthetic_patch(color_name, rng):
    bg_b = rng.uniform(175, 210)
    bg_g = rng.uniform(195, 225)
    bg_r = rng.uniform(205, 235)
    img = np.ones((32, 32, 3), dtype=np.float32)
    img[:, :, 0] = bg_b
    img[:, :, 1] = bg_g
    img[:, :, 2] = bg_r

    illum = rng.uniform(0.45, 1.35)
    if color_name == 'red':
        base = np.array([15.0, 15.0, 220.0]) * illum
    elif color_name == 'green':
        base = np.array([15.0, 215.0, 15.0]) * illum
    else:
        base = np.array([225.0, 45.0, 15.0]) * illum

    base = np.clip(base, 0, 255)
    cx = int(rng.integers(12, 20))
    cy = int(rng.integers(12, 20))
    radius = int(rng.integers(8, 12))
    cv2.circle(img, (cx, cy), radius, base.tolist(), -1)

    if rng.random() > 0.4:
        spec_x = cx + int(rng.integers(-4, 4))
        spec_y = cy + int(rng.integers(-4, 4))
        cv2.circle(img, (spec_x, spec_y), 3, (245, 245, 245), -1)

    noise = rng.normal(0, 8.0, img.shape)
    img = np.clip(img + noise, 0, 255).astype(np.uint8)
    return img

def extract_features(bgr_patch):
    resized = cv2.resize(bgr_patch, (32, 32))
    hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(resized, cv2.COLOR_BGR2LAB)

    center_bgr = resized[8:24, 8:24].reshape(-1, 3) / 255.0
    center_hsv = hsv[8:24, 8:24].reshape(-1, 3) / 255.0
    center_lab = lab[8:24, 8:24].reshape(-1, 3) / 255.0

    hist_h = cv2.calcHist([hsv], [0], None, [12], [0, 180]).flatten()
    hist_h /= (hist_h.sum() + 1e-6)

    feats = np.concatenate([
        center_bgr.mean(axis=0), center_bgr.std(axis=0),
        center_hsv.mean(axis=0), center_hsv.std(axis=0),
        center_lab.mean(axis=0),
        hist_h
    ])
    return feats

def classify_classical_hsv(bgr_patch):
    resized = cv2.resize(bgr_patch, (32, 32))
    hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)

    mask_red1 = cv2.inRange(hsv, (0, 90, 60), (12, 255, 255))
    mask_red2 = cv2.inRange(hsv, (165, 90, 60), (180, 255, 255))
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    mask_green = cv2.inRange(hsv, (35, 90, 60), (85, 255, 255))
    mask_blue = cv2.inRange(hsv, (95, 90, 60), (135, 255, 255))

    counts = [
        cv2.countNonZero(mask_red),
        cv2.countNonZero(mask_green),
        cv2.countNonZero(mask_blue)
    ]
    return int(np.argmax(counts))

def main():
    rng = np.random.default_rng(42)
    patches, X, y = [], [], []

    for label_idx, cname in enumerate(CLASSES):
        for _ in range(300):
            patch = generate_synthetic_patch(cname, rng)
            patches.append(patch)
            X.append(extract_features(patch))
            y.append(label_idx)

    patches = np.array(patches)
    X = np.array(X)
    y = np.array(y)

    idx_train, idx_test = train_test_split(np.arange(len(y)), test_size=0.25, random_state=42, stratify=y)
    X_train, y_train = X[idx_train], y[idx_train]
    X_test, y_test = X[idx_test], y[idx_test]
    patches_test = patches[idx_test]

    clf = MLPClassifier(hidden_layer_sizes=(64, 32), max_iter=500, random_state=42)
    clf.fit(X_train, y_train)

    t0 = time.perf_counter()
    preds_ai = [clf.predict([extract_features(p)])[0] for p in patches_test]
    t_ai_ms = ((time.perf_counter() - t0) / len(patches_test)) * 1000.0
    acc_ai = accuracy_score(y_test, preds_ai) * 100.0

    t0 = time.perf_counter()
    preds_cv = [classify_classical_hsv(p) for p in patches_test]
    t_cv_ms = ((time.perf_counter() - t0) / len(patches_test)) * 1000.0
    acc_cv = accuracy_score(y_test, preds_cv) * 100.0

    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    models_dir = os.path.join(base_dir, 'models')
    os.makedirs(models_dir, exist_ok=True)

    model_path = os.path.join(models_dir, 'color_mlp_model.joblib')
    joblib.dump(clf, model_path)

    report_text = f"""=== BENCHMARK: IA (MLP NEURAL NETWORK) VS VISION CLASICA (HSV) ===
Dataset de evaluacion: {len(y_test)} muestras sinteticas con variacion de iluminacion (0.45x - 1.35x) y brillo especular.

1. Metodo Clasico (Segmentacion HSV por umbrales):
   - Exactitud (Accuracy): {acc_cv:.2f} %
   - Tiempo promedio por parche: {t_cv_ms:.4f} ms

2. Componente de IA (Red Neuronal MLP 64-32 sobre caracteristicas HSV+LAB+RGB):
   - Exactitud (Accuracy): {acc_ai:.2f} %
   - Tiempo promedio por parche: {t_ai_ms:.4f} ms

Reporte detallado IA:
{classification_report(y_test, preds_ai, target_names=CLASSES)}
"""
    report_path = os.path.join(models_dir, 'benchmark_ia_vs_clasico.txt')
    with open(report_path, 'w') as f:
        f.write(report_text)

    print(report_text)

if __name__ == '__main__':
    main()
