import argparse
import csv
import json
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares


def load_csv(filename):

    rows = []

    with open(filename, "r", newline="") as f:
        reader = csv.DictReader(f)

        if not reader.fieldnames:
            raise ValueError("CSV file has no header.")

        columns = {
            name.strip().lower(): name
            for name in reader.fieldnames
            if name is not None
        }

        candidates = [
            ("x_gauss", "y_gauss", "z_gauss"),
            ("x", "y", "z"),
        ]

        selected = None

        for names in candidates:
            if all(name in columns for name in names):
                selected = [columns[name] for name in names]
                break

        if selected is None:
            raise ValueError(
                "Could not find magnetometer columns. "
                "Expected x_gauss,y_gauss,z_gauss or x,y,z."
            )

        for row in reader:
            try:
                xyz = [float(row[col]) for col in selected]
                if np.all(np.isfinite(xyz)):
                    rows.append(xyz)
            except (ValueError, TypeError, KeyError):
                continue

    data = np.asarray(rows, dtype=float)

    if len(data) < 30:
        raise ValueError(
            f"Only {len(data)} valid samples found. "
            "At least 30 are required; substantially more and "
            "good 3D coverage are recommended."
        )

    if np.any(np.ptp(data, axis=0) < 1e-10):
        raise ValueError(
            "At least one axis has no meaningful variation."
        )

    return data


def initial_ellipsoid_fit(points):

    mean = np.mean(points, axis=0)

    scale = np.sqrt(np.mean(np.sum((points - mean) ** 2, axis=1)))

    if not np.isfinite(scale) or scale < 1e-12:
        raise ValueError("Measurements have insufficient variation.")

    u = (points - mean) / scale

    x, y, z = u.T

    design = np.column_stack([
        x * x,
        y * y,
        z * z,
        2 * y * z,
        2 * x * z,
        2 * x * y,
        2 * x,
        2 * y,
        2 * z,
    ])

    coefficients, _, rank, _ = np.linalg.lstsq(
        design, np.ones(len(u)), rcond=None
    )

    if rank < 9:
        raise ValueError(
            "The measurements do not constrain a full ellipsoid. "
            "Rotate the sensor through more 3D orientations."
        )

    a, b, c, f, g, h, p, q, r = coefficients

    Q = np.array([
        [a, h, g],
        [h, b, f],
        [g, f, c],
    ])

    linear = np.array([p, q, r])

    try:
        center = -np.linalg.solve(Q, linear)
    except np.linalg.LinAlgError as exc:
        raise ValueError("Degenerate ellipsoid fit.") from exc

    k = 1.0 + center @ Q @ center

    if not np.isfinite(k) or k <= 0:
        raise ValueError(
            "The fitted quadric is not a valid ellipsoid."
        )

    S = Q / k

    eigenvalues = np.linalg.eigvalsh(S)

    if np.any(eigenvalues <= 0):
        raise ValueError(
            "The fit is not a positive-definite ellipsoid. "
            "Check orientation coverage and magnetic interference."
        )

    return mean, scale, center, S


def unpack_parameters(params):

    L = np.array([
        [np.exp(params[0]), 0.0, 0.0],
        [params[1], np.exp(params[2]), 0.0],
        [params[3], params[4], np.exp(params[5])],
    ])

    return L @ L.T


def fit_ellipsoid(points):

    mean, scale, center0, S0 = initial_ellipsoid_fit(points)

    u = (points - mean) / scale

    try:
        L0 = np.linalg.cholesky(S0)
    except np.linalg.LinAlgError as exc:
        raise ValueError("Cannot initialize ellipsoid refinement.") from exc

    params0 = np.array([
        np.log(L0[0, 0]),
        L0[1, 0],
        np.log(L0[1, 1]),
        L0[2, 0],
        L0[2, 1],
        np.log(L0[2, 2]),
    ])

    initial = np.concatenate([center0, params0])

    def residuals(parameters):
        center = parameters[:3]
        S = unpack_parameters(parameters[3:])

        delta = u - center

        radius = np.sqrt(
            np.maximum(
                np.einsum("ni,ij,nj->n", delta, S, delta),
                1e-15,
            )
        )

        return radius - 1.0

    result = least_squares(
        residuals,
        initial,
        loss="soft_l1",
        f_scale=0.01,
        max_nfev=3000,
    )

    if not result.success and result.status <= 0:
        raise ValueError(
            f"Nonlinear ellipsoid refinement failed: {result.message}"
        )

    center_norm = result.x[:3]
    S = unpack_parameters(result.x[3:])

    eigenvalues, eigenvectors = np.linalg.eigh(S)

    if np.any(eigenvalues <= 0) or not np.all(np.isfinite(eigenvalues)):
        raise ValueError("Refined ellipsoid is not positive definite.")

    offset = mean + scale * center_norm

    sqrt_S = (
        eigenvectors
        @ np.diag(np.sqrt(eigenvalues))
        @ eigenvectors.T
    )

    log_det_S = np.sum(np.log(eigenvalues))
    scale_factor = np.exp(-log_det_S / 6.0)

    correction = scale_factor * sqrt_S

    return offset, correction, scale, center_norm, S, result


def coverage_score(points, offset, bins=200):

    centered = points - offset

    norms = np.linalg.norm(centered, axis=1)
    valid = norms > 1e-12

    directions = centered[valid] / norms[valid, None]

    if len(directions) == 0:
        return 0.0, 0

    i = np.arange(bins)

    z = 1.0 - 2.0 * (i + 0.5) / bins
    phi = np.pi * (3.0 - np.sqrt(5.0)) * i
    radial = np.sqrt(np.maximum(0.0, 1.0 - z * z))

    reference = np.column_stack([
        radial * np.cos(phi),
        radial * np.sin(phi),
        z,
    ])

    assignments = np.argmax(directions @ reference.T, axis=1)

    occupied = np.unique(assignments).size

    return 100.0 * occupied / bins, int(occupied)


def print_statistics(raw, calibrated):
    raw_mag = np.linalg.norm(raw, axis=1)
    cal_mag = np.linalg.norm(calibrated, axis=1)

    def describe(name, values):
        mean = np.mean(values)
        std = np.std(values)
        cv = 100.0 * std / mean if abs(mean) > 1e-12 else float("nan")

        print(f"\n{name}")
        print(f"  Mean magnitude:       {mean:.8g}")
        print(f"  Standard deviation:   {std:.8g}")
        print(f"  Relative variation:   {cv:.3f}%")
        print(f"  5th percentile:       {np.percentile(values, 5):.8g}")
        print(f"  95th percentile:      {np.percentile(values, 95):.8g}")

    describe("RAW FIELD MAGNITUDE", raw_mag)
    describe("CALIBRATED FIELD MAGNITUDE", cal_mag)

    return raw_mag, cal_mag


def make_ellipsoid_surface(mean, scale, center_norm, S, resolution=35):

    eigenvalues, eigenvectors = np.linalg.eigh(S)

    if np.any(eigenvalues <= 0):
        raise ValueError("Cannot plot an invalid ellipsoid.")

    u = np.linspace(0, 2 * np.pi, resolution)
    v = np.linspace(0, np.pi, resolution)

    xs = np.outer(np.cos(u), np.sin(v))
    ys = np.outer(np.sin(u), np.sin(v))
    zs = np.outer(np.ones_like(u), np.cos(v))

    sphere = np.stack([xs, ys, zs], axis=-1)

    transform = eigenvectors @ np.diag(1.0 / np.sqrt(eigenvalues))

    flat = sphere.reshape(-1, 3)
    ellipsoid_norm = center_norm + flat @ transform.T
    ellipsoid_raw = mean + scale * ellipsoid_norm

    surface = ellipsoid_raw.reshape(sphere.shape)

    return surface[:, :, 0], surface[:, :, 1], surface[:, :, 2]


def set_equal_axes(ax, points):

    low = np.min(points, axis=0)
    high = np.max(points, axis=0)

    center = (low + high) / 2.0
    radius = max(np.max(high - low) / 2.0, 1e-9) * 1.1

    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)

    try:
        ax.set_box_aspect((1, 1, 1))
    except AttributeError:
        pass


def make_diagnostic_plot(
    raw,
    calibrated,
    offset,
    mean,
    scale,
    center_norm,
    S,
    raw_mag,
    cal_mag,
    output_path,
):
    # Downsample for visualization only; calculations use all samples.
    max_plot_points = 5000
    step = max(1, len(raw) // max_plot_points)

    raw_plot = raw[::step]
    cal_plot = calibrated[::step]

    fig = plt.figure(figsize=(16, 10))

    ax1 = fig.add_subplot(221, projection="3d")
    ax2 = fig.add_subplot(222, projection="3d")
    ax3 = fig.add_subplot(223)
    ax4 = fig.add_subplot(224)

    ax1.scatter(
        raw_plot[:, 0],
        raw_plot[:, 1],
        raw_plot[:, 2],
        s=3,
        alpha=0.35,
        label="Raw samples",
    )

    ex, ey, ez = make_ellipsoid_surface(
        mean, scale, center_norm, S
    )

    ax1.plot_wireframe(
        ex, ey, ez,
        rstride=3,
        cstride=3,
        color="red",
        linewidth=0.6,
        alpha=0.6,
    )

    ax1.scatter(
        *offset,
        color="black",
        marker="x",
        s=80,
        label="Estimated center",
    )

    ax1.set_title("Raw measurements + fitted ellipsoid")
    ax1.set_xlabel("X")
    ax1.set_ylabel("Y")
    ax1.set_zlabel("Z")
    ax1.legend(loc="upper right")
    set_equal_axes(ax1, raw_plot)

    ax2.scatter(
        cal_plot[:, 0],
        cal_plot[:, 1],
        cal_plot[:, 2],
        s=3,
        alpha=0.4,
        color="tab:green",
    )

    ax2.set_title("Calibrated measurements")
    ax2.set_xlabel("X")
    ax2.set_ylabel("Y")
    ax2.set_zlabel("Z")
    set_equal_axes(ax2, cal_plot)

    ax3.hist(raw_mag, bins=70, alpha=0.55, label="Raw")
    ax3.hist(cal_mag, bins=70, alpha=0.7, label="Calibrated")
    ax3.set_title("Magnetic-field magnitude distribution")
    ax3.set_xlabel("Magnitude (input units)")
    ax3.set_ylabel("Number of samples")
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    mean_cal = np.mean(cal_mag)
    residual = cal_mag - mean_cal

    ax4.hist(residual, bins=70, color="tab:green", alpha=0.75)
    ax4.axvline(0, color="black", linewidth=1)
    ax4.set_title("Calibrated magnitude residual")
    ax4.set_xlabel("Magnitude minus mean (input units)")
    ax4.set_ylabel("Number of samples")
    ax4.grid(True, alpha=0.3)

    fig.suptitle(
        "Magnetometer Calibration Diagnostics",
        fontsize=15,
    )

    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    print(f"\nDiagnostic plot saved to: {output_path}")

    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Fit a hard/soft-iron magnetometer calibration."
    )

    parser.add_argument(
        "csv_file",
        help="CSV file containing raw magnetometer measurements",
    )

    parser.add_argument(
        "--output",
        default="mag_calibration.json",
        help="JSON output filename",
    )

    parser.add_argument(
        "--plot",
        default="mag_calibration_diagnostics.png",
        help="Diagnostic plot filename",
    )

    args = parser.parse_args()

    filename = Path(args.csv_file)

    print(f"Loading: {filename}")

    raw = load_csv(filename)

    print(f"Valid measurements: {len(raw)}")
    print("Fitting ellipsoid...")

    offset, correction, scale, center_norm, S, result = (
        fit_ellipsoid(raw)
    )

    calibrated = (raw - offset) @ correction.T

    raw_mag, cal_mag = print_statistics(raw, calibrated)

    coverage, occupied = coverage_score(raw, offset)

    print("\nHARD-IRON OFFSET (same units as CSV)")
    for axis, value in zip("XYZ", offset):
        print(f"  {axis}: {value:.10g}")

    print("\nSOFT-IRON CORRECTION MATRIX")
    print(correction)

    print("\nCALIBRATION DIAGNOSTICS")
    print(f"  Solver converged:            {result.success}")
    print(f"  Optimization cost:           {result.cost:.8g}")
    print(f"  Approx. directional coverage: {coverage:.1f}% "
          f"({occupied}/200 bins)")
    print(f"  Mean corrected magnitude:    {np.mean(cal_mag):.8g}")
    print(f"  Corrected magnitude std:     {np.std(cal_mag):.8g}")
    print(f"  Corrected relative variation:"
          f" {100*np.std(cal_mag)/np.mean(cal_mag):.3f}%")

    output = {
        "input_file": str(filename),
        "sample_count": int(len(raw)),
        "input_units": "same as CSV (e.g. gauss)",
        "hard_iron_offset": offset.tolist(),
        "soft_iron_correction_matrix": correction.tolist(),
        "application": "calibrated = correction @ (raw - offset)",
        "mean_calibrated_magnitude": float(np.mean(cal_mag)),
        "std_calibrated_magnitude": float(np.std(cal_mag)),
        "relative_magnitude_variation_percent": float(
            100 * np.std(cal_mag) / np.mean(cal_mag)
        ),
        "approx_directional_coverage_percent": float(coverage),
        "occupied_direction_bins": int(occupied),
        "direction_bin_count": 200,
        "optimizer_success": bool(result.success),
        "optimizer_message": str(result.message),
    }

    with open(args.output, "w") as f:
        json.dump(output, f, indent=4)

    print(f"\nCalibration saved to: {args.output}")

    make_diagnostic_plot(
        raw,
        calibrated,
        offset,
        np.mean(raw, axis=0),
        scale,
        center_norm,
        S,
        raw_mag,
        cal_mag,
        args.plot,
    )


if __name__ == "__main__":
    main()
