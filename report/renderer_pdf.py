"""
report/renderer_pdf.py
======================
PDF report renderer for the drone sizing tool.

This module generates a PDF report by:
1. Rendering the HTML report to a temporary file via renderer_html
2. Converting the HTML to PDF using wkhtmltopdf

wkhtmltopdf is used as the PDF backend because:
- It is available as a system binary (no Python package install required)
- It produces accurate, print-quality PDF output from HTML+CSS
- It handles inline SVG rendering correctly
- It supports page headers, footers, and margins via CLI arguments

The output PDF is named identically to the HTML report:
    {mission_name}_{YYYY-MM-DD}.pdf

Dependencies:
    - wkhtmltopdf must be installed and available in the system PATH.
      Verified on: wkhtmltopdf 0.12.6

Usage:
    from report.renderer_pdf import render

    output_path = render(data, output_dir=".")
    print(f"PDF written to: {output_path}")
"""

import os
import subprocess
import tempfile

from report.builder import ReportData
from report.renderer_html import render as render_html


# ---------------------------------------------------------------------------
# wkhtmltopdf settings
# ---------------------------------------------------------------------------

# Page margins in millimetres
_MARGIN_TOP_MM = 15
_MARGIN_BOTTOM_MM = 15
_MARGIN_LEFT_MM = 14
_MARGIN_RIGHT_MM = 14

# DPI for SVG/image rendering
_DPI = 150

# Page size
_PAGE_SIZE = "A4"


def _wkhtmltopdf_available() -> bool:
    """Check whether wkhtmltopdf is installed and accessible in PATH.

    Returns:
        bool: True if the binary is found and executable, False otherwise.
    """
    try:
        result = subprocess.run(
            ["wkhtmltopdf", "--version"],
            capture_output=True,
            timeout=5
        )
        return result.returncode == 0
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False


def render(data: ReportData, output_dir: str = ".") -> str:
    """Render the sizing report as a PDF file.

    Generates the HTML report to a temporary file, then converts it to PDF
    using wkhtmltopdf. The temporary HTML file is deleted after conversion.

    Args:
        data (ReportData): Fully populated report data from builder.build().
        output_dir (str): Directory where the PDF file will be written.
                          Default is the current working directory.

    Returns:
        str: Absolute path to the generated PDF file.

    Output filename:
        {mission_name}_{YYYY-MM-DD}.pdf
        Example: HELIOX_X1_baseline_2026-03-22.pdf

    Raises:
        EnvironmentError: If wkhtmltopdf is not found in the system PATH.
        RuntimeError: If wkhtmltopdf exits with a non-zero return code,
                      indicating a conversion failure.
    """
    if not _wkhtmltopdf_available():
        raise EnvironmentError(
            "wkhtmltopdf is not installed or not found in PATH. "
            "Install it from https://wkhtmltopdf.org/downloads.html "
            "or via your system package manager (e.g. apt install wkhtmltopdf)."
        )

    os.makedirs(output_dir, exist_ok=True)

    # --- Step 1: render HTML to a temporary file ---
    with tempfile.TemporaryDirectory() as tmp_dir:
        html_path = render_html(data, output_dir=tmp_dir)

        # --- Step 2: define PDF output path ---
        pdf_filename = f"{data.mission_name}_{data.report_date}.pdf"
        pdf_path = os.path.join(output_dir, pdf_filename)

        # --- Step 3: convert HTML -> PDF via wkhtmltopdf ---
        cmd = [
            "wkhtmltopdf",
            "--page-size", _PAGE_SIZE,
            "--margin-top", str(_MARGIN_TOP_MM),
            "--margin-bottom", str(_MARGIN_BOTTOM_MM),
            "--margin-left", str(_MARGIN_LEFT_MM),
            "--margin-right", str(_MARGIN_RIGHT_MM),
            "--dpi", str(_DPI),
            "--enable-local-file-access",
            "--print-media-type",
            "--no-outline",
            "--quiet",
            "--footer-center", (
                f"Drone Sizing Report — {data.mission_name} — "
                f"{data.report_date} — Page [page] / [topage]"
            ),
            "--footer-font-size", "8",
            "--footer-spacing", "4",
            html_path,
            pdf_path,
        ]

        result = subprocess.run(
            cmd,
            capture_output=True,
            timeout=60
        )

        if result.returncode != 0:
            stderr = result.stderr.decode("utf-8", errors="replace")
            raise RuntimeError(
                f"wkhtmltopdf failed (exit code {result.returncode}):\n{stderr}"
            )

    return os.path.abspath(pdf_path)
