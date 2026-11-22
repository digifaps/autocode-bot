#!/usr/bin/env python3
"""Printable chessboard for stereo_calibrator.py, as a PDF at exact scale (no dependencies).

    scripts/make_calibration_board.py                          # A4, 10x7 squares of 25 mm (9x6 inner corners)
    scripts/make_calibration_board.py --paper A3 --square 35 -o board_a3.pdf

--cols/--rows are SQUARES; the calibrator wants inner corners (cols-1, rows-1). Print at 100 % /
"Actual size" and check the 100 mm line with a ruler; pass the measured square size (in metres) as
square_size.
"""
import argparse

MM = 72 / 25.4
PAPERS = {"A4": (297, 210), "A3": (420, 297)}  # landscape, mm


def board_pdf(cols, rows, sq, paper):
    pw, ph = PAPERS[paper]
    bw, bh = cols * sq, rows * sq
    if bw > pw - 20 or bh > ph - 30:  # 10 mm side margins, 30 mm for the check line and text
        raise SystemExit("board %.0f x %.0f mm does not fit on %s with margins; use smaller squares"
                         % (bw, bh, paper))
    x0, y0 = (pw - bw) / 2, (ph - bh) / 2 + 4  # mm from bottom-left, room for the text below
    ops = ["0 g"]
    for r in range(rows):
        for c in range(cols):
            if (r + c) % 2 == 0:
                ops.append("%.3f %.3f %.3f %.3f re f" % ((x0 + c * sq) * MM, (y0 + r * sq) * MM, sq * MM, sq * MM))
    lx, ly = x0 * MM, (y0 - 9) * MM  # 100 mm check line with end ticks
    ops += ["0.5 w", "%.3f %.3f m %.3f %.3f l S" % (lx, ly, lx + 100 * MM, ly),
            "%.3f %.3f m %.3f %.3f l S" % (lx, ly - 4, lx, ly + 4),
            "%.3f %.3f m %.3f %.3f l S" % (lx + 100 * MM, ly - 4, lx + 100 * MM, ly + 4)]
    texts = [("must measure exactly 100 mm", lx + 100 * MM + 8, ly - 3),
             ("autocode-bot stereo calibration - %dx%d inner corners, %g mm squares (board %g x %g mm). "
              "Print at 100%% / Actual size, glue flat on cardboard." % (cols - 1, rows - 1, sq, bw, bh),
              lx, ly - 18)]
    for t, x, y in texts:
        t = t.replace("\\", "\\\\").replace("(", "\\(").replace(")", "\\)")
        ops.append("BT /F1 8 Tf %.3f %.3f Td (%s) Tj ET" % (x, y, t))
    content = "\n".join(ops).encode()
    objs = [b"<< /Type /Catalog /Pages 2 0 R >>",
            b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>",
            b"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 %.2f %.2f] /Contents 4 0 R "
            b"/Resources << /Font << /F1 5 0 R >> >> >>" % (pw * MM, ph * MM),
            b"<< /Length %d >>\nstream\n" % len(content) + content + b"\nendstream",
            b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>"]
    out, offs = bytearray(b"%PDF-1.4\n"), []
    for i, o in enumerate(objs, 1):
        offs.append(len(out))
        out += b"%d 0 obj\n" % i + o + b"\nendobj\n"
    xref = len(out)
    out += b"xref\n0 %d\n0000000000 65535 f \n" % (len(objs) + 1)
    out += b"".join(b"%010d 00000 n \n" % o for o in offs)
    out += b"trailer\n<< /Size %d /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF\n" % (len(objs) + 1, xref)
    return bytes(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cols", type=int, default=10, help="squares across (default 10)")
    ap.add_argument("--rows", type=int, default=7, help="squares down (default 7)")
    ap.add_argument("--square", type=float, default=25.0, help="square size in mm (default 25)")
    ap.add_argument("--paper", choices=sorted(PAPERS), default="A4")
    ap.add_argument("-o", "--output", default="calibration_board.pdf")
    a = ap.parse_args()
    with open(a.output, "wb") as f:
        f.write(board_pdf(a.cols, a.rows, a.square, a.paper))
    print("wrote %s: %dx%d inner corners, %g mm squares, %s -> -p board_cols:=%d -p board_rows:=%d "
          "-p square_size:=%g" % (a.output, a.cols - 1, a.rows - 1, a.square, a.paper,
                                  a.cols - 1, a.rows - 1, a.square / 1000))


if __name__ == "__main__":
    main()
