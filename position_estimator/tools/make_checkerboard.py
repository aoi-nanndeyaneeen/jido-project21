"""
tools/make_checkerboard.py

キャリブレーション用チェッカーボードの印刷データ(SVG)を生成する。

tools/calibrate_intrinsics.py で使う「格子模様」を作るためのスクリプト。
SVGなので拡大縮小しても劣化せず、mm単位で正確に印刷できる。

--------------------------------------------------------------------------
使い方
--------------------------------------------------------------------------
    # A4・既定サイズ（内側交点 7x5、マス30mm）
    python tools/make_checkerboard.py

    # A3で大きめに作る（推奨。コンビニでA3印刷できる）
    python tools/make_checkerboard.py --paper a3

    # マス目のサイズを指定
    python tools/make_checkerboard.py --paper a4 --cols 7 --rows 5 --square 28

出力: calib/checkerboard_<cols>x<rows>_<square>mm_<paper>.svg

--------------------------------------------------------------------------
印刷時の注意（ここを外すと全部やり直しになる）
--------------------------------------------------------------------------
1. **必ず「実際のサイズ」「100%」で印刷する。**
   「用紙に合わせる」「フィット」を選ぶと縮尺が変わる。
   Adobe Reader なら [ページサイズ処理] → [実際のサイズ]。
   ブラウザ印刷なら [詳細設定] → [倍率] を 100% に。

2. **印刷後、定規でマス目を実測する。**
   紙に印字された「square = XX.X mm」と一致するか確認すること。
   ずれていたら、実測値を calibrate_intrinsics.py の --square に渡す。

3. **平らな板に全面のりで貼る。**
   反りがあると精度が出ない。平面性がいちばん効く。
   スチレンボード・厚紙・クリップボードなど。
   ★ 波打った紙を手で持って撮るのは失敗のもと。

4. 白い余白（枠）は切り落とさない。検出に必要。

--------------------------------------------------------------------------
「内側の交点」とは
--------------------------------------------------------------------------
    --cols / --rows はマスの数ではなく、**内側の交点の数**。

        8マス x 6マス の市松模様  →  内側交点は 7 x 5

    OpenCV に渡すのはこの交点の数。生成されるSVGにも印字される。
"""

import argparse
from pathlib import Path

# calibrate_intrinsics.py / calib_store.py と同じ場所にまとめる
# （calib/ の実体は src/calib/。src/utils/config.py の PROJECT_DIR 参照）
CALIB_DIR = Path(__file__).resolve().parent.parent / "src" / "calib"

# 用紙サイズ [mm] (幅, 高さ) — 横長で使う
PAPERS = {
    "a4": (297.0, 210.0),
    "a3": (420.0, 297.0),
    "a2": (594.0, 420.0),
}

MARGIN_MM = 12.0     # 白余白（検出に必要なので削らない）
CAPTION_MM = 12.0    # 下部の説明文スペース


def build_svg(cols: int, rows: int, square: float, paper: str) -> str:
    pw, ph = PAPERS[paper]

    n_sq_x = cols + 1          # 交点 cols 個 → マスは cols+1 列
    n_sq_y = rows + 1
    bw, bh = n_sq_x * square, n_sq_y * square

    avail_w = pw - 2 * MARGIN_MM
    avail_h = ph - 2 * MARGIN_MM - CAPTION_MM
    if bw > avail_w or bh > avail_h:
        max_sq = min(avail_w / n_sq_x, avail_h / n_sq_y)
        raise SystemExit(
            f"[ERROR] {paper.upper()} に収まりません。\n"
            f"        盤面 {bw:.0f}x{bh:.0f}mm > 印刷可能 {avail_w:.0f}x{avail_h:.0f}mm\n"
            f"        --square を {max_sq:.1f} 以下にするか、--paper a3 にしてください。")

    # 中央に配置（キャプション分だけ上へ寄せる）
    ox = (pw - bw) / 2
    oy = (ph - CAPTION_MM - bh) / 2

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" '
        f'width="{pw}mm" height="{ph}mm" viewBox="0 0 {pw} {ph}">',
        f'<rect x="0" y="0" width="{pw}" height="{ph}" fill="#ffffff"/>',
    ]

    for iy in range(n_sq_y):
        for ix in range(n_sq_x):
            if (ix + iy) % 2 == 0:
                continue          # 市松模様: 偶数マスは白のまま
            x = ox + ix * square
            y = oy + iy * square
            parts.append(
                f'<rect x="{x:.4f}" y="{y:.4f}" '
                f'width="{square:.4f}" height="{square:.4f}" fill="#000000"/>')

    # 実寸確認用の基準線（1マスぶん）
    ly = oy + bh + 5.0
    parts.append(
        f'<line x1="{ox:.3f}" y1="{ly:.3f}" x2="{ox + square:.3f}" y2="{ly:.3f}" '
        f'stroke="#000000" stroke-width="0.4"/>')
    for xx in (ox, ox + square):
        parts.append(
            f'<line x1="{xx:.3f}" y1="{ly - 1.6:.3f}" x2="{xx:.3f}" '
            f'y2="{ly + 1.6:.3f}" stroke="#000000" stroke-width="0.4"/>')

    caption = (f'inner corners = {cols} x {rows}   |   '
               f'square = {square:.1f} mm   |   {paper.upper()} 100% scale')
    parts.append(
        f'<text x="{ox + square + 4:.3f}" y="{ly + 1.6:.3f}" '
        f'font-family="monospace" font-size="4.2" fill="#000000">{caption}</text>')
    parts.append(
        f'<text x="{ox:.3f}" y="{ly + 8.0:.3f}" '
        f'font-family="monospace" font-size="3.4" fill="#444444">'
        f'PRINT AT 100% (do NOT "fit to page"), then measure one square with a ruler.'
        f'</text>')

    parts.append('</svg>')
    return "\n".join(parts)


def main():
    ap = argparse.ArgumentParser(
        description="キャリブレーション用チェッカーボードのSVGを生成する")
    ap.add_argument("--cols", type=int, default=7,
                    help="内側交点の列数（マスの数ではない）。既定 7")
    ap.add_argument("--rows", type=int, default=5,
                    help="内側交点の行数（マスの数ではない）。既定 5")
    ap.add_argument("--square", type=float, default=None,
                    help="マス目の一辺 [mm]。省略時は用紙に合わせて自動")
    ap.add_argument("--paper", choices=sorted(PAPERS), default="a4",
                    help="用紙サイズ。既定 a4")
    args = ap.parse_args()

    if args.cols == args.rows:
        raise SystemExit("[ERROR] --cols と --rows は違う値にしてください。"
                         "正方形だと向きが一意に決まらず検出が不安定になります。")

    pw, ph = PAPERS[args.paper]
    square = args.square
    if square is None:
        avail_w = pw - 2 * MARGIN_MM
        avail_h = ph - 2 * MARGIN_MM - CAPTION_MM
        square = min(avail_w / (args.cols + 1), avail_h / (args.rows + 1))
        square = int(square)        # 定規で測りやすいよう整数mmに丸める

    svg = build_svg(args.cols, args.rows, square, args.paper)

    CALIB_DIR.mkdir(parents=True, exist_ok=True)
    out = CALIB_DIR / (f"checkerboard_{args.cols}x{args.rows}_"
                       f"{square:g}mm_{args.paper}.svg")
    out.write_text(svg, encoding="utf-8")

    print(f"\n  生成しました: {out}")
    print(f"    内側交点 : {args.cols} x {args.rows}")
    print(f"    マス目   : {square:g} mm")
    print(f"    盤面     : {(args.cols+1)*square:g} x {(args.rows+1)*square:g} mm "
          f"({args.paper.upper()})")
    print("\n  次の手順:")
    print("    1. このSVGをブラウザで開いて [100%/実際のサイズ] で印刷")
    print("    2. 定規でマス目を実測（印字値と一致するか確認）")
    print("    3. 平らな板に全面のりで貼る（反りが最大の敵）")
    print(f"    4. python tools/calibrate_intrinsics.py --label Camera1 "
          f"--source <ID> --cols {args.cols} --rows {args.rows} "
          f"--square {square:g}")


if __name__ == "__main__":
    main()
