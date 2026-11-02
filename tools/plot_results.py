"""Regenerate the README figures from CSVs in results/.

    python3 tools/plot_results.py

Requires matplotlib. Figures go to docs/figures/.
"""
import csv, os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RES, FIG = os.path.join(ROOT, "results"), os.path.join(ROOT, "docs", "figures")
os.makedirs(FIG, exist_ok=True)

# Categorical palette (fixed order), validated for CVD separation; every series is
# also direct-labelled so colour never carries identity alone.
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"]
INK, MUTED, GRID, SURFACE = "#1f1e1c", "#6b6a64", "#e6e4df", "#fcfcfb"
plt.rcParams.update({
    "font.family": "DejaVu Sans", "font.size": 10, "axes.edgecolor": MUTED, "axes.labelcolor": INK,
    "xtick.color": MUTED, "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False,
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
})

def read(path):
    with open(path) as f:
        return list(csv.DictReader(f))

def slip_figure():
    rovers = [("mer_class", "MER-class"), ("cfrp_titanium", "CFRP + Ti"), ("mini_sojourner", "Microrover"), ("steel_tank", "Steel tank")]
    soils = [("m90_simulant", "M90 Mars simulant"), ("loose_drift", "Loose drift")]
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.2), sharey=True)
    for ax, (sid, sname) in zip(axes, soils):
        for k, (rid, rname) in enumerate(rovers):
            p = os.path.join(RES, f"slip_{rid}_{sid}.csv")
            if not os.path.exists(p):
                continue
            rows = read(p)
            x = [float(r["slope_deg"]) for r in rows]
            pred = [100 * float(r["predicted_slip"]) for r in rows]
            sim = [100 * float(r["simulated_slip"]) for r in rows]
            c = SERIES[k]
            ax.plot(x, pred, color=c, lw=2)
            ax.plot(x, sim, "o", color=c, ms=6, mec=SURFACE, mew=1.5)
            # direct label at the last point under 95 % slip
            last = max(i for i in range(len(x)) if pred[i] < 95) if any(v < 95 for v in pred) else 0
            ax.annotate(rname, (x[last], pred[last]), xytext=(4, -2), textcoords="offset points", color=INK, fontsize=9)
        ax.set_title(sname, color=INK, loc="left", fontsize=11, fontweight="bold")
        ax.set_xlabel("Slope (deg)")
        ax.grid(True, color=GRID, lw=0.8)
        ax.set_ylim(0, 102)
        ax.set_xlim(0, 31)
    axes[0].set_ylabel("Wheel slip (%)")
    fig.text(0.01, 0.01, "Lines: steady-state Bekker-Wong/Janosi model.  Dots: full dynamic simulation (backward-Euler, 6 wheels).",
             color=MUTED, fontsize=8.5)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(os.path.join(FIG, "slip_vs_slope.png"), dpi=160)
    print("wrote docs/figures/slip_vs_slope.png")

def bench_figure(name, title):
    p = os.path.join(RES, f"bench_{name}.csv")
    if not os.path.exists(p):
        return
    rows = read(p)
    order = ["reference", "ablate_value", "ablate_vo", "ablate_landmarks", "simple_drive"]
    labels = {"reference": "Full stack", "ablate_value": "No value ranking", "ablate_vo": "No visual odometry",
              "ablate_landmarks": "No landmarks", "simple_drive": "Dead reckoning only"}
    stats = []
    for c in order:
        s = [float(r["score"]) for r in rows if r["controller"] == c]
        if s:
            m = sum(s) / len(s)
            sd = (sum((v - m) ** 2 for v in s) / max(1, len(s) - 1)) ** 0.5
            stats.append((labels[c], m, sd, len(s)))
    fig, ax = plt.subplots(figsize=(7.5, 0.55 * len(stats) + 1.2))
    y = list(range(len(stats)))[::-1]
    ax.barh(y, [s[1] for s in stats], height=0.55, color=SERIES[0], xerr=[s[2] for s in stats],
            error_kw={"ecolor": MUTED, "elinewidth": 1, "capsize": 3})
    for yi, s in zip(y, stats):
        ax.text(s[1] + s[2] + max(1, 0.01 * max(t[1] for t in stats)), yi, f"{s[1]:.0f}", va="center", color=INK, fontsize=9)
    ax.set_yticks(y)
    ax.set_yticklabels([s[0] for s in stats])
    ax.set_xlabel(f"Mission score (mean ± sd over {stats[0][3]} random terrains)")
    ax.set_title(title, color=INK, loc="left", fontsize=11, fontweight="bold")
    ax.grid(True, axis="x", color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, f"ablation_{name}.png"), dpi=160)
    print(f"wrote docs/figures/ablation_{name}.png")

if __name__ == "__main__":
    slip_figure()
    bench_figure("crater", "Ablation: 02 Crater Rim Science")
    bench_figure("storm", "Ablation: 04 Dust Storm")
