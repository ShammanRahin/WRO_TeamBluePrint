#!/usr/bin/env python3
"""
Generates schemes/wiring_block_diagram.png — WRO FE 2026, Team Blueprint.

The rules require schemes/ to contain "one or several schematic diagrams in form of
JPEG, PNG or PDF of the electromechanical components illustrating all the elements
(electronic components and motors) used in the vehicle and how they connect to each
other." Rubric Criterion 2 scores 0 without it.

Regenerate after any electrical change:   python3 electrical/make_block_diagram.py
"""

import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

C_PWR, C_MCU, C_SENS, C_ACT, C_COMP = "#F4B942", "#4A90D9", "#5CB85C", "#D9534F", "#9B59B6"


def box(ax, x, y, w, h, label, color, sub="", fs=8.5):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.10",
                                fc=color, ec="black", lw=1.1, alpha=0.85))
    ax.text(x + w / 2, y + h / 2 + (0.12 if sub else 0), label, ha="center",
            va="center", fontsize=fs, fontweight="bold", zorder=5)
    if sub:
        ax.text(x + w / 2, y + h / 2 - 0.20, sub, ha="center", va="center",
                fontsize=fs - 2.0, style="italic", zorder=5)


def link(ax, p1, p2, label="", color="black", ls="-", lw=1.3, off=0.0):
    ax.add_patch(FancyArrowPatch(p1, p2, arrowstyle="-", color=color, ls=ls,
                                 lw=lw, shrinkA=2, shrinkB=2, zorder=1))
    if label:
        ax.text((p1[0] + p2[0]) / 2 + off, (p1[1] + p2[1]) / 2 + 0.14, label,
                ha="center", fontsize=6.6, color=color,
                bbox=dict(fc="white", ec="none", alpha=0.85, pad=0.8), zorder=6)


def main():
    fig, ax = plt.subplots(figsize=(15.5, 9.5))
    ax.set_xlim(0, 26)
    ax.set_ylim(0, 16)
    ax.axis("off")

    ax.text(13, 15.4, "WRO Future Engineers 2026 — Team Blueprint — Electrical Block Diagram",
            ha="center", fontsize=13.5, fontweight="bold")
    ax.text(13, 14.9, "As built 2026-07-26  |  STM32F411CEU6  |  one master switch (rule 9.10)  "
                      "|  separate start button (rule 9.11)", ha="center", fontsize=8.5,
            style="italic")

    # ---------------- power chain ----------------
    box(ax, 0.5, 12.2, 2.6, 1.3, "3S LiPo", C_PWR, "XT30")
    box(ax, 3.6, 12.2, 2.2, 1.3, "FUSE", C_PWR, "10 A blade")
    box(ax, 6.3, 12.2, 3.0, 1.3, "MASTER SWITCH", C_PWR, "SPST 10 A — rule 9.10")
    link(ax, (3.1, 12.85), (3.6, 12.85))
    link(ax, (5.8, 12.85), (6.3, 12.85))

    box(ax, 10.6, 12.2, 3.0, 1.3, "BEC-S  6.0 V / 3 A", C_PWR)
    box(ax, 10.6, 10.5, 3.0, 1.3, "BEC-L  5.0 V / 2 A", C_PWR)
    box(ax, 10.6, 8.8, 3.0, 1.3, "BEC-C  5.1 V / 3 A", C_PWR, "Pi only — removed in Open")
    box(ax, 10.6, 6.9, 3.0, 1.3, "VBAT direct", C_PWR, "to motor driver")
    for yy in (12.85, 11.15, 9.45, 7.55):
        link(ax, (9.3, 12.85), (10.0, 12.85))
        link(ax, (10.0, 12.85), (10.0, yy))
        link(ax, (10.0, yy), (10.6, yy))

    box(ax, 6.3, 9.6, 3.0, 1.2, "STAR GROUND", "#BBBBBB",
        "single M3 brass standoff")
    link(ax, (1.8, 12.2), (1.8, 10.2), color="0.35")
    link(ax, (1.8, 10.2), (6.3, 10.2), color="0.35", label="all returns")

    # ---------------- MCU ----------------
    box(ax, 15.2, 10.2, 4.2, 2.2, "STM32F411CEU6", C_MCU,
        "26 / 28 pins used  |  no WiFi (rule 11.10)", fs=10)
    link(ax, (13.6, 11.15), (15.2, 11.30), label="5 V")

    # ---------------- sensors ----------------
    box(ax, 20.6, 13.0, 4.9, 1.5, "5 x VL53L0X   (I2C1)", C_SENS,
        "XSHUT -> 0x30-0x34  |  F / FL30 / FR30 / L90 / R90")
    box(ax, 20.6, 11.2, 4.9, 1.4, "TCS34725   (I2C2, ALONE)", C_SENS,
        "turn trigger + direction decode")
    box(ax, 20.6, 9.5, 4.9, 1.3, "MPU9250   (SPI1)", C_SENS,
        "gyro-Z only, magnetometer OFF")
    box(ax, 20.6, 7.8, 4.9, 1.3, "25GA encoder  (TIM3)", C_SENS,
        "quadrature, 3995 Hz max")
    link(ax, (19.4, 12.1), (20.6, 13.75), label="I2C1 + 5x XSHUT")
    link(ax, (19.4, 11.6), (20.6, 11.9), label="I2C2")
    link(ax, (19.4, 11.0), (20.6, 10.15), label="SPI1 8 MHz")
    link(ax, (19.4, 10.5), (20.6, 8.45), label="A/B twisted")

    # ---------------- actuators ----------------
    box(ax, 15.2, 7.2, 4.2, 1.4, "BTS7960", C_ACT, "43 A H-bridge")
    box(ax, 15.2, 5.3, 4.2, 1.4, "MG996R servo", C_ACT, "ANALOG — 50 Hz ceiling")
    box(ax, 20.6, 5.9, 4.9, 2.0, "25GA motor -> 5:1 gear\n-> SOLID rear axle", C_ACT,
        "266 RPM  |  0.70 m/s  |  0.175 mm/count")
    link(ax, (17.3, 10.2), (17.3, 8.6), label="RPWM/LPWM/EN")
    link(ax, (16.0, 10.2), (16.0, 6.7), label="PWM 50 Hz", off=-0.9)
    link(ax, (19.4, 7.9), (20.6, 7.2), label="M+/M-")
    link(ax, (13.6, 7.55), (15.2, 7.55), label="VBAT")
    link(ax, (13.6, 12.85), (15.2, 6.0), label="6 V", color="0.4", ls=":")

    # ---------------- compute ----------------
    box(ax, 15.2, 2.9, 4.2, 1.6, "Raspberry Pi 4B 1 GB", C_COMP,
        "OBSTACLE ROUND ONLY")
    box(ax, 20.6, 2.9, 4.9, 1.6, "Fisheye camera 160 deg", C_COMP,
        "pillar colour only")
    link(ax, (19.4, 3.7), (20.6, 3.7), label="CSI")
    link(ax, (17.3, 10.2), (17.3, 4.5), label="USART1 checksummed", off=-1.5)
    link(ax, (13.6, 9.45), (15.2, 3.9), label="5.15 V", color="0.4", ls=":")

    # ---------------- start button ----------------
    box(ax, 15.2, 8.9, 4.2, 1.0, "START BUTTON", "#DDDDDD",
        "momentary, debounced — rule 9.11")
    link(ax, (17.3, 9.9), (17.3, 10.2))

    # ---------------- notes ----------------
    notes = (
        "KEY DESIGN POINTS\n"
        "1. ONE master switch powers every rail (rule 9.10). The start button is separate (rule 9.11).\n"
        "2. Four isolated power domains, joined at ONE star ground. Motor return has its own >=1.0 mm2 leg.\n"
        "3. VL53L0X and TCS34725 both ship at I2C address 0x29 -> buses are SPLIT. The TCS (turn trigger)\n"
        "   sits alone on I2C2 so no ToF cable fault can take it down. MPU9250 moved to SPI.\n"
        "4. XSHUT is OPEN-DRAIN (the L0X die is 2.8 V). Addresses are volatile: enumeration re-runs on\n"
        "   every reset and SKIPS a failed sensor rather than stalling.\n"
        "5. Every VL53L0X needs a printed collimator snout (2.5 x 10 x 20 mm slot) + 2 deg wedge:\n"
        "   the L0X has NO ROI, the mat is white and the walls are black, so the floor out-reflects the wall.\n"
        "6. Domain C (Pi) is PHYSICALLY UNPLUGGED for the Open Challenge — a claim a judge can inspect."
    )
    ax.text(0.5, 5.6, notes, fontsize=8.2, va="top", family="monospace",
            bbox=dict(fc="#FAFAFA", ec="0.6", pad=8))

    handles = [plt.Rectangle((0, 0), 1, 1, fc=c, ec="k", alpha=0.85) for c in
               (C_PWR, C_MCU, C_SENS, C_ACT, C_COMP)]
    ax.legend(handles, ["Power", "MCU", "Sensors", "Actuators", "Compute (obstacle only)"],
              loc="lower right", fontsize=8.5, ncol=5, bbox_to_anchor=(1.0, -0.02))

    os.makedirs("schemes", exist_ok=True)
    out = "schemes/wiring_block_diagram.png"
    plt.savefig(out, dpi=145, bbox_inches="tight", facecolor="white")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
