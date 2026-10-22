#!/usr/bin/env python3
"""
Analyze Flight Data — Post-processing and comparison plots.

Reads the CSV produced by data_logger and generates:
0. Cover page with all calculation formulas
1. Per-config altitude hold performance
2. Per-config attitude stability (roll/pitch)
3. Stability index comparison (box plot)
4. Power consumption comparison
5. Motor command distribution
6. Summary statistics bar charts
"""

import os
import sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend for WSL
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

import rclpy
from rclpy.node import Node

# Larger default font so plots are readable at full size
plt.rcParams.update({
    'font.size': 14,
    'axes.titlesize': 18,
    'axes.labelsize': 15,
    'xtick.labelsize': 12,
    'ytick.labelsize': 12,
    'legend.fontsize': 12,
    'figure.titlesize': 20,
    'lines.linewidth': 1.8,
    'axes.grid': True,
    'grid.alpha': 0.3,
})


class FlightDataAnalyzer(Node):

    def __init__(self):
        super().__init__('flight_data_analyzer')

        self.declare_parameter(
            'data_dir', os.path.expanduser('~/morphing_drone_data')
        )
        self.data_dir = self.get_parameter('data_dir').value
        self.csv_path = os.path.join(self.data_dir, 'flight_data.csv')

        if not os.path.exists(self.csv_path):
            self.get_logger().error(f'No data file found at {self.csv_path}')
            self.get_logger().info('Run the full simulation first.')
            return

        self.get_logger().info(f'Analyzing {self.csv_path}...')
        self.analyze()

    def _cover_page(self, pdf, df, configs):
        """Write a cover page explaining every calculation."""
        fig = plt.figure(figsize=(16, 20))
        fig.patch.set_facecolor('white')
        ax = fig.add_subplot(111)
        ax.axis('off')

        n_samples = len(df)
        duration = df['timestamp_s'].max() - df['timestamp_s'].min()
        sample_rate = n_samples / duration if duration > 0 else 0

        lines = []
        lines.append(("MORPHING DRONE — FLIGHT DATA ANALYSIS", 'title'))
        lines.append(("Cal Poly Pomona — ECE Senior Design", 'subtitle'))
        lines.append(("", 'space'))
        lines.append((f"Dataset: {self.csv_path}", 'body'))
        lines.append((f"Samples: {n_samples:,}   |   Duration: {duration:.1f} s   "
                      f"|   Sample rate: {sample_rate:.1f} Hz", 'body'))
        lines.append((f"Configurations observed: {', '.join(sorted(configs))}", 'body'))
        lines.append(("", 'space'))
        lines.append(("CALCULATIONS USED IN THIS REPORT", 'section'))
        lines.append(("", 'small_space'))

        lines.append(("1. Altitude RMSE (Root Mean Square Error)", 'subsection'))
        lines.append(("    Formula:  RMSE = sqrt( mean( (z_actual - z_target)² ) )", 'formula'))
        lines.append(("    Units:    meters", 'body'))
        lines.append(("    Target:   z_target = 5.0 m (hover altitude)", 'body'))
        lines.append(("    Purpose:  Measures how tightly the altitude controller held the setpoint.", 'body'))
        lines.append(("              Lower = better. Ideal < 0.1 m.", 'body'))
        lines.append(("    Note:     First 5 s of each config excluded to skip morph transients.", 'note'))
        lines.append(("", 'small_space'))

        lines.append(("2. Stability Index", 'subsection'))
        lines.append(("    Formula:  S = sqrt( roll² + pitch² )", 'formula'))
        lines.append(("    Units:    degrees (instantaneous tilt magnitude)", 'body'))
        lines.append(("    Purpose:  Combined measure of attitude deviation. Reflects the total", 'body'))
        lines.append(("              angle between the drone's thrust vector and vertical.", 'body'))
        lines.append(("              Lower = more stable hover.", 'body'))
        lines.append(("", 'small_space'))

        lines.append(("3. Attitude Plots — Roll and Pitch", 'subsection'))
        lines.append(("    Formula:  roll_deg = roll_rad × (180/π)", 'formula'))
        lines.append(("              pitch_deg = pitch_rad × (180/π)", 'formula'))
        lines.append(("    Source:   Quaternion from Gazebo ground-truth odometry, converted to", 'body'))
        lines.append(("              Euler angles using standard ZYX intrinsic rotation sequence.", 'body'))
        lines.append(("", 'small_space'))

        lines.append(("4. Estimated Power", 'subsection'))
        lines.append(("    Formula:  P_est = Σ(i=1..4) k_motor × thrust_i^(3/2)", 'formula'))
        lines.append(("    Units:    watts", 'body'))
        lines.append(("    Purpose:  Approximates electrical power draw from motor thrust via", 'body'))
        lines.append(("              propeller momentum theory (P ∝ T^1.5 for constant disc area).", 'body'))
        lines.append(("              k_motor is calibrated from the D2842-2800KV thrust table.", 'body'))
        lines.append(("    Note:     Steady-state estimate only. Transient currents during", 'note'))
        lines.append(("              acceleration can exceed this by 1.5–2×.", 'note'))
        lines.append(("", 'small_space'))

        lines.append(("5. Per-Motor Thrust (CG-Centering Allocation)", 'subsection'))
        lines.append(("    Formula:  Solve for T_i:                                                  ", 'formula'))
        lines.append(("              Σ T_i        = T_total   (total thrust = weight)", 'formula'))
        lines.append(("              Σ T_i · x_i  = 0         (cancel pitch moment)", 'formula'))
        lines.append(("              Σ T_i · y_i  = 0         (cancel roll moment)", 'formula'))
        lines.append(("    Method:   Least-squares (numpy.linalg.lstsq) — 3 equations, 4 unknowns.", 'body'))
        lines.append(("              Produces equal thrust for X/O/H (symmetric) and differential", 'body'))
        lines.append(("              thrust for Y/T (asymmetric) to keep the thrust centroid at", 'body'))
        lines.append(("              the geometric center of mass.", 'body'))
        lines.append(("", 'small_space'))

        lines.append(("6. Thrust Centroid Offset (CG offset)", 'subsection'))
        lines.append(("    Formula:  cx = mean(x_i),  cy = mean(y_i)", 'formula'))
        lines.append(("              offset = sqrt(cx² + cy²)", 'formula'))
        lines.append(("    Units:    millimeters", 'body'))
        lines.append(("    Values:   X, O, H = 0 mm   |   Y = 23.3 mm   |   T = 79.5 mm", 'body'))
        lines.append(("", 'small_space'))

        lines.append(("7. Transition Analysis (across config changes)", 'subsection'))
        lines.append(("    Servo travel:   max(|target_angle - current_angle|)  degrees", 'formula'))
        lines.append(("    Transition dt:  travel / servo_speed  (servo_speed = 300 °/s nominal)", 'formula'))
        lines.append(("    Thrust rate:    ΔT_max / transition_dt  (N/s)", 'formula'))
        lines.append(("", 'small_space'))

        lines.append(("DATA EXCLUSION RULES", 'section'))
        lines.append(("• First 5 seconds (~250 samples at 50 Hz) of each config excluded from RMSE", 'body'))
        lines.append(("  and stability calculations to avoid morph transients biasing the metrics.", 'body'))
        lines.append(("• Samples with invalid altitude (< 0 m or > 20 m) are treated as outliers.", 'body'))
        lines.append(("", 'space'))

        lines.append(("INTERPRETATION GUIDE", 'section'))
        lines.append(("• Altitude RMSE < 0.10 m = excellent altitude hold", 'body'))
        lines.append(("• Altitude RMSE 0.10 – 0.25 m = acceptable", 'body'))
        lines.append(("• Altitude RMSE > 0.25 m = needs tuning", 'body'))
        lines.append(("• Stability Index < 2° RMS = solid hover", 'body'))
        lines.append(("• Stability Index 2° – 5° RMS = functional, minor drift", 'body'))
        lines.append(("• Stability Index > 5° RMS = instability — likely CG or mixer issue", 'body'))

        y_pos = 0.97
        for text, style in lines:
            if style == 'title':
                ax.text(0.5, y_pos, text, transform=ax.transAxes,
                        ha='center', va='top', fontsize=22, fontweight='bold')
                y_pos -= 0.035
            elif style == 'subtitle':
                ax.text(0.5, y_pos, text, transform=ax.transAxes,
                        ha='center', va='top', fontsize=14, style='italic',
                        color='#555')
                y_pos -= 0.028
            elif style == 'section':
                ax.text(0.04, y_pos, text, transform=ax.transAxes,
                        ha='left', va='top', fontsize=15, fontweight='bold',
                        color='#1a4080')
                y_pos -= 0.028
            elif style == 'subsection':
                ax.text(0.04, y_pos, text, transform=ax.transAxes,
                        ha='left', va='top', fontsize=13, fontweight='bold')
                y_pos -= 0.022
            elif style == 'formula':
                ax.text(0.06, y_pos, text, transform=ax.transAxes,
                        ha='left', va='top', fontsize=11, family='monospace',
                        color='#222', bbox=dict(facecolor='#f4f4f8',
                                                edgecolor='#ccc', pad=3))
                y_pos -= 0.022
            elif style == 'body':
                ax.text(0.06, y_pos, text, transform=ax.transAxes,
                        ha='left', va='top', fontsize=11)
                y_pos -= 0.02
            elif style == 'note':
                ax.text(0.06, y_pos, text, transform=ax.transAxes,
                        ha='left', va='top', fontsize=10, style='italic',
                        color='#666')
                y_pos -= 0.018
            elif style == 'space':
                y_pos -= 0.02
            elif style == 'small_space':
                y_pos -= 0.01

        pdf.savefig(fig, bbox_inches='tight')
        plt.close()

    def analyze(self):
        df = pd.read_csv(self.csv_path)

        if df.empty:
            self.get_logger().error('CSV is empty — no data to analyze')
            return

        configs = df['config'].unique()
        self.get_logger().info(f'Configurations found: {list(configs)}')

        # Color map for configs
        colors = {'X': '#2196F3', 'O': '#4CAF50', 'T': '#FF9800',
                  'Y': '#FF5722', 'H': '#E91E63'}

        output_pdf = os.path.join(self.data_dir, 'flight_analysis.pdf')

        with PdfPages(output_pdf) as pdf:

            # ---- PAGE 0: Cover page explaining all calculations ----
            self._cover_page(pdf, df, configs)

            # ---- PLOT 1: Altitude over time (BIGGER) ----
            fig, ax = plt.subplots(figsize=(16, 9))
            for cfg in configs:
                subset = df[df['config'] == cfg]
                ax.plot(subset['timestamp_s'].to_numpy(),
                        subset['pos_z'].to_numpy(),
                        label=f'{cfg}-config',
                        color=colors.get(cfg, 'gray'),
                        alpha=0.85, linewidth=2.2)
            ax.axhline(y=5.0, color='red', linestyle='--', linewidth=2,
                       alpha=0.6, label='Target (5m)')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Altitude (m)')
            ax.set_title('Altitude Hold Performance by Configuration\n'
                         r'$z_{error} = z_{actual} - z_{target}$;  '
                         r'$RMSE = \sqrt{\langle z_{error}^2 \rangle}$')
            ax.legend(loc='best', ncol=2)
            plt.tight_layout()
            pdf.savefig(fig)
            plt.close()

            # ---- PLOT 2: Roll and Pitch stability (BIGGER) ----
            fig, axes = plt.subplots(2, 1, figsize=(16, 11), sharex=True)
            for cfg in configs:
                subset = df[df['config'] == cfg]
                axes[0].plot(subset['timestamp_s'].to_numpy(),
                             np.degrees(subset['roll_rad'].to_numpy()),
                             label=f'{cfg}',
                             color=colors.get(cfg, 'gray'),
                             alpha=0.8, linewidth=1.8)
                axes[1].plot(subset['timestamp_s'].to_numpy(),
                             np.degrees(subset['pitch_rad'].to_numpy()),
                             label=f'{cfg}',
                             color=colors.get(cfg, 'gray'),
                             alpha=0.8, linewidth=1.8)

            axes[0].axhline(y=0, color='k', linestyle=':', alpha=0.4)
            axes[1].axhline(y=0, color='k', linestyle=':', alpha=0.4)
            axes[0].set_ylabel('Roll (deg)')
            axes[0].set_title(r'Attitude Stability — Roll   '
                              r'$(\phi = atan2(2(q_wq_x + q_yq_z), 1 - 2(q_x^2 + q_y^2)))$')
            axes[0].legend(loc='best', ncol=len(configs))

            axes[1].set_ylabel('Pitch (deg)')
            axes[1].set_xlabel('Time (s)')
            axes[1].set_title(r'Attitude Stability — Pitch   '
                              r'$(\theta = asin(2(q_wq_y - q_zq_x)))$')
            axes[1].legend(loc='best', ncol=len(configs))

            plt.tight_layout()
            pdf.savefig(fig)
            plt.close()

            # ---- PLOT 3: Stability index comparison (BIGGER box plot) ----
            fig, ax = plt.subplots(figsize=(14, 8))
            stability_data = []
            stability_labels = []
            for cfg in configs:
                subset = df[df['config'] == cfg]
                # Skip transition periods (first 5s after config change)
                if len(subset) > 250:
                    subset = subset.iloc[250:]
                stability_data.append(
                    np.degrees(subset['stability_index'].values)
                )
                stability_labels.append(f'{cfg}-config')

            bp = ax.boxplot(stability_data, labels=stability_labels,
                            patch_artist=True, widths=0.6,
                            medianprops=dict(color='black', linewidth=2))
            for i, cfg in enumerate(configs):
                bp['boxes'][i].set_facecolor(colors.get(cfg, 'gray'))
                bp['boxes'][i].set_alpha(0.6)

            ax.set_ylabel('Stability Index (deg, tilt magnitude)')
            ax.set_title(r'Hover Stability Comparison '
                         r'$(S = \sqrt{\phi^2 + \theta^2})$ — lower = more stable')
            ax.grid(True, alpha=0.3, axis='y')
            plt.tight_layout()
            pdf.savefig(fig)
            plt.close()

            # ---- PLOT 4: Power consumption (BIGGER) ----
            fig, ax = plt.subplots(figsize=(16, 9))
            for cfg in configs:
                subset = df[df['config'] == cfg]
                ax.plot(subset['timestamp_s'].to_numpy(),
                        subset['estimated_power_W'].to_numpy(),
                        label=f'{cfg}-config',
                        color=colors.get(cfg, 'gray'),
                        alpha=0.8, linewidth=1.8)

            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Estimated Power (W)')
            ax.set_title(r'Power Consumption by Configuration   '
                         r'$(P_{est} = \sum_{i=1}^{4} k \cdot T_i^{1.5})$')
            ax.legend(loc='best', ncol=2)
            plt.tight_layout()
            pdf.savefig(fig)
            plt.close()

            # ---- PLOT 5: Motor thrust distribution per config (BIGGER) ----
            n = len(configs)
            cols = 2 if n <= 4 else 3
            rows = int(np.ceil(n / cols))
            fig, axes = plt.subplots(rows, cols, figsize=(16, 5 * rows))
            if rows == 1 and cols == 1:
                axes = np.array([[axes]])
            elif rows == 1:
                axes = axes.reshape(1, -1)
            elif cols == 1:
                axes = axes.reshape(-1, 1)

            motor_cols = ['motor_1_N', 'motor_2_N', 'motor_3_N', 'motor_4_N']
            motor_labels = ['FR (arm 1)', 'FL (arm 2)', 'RL (arm 3)', 'RR (arm 4)']
            motor_colors = ['#3b82f6', '#22c55e', '#f59e0b', '#ef4444']

            for idx, cfg in enumerate(configs):
                ax = axes[idx // cols, idx % cols]
                subset = df[df['config'] == cfg]
                if len(subset) > 250:
                    subset = subset.iloc[250:]

                means = []
                for j, col in enumerate(motor_cols):
                    ax.hist(subset[col], bins=40, alpha=0.55,
                            label=motor_labels[j], color=motor_colors[j])
                    means.append(subset[col].mean())

                mean_str = " | ".join(f'{motor_labels[j].split()[0]}={means[j]:.2f}'
                                      for j in range(4))
                ax.set_xlabel('Thrust (N)')
                ax.set_ylabel('Sample count')
                ax.set_title(f'{cfg}-Config  ·  Mean: {mean_str}')
                ax.legend(fontsize=10)
                ax.grid(True, alpha=0.3)

            # Hide unused axes
            for idx in range(len(configs), rows * cols):
                axes[idx // cols, idx % cols].axis('off')

            plt.suptitle('Per-Motor Thrust Distribution (post-transient, 5s skip)',
                         fontsize=18, y=1.00)
            plt.tight_layout()
            pdf.savefig(fig)
            plt.close()

            # ---- PLOT 6: Bar chart summary (BIGGER) ----
            fig, axes = plt.subplots(1, 3, figsize=(18, 7))

            summary = {}
            for cfg in configs:
                subset = df[df['config'] == cfg]
                if len(subset) > 250:
                    subset = subset.iloc[250:]

                summary[cfg] = {
                    'alt_rmse': np.sqrt(np.mean(
                        (subset['pos_z'] - 5.0) ** 2
                    )),
                    'stability_mean': np.degrees(
                        subset['stability_index'].mean()
                    ),
                    'power_mean': subset['estimated_power_W'].mean(),
                }

            cfgs = list(summary.keys())
            x = np.arange(len(cfgs))

            # Altitude RMSE
            vals = [summary[c]['alt_rmse'] for c in cfgs]
            bars = axes[0].bar(x, vals,
                               color=[colors.get(c, 'gray') for c in cfgs],
                               alpha=0.8, edgecolor='black', linewidth=1)
            for bar, v in zip(bars, vals):
                axes[0].text(bar.get_x() + bar.get_width() / 2, v,
                             f'{v:.3f}', ha='center', va='bottom',
                             fontsize=11, fontweight='bold')
            axes[0].set_xticks(x)
            axes[0].set_xticklabels([f'{c}-config' for c in cfgs])
            axes[0].set_ylabel('RMSE (m)')
            axes[0].set_title(r'Altitude Hold RMSE' + '\n' +
                              r'$\sqrt{\langle (z - z_{target})^2 \rangle}$')
            axes[0].grid(True, alpha=0.3, axis='y')

            # Stability
            vals = [summary[c]['stability_mean'] for c in cfgs]
            bars = axes[1].bar(x, vals,
                               color=[colors.get(c, 'gray') for c in cfgs],
                               alpha=0.8, edgecolor='black', linewidth=1)
            for bar, v in zip(bars, vals):
                axes[1].text(bar.get_x() + bar.get_width() / 2, v,
                             f'{v:.2f}°', ha='center', va='bottom',
                             fontsize=11, fontweight='bold')
            axes[1].set_xticks(x)
            axes[1].set_xticklabels([f'{c}-config' for c in cfgs])
            axes[1].set_ylabel('Mean Tilt (deg)')
            axes[1].set_title(r'Attitude Stability' + '\n' +
                              r'$\langle \sqrt{\phi^2 + \theta^2} \rangle$')
            axes[1].grid(True, alpha=0.3, axis='y')

            # Power
            vals = [summary[c]['power_mean'] for c in cfgs]
            bars = axes[2].bar(x, vals,
                               color=[colors.get(c, 'gray') for c in cfgs],
                               alpha=0.8, edgecolor='black', linewidth=1)
            for bar, v in zip(bars, vals):
                axes[2].text(bar.get_x() + bar.get_width() / 2, v,
                             f'{v:.0f}W', ha='center', va='bottom',
                             fontsize=11, fontweight='bold')
            axes[2].set_xticks(x)
            axes[2].set_xticklabels([f'{c}-config' for c in cfgs])
            axes[2].set_ylabel('Mean Power (W)')
            axes[2].set_title(r'Power Consumption' + '\n' +
                              r'$\langle \sum_i k \cdot T_i^{1.5} \rangle$')
            axes[2].grid(True, alpha=0.3, axis='y')

            plt.suptitle('Per-Configuration Summary Metrics',
                         fontsize=18, y=1.02)
            plt.tight_layout()
            pdf.savefig(fig, bbox_inches='tight')
            plt.close()

        self.get_logger().info(f'Analysis PDF saved: {output_pdf}')

        # Print summary table
        self.get_logger().info('\n' + '=' * 70)
        self.get_logger().info('FLIGHT DATA SUMMARY')
        self.get_logger().info('=' * 70)
        self.get_logger().info(
            f'{"Config":<10} {"Alt RMSE (m)":<15} '
            f'{"Stability (deg)":<18} {"Avg Power (W)":<15}'
        )
        self.get_logger().info('-' * 70)
        for cfg in summary:
            s = summary[cfg]
            self.get_logger().info(
                f'{cfg + "-config":<10} {s["alt_rmse"]:<15.4f} '
                f'{s["stability_mean"]:<18.4f} {s["power_mean"]:<15.1f}'
            )
        self.get_logger().info('=' * 70)

        # Also save summary as CSV
        summary_path = os.path.join(self.data_dir, 'summary_stats.csv')
        summary_df = pd.DataFrame(summary).T
        summary_df.index.name = 'config'
        summary_df.to_csv(summary_path)
        self.get_logger().info(f'Summary CSV: {summary_path}')


def main(args=None):
    rclpy.init(args=args)
    node = FlightDataAnalyzer()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
