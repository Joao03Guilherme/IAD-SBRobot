"""Visualization tools for parameter training."""

import matplotlib.pyplot as plt
import numpy as np
import os
import threading
import time
from datetime import datetime

class TrainingVisualizer:
    """Real-time visualization for parameter training."""
    
    def __init__(self, param_ranges):
        """Initialize visualizer.
        
        Args:
            param_ranges: Dictionary of parameter ranges
        """
        self.param_ranges = param_ranges
        self.param_history = []
        self.reward_history = []
        self.score_history = []
        self.episode_markers = []
        self.episode_numbers = []
        
        # Create directory for plots
        os.makedirs('training_plots', exist_ok=True)
        
        # Setup plots - will be initialized in first update
        self.fig = None
        self.axs = None
        
        # Real-time update thread
        self.is_running = False
        self.thread = None
        self.lock = threading.Lock()  # Thread safety
    
    def add_data_point(self, parameters, reward, score, new_episode=False):
        """Add new data point to visualizer.
        
        Args:
            parameters: Dictionary of parameter values
            reward: Reward value
            score: Performance score (lower is better)
            new_episode: Whether this marks the start of a new episode
        """
        # Thread-safe data update
        with self.lock:
            self.param_history.append(parameters.copy())
            self.reward_history.append(reward)
            self.score_history.append(score)
            
            if new_episode:
                self.episode_markers.append(len(self.reward_history) - 1)
                # Extract episode number from length of episode_markers
                self.episode_numbers.append(len(self.episode_markers))
                print(f"Visualization: Starting episode {len(self.episode_markers)}")
    
    def start_real_time_visualization(self, update_interval=2.0):
        """Start real-time visualization in a separate thread.
        
        Args:
            update_interval: Time between plot updates in seconds
        """
        if self.is_running:
            return
            
        try:
            # Check if matplotlib is available
            import matplotlib
            # Use non-interactive backend if no display is available
            if os.environ.get('DISPLAY', '') == '':
                print("No display found. Using non-interactive Agg backend.")
                matplotlib.use('Agg')
            
            self.is_running = True
            self.thread = threading.Thread(target=self._update_plots_thread, args=(update_interval,))
            self.thread.daemon = True
            self.thread.start()
            print("Real-time visualization started")
        except ImportError:
            print("WARNING: Matplotlib not available. Visualization disabled.")
            self.is_running = False
        except Exception as e:
            print(f"Error starting visualization: {e}")
            self.is_running = False
    
    def _initialize_plots(self):
        """Initialize plot figure and axes if needed."""
        if self.fig is None:
            plt.ion()  # Enable interactive mode
            self.fig, self.axs = plt.subplots(3, 1, figsize=(10, 15))
            self.fig.tight_layout(pad=5.0)
            plt.subplots_adjust(hspace=0.4)  # Add space between subplots
    
    def _update_plots_thread(self, interval):
        """Thread for updating plots."""
        while self.is_running:
            try:
                with self.lock:
                    if not self.param_history:
                        # No data yet, wait for more
                        time.sleep(interval)
                        continue
                        
                # Update plots
                self.update_plots()
                
                # Pause to allow UI to update
                plt.pause(0.1)
                
                # Sleep for remaining interval time
                time.sleep(max(0.1, interval - 0.1))
            except Exception as e:
                print(f"Error updating plots: {e}")
                time.sleep(interval)
    
    def stop_visualization(self):
        """Stop real-time visualization."""
        self.is_running = False
        if self.thread:
            self.thread.join(timeout=1.0)
            self.thread = None
        
        # Save final plots
        try:
            if len(self.param_history) > 0:
                self.update_plots()
                self.save_plots("final")
        except Exception as e:
            print(f"Error saving final plots: {e}")
    
    def update_plots(self):
        """Update all plots with current data."""
        # Thread-safe copy of data
        with self.lock:
            if not self.param_history:
                return
                
            # Make copies to avoid thread issues
            param_history = [p.copy() for p in self.param_history]
            reward_history = self.reward_history.copy()
            score_history = self.score_history.copy()
            episode_markers = self.episode_markers.copy()
            episode_numbers = self.episode_numbers.copy()
        
        # Initialize plots if needed
        self._initialize_plots()
            
        # Clear axes
        for ax in self.axs:
            ax.clear()
            
        # Plot parameter evolution
        self._plot_parameters(self.axs[0], param_history, episode_markers, episode_numbers)
        
        # Plot reward history
        self._plot_rewards(self.axs[1], reward_history, episode_markers, episode_numbers)
        
        # Plot performance score
        self._plot_scores(self.axs[2], score_history, episode_markers, episode_numbers)
        
        # Update layout
        self.fig.tight_layout(pad=5.0)
        self.fig.canvas.draw_idle()
    
    def _plot_parameters(self, ax, param_history, episode_markers, episode_numbers):
        """Plot parameter evolution."""
        param_names = list(self.param_ranges.keys())
        x = range(len(param_history))
        
        for i, param in enumerate(param_names):
            # Extract parameter values, normalizing to [0,1] for comparison
            values = []
            for params in param_history:
                min_val, max_val = self.param_ranges[param]
                val = params.get(param, min_val)
                # Normalize to [0,1]
                norm_val = (val - min_val) / (max_val - min_val) if max_val > min_val else 0.5
                values.append(norm_val)
            
            # Plot with different colors and markers
            ax.plot(x, values, label=param, 
                   marker='o' if len(x) < 50 else None, 
                   markersize=4,
                   linewidth=2)
        
        # Mark episode boundaries
        self._add_episode_markers(ax, episode_markers, episode_numbers)
        
        ax.set_title('Parameter Evolution (Normalized)')
        ax.set_xlabel('Training Steps')
        ax.set_ylabel('Parameter Value (normalized)')
        ax.set_ylim(-0.1, 1.1)  # Give some padding
        ax.legend(loc='upper right', fontsize='small')
        ax.grid(True, alpha=0.3)
    
    def _plot_rewards(self, ax, reward_history, episode_markers, episode_numbers):
        """Plot reward history."""
        x = range(len(reward_history))
        ax.plot(x, reward_history, label='Reward', color='green', 
               marker='o' if len(x) < 50 else None,
               markersize=4)
        
        # Add moving average
        window = min(20, max(2, len(reward_history) // 5))
        if len(reward_history) > window:
            moving_avg = np.convolve(reward_history, np.ones(window)/window, mode='valid')
            ax.plot(range(window-1, len(reward_history)), moving_avg, 
                    label=f'{window}-step Moving Avg', color='darkgreen', linestyle='--',
                    linewidth=2)
        
        # Mark episode boundaries
        self._add_episode_markers(ax, episode_markers, episode_numbers)
        
        ax.set_title('Reward History')
        ax.set_xlabel('Training Steps')
        ax.set_ylabel('Reward')
        
        # Dynamic y-axis limits based on data
        if reward_history:
            min_r, max_r = min(reward_history), max(reward_history)
            padding = max(1.0, (max_r - min_r) * 0.1)
            ax.set_ylim(min_r - padding, max_r + padding)
            
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
    
    def _plot_scores(self, ax, score_history, episode_markers, episode_numbers):
        """Plot performance score history (lower is better)."""
        # Filter out infinite scores for plotting
        filtered_scores = [min(s, 1000) for s in score_history]  # Cap at 1000 for visualization
        
        x = range(len(filtered_scores))
        ax.plot(x, filtered_scores, label='Score', color='red',
               marker='o' if len(x) < 50 else None,
               markersize=4)
        
        # Add moving average
        window = min(20, max(2, len(filtered_scores) // 5))
        if len(filtered_scores) > window:
            moving_avg = np.convolve(filtered_scores, np.ones(window)/window, mode='valid')
            ax.plot(range(window-1, len(filtered_scores)), moving_avg, 
                    label=f'{window}-step Moving Avg', color='darkred', linestyle='--',
                    linewidth=2)
        
        # Mark episode boundaries
        self._add_episode_markers(ax, episode_markers, episode_numbers)
        
        ax.set_title('Performance Score (lower is better)')
        ax.set_xlabel('Training Steps')
        ax.set_ylabel('Score')
        
        # Dynamic y-axis limits based on data
        if filtered_scores:
            min_s, max_s = min(filtered_scores), max(filtered_scores)
            padding = max(1.0, (max_s - min_s) * 0.1)
            ax.set_ylim(0, max_s + padding)  # Start at 0 since scores are positive
            
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
    
    def _add_episode_markers(self, ax, episode_markers, episode_numbers):
        """Add episode boundary markers with numbers."""
        for i, marker in enumerate(episode_markers):
            if i < len(episode_numbers):
                episode = episode_numbers[i]
                ax.axvline(x=marker, color='gray', linestyle='--', alpha=0.5)
                # Add episode number text
                y_min, y_max = ax.get_ylim()
                text_y = y_max - (y_max - y_min) * 0.05
                ax.text(marker + 0.5, text_y, f"Ep {episode}", 
                       fontsize=8, color='gray', ha='left', va='top')
    
    def save_plots(self, filename_prefix='training_visualization'):
        """Save current plots to file."""
        if not self.param_history:
            print("No data to plot")
            return
            
        try:
            # Update plots before saving
            self.update_plots()
            
            # Create timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filepath = f'training_plots/{filename_prefix}_{timestamp}.png'
            
            # Save figure
            plt.savefig(filepath, dpi=300, bbox_inches='tight')
            print(f"Plots saved to {filepath}")
            
            # Also save with overwriting for latest version
            latest_path = f'training_plots/{filename_prefix}_latest.png'
            plt.savefig(latest_path, dpi=300, bbox_inches='tight')
        except Exception as e:
            print(f"Error saving plots: {e}")