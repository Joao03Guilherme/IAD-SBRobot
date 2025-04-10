"""Main script for training robot parameters using SAC."""

import argparse
import json
import time
import os
import sys
from raspberry.training.sac_trainer import SACTrainer
from raspberry.training.robot_interface import RobotInterface

def main():
    """Main function to run parameter training."""
    parser = argparse.ArgumentParser(description='Train self-balancing robot parameters using SAC')
    parser.add_argument('--episodes', type=int, default=30, help='Number of training episodes')
    parser.add_argument('--steps', type=int, default=100, help='Maximum steps per episode')
    parser.add_argument('--checkpoint', type=str, help='Load checkpoint file')
    parser.add_argument('--device', type=str, default='cpu', choices=['cpu', 'cuda'], help='Device to run on')
    parser.add_argument('--config', type=str, default='param_ranges.json', help='Parameter ranges configuration file')
    parser.add_argument('--warm-up', type=int, default=500, help='Warm-up steps with random actions')
    parser.add_argument('--connection', type=str, default='bluetooth', choices=['bluetooth', 'direct'], 
                        help='Connection method to robot')
    parser.add_argument('--no-visualization', action='store_true', help='Disable real-time visualization')
    parser.add_argument('--test-only', action='store_true', help='Only test best parameters without training')
    args = parser.parse_args()
    
    # Directory for saving results
    os.makedirs('results', exist_ok=True)
    os.makedirs('checkpoints', exist_ok=True)
    
    # Configure logging
    import logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(message)s',
        handlers=[
            logging.FileHandler(f'results/training_log_{time.strftime("%Y%m%d_%H%M%S")}.log'),
            logging.StreamHandler(sys.stdout)
        ]
    )
    logger = logging.getLogger(__name__)
    
    # Log startup info
    logger.info(f"Starting parameter training with args: {args}")
    
    # Load parameter ranges
    try:
        with open(args.config, 'r') as f:
            param_ranges = json.load(f)
        logger.info(f"Loaded parameter ranges from {args.config}")
    except Exception as e:
        logger.warning(f"Error loading parameter ranges: {e}")
        logger.info("Using default parameter ranges")
        param_ranges = {
            "balance_kp": (1.0, 10.0),
            "balance_ki": (0.0, 0.5),
            "balance_kd": (0.0, 2.0),
            "k_acc": (0.5, 5.0),
            "k_torque_per_pw": (0.1, 1.0),
            "drag": (0.5, 2.0),
            "max_safe_tilt": (3.0, 8.0),
        }
    
    # Initialize robot interface using existing communication modules
    logger.info("Connecting to robot...")
    robot = RobotInterface(connection_method=args.connection)
    if not robot.connected:
        logger.error("Failed to connect to robot. Exiting.")
        return
    
    # Check for visualization dependencies
    vis_available = True
    try:
        import matplotlib
        # Check if display is available
        if args.no_visualization:
            logger.info("Visualization disabled by command-line argument")
            vis_available = False
        elif os.environ.get('DISPLAY', '') == '' and not sys.platform.startswith('win'):
            logger.warning("No display detected. Setting non-interactive Agg backend.")
            matplotlib.use('Agg')
            # On headless systems, we can still save plots but not show them interactively
    except ImportError:
        logger.warning("Matplotlib not available. Visualization will be disabled.")
        vis_available = False

    # Initialize SAC trainer
    trainer = SACTrainer(
        param_ranges=param_ranges,
        robot_interface=robot,
        device=args.device,
        gamma=0.99,
        tau=0.005,
        lr=0.0003,
        batch_size=64,
        autotuned_entropy=True,
        use_visualization=vis_available and not args.no_visualization
    )
    
    # Load checkpoint if specified
    if args.checkpoint:
        logger.info(f"Loading checkpoint: {args.checkpoint}")
        trainer.load_checkpoint(args.checkpoint)
    
    # Test only mode
    if args.test_only:
        if not trainer.best_parameters:
            logger.error("No best parameters available. Run training first or provide a checkpoint.")
            return
            
        logger.info("Running test with best parameters:")
        for param, value in trainer.best_parameters.items():
            logger.info(f"  {param}: {value:.4f}")
            
        robot.update_parameters(trainer.best_parameters)
        
        for pattern in ["sine", "step", "random"]:
            logger.info(f"Testing with pattern: {pattern}")
            metrics = robot.run_test_pattern(duration=10.0, pattern_type=pattern)
            logger.info(f"Results: {metrics}")
            
        return
    
    # Run training
    try:
        logger.info("Starting training...")
        best_params = trainer.train(
            num_episodes=args.episodes,
            max_steps=args.steps,
            warm_up_steps=args.warm_up
        )
        
        # Save final results
        results_file = f"results/training_results_{time.strftime('%Y%m%d_%H%M%S')}.json"
        with open(results_file, 'w') as f:
            json.dump({
                'best_parameters': best_params,
                'best_score': trainer.best_score,
                'episode_rewards': trainer.episode_rewards
            }, f, indent=2)
        logger.info(f"Results saved to {results_file}")
        
    except KeyboardInterrupt:
        logger.info("\nTraining interrupted by user.")
    except Exception as e:
        logger.error(f"Error during training: {e}", exc_info=True)
    finally:
        # Disconnect from robot
        logger.info("Disconnecting from robot...")
        robot.disconnect()
        logger.info("Robot disconnected.")

if __name__ == "__main__":
    main()