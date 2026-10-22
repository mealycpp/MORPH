#!/usr/bin/env python3
"""
Train RL agent for morphing drone stabilization.
Uses PPO from Stable-Baselines3.

Usage:
  pip install gymnasium stable-baselines3[extra] torch --break-system-packages
  python3 train_agent.py --quick    # ~5 min test
  python3 train_agent.py            # ~30 min full training
  python3 train_agent.py --eval     # evaluate existing model
"""
import os, sys, json, time, argparse
import numpy as np

def check_deps():
    missing = []
    for pkg in ['gymnasium', 'stable_baselines3', 'torch']:
        try:
            __import__(pkg)
        except ImportError:
            missing.append(pkg.replace('_', '-'))
    if missing:
        print(f"Install missing: pip install {' '.join(missing)} --break-system-packages")
        sys.exit(1)

check_deps()

import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv
from stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback
from stable_baselines3.common.monitor import Monitor

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ml.morph_drone_env import MorphDroneEnv, CONFIG_NAMES

OUTPUT_DIR = os.path.expanduser("~/morphing_drone_data/ml")
os.makedirs(OUTPUT_DIR, exist_ok=True)

def make_env(rank, seed=0):
    def _init():
        env = Monitor(MorphDroneEnv())
        env.reset(seed=seed + rank)
        return env
    return _init

def train(args):
    print("=" * 50)
    print("MORPHING DRONE RL TRAINING")
    print("=" * 50)

    if args.quick:
        total_timesteps, n_envs, batch_size, n_epochs = 200_000, 4, 256, 5
        print("Quick mode: 200K steps")
    else:
        total_timesteps, n_envs, batch_size, n_epochs = 2_000_000, 8, 512, 10
        print(f"Full training: 2M steps, {n_envs} envs")

    env = SubprocVecEnv([make_env(i) for i in range(n_envs)])
    eval_env = DummyVecEnv([make_env(99, seed=42)])

    eval_cb = EvalCallback(eval_env, best_model_save_path=OUTPUT_DIR,
                           log_path=OUTPUT_DIR, eval_freq=max(total_timesteps // 20, 5000),
                           n_eval_episodes=10, deterministic=True)
    ckpt_cb = CheckpointCallback(save_freq=max(total_timesteps // 10, 10000),
                                 save_path=os.path.join(OUTPUT_DIR, "checkpoints"),
                                 name_prefix="morph_drone")

    model_path = os.path.join(OUTPUT_DIR, "best_model.zip")
    if args.resume and os.path.exists(model_path):
        print(f"Resuming from {model_path}")
        model = PPO.load(model_path, env=env)
    else:
        model = PPO("MlpPolicy", env, learning_rate=3e-4, n_steps=2048,
                     batch_size=batch_size, n_epochs=n_epochs, gamma=0.99,
                     gae_lambda=0.95, clip_range=0.2, ent_coef=0.01,
                     policy_kwargs=dict(net_arch=dict(pi=[256, 256, 128], vf=[256, 256, 128])),
                     verbose=1, tensorboard_log=os.path.join(OUTPUT_DIR, "tb_logs"))

    t0 = time.time()
    model.learn(total_timesteps=total_timesteps, callback=[eval_cb, ckpt_cb], progress_bar=True)
    print(f"\nDone in {(time.time()-t0)/60:.1f} min. Model: {model_path}")
    model.save(os.path.join(OUTPUT_DIR, "final_model.zip"))
    env.close(); eval_env.close()
    return model

def evaluate(model=None):
    print("\nEVALUATING TRAINED AGENT")
    model_path = os.path.join(OUTPUT_DIR, "best_model.zip")
    if model is None:
        if not os.path.exists(model_path):
            print(f"No model at {model_path}. Train first."); return
        model = PPO.load(model_path)

    results = {}
    for fc in CONFIG_NAMES:
        for tc in CONFIG_NAMES:
            if fc == tc: continue
            key = f"{fc}_to_{tc}"
            morph_times, att_errors, completions = [], [], 0
            for _ in range(20):
                env = MorphDroneEnv()
                obs, _ = env.reset(options={"from_config": fc, "to_config": tc})
                done = False
                ep_att = []
                while not done:
                    action, _ = model.predict(obs, deterministic=True)
                    obs, _, terminated, truncated, info = env.step(action)
                    done = terminated or truncated
                    ep_att.append(info["attitude_error_deg"])
                att_errors.append(np.mean(ep_att))
                if info["morph_progress"] >= 0.99:
                    completions += 1
                    morph_times.append((env.step_count - env.morph_start_step) * env.dt)

            results[key] = {
                "from": fc, "to": tc,
                "completion_rate": completions / 20,
                "avg_morph_time_s": float(np.mean(morph_times)) if morph_times else 0,
                "avg_attitude_error_deg": float(np.mean(att_errors)),
            }
            r = results[key]
            print(f"  {fc}->{tc}: {r['completion_rate']:.0%} complete, {r['avg_morph_time_s']:.2f}s, {r['avg_attitude_error_deg']:.1f} deg err")

    with open(os.path.join(OUTPUT_DIR, "eval_results.json"), "w") as f:
        json.dump(results, f, indent=2)
    timing = {k: {"recommended_time_s": round(v["avg_morph_time_s"], 2)}
              for k, v in results.items() if v["completion_rate"] > 0}
    with open(os.path.join(OUTPUT_DIR, "morph_timing_table.json"), "w") as f:
        json.dump(timing, f, indent=2)
    print(f"\nResults: {OUTPUT_DIR}/eval_results.json")

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--quick", action="store_true")
    p.add_argument("--resume", action="store_true")
    p.add_argument("--eval", action="store_true")
    args = p.parse_args()
    if args.eval:
        evaluate()
    else:
        model = train(args)
        evaluate(model)

if __name__ == "__main__":
    main()
