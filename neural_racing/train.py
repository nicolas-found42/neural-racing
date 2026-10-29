"""Train the driving policy with REINFORCE."""

import os

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

import argparse
import time

import numpy as np
import pygame

from .game import Env, build_screen, draw_scene
from .network import Adam, Policy

GAMMA = 0.97
LR = 1e-3
SIGMA_START = 0.4
SIGMA_END = 0.08
SIGMA_DECAY = 2000
EPISODES_PER_UPDATE = 4


def sigma_at(episode):
    frac = min(1.0, (episode - 1) / SIGMA_DECAY)
    return SIGMA_START + (SIGMA_END - SIGMA_START) * frac


def discounted_returns(rewards):
    returns = np.zeros(len(rewards))
    running = 0.0
    for t in reversed(range(len(rewards))):
        running = rewards[t] + GAMMA * running
        returns[t] = running
    return returns


def run_episode(env, policy, sigma, rng):
    """Collect one episode. Returns observations, eps, discounted returns, info."""
    obs = env.reset()
    obs_log, eps_log, rew_log = [], [], []
    while True:
        action, eps, _ = policy.act(obs, sigma, rng)
        obs_log.append(obs)
        eps_log.append(eps)
        obs, reward, done, info = env.step(action)
        rew_log.append(reward)
        if done:
            info["reward"] = float(sum(rew_log))
            return obs_log, np.stack(eps_log), discounted_returns(rew_log), info

def update(policy, optimizer, obs, eps, returns, sigma):
    """Ascend the policy gradient over a batch of episodes.

    Advantages are normalized across the whole batch so episodes compete with
    each other. Normalizing per episode would erase the difference between a
    clean run and a crash.
    """
    adv = (returns - returns.mean()) / (returns.std() + 1e-8)
    # d log p / dz for a Gaussian in pre-tanh space is eps / sigma, so the tanh
    # Jacobian never enters the gradient.
    dz = adv[:, None] * eps / sigma
    _, cache = policy.forward(obs)
    grads = policy.backward(dz, cache)
    # Adam steps downhill, and this gradient points uphill, so negate it.
    optimizer.step(policy.params, {k: -v / len(obs) for k, v in grads.items()})


def train(episodes=500, render=False, out="weights.npz", seed=0, verbose=True):
    env = Env(seed=seed)
    policy = Policy(seed=seed)
    optimizer = Adam(policy.params, lr=LR)
    rng = np.random.default_rng(seed + 1)

    screen = font = None
    if render:
        screen = build_screen("Training")
        font = pygame.font.SysFont("menlo", 14)

    history = {"distance": [], "reward": [], "lap": [], "off_track": []}
    obs_batch, eps_batch, ret_batch = [], [], []
    best_distance = -1.0

    for episode in range(1, episodes + 1):
        sigma = sigma_at(episode)
        obs_log, eps_log, ret_log, info = run_episode(env, policy, sigma, rng)
        obs_batch += obs_log
        eps_batch.append(eps_log)
        ret_batch.append(ret_log)

        history["distance"].append(info["distance"])
        history["reward"].append(info["reward"])
        history["lap"].append(info["lap_done"])
        history["off_track"].append(info["off_track"])

        if render:
            draw_scene(screen, env, font, [
                f"episode {episode}   sigma {sigma:.2f}",
                f"distance {info['distance']:6.0f} / {env.track.LAP_LENGTH:.0f}",
                f"speed {env.speed:5.0f}   reward {ret_log[0]:7.2f}",
                f"avg distance {np.mean(history['distance'][-20:]):6.0f}",
            ])
            pygame.display.flip()
            pygame.event.pump()

        if episode % EPISODES_PER_UPDATE == 0:
            update(policy, optimizer, np.stack(obs_batch), np.concatenate(eps_batch),
                   np.concatenate(ret_batch), sigma)
            obs_batch, eps_batch, ret_batch = [], [], []

        if info["distance"] > best_distance:
            best_distance = info["distance"]
            policy.save(out)

        if verbose and episode % 10 == 0:
            recent = history["distance"][-20:]
            print(f"ep {episode:4d}  dist {np.mean(recent):6.1f}  best {best_distance:6.1f}  "
                  f"reward {np.mean(history['reward'][-20:]):7.2f}  laps {sum(history['lap'])}  "
                  f"off {sum(history['off_track'][-20:])}/20  sigma {sigma:.2f}", flush=True)

    if render:
        pygame.quit()
    return policy, history


def main():
    parser = argparse.ArgumentParser(description="Train a self-driving car with REINFORCE.")
    parser.add_argument("--episodes", type=int, default=800)
    parser.add_argument("--render", action="store_true", help="show the training in a pygame window")
    parser.add_argument("--out", default="weights.npz")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    started = time.perf_counter()
    policy, history = train(args.episodes, args.render, args.out, args.seed)
    elapsed = time.perf_counter() - started
    print(f"saved {args.out} ({policy.num_params()} parameters, "
          f"{elapsed:.1f}s for {args.episodes} episodes)")


if __name__ == "__main__":
    main()
