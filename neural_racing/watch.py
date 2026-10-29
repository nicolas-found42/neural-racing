"""Watch a trained deep residual policy drive with the interactive telemetry dashboard."""

import os

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

import argparse

import numpy as np
import pygame

from .game import DECISION_DT, Env, build_screen, draw_scene
from .network import Policy

FPS = 30


def main():
    parser = argparse.ArgumentParser(
        description="Watch the trained car drive a lap with full brain visualization."
    )
    parser.add_argument("--weights", default="weights.npz")
    parser.add_argument(
        "--episodes", type=int, default=0, help="number of runs, 0 means forever"
    )
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    policy = Policy.load(args.weights)
    env = Env(seed=args.seed)
    screen = build_screen("Watch Mode - Deep Neural Network & Car Telemetry")
    font = pygame.font.SysFont("menlo", 14)
    clock = pygame.time.Clock()
    rng = np.random.default_rng(args.seed)

    episode = 1
    lap_time = 0.0
    best = 0.0
    laps_done = 0
    history_dist = []
    obs = env.reset()
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (
                event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE
            ):
                running = False

        action, _, _ = policy.act(obs, 0.0, rng)
        obs, reward, done, info = env.step(action)
        lap_time += DECISION_DT
        best = max(best, info["distance"])

        stats = {
            "episode": episode,
            "total_episodes": args.episodes,
            "sigma": 0.0,
            "best_distance": best,
            "laps": laps_done,
            "history_distance": history_dist + [info["distance"]],
        }

        draw_scene(screen, env, font, policy=policy, stats=stats)
        pygame.display.flip()
        clock.tick(FPS)

        if done:
            history_dist.append(info["distance"])
            if info["lap_done"]:
                laps_done += 1
            print(
                f"episode {episode}: distance {info['distance']:.0f} px, "
                f"lap {'yes' if info['lap_done'] else 'no'}, "
                f"off track {'yes' if info['off_track'] else 'no'}, {lap_time:.2f}s"
            )
            if args.episodes and episode >= args.episodes:
                running = False
            episode += 1
            obs = env.reset()
            lap_time = 0.0

    pygame.quit()


if __name__ == "__main__":
    main()
